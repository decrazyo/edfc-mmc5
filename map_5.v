

module map_5
(map_out, cpu_dat, cpu_addr, cpu_ce, cpu_rw, m2,
ppu_dat, ppu_addr, ppu_oe, ppu_we, 
clk, map_cfg, rst, mmc_cpu_do, mmc_cpu_out, mmc_ppu_do, mmc_ppu_out);
 
	
	output [48:0]map_out;
 
	//cpu bus
	input [7:0]cpu_dat;
	input [14:0]cpu_addr;
	input cpu_ce, cpu_rw, m2;
	
	//ppu bus
	input [7:0]ppu_dat;
	input [13:0]ppu_addr;
	input ppu_oe, ppu_we;
	
	input clk, rst;
	input [7:0]map_cfg;
	//input [7:0]map_idx;
	
	//mapper outputs
	wire ciram_a10, ciram_ce;
	wire [19:0]prg_addr;
	wire [19:0]chr_addr;
	wire prg_we;
	wire chr_we;
	wire irq;
	wire rom_ce;
	wire ram_ce;
	wire snd = 1'bzz;
	wire chr_ce;
	
	assign map_out[48:0] = {snd, chr_ce, ram_ce, rom_ce, irq, chr_we, prg_we, ciram_ce, ciram_a10, chr_addr[19:0], prg_addr[19:0]};
	
	output [7:0]mmc_cpu_do;
	output mmc_cpu_out;
	output [7:0]mmc_ppu_do;
	output mmc_ppu_out;
	//*************************************************************

	assign prg_we = !cpu_rw & ram_ce & m2;
	assign ram_ce = ram_mank_ce | (cpu_ce & cpu_addr[14:13] == 2'b11);
	assign rom_ce = !cpu_ce & !ram_mank_ce;
	assign chr_ce = !ppu_addr[13];
	assign chr_we = 0;//map_cfg[7] ? !ppu_we & ciram_ce : 0;

	//*************************************************************mirroring
	//A10-Vmir, A11-Hmir

	assign ciram_a10 = ntb_ctrl[0];//current_mir;
		
	//assign ciram_ce = !ppu_addr[13];
	
	wire [1:0]ntb_ctrl = 
	ppu_addr[11:10] == 0 ? ntb_map[1:0] : 
	ppu_addr[11:10] == 1 ? ntb_map[3:2] : 
	ppu_addr[11:10] == 2 ? ntb_map[5:4] : ntb_map[7:6];
	
	//assign ciram_ce = !ppu_addr[13];
	
	assign ciram_ce = ppu_addr[13:12] == 2'b10 & !ntb_ctrl[1] ? 0 : 1;
	
	wire fill_mode = ntb_ctrl[1:0] == 2'b11 & ppu_addr[13];
	wire fill_at_ce = fill_mode & ppu_addr[9:6] == 6'b1111;
	wire fill_nt_ce = fill_mode & ppu_addr[9:6] != 6'b1111;
	wire ppu_ext_ce = ntb_ctrl[1:0] == 2'b10 & ppu_addr[13] & ext_cfg == 1;
	
	assign mmc_ppu_do[7:0] = fill_nt_ce ? tile_idx[7:0] : fill_at_ce ? {tile_idx[1:0], tile_idx[1:0], tile_idx[1:0], tile_idx[1:0]} : ex_do[7:0];
	assign mmc_ppu_out = ppu_oe ? 0 : fill_at_ce | fill_nt_ce | ppu_ext_ce;

//***************************************************************** chr stuff		
	assign chr_addr[9:0] = ppu_addr[9:0];
	
	assign chr_addr[18:10] = sprite_mode ? (sprt_fetch ? chr_ax[8:0] : chr_bx[8:0]) : last_set ? chr_bx[8:0] : chr_ax[8:0];
	
	wire [9:0]chr_ax = chr_md[chr_mode];
	wire [9:0]chr_bx = chr_md_b[chr_mode];
	
	
	
	wire [9:0]chr_md[4];
	assign chr_md[0][9:0] = {chr_a[7][6:0], ppu_addr[12:10]};
	assign chr_md[1][9:0] = !ppu_addr[12] ? {chr_a[3][7:0], ppu_addr[11:10]} : {chr_a[7][7:0], ppu_addr[11:10]};
	assign chr_md[2][9:0] = 
	ppu_addr[12:11] == 0 ? {chr_a[1][8:0], ppu_addr[10]} : 
	ppu_addr[12:11] == 1 ? {chr_a[3][8:0], ppu_addr[10]} : 
	ppu_addr[12:11] == 2 ? {chr_a[5][8:0], ppu_addr[10]} : {chr_a[7][8:0], ppu_addr[10]}; 
	assign chr_md[3][9:0] = chr_a[ppu_addr[12:10]][9:0];
	
	wire [9:0]chr_md_b[4];
	assign chr_md_b[0] = {chr_b[3][6:0], 1'b0, ppu_addr[11:10]};
	assign chr_md_b[1] = {chr_b[3][7:0], ppu_addr[11:10]};
	assign chr_md_b[2] = !ppu_addr[11] ? {chr_b[1][8:0], ppu_addr[10]} : {chr_b[3][8:0], ppu_addr[10]};
	assign chr_md_b[3] = chr_b[ppu_addr[11:10]];
	
//***************************************************************** prg stuff	
	assign prg_addr[12:0] = cpu_addr[12:0];
	assign prg_addr[18:13] = cpu_ce ? {3'b000, ram_bank[2:0]} : prg_bk[5:0];
	
	wire ram_mank_ce;// = prg_bak_idx != 3 & prg_bank[prg_bak_idx][7] == 0 & !cpu_ce;
	
	wire [5:0]prg_bk = cpu_ce | ram_mank_ce ? {3'b000, prg_bank[prg_bak_idx][2:0]} : prg_md[prg_mode][5:0];
	wire [7:0]prg_md[4];
	assign prg_md[0][7:0] = {prg_bank[3][7:2], cpu_addr[14:13]};
	assign prg_md[1][7:0] = !cpu_addr[14] ? {prg_bank[1][7:1], cpu_addr[13]} : {prg_bank[3][7:1], cpu_addr[13]};
	assign prg_md[2][7:0] = !cpu_addr[14] ? {prg_bank[1][7:1], cpu_addr[13]} : !cpu_addr[13] ? prg_bank[2][7:0] : prg_bank[3][7:0];
	assign prg_md[3][7:0] = prg_bank[cpu_addr[14:13]][7:0];
	
	wire [1:0]prg_bak_idx = prg_bak_idx_[prg_mode];
	wire [1:0]prg_bak_idx_[4];
	assign prg_bak_idx_[0][1:0] = 3;
	assign prg_bak_idx_[1][1:0] = !cpu_addr[14] ? 1 : 3;
	assign prg_bak_idx_[2][1:0] = !cpu_addr[14] ? 1 : !cpu_addr[13] ? 2 : 3;
	assign prg_bak_idx_[3][1:0] = cpu_addr[1:0];

//***************************************************************** 
	
	wire reg_ce_51 = cpu_addr[14:8] == 7'h51 & cpu_ce & !rst;
	wire reg_ce_52 = cpu_addr[14:8] == 7'h52 & cpu_ce & !rst;
	wire prg_bank_we =  !cpu_rw & reg_ce_51 & cpu_addr[7:4] == 4'h01 & cpu_addr[3:2] == 2'b01;
	wire chr_bank_a_we = !cpu_rw & reg_ce_51 & cpu_addr[7:4] == 4'h02 & cpu_addr[3] == 0;
	wire chr_bank_b_we = !cpu_rw & reg_ce_51 & cpu_addr[7:4] == 4'h02 & cpu_addr[3:2] == 2'b10;
	
	
	reg [1:0]prg_mode;
	reg [7:0]prg_bank[4];
	reg [2:0]ram_bank;
	reg [7:0]ntb_map;
	reg [7:0]tile_idx;
	reg [1:0]tile_at;
	reg [1:0]ext_cfg;
	reg sprite_mode;
	
	reg [9:0]chr_a[8];
	reg [9:0]chr_b[4];
	reg [1:0]chr_hi;
	reg [1:0]chr_mode;
	
	reg bgr_on;
	reg last_set;
	
	reg [7:0]mul_a;
	reg [7:0]mul_b;
	reg [15:0]mul_rez;
	
	always @(negedge m2)
	begin
		
		if(rst)
		begin
			//prg_bank[0] <= 8'hff;
			//prg_bank[1] <= 8'hff;
			//prg_bank[2] <= 8'hff;
			prg_bank[0][7] <= 1;
			prg_bank[1][7] <= 1;
			prg_bank[2][7] <= 1;
			prg_bank[3] <= 8'hff;
			prg_mode[1:0] <= 2'h3;
		end
		
		if(chr_bank_a_we)last_set <= 0;
		if(chr_bank_b_we)last_set <= 1;
		//if(!cpu_rw & cpu_addr[14:0] == 15'h5100 & cpu_ce)prg_mode[1:0] <= 3;//cpu_dat[1:0];
		
		if(!cpu_rw & cpu_addr[14:0] == 15'h2000 & cpu_ce)sprite_mode <= cpu_dat[5];
		if(!cpu_rw & cpu_addr[14:0] == 15'h2001 & cpu_ce)bgr_on <= cpu_dat[3];
		
		if(prg_bank_we)prg_bank[cpu_addr[1:0]][7:0] <= cpu_dat[7:0];
		if(chr_bank_a_we)chr_a[cpu_addr[2:0]][9:0] <= {chr_hi[1:0], cpu_dat[7:0]};
		if(chr_bank_b_we)chr_b[cpu_addr[1:0]][9:0] <= {chr_hi[1:0], cpu_dat[7:0]};
		
		
		
		if(!cpu_rw & reg_ce_51)
		case(cpu_addr[7:0])
			8'h00:prg_mode[1:0] <= cpu_dat[1:0];
			8'h01:chr_mode[1:0] <= cpu_dat[1:0];
			8'h04:ext_cfg[1:0] <= cpu_dat[1:0];
			8'h05:ntb_map[7:0] <= cpu_dat[7:0];
			8'h06:tile_idx[7:0] <= cpu_dat[7:0];
			8'h07:tile_at[1:0] <= cpu_dat[1:0];
			8'h13:ram_bank[2:0] <= cpu_dat[2:0];
			8'h30:chr_hi[1:0] <= cpu_dat[1:0];
		endcase
		
		if(!cpu_rw & reg_ce_52)
		case(cpu_addr[7:0])
			8'h03:irq_val[7:0] <= cpu_dat[7:0];
			8'h04:irq_on <= cpu_dat[7];
			8'h05:begin
				mul_a <= cpu_dat[7:0];
				mul_rez <= cpu_dat * mul_b;
			end
			8'h06:begin
				mul_b <= cpu_dat[7:0];
				mul_rez <= cpu_dat * mul_a;
			end
		endcase
		
		if(cpu_rw & reg_ce_52 & cpu_addr[7:0] == 8'h04)irq_pend <= 0;
		
		nt_strobe_st[7:0] <= {nt_strobe_st[6:0], ppu_nt_strobe};
		
		if(!bgr_on | nt_strobe_st == 255 | nt_strobe_st == 0)in_frame <= 0;
			else
		if(!in_frame)
		begin
			irq_ctr <= 0;
			in_frame <= 1;
			irq_pend <= 0;
		end
			else
		if(ppu_line_strobe != ppu_line_strobe_st)
		begin
			//line_ctr <= 0;
			irq_ctr <= irq_ctr + 1;
			if(irq_ctr+1 == irq_val)irq_pend <= 1;
		end
		
		
		ppu_line_strobe_st <= ppu_line_strobe;
		
		
	end
	
	reg [8:0]line_ctr;
	
	wire status_oe = cpu_rw & reg_ce_52 & cpu_addr[7:0] == 4'h04;
	wire mul_lo_oe = cpu_rw & reg_ce_52 & cpu_addr[7:0] == 4'h05;
	wire mul_hi_oe = cpu_rw & reg_ce_52 & cpu_addr[7:0] == 4'h06;
	wire ex_ram_oe = cpu_rw & ex_ram_ce;
	
	assign mmc_cpu_do[7:0] = 
	ex_ram_ce ? ex_do : 
	status_oe ? {irq_pend, in_frame, 6'd0} : 
	mul_lo_oe ? mul_rez[7:0] :
	mul_hi_oe ? mul_rez[15:8] : 0;
	
	assign mmc_cpu_out = status_oe | mul_lo_oe | mul_hi_oe | ex_ram_oe;
	
	assign irq = irq_pend & irq_on;
	reg [7:0]irq_val;
	reg irq_on;
	reg irq_pend;
	
	reg [7:0]irq_ctr;
	reg [7:0]nt_strobe_st;
	reg ppu_line_strobe_st;
	reg in_frame;
	reg irq_flag;
	
	wire nt_ce = ppu_addr[13:12] == 2'b10 &  ppu_addr[9:6] != 6'b1111;
	reg ppu_line_strobe;
	reg ppu_nt_strobe;
	wire sprt_fetch = line_ctr > 128-3 & line_ctr < 171 - 11;
	
	reg [13:0]last_ppu_addr;
	reg ppu_addr_eq;
	
	always @(negedge ppu_oe)
	begin
		
		ppu_addr_eq <= last_ppu_addr == ppu_addr;
		
		if(last_ppu_addr == ppu_addr & ppu_addr_eq)
		begin
			line_ctr <= 0;
		end
			else
		line_ctr <= line_ctr + 1;
	
		if(line_ctr == 170-1)ppu_line_strobe <= !ppu_line_strobe;
		
		last_ppu_addr <= ppu_addr;
		
		
		if(nt_ce)ppu_nt_strobe <= !ppu_nt_strobe;
	end
	
	
	wire ex_ram_ce = cpu_addr[14:10] == 5'h17 & cpu_ce;
	wire ex_we = ext_cfg[1] == 0 ? (ppu_ext_ce & !ppu_we) : ext_cfg[1:0] == 2 & !cpu_rw & ex_ram_ce & m2;
	wire ex_clk = clk;
	wire [7:0]ex_do;
	wire [7:0]ex_di = ext_cfg[1] == 0 ? ppu_dat[7:0] : cpu_dat[7:0];
	wire [9:0]ex_addr = ext_cfg[1] == 0 ? ppu_addr[9:0] : cpu_addr[9:0];
	exram exram_inst(ex_addr, ex_clk, ex_di, ex_we, ex_do);

endmodule

