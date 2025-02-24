module map_255 //OSMAP
(map_out, cpu_dat, cpu_addr, cpu_ce, cpu_rw, m2,
ppu_dat, ppu_addr, ppu_oe, ppu_we,
clk, map_cfg, map_idx, rst_out, prg_mask, chr_mask, os_map_dout, os_map_oe, m2_ok, bios_off);
 

	output [48:0]map_out;
	 
	//cpu bus
	input [7:0]cpu_dat;
	input [14:0]cpu_addr;
	input cpu_ce, cpu_rw, m2;
	 
	//ppu bus
	input [7:0]ppu_dat;
	input [13:0]ppu_addr;
	input ppu_oe, ppu_we;

	input clk, bios_off;
	output reg [7:0]map_cfg;
	output reg [7:0]map_idx;
	output reg rst_out;
	reg rst;

	//mapper outputs
	wire ciram_a10, ciram_ce;
	wire [19:0]prg_addr;
	wire [19:0]chr_addr;
	wire prg_we;
	wire chr_we;	 
	wire irq;
	wire rom_ce;
	wire ram_ce;
	wire chr_ce;
	output [4:0]prg_mask;
	output [5:0]chr_mask;
	
	initial map_idx[7:0] = 8'hff;
	wire snd;
	assign map_out[48:0] = {snd, chr_ce, ram_ce, rom_ce, irq, chr_we, prg_we, ciram_ce, ciram_a10, chr_addr[19:0], prg_addr[19:0]};
	
	
	output [7:0]os_map_dout;
	output os_map_oe;
	output m2_ok;
	//*************************************************************
	
	parameter REG_PRG_MAPA = 128;
	parameter REG_SRM_MAPA = 129;
	parameter REG_CHR_MAPA = 130;
	parameter REG_MAPPER = 131;
	parameter REG_PRG_SIZE = 132;
	parameter REG_CHR_SIZE= 133;
	parameter REG_MAP_CFG = 134;
	parameter REG_GG = 135;
	parameter REG_CART_CFG = 136;
	 
	assign prg_we = !cpu_rw & m2;
	assign chr_we = !ppu_we;
	assign chr_ce = ciram_ce;
	 
	assign ciram_a10 = 0;
	assign ciram_ce = !ppu_addr[13];
	 
	assign prg_addr[9:0] = cpu_addr[9:0];
	assign prg_addr[18:10] = 
	!cpu_ce ? {4'b0011, cpu_addr[14:10]} : 
	ram_area ? {reg_srm_map[5:0], cpu_addr[12:10]} :
	{reg_prg_map[7:0], cpu_addr[10]};
	
	assign chr_addr[11:0] = ppu_addr[11:0];
	assign chr_addr[18:12] = ppu_addr[12] ? reg_chr_map[6:0] : reg_chr_map[6:0]-1;
	 
	assign rom_ce = exp_area & m2;
	assign ram_ce = !m2 ? 0 : bios_off ? (rom_area | ram_area) : ram_area;

	reg [7:0]reg_prg_map;
	reg [5:0]reg_srm_map;
	reg [6:0]reg_chr_map;
	reg [4:0]prg_size;
	reg [5:0]chr_size;
	
	wire exp_area = cpu_addr[14:11] == 4'b1001 & cpu_ce;
	wire ram_area = cpu_addr[14:13] == 2'b11 & cpu_ce;
	wire rom_area = !cpu_ce;
	wire regs_area =  cpu_ce & cpu_addr[14:9] == 6'h22;
	wire cmd_addr_ce = regs_area & cpu_addr[1:0] == 2'b00;
	wire cmd_data_ce = regs_area & cpu_addr[1:0] == 2'b01 & osm_map_on;
	
	reg [7:0]cmd_addr;
	reg reset_mode;
	
	assign prg_mask[4:0] = osm_map_on ? 5'b11111 : prg_size[4:0];
	assign chr_mask[5:0] = osm_map_on ? 6'b111111 : chr_size[5:0];
	
	wire osm_map_on = map_idx[7:0] == 8'hff;
	 
	always @(negedge m2)
	begin
		
		m2_strobe <= !m2_strobe;
		
		if(cmd_addr_ce & !cpu_rw)
		begin
			cmd_addr[7:0] <= cpu_dat[7:0];
		end

		if(rst)
		begin
			map_idx[7:0] <= 8'hff;
		end
			else
		if(cmd_data_ce & !cpu_rw)
		begin
			
			case(cmd_addr[7:0])
				
				
				REG_PRG_MAPA:begin
					reg_prg_map[7:0] <= cpu_dat[7:0];
				end
				
				REG_SRM_MAPA:begin
					reg_srm_map[5:0] <= cpu_dat[5:0];
				end
				
				REG_CHR_MAPA:begin
					reg_chr_map[6:0] <= cpu_dat[6:0];
				end
				
				REG_MAPPER:begin
					map_idx[7:0] <= cpu_dat[7:0];
				end
				
				REG_PRG_SIZE:begin
					prg_size[4:0] <= cpu_dat[4:0];
				end
				
				REG_CHR_SIZE:begin
					chr_size[5:0] <= cpu_dat[5:0];
					map_cfg[7] <= cpu_dat[7];
				end
				
				REG_MAP_CFG:begin
					map_cfg[6:0] <= cpu_dat[6:0];
				end
				
				REG_CART_CFG:begin
					reset_mode <= cpu_dat[0];
				end
				
			endcase
		end
		
	end
	

	reg [25:0]cpu_rst_ctr;
	reg m2_strobe;
	reg m2_strobe_st;
	
	
	reg [3:0]m2_st;
	assign m2_ok = m2_st[3:0] == 4'b1111 & m2;
	assign os_map_oe = gg_act;
	
	always @(negedge clk)
	begin
	
		m2_st <= {m2_st[2:0], m2};
	 
      m2_strobe_st <= m2_strobe;
		if(m2_strobe_st != m2_strobe)
		begin
			cpu_rst_ctr <= 0;
			rst <= 0;
			rst_out <= 0;
		end
			else
		begin
			if(cpu_rst_ctr[7])rst_out <= 1;
			
			if(reset_mode)
			begin
				if(!cpu_rst_ctr[25])cpu_rst_ctr <= cpu_rst_ctr + 1;
				rst <= cpu_rst_ctr[25];
			end
				else
			begin
				if(!cpu_rst_ctr[7])cpu_rst_ctr[7:0] <= cpu_rst_ctr[7:0] + 1;
				rst <= cpu_rst_ctr[7];
			end
		end
	end
	
	
	assign os_map_dout[7:0] =  gg_dout[7:0];
	wire gg_on = !osm_map_on;
	 
	wire [7:0]gg_dout;
	wire gg_ce = cmd_data_ce & cmd_addr == REG_GG;
	wire gg_act;
	gg_cnt gg_control(cpu_dat, cpu_addr, m2, cpu_ce, cpu_rw, gg_ce, gg_dout, gg_act, clk, gg_on);//!osm_map_on

endmodule

module gg_cnt
(dat, addr, m2, cpu_ce, cpu_rw, gg_ce, dout, act, clk, gg_on);

	input [7:0]dat;
	input [14:0]addr;
	input cpu_ce, m2, cpu_rw, gg_ce, clk, gg_on;
	output act;
	output [7:0]dout;
	
	reg [2:0]gg_idx;
	reg [1:0]idx_sub_ctr;
	reg ctr_latch;
	
	always @(negedge m2)
	begin
		if(gg_ce & cpu_rw)
		begin
			gg_idx <= 0;
			idx_sub_ctr <= 1;
		end
			else
		if(gg_ce & !cpu_rw)
		begin
			idx_sub_ctr <= idx_sub_ctr + 1;
			if(idx_sub_ctr == 0)gg_idx <= gg_idx + 1;
		end
		
	end
	
	/*
	reg [1:0]act_st;
	assign act_out = act_st[1:0] == 2'b11 & act;
	always @(negedge clk)
	begin
		act_st[1:0] <= {act_st[0], act};
	end*/
	
	assign act = !m2 ? 0 : gg_act0 | gg_act1 | gg_act2 | gg_act3 | gg_act4;
	wire gg_rst = gg_ce & cpu_rw;
	assign dout = gg_dout0 | gg_dout1 | gg_dout2 | gg_dout3 | gg_dout4;
	wire gg_we = gg_ce & !cpu_rw;
	wire gg_act0;
	wire gg_act1;
	wire gg_act2;
	wire gg_act3;
	wire gg_act4;
	wire [7:0]gg_dout0;
	wire [7:0]gg_dout1;
	wire [7:0]gg_dout2;
	wire [7:0]gg_dout3;
	wire [7:0]gg_dout4;
	gg_code gg0(0, gg_act0, gg_dout0, dat, addr, m2, cpu_ce, cpu_rw, gg_idx, gg_we, clk, gg_on, gg_rst);
	gg_code gg1(1, gg_act1, gg_dout1, dat, addr, m2, cpu_ce, cpu_rw, gg_idx, gg_we, clk, gg_on, gg_rst);
	gg_code gg2(2, gg_act2, gg_dout2, dat, addr, m2, cpu_ce, cpu_rw, gg_idx, gg_we, clk, gg_on, gg_rst);
	gg_code gg3(3, gg_act3, gg_dout3, dat, addr, m2, cpu_ce, cpu_rw, gg_idx, gg_we, clk, gg_on, gg_rst);
	gg_code gg4(4, gg_act4, gg_dout4, dat, addr, m2, cpu_ce, cpu_rw, gg_idx, gg_we, clk, gg_on, gg_rst);

	
endmodule

module gg_code
(gg_idx, act, dout, dat, addr, m2, cpu_ce, cpu_rw, gg_cur_idx, gg_we, clk, gg_on, gg_rst);


	input [7:0]dat;
	input [14:0]addr;
	input cpu_ce, m2, cpu_rw, gg_we, clk, gg_on, gg_rst;
	input [3:0]gg_idx;
	input [3:0]gg_cur_idx;
	output act;
	output [7:0]dout;
	//input gg_we;

	reg [31:0]code;
	wire [7:0]gg_cmp_dat = code[31:24];
	wire [7:0]gg_dat = code[23:16];
	wire [14:0]gg_addr = code[14:0];
	wire long_code = code[15];
	reg code_ready;
	
	assign dout = act ? gg_dat : 0;
	
	wire gg_addr_ok;
	reg gg_cmp_ok;
	reg gg_cmp_latch;
	reg [3:0]cmp_ctr;
	
	assign act =  code_ready & !cpu_ce & cpu_rw & gg_addr_ok & (!long_code | gg_cmp_ok) & gg_on;
	assign gg_addr_ok = addr[14:0] == gg_addr[14:0];
	
	always @(negedge clk)
	begin
		
		
		if(!m2 | cpu_ce | !cpu_rw)
		begin
			cmp_ctr <= 0;
			gg_cmp_latch <= 0;
			gg_cmp_ok <= 0;
		end
			else
		if(cmp_ctr < 8)
		begin
			cmp_ctr <= cmp_ctr + 1;
		end
			else
		if(!gg_cmp_latch)
		begin
			gg_cmp_latch <= 1;
			gg_cmp_ok <= gg_cmp_dat[7:0] == dat;
		end
		
	end
	
	always @(negedge m2)
	begin
	
		//gg_addr_ok <= addr[14:0] == gg_addr[14:0] & !cpu_ce;
		if(gg_rst)
		begin
			code_ready <= 0;
		end
			else
		if(gg_we & gg_idx == gg_cur_idx)
		begin
			code[31:0] <= {code[23:0], dat[7:0]};
			code_ready <= 1;
		end
	end

endmodule

