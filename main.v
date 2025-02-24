module main
(cpu_dat, cpu_addr, cpu_ce, cpu_rw, cpu_m2, cpu_irq, cpu_exp, mclk,
ppu_dat, ppu_addr, ppu_rd, ppu_wr, ppu_ciram_ce, ppu_ciram_a10,
ppu_dir, ppu_ex,
prg_map, prg_ce, prg_oe, prg_we,
chr_map, chr_ce, chr_oe, chr_we,
ram_ce, ram_oe, ram_we,
clk, max0, max1, max2 
);
	
	//cpu bus
	inout [7:0]cpu_dat;
	input [14:0]cpu_addr;
	input cpu_ce, cpu_rw, cpu_m2, mclk;//mclk = m2 line, but connected to clock input
	output cpu_irq, cpu_exp;
	
	//ppu bus
	inout [7:0]ppu_dat;
	input [13:0]ppu_addr;
	input ppu_rd, ppu_wr;
	output ppu_ciram_ce, ppu_ciram_a10;
	
	//ppu bus control
	output ppu_dir, ppu_ex;
	
	//prg ram
	output [8:0]prg_map;
	output prg_ce, prg_oe, prg_we;
	
	//chr ram
	output [8:0]chr_map;
	output chr_ce, chr_oe, chr_we;
	
	//ram 
	output ram_ce, ram_oe, ram_we;
	
	//misc
	input clk, max2;
	output max0, max1;
	

	//*******************************************************Some glue
	parameter BMOD_REGS = 2'b11;//cpld regs unlocked
	parameter BMOD_GAME = 2'b01;//cpld regs locked
	parameter BMOD_LED = 2'b00;//led or bus spy.
	parameter BMOD_OE = 2'b10;//force bus outputs.
	
	assign ppu_ex = !ppu_ciram_ce;
	assign ppu_dir = !ppu_rd ? 0 : 1;
	assign ppu_dat = !os_map_act & mmc_ppu_out ? mmc_ppu_do[7:0] : 8'hzz;
	

	assign max0 = bus_mode[0];
	assign max1 = bus_mode[1];
	wire [1:0]bus_mode;
	
	
	assign prg_map[8:0] = {prg_m[8:4] & prg_mask[4:0], prg_m[3:0]};
	assign chr_map[8:0] = {chr_m[8], chr_m[7:3] & chr_mask[4:0], chr_m[2:0]};
	
	wire [8:0]prg_m = map_out[18:10];
	wire [8:0]chr_m = map_out[38:30];
	
	wire rom_we;
	assign ppu_ciram_a10 = map_out[40];
	assign ppu_ciram_ce = map_out[41];
	assign rom_we = rst ? 1 : !map_out[42];
	assign chr_we = rst ? 1 : !map_out[43];
	assign cpu_irq = map_out[44] ? 0 : 1'bz;
	assign prg_ce = !m2_ok ? 1 : !map_out[45];
	assign ram_ce = !m2_ok ? 0 : map_out[46] & !map_out[45];
	assign chr_ce = !map_out[47];
	assign cpu_exp = 1'bz;
	
	assign prg_we = !os_map_act ? 1 : rom_we | !mclk;
	assign prg_oe = !cpu_rw | bus_oe;
	assign ram_we = rom_we | !mclk;
	assign ram_oe = !cpu_rw | bus_oe;
	assign chr_oe = ppu_rd;
	
	reg bus_oe_st;
	always @(negedge clk)
	begin
		bus_oe_st <= bus_oe;
	end

	//*******************************************************
	assign cpu_dat = !mclk | !bus_oe_st | !bus_oe ? 8'hzz :
	os_map_oe ? os_map_dout : 
	!os_map_act & mmc_cpu_out ? mmc_cpu_do[7:0] : 8'hzz;
	
	wire bus_oe = os_map_oe | (!os_map_act & mmc_cpu_out);
	assign bus_mode[1:0] = rst ? 2'b00 : os_map_act ? BMOD_REGS : BMOD_GAME;
	
	//*******************************************************OS map
	wire os_map_act = map_rst;
	wire map_rst = map_idx[7:0] == 255;
	wire [48:0]map_out255;
	wire [7:0]map_cfg;
	wire [7:0]map_idx;
	wire [4:0]prg_mask;
	wire [5:0]chr_mask;
	wire rst;
	wire [7:0]os_map_dout;
	wire os_map_oe;
	wire m2_ok;
	
	map_255 m255_inst(map_out255, cpu_dat, cpu_addr, cpu_ce, cpu_rw, mclk,
	ppu_dat, ppu_addr, ppu_rd, ppu_wr, 
	clk, map_cfg, map_idx, rst, prg_mask, chr_mask, os_map_dout, os_map_oe, m2_ok, max2);
	
	//*******************************************************Game map
	
	wire [7:0]mmc_cpu_do;
	wire mmc_cpu_out;
	wire [7:0]mmc_ppu_do;
	wire mmc_ppu_out;
	
	wire [48:0]map_out = 
	map_idx == 5 ? map_out5 : 
	map_out255;
	
	wire [48:0]map_out5;
	map_5 m5_inst(map_out5, cpu_dat, cpu_addr, cpu_ce, cpu_rw, mclk,
	ppu_dat, ppu_addr, ppu_rd, ppu_wr, 
	clk, map_cfg, map_rst | rst, mmc_cpu_do, mmc_cpu_out, mmc_ppu_do, mmc_ppu_out);
	
	
	mmc5_snd snd_inst(cpu_dat, cpu_addr, cpu_rw, cpu_ce, mclk, rst, cpu_exp, clk);


	


	
endmodule
