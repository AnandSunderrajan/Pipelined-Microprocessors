//module to calculate four input or

module fourinOR(
input wire a, b, c, d,
output wire o);
assign o = a | b | c | d;
endmodule


//module to calculate two input or

module twoinOR(
input wire a, b,
output wire o);
assign o = a | b;
endmodule


//module for L2 cache pseudoLRU

module pseudoLRU(
input logic clk, out0, out1, out2, out3, pseudoLRU_write,
output logic [1:0] LRU);
logic [2:0] state;
always @(posedge clk)
begin
if(pseudoLRU_write == 1)
	begin// Start references to lines based on statevals
	if((state[1] == 0) && (state[2] == 0))
	begin
		LRU = 2'b00;
		state = {2'b11, state[0]};
	end
	else if((state[1] == 1) &&(state[2] == 0))
	begin
		LRU = 2'b01;
		state = {2'b10, state[0]};
	end
	else if((state[0] == 0) && (state[2] == 1))
	begin
		LRU = 2'b10;
		state = {1'b0, state[1], 1'b1};
	end
	else if((state[0] == 1) && (state[2] == 1))
	begin
		LRU = 2'b11;
		state = {1'b0, state[1], 1'b0};
	end
	end
else
	begin
		LRU = 2'b00;
		state = 3'b000;
	end
end
endmodule


//Module to detect pipeline hazard and then stall if needed

import LC_3B_2::*;
module hazard_detection(
input LC_3B_reg IF_IDC_A, 
input LC_3B_reg IF_IDC_B, 
input LC_3B_reg SDR_IF_ID, 
input LC_3B_reg DRM_IDC_EX,
input logic IDC_EX_AREQ, IDC_EX_BREQ, DCacheR, write_DCache, OPUNCO,
output logic stallhazard, flushpipe);
always_comb
begin
stallhazard = 0;
if(DCacheR) //basically, if there is stuff in the pipeline that can cause a hazard, this will stall the pipeline so that it can be cleared
	if(((DRM_IDC_EX == IF_IDC_A) && (!IDC_EX_AREQ)) || ((DRM_IDC_EX == IF_IDC_B) && (IDC_EX_BREQ)) || ((write_DCache) && (DRM_IDC_EX == SDR_IF_ID)))
		stallhazard = 1;
else
	stallhazard = 0;
end
always_comb
begin
if(OPUNCO) //if signal is high it will flush/clear pipeline
	flushpipe = 1;
else
	flushpipe = 0;
end
endmodule


//This module will output selects for mux in ALU

import LC_3B_2::*;
module forwardselectunit(
input LC_3B_reg SR1_IDC_EX, 
input LC_3B_reg SR2_IDC_EX, 
input LC_3B_reg DRM_EX_MEM, 
input LC_3B_reg DRM_MEM_WB, 
input LC_3B_reg out_IDC_EX,
input logic clk, IDC_EX_AREQ, IDC_EX_BREQ, SR1_EX_MEM_REQ, SR2_EX_MEM_REQ, SR1_MEM_WB_REQ, SR2_MEM_WB_REQ, registerload,
input logic DRM_IDC_EX_REQ, DRM_EX_MEM_REQ, DRM_MEM_WB_REQ, VALIDC_EX_MEM, VALIDC_mem_WB,
output logic [1:0] selectFM1, selectFM2);
always_comb
begin
selectFM1 = 2'b00;
selectFM2 = 2'b00;
if(registerload == 1)
begin
	if((IDC_EX_AREQ)&&(VALIDC_EX_MEM)&&(DRM_EX_MEM == SR1_IDC_EX)&&(DRM_IDC_EX_REQ)&&(DRM_EX_MEM_REQ))
		selectFM1 = 2'b10;
	else if((IDC_EX_AREQ)&&(VALIDC_mem_WB)&&(DRM_MEM_WB == SR1_IDC_EX)&&(DRM_IDC_EX_REQ)&&(DRM_MEM_WB_REQ))
		selectFM1 = 2'b01;
	else 
		selectFM1 = 2'b00; //SR1 check
	if((IDC_EX_BREQ)&&(VALIDC_EX_MEM)&&(DRM_EX_MEM == SR2_IDC_EX)&&(DRM_IDC_EX_REQ)&&(DRM_EX_MEM_REQ))
		selectFM2 = 2'b10;
	else if((IDC_EX_BREQ)&&(VALIDC_mem_WB)&&(DRM_MEM_WB == SR2_IDC_EX)&&(DRM_IDC_EX_REQ)&&(DRM_MEM_WB_REQ))
		selectFM2 = 2'b01;	
	else 
		selectFM2 = 2'b00; //SR2 check
end
else
begin
	if((out_IDC_EX == DRM_MEM_WB)&&(DRM_IDC_EX_REQ))
		selectFM2 = 2'b01;
	else if((DRM_EX_MEM == SR1_IDC_EX)&&(IDC_EX_AREQ))
		selectFM1 = 2'b11;
	else
		selectFM2 = 2'b00;
end
end
endmodule


//This module has the 2bit counter which we need for ldisti instructions. lmao sti 

module bitcounter(
input logic clk, OP_LDI, OP_STI, DC_mem_response, STALL_DCache,
output logic [1:0] counter);
logic [1:0] data;

always @(posedge clk)
begin
if((DC_mem_response) &&(OP_LDI || OP_STI)&&(!STALL_DCache))
	data++;
else if((OP_LDI || OP_STI) &&(STALL_DCache))
	data = data;
else
	data = 2'b00;
end 
always_comb
begin
	counter = data;
end
endmodule


//This module basically builds the logic for the instruc decode n execute connection.

module ISVAL_IDC_EX(
input logic clk, TRUE_BR, internal_mem_response, STALL_LDI, STALL_DCache,
output logic out);
logic [1:0] data;
initial
begin
    data = 2'b11;
end
always @(posedge clk)
begin
if(internal_mem_response == 1)
	data = 2'b11;
else 
	data = data - 1'b1;
if(data == 2'b00)
	data = 2'b01;
end
always_comb
begin
if(STALL_DCache == 1)
	out = 0;
else if(STALL_LDI)
	out = 0;
else if((data == (2'b11)) || (data == (2'b10)))
	out = 1;
else if(TRUE_BR)
	out = 0;
else 
	out = 0;
end
endmodule


//two to one decoder

module one_two_decoder #(parameter width = 16)(
input logic select,
input logic [width-1:0] in,
output logic [width-1:0] Aout, Bout);
always_comb
begin
if(select == 0)
begin
	Aout = in;
	Bout = 1'b0;
end
else
begin
	Aout = 1'b0;
	Bout = in;
end
end
endmodule


//This module is an 8-1 mux

module
eight_one_mux #(parameter width = 16)(
input [2:0] select,
input [width-1:0] a, b, c, d, e, f, g, h,
output logic [width-1:0] out);
always_comb
begin
if (select == 3'b000)
	out = a;
else if (select == 3'b001)
	out = b;
else if (select == 3'b010)
	out = c;
else if (select == 3'b011)
	out = d;
else if (select == 3'b100)
	out = e;
else if (select == 3'b101)
	out = f;
else if (select == 3'b110)
	out = g;
else 
	out = h;
end
endmodule


//module to create arrays for the cache datapath. need arrays for dirty cache, LRU, .... basically the advanced cache design options that were in the mp3 doc from piazza. pain in the ass.

import LC_3B_2::*;
module arraycreaTOR #(parameter width = 128)(
input clk, write,
input LC_3B_c_index index,
input [width-1:0] datain,
output logic [width-1:0] dataout);
logic [width-1:0] data [7:0];
initial
begin
for (int i = 0; i < $size(data); i++)
	begin
		data[i] = 1'b0;
	end
end
always @(posedge clk)
begin
if (write == 1)
	begin
		data[index] = datain;
	end
end
assign dataout = data[index];
endmodule


//module for tag comparison b/w memadd n cacheindex

module COMPARATOR_TAGX(
input [8:0] inputCache, mem_addr_in,
output logic out);
always_comb
begin
if(inputCache == mem_addr_in)
	out = 1;
else
	out = 0;
end
endmodule


//combined concantenatormux block 16bit in 128 out

import LC_3B_2::*;
module CONCATENATOR_MUX(
input LC_3B_word data_wr,
input cache_flow data_r,
input [2:0] select,
output cache_flow out);
always_comb
begin
if (select == 3'b000)
	out = {data_r[127:16],data_wr};
else if (select == 3'b001)
	out = {data_r[127:32],data_wr,data_r[15:0]};
else if (select == 3'b010)
	out = {data_r[127:48],data_wr,data_r[31:0]};
else if (select == 3'b011)
	out = {data_r[127:64],data_wr,data_r[47:0]};
else if (select == 3'b100)
	out = {data_r[127:80],data_wr,data_r[63:0]};
else if (select == 3'b101)
	out = {data_r[127:96],data_wr,data_r[79:0]};
else if (select == 3'b110)
	out = {data_r[127:112],data_wr,data_r[95:0]};	
else
	out = {data_wr,data_r[111:0]};
end
endmodule


//combined concantenatormux block 8it in 8 out

import LC_3B_2::*;
module CONCATENATORDOSMX #(parameter width = 8)(
input select,
input [width-1:0] a,b,
output LC_3B_word out);

always_comb
begin
if (select == 0)
	out = {a,b};
else
	out = {b,a};
end
endmodule


//modules used for ZEXT[offset]

import LC_3B_2::*;
module ZEXTblock #(parameter width = 8)(
input ZEXTblockselect,
input [width-1:0] in,
output LC_3B_word out);
always_comb
begin
if (ZEXTblockselect == 0)
	out = {8'b00000000, in};
else
	out = {in, 8'b00000000};
end
endmodule

import LC_3B_2::*;
module zext #(parameter width = 4)(
input [width-1:0] in,
output LC_3B_word out);
assign out = in;
endmodule

import LC_3B_2::*;
module LeftShift_ZEXT #(parameter width = 8)(
input [width-1:0] in,
output LC_3B_word out);
assign out = { 8'b00000000,in} << 1;
endmodule


//module used for shifting operation ins the alu

import LC_3B_2::*;
module ShiftBlock #(parameter width = 16)(
input [5:0] shiftword,
input [width-1:0] in,
output LC_3B_word out);
always_comb
begin
if(shiftword[4] == 0)
	out = in << shiftword[3:0];
else
begin
	if(shiftword[5] == 0)
		out = in >> shiftword[3:0];
	else
		out = $signed(in) >>> shiftword[3:0];
end
end
endmodule


//4to1mux for the br operations rap/br/jsr/jmp

module BRMODS(
input logic a, b, c, d,
output logic [1:0] out);
always_comb
begin
if (a == 1)
	out = 2'b10;
else if(c || b)
	out = 2'b01;
else if(d)
	out = 2'b11;
else 
	out = 2'b00;
end
endmodule


//to check if the pipeline has no haz in execute mem pipe part so its on mem-wb

module VALCHECK_MEM_WB(
input logic LDI_cs, STALL_LDI, STALL_DCache,
output logic out);
always_comb
begin
if(STALL_DCache)
	out = 1'b0;
else if(STALL_LDI)
	out = 1'b0;
else
	out = 1'b1;
end
endmodule


//generate the condition code

import LC_3B_2::*;
module CondCode(
input LC_3B_word in,
output LC_3B_nzp out);
always_comb
begin
if (in[15] == 1'b1)
	out = 3'b100;
else if (|in)
	out = 3'b001;
else
	out = 3'b010;
end
endmodule


//two input and calculator
module twoinAND(
input logic a, b,
output logic o);
assign o = a & b;
endmodule


//threein and calcu
module threeinAND(
input logic a, b, c,
output logic o);
assign o = a & b & c;
endmodule


//thretoone mux

module
threetoonemux #(parameter width = 16)(
input [1:0] select,
input [width-1:0] a, b, c,
output logic [width-1:0] o);
always_comb
begin
if (select == 2'b00)
	o = a;
else if (select == 2'b01)
	o = b;
else 
	o = c;
end
endmodule


//fourtoonemux

module
fourtoonemux #(parameter width = 16)(
input [1:0] select,
input [width-1:0] a, b, c, d,
output logic [width-1:0] o);
always_comb
begin
if (select == 2'b00)
	o = a;
else if (select == 2'b01)
	o = b;
else if (select == 2'b10)
	o = c;
else 
	o = d;
end
endmodule


//comapres cc for the reg

import LC_3B_2::*;
module CondCodeComp(
input LC_3B_nzp a,
input LC_3B_reg b,
output logic out);
always_comb
begin
if ((a[0]& b[0])| (a[1] &b[1]) | (a[2] &b[2]))
	out <= 1'b1;
else 
	out <= 1'b0;
end
endmodule


//the alu model

import LC_3B_2::*;
module Alu(
input LC_3B_OPAlu OPAlu,
input LC_3B_word a, b,
output LC_3B_word f);
always_comb
begin
case (OPAlu)
	Alu_add: f = a + b;
   Alu_and: f = a & b;
	Alu_passa: f = a;
	Alu_passb: f = b;
   Alu_not: f = ~a;
   Alu_shiftleft: f = a << b;
   Alu_shiftrile: f = a >> b;
   Alu_signedshiftright: f = $signed(a) >>> b;
   default:;
   endcase
end
endmodule



//a 16itadder

import LC_3B_2::*;
module adderst (
input LC_3B_word a,b,
output LC_3B_word out);
always_comb
begin
	out = a + b;
end
endmodule


//used for SEXT

import LC_3B_2::*;
module SEXT #(parameter width = 5)(
input [width-1:0] in,
output LC_3B_word out);
assign out = $signed(in);
endmodule


//constrol singal register

import LC_3B_2::*;
module contrsigreg #(parameter width = 24)(
input clk, load,
input LC_3B_control_word in,
output LC_3B_control_word out);
LC_3B_control_word data;
always @(posedge clk)
begin
	if (load)
   begin
		data = in;
	end
	else
	begin
		data = 1'b0;
	end
end
always_comb
begin
    out = data;
end
endmodule


//REGFILEEEEEEE

import LC_3B_2::*;
module REGFILE(
input clk, load,
input LC_3B_word in,
input LC_3B_reg AREF, BREF, Destination,
output LC_3B_word reg_a, reg_b);
LC_3B_word data [7:0];
initial
begin
for (int i = 0; i < $size(data); i++)
	begin
   data[i] = 16'b0;
   end
end
always @(negedge clk)
begin
if (load == 1)
	begin
		data[Destination] = in;
	end
end
always_comb
begin
   reg_a = data[AREF];
   reg_b = data[BREF];
end
endmodule



//the conrol sig for the opcode n pipelin lc3b

import LC_3B_2::*;
module controlla(
input LC_3B_word PC,
input LC_3B_OPCode OPCode,
input logic IR5, IR11,
output LC_3B_control_word control);
always_comb
begin 
	control.OPCode = OPCode;
	control.OPAlu = Alu_passa;
	control.OPTRAP = 1'b0;
	control.OPBR = 1'b0;
	control.OPJSR = 1'b0;
	control.OPJMP = 1'b0;
	control.OPUNCO = 1'b0;
	control.OPLDB = 1'b0;
	control.OPSTB = 1'b0;
	control.OP_LDI = 1'b0;
	control.OP_STI = 1'b0;
	control.LeftSHIFT = 1'b0;
	control.DCacheR = 1'b0;
	control.addr1mux_select = 1'b0;
	control.addr2mux_select = 2'b00;
	control.addr3mux_select = 1'b0;
	control.Alu_result_mux_select = 1'b0;	
	control.Destination_mux_select = 1'b0;
	control.DC_mem_byte_select = 1'b0;
	control.wbmux_select = 2'b00;
	control.sr2mux_select = 1'b0;
	control.storemux_select = 1'b0;
   control.LDC_CC = 1'b0;
	control.PC = PC;
   control.registerload = 1'b0;
	control.write_DCache = 1'b0;
	control.DCache_enable = 1'b0;
	control.REQSR1 = 1'b0;
	control.REQSR2 = 1'b0;
	control.dr_REQ = 1'b0;
   case(OPCode)
		op_add: 
		begin 
			control.OPAlu = Alu_add;
			control.LDC_CC = 1;
			control.REQSR1 = 1;
			control.REQSR2 = 1;
			control.registerload = 1;
			control.dr_REQ = 1;
			control.wbmux_select = 2'b11;
			if(IR5)
			begin
				control.REQSR2 = 0;
				control.sr2mux_select = 1;
			end
		end
      op_and: 
      begin
			control.OPAlu = Alu_and;
			control.LDC_CC = 1;
			control.REQSR1 = 1;
			control.REQSR2 = 1;
			control.registerload = 1;
			control.dr_REQ = 1;
			control.wbmux_select = 2'b11;
			if(IR5)
			begin
				control.REQSR2 = 0;
				control.sr2mux_select = 1;
			end
		end	
		op_not:
		begin
			control.OPAlu = Alu_not;
			control.LDC_CC = 1;
			control.registerload = 1;
			control.REQSR1 = 1;
			control.dr_REQ = 1;
			control.wbmux_select = 2'b11;
		end  
		op_ldr:
		begin
			control.LDC_CC = 1;
			control.registerload = 1;
			control.LeftSHIFT = 1;
			control.REQSR1 = 1;
			control.DCacheR = 1;
			control.DCache_enable = 1'b1;
			control.dr_REQ = 1;
			control.addr1mux_select = 1;
			control.addr2mux_select = 2'b01;
			control.wbmux_select = 2'b01;
		end
		op_str:
		begin
			control.OPAlu = Alu_passb;
			control.LeftSHIFT = 1;
			control.REQSR1 = 1;
			control.write_DCache = 1;
			control.DCache_enable = 1'b1;
			control.dr_REQ = 1;
			control.addr1mux_select = 1;
			control.addr2mux_select = 2'b01;
			control.storemux_select = 1;
		end
		op_br:
		begin
			control.OPBR = 1;
			control.LeftSHIFT = 1;
			control.addr2mux_select = 2'b10;
		end
		op_lea:
		begin
			control.LDC_CC = 1;
			control.registerload = 1;
			control.LeftSHIFT = 1;
			control.dr_REQ = 1;
			control.addr2mux_select = 2'b10;
			control.storemux_select = 1;
		end
		op_jmp:
		begin
			control.OPJMP = 1;
			control.OPUNCO = 1;
			control.REQSR1 = 1;
		end
		op_jsr:
		begin
			control.OPJSR = 1;
			control.OPUNCO = 1;
			control.registerload = 1;
			control.Destination_mux_select = 1;
			if(IR11 == 0)
			begin
				control.OPJMP = 1;
				control.OPJSR = 0;
				control.REQSR1 = 1;
				control.wbmux_select = 2'b10;
			end
			if(IR11 == 1)
			begin
				control.OPBR = 1;
				control.LeftSHIFT = 1;
				control.addr2mux_select = 2'b11;
				control.wbmux_select = 2'b10;
			end
		end
		op_trap:
		begin
			control.OPTRAP = 1;
			control.OPUNCO = 1;
			control.registerload = 1;
			control.DCache_enable = 1;
			control.DCacheR = 1;
			control.addr3mux_select = 1;
			control.Destination_mux_select = 1;
			control.wbmux_select = 2'b10;
		end
		op_shf:
		begin
			control.LDC_CC = 1;
			control.registerload = 1;
			control.dr_REQ = 1;
			control.REQSR1 = 1;
			control.Alu_result_mux_select = 1;
			control.wbmux_select = 2'b11;
		end
		op_ldb:
		begin
			control.OPLDB = 1;
			control.LDC_CC = 1;
			control.registerload = 1;
			control.dr_REQ = 1;
			control.REQSR1 = 1;
			control.DCacheR = 1;
			control.DCache_enable = 1;
			control.DC_mem_byte_select = 1;
			control.addr1mux_select = 1;
			control.addr2mux_select = 2'b01;
			control.wbmux_select = 2'b01;
		end
		op_stb:
		begin
			control.OPAlu = Alu_passb;
			control.OPSTB = 1;
			control.DCache_enable = 1;
			control.write_DCache = 1;
			control.dr_REQ = 1;
			control.REQSR1 = 1;
			control.addr2mux_select = 2'b01;
			control.addr1mux_select = 1;
			control.storemux_select = 1;
		end
		op_ldi:
		begin
			control.OP_LDI = 1;
			control.LDC_CC = 1;
			control.registerload = 1;
			control.LeftSHIFT = 1;
			control.dr_REQ = 1;
			control.REQSR1 = 1;
			control.DCacheR = 1;
			control.DCache_enable = 1;
			control.addr1mux_select = 1;
			control.addr2mux_select = 2'b01;
			control.wbmux_select = 2'b01;
		end
		op_sti:
		begin
			control.OPAlu = Alu_passb;
			control.OP_STI = 1;
			control.LeftSHIFT = 1;
			control.dr_REQ = 1;
			control.REQSR1 = 1;
			control.write_DCache = 1;
			control.DCache_enable = 1;
			control.addr2mux_select = 2'b01;
			control.addr1mux_select = 1;
			control.storemux_select = 1;
		end
		default: 
      begin
			control = 0;
      end
endcase
end
endmodule


//adds two to input

module twoadd #(parameter width = 16)(
input [width-1:0] in,
output logic [width-1:0] out);
assign out = in + 4'h2;
endmodule


//regi module the regi pokemon trio lul

module register #(parameter width = 16)(
input clk, load,
input [width-1:0] in,
output logic [width-1:0] out);
logic [width-1:0] data;
always @(posedge clk)
begin
if (load)
begin
	data = in;
end
else
begin
	data = 1'b0;
end
end
always_comb
begin
   out = data;
end
endmodule



//twotonemux

module
twotoonemux #(parameter width = 16)(
input select,
input [width-1:0] a, b,
output logic [width-1:0] o);
always_comb
begin
if (select == 0)
	o = a;
else
	o = b;
end
endmodule


//1bitleftshit

import LC_3B_2::*;
module LeftSHIFTone #(parameter width = 16)(
input logic select,
input [width-1:0] in,
output LC_3B_word out);
always_comb
begin
if(select == 1)
	out = in << 1;
else
	out = in;
end
endmodule
