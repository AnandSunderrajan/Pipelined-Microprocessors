//////////////////////////////////////////////////////////
////////////////L2CACHE MODULES NOW////////////////////////
//////////////////////////////////////////////////////////


//Toplevel module for the L2 cache structure - includes the controller and datapth

import LC_3B_2::*;
module L2Cache(
input clk, l2_mem_read,l2_mem_write,
input cache_flow physicalmem_rdata, l2_mem_wdata,
input LC_3B_word l2_mem_addr,
input LC_3B_mem_wmask l2_mem_byte_enable,
input logic physicalmem_response,
output l2_mem_response,
output cache_flow l2_mem_rdata, physicalmem_wdata,
output LC_3B_word physicalmem_addr,
output logic readphysicalmem, writephysicalmem, l2hit);

logic [1:0] pseudooutputLRU;
logic hit, writedata0, writedata1, writevalid0, writevalid1, writetag0, writetag1, writedata2, writedata3, writevalid2;
logic writevalid3, writetag2, writetag3, LRU_in, p0ANDo, pseudoLRU_write, dirtymux_out, stbwritemux_select;
logic MUXRW_select, writedirty0, writedirty1, inputdirty0, inputdirty1, writedirty2, writedirty3, inputdirty2, inputdirty3, physicalmemMXselect;

assign l2hit = hit;

L2Cache_control L2Cache_control(
.clk, .mem_addr(l2_mem_addr), .mem_byte_enable(l2_mem_byte_enable), .mem_write(l2_mem_write), .mem_read(l2_mem_read), .mem_wdata(l2_mem_wdata),
.pseudooutputLRU, .p0ANDo, .pseudoLRU_write, .physicalmem_rdata, .physicalmem_response, .dirtymux_out,
.hit, .physicalmemMXselect, .MUXRW_select, .stbwritemux_select, .mem_response(l2_mem_response), .readphysicalmem,  .writephysicalmem,
.writedata0, .writedata1, .writetag0, .writetag1, .writevalid0, .writevalid1, .writedirty0, .writedirty1, .inputdirty0, .inputdirty1,
.writedata2, .writedata3, .writetag2, .writetag3, .writevalid2, .writevalid3, .writedirty2, .writedirty3, .inputdirty2, .inputdirty3);

l2DatapathCache  l2DatapathCache(
.clk, .hit, .mem_addr(l2_mem_addr), .writedata0, .writedata1, .writetag0, .writetag1, .writevalid0, .writevalid1, .writedirty0, .writedirty1,
.inputdirty0, .inputdirty1, .writedata2, .writedata3, .writetag2, .writetag3, .writevalid2, .writevalid3, .writedirty2, .writedirty3, .inputdirty2, .inputdirty3,
.physicalmem_rdata, .l2_mem_wdata, .l2_mem_rdata, .mem_byte_enable(l2_mem_byte_enable), .pseudoLRU_write, .p0ANDo(p0ANDo), .MUXRW_select,
.stbwritemux_select, .physicalmemMXselect, .dirtymux_out, .pseudooutputLRU, .physicalmem_addr, .physicalmem_wdata);
endmodule


//This module hold the cache controller whihc basically contains the state machine that is there for the cache

import LC_3B_2::*;
module L2Cache_control(
input clk, mem_write, mem_read, physicalmem_response,
input LC_3B_word mem_addr,
input cache_flow mem_wdata, physicalmem_rdata,
input [1:0] mem_byte_enable,
input logic [1:0] pseudooutputLRU,
input logic hit, p0ANDo, dirtymux_out,
output logic writedata0, writedata1, writetag0, writetag1, writevalid0, writevalid1, writedirty0, writedirty1, inputdirty0, inputdirty1,
output logic writedata2, writedata3, writetag2, writetag3, writevalid2, writevalid3, writedirty2, writedirty3, inputdirty2, inputdirty3,
output logic mem_response, MUXRW_select, stbwritemux_select, readphysicalmem, writephysicalmem, physicalmemMXselect, pseudoLRU_write);

enum int unsigned {idless, cacheupdatess, physicalmemss, writebackss, aloss} state, next_state;

always_comb
begin : state_actions
	writephysicalmem = 1'b0;
	pseudoLRU_write = 1'b0;
	writedata0 = 1'b0;
	writedata1 = 1'b0;
	writedata2 = 1'b0;
	writedata3 = 1'b0;
	writedirty0 = 1'b0;
	writedirty1 = 1'b0;
	writedirty2 = 1'b0;
	writedirty3 = 1'b0;
	writetag0 = 1'b0;
	writetag1 = 1'b0;
	writetag2 = 1'b0;
	writetag3 = 1'b0;
	writevalid0 = 1'b0;
	writevalid1 = 1'b0;
	writevalid2 = 1'b0;
	writevalid3 = 1'b0;
	inputdirty0 = 1'b0;
	inputdirty1 = 1'b0;
	inputdirty2 = 1'b0;
	inputdirty3 = 1'b0;
	mem_response = 1'b0;
	readphysicalmem = 1'b0;
	stbwritemux_select = 1'b0;
	MUXRW_select = 1'b0;
	physicalmemMXselect = 1'b0;
     
	case(state)
	idless: begin
	pseudoLRU_write = 0;
	if(mem_read && hit)
	begin
		mem_response = 1;
		pseudoLRU_write = 1;
	end
	else if((mem_write && hit) && (mem_byte_enable != 2'b00))
	begin
		pseudoLRU_write = 1;
		if(p0ANDo)
		begin
			inputdirty0 = 1;
			writedata0 = 1;
			writedirty0 = 1;
			writetag0 = 1;
			writevalid0 = 1;
		end
	else 
	begin
		inputdirty1 = 1;
		writedata1 = 1;
		writevalid1 = 1;
		writetag1 = 1;
		writedirty1 = 1;
	end
	mem_response = 1;
	MUXRW_select = 1;
	end
	end
	cacheupdatess: begin
	if(mem_read)
		pseudoLRU_write = 1;
	end
	physicalmemss: begin
	if(mem_write)
	begin
		if((mem_byte_enable == 2'b10) | (mem_byte_enable == 2'b01))
			stbwritemux_select = 1;
			MUXRW_select = 1;
			if(pseudooutputLRU == 2'b00)
			begin
				inputdirty0 = 1;
				writedata0 = 1;
				writevalid0 = 1;
				writetag0 = 1;
				writedirty0 = 1;
			end
			else if(pseudooutputLRU == 2'b01)
			begin
				inputdirty1 = 1;
				writedata1 = 1;
				writevalid1 = 1;
				writetag1 = 1;
				writedirty1 = 1;
			end
			else if(pseudooutputLRU == 2'b10)
			begin
				inputdirty2 = 1;
				writedata2 = 1;
				writevalid2 = 1;
				writetag2 = 1;
				writedirty2 = 1;
			end
			else if(pseudooutputLRU == 2'b11)
			begin
				inputdirty3 = 1;
				writedata3 = 1;
				writevalid3 = 1;
				writetag3 = 1;
				writedirty3 = 1;
			end
		end
		mem_response = 1;
	end
	writebackss: begin
	writephysicalmem = 1;
	physicalmemMXselect = 1;
	end
	aloss: begin
	readphysicalmem = 1;
	if(pseudooutputLRU == 2'b00)
	begin
		inputdirty0 = 1;
		writedata0 = 1;
		writevalid0 = 1;
		writetag0 = 1;
		writedirty0 = 1;
	end
	else if(pseudooutputLRU == 2'b01)
	begin
		inputdirty1 = 1;
		writedata1 = 1;
		writevalid1 = 1;
		writetag1 = 1;
		writedirty1 = 1;
	end
	else if(pseudooutputLRU == 2'b10)
	begin
		inputdirty2 = 1;
		writedata2 = 1;
		writevalid2 = 1;
		writetag2 = 1;
		writedirty2 = 1;
	end
	else if(pseudooutputLRU == 2'b11)
	begin
		inputdirty3 = 1;
		writedata3 = 1;
		writevalid3 = 1;
		writetag3 = 1;
		writedirty3 = 1;
	end
	end
   default:;
	endcase 
end

always_comb
begin : next_state_logic
next_state = state;
case(state)
	idless:
	if(mem_read && hit)
		next_state = idless;
	else if(!hit && !dirtymux_out && (mem_read || mem_write))
		next_state = aloss;
	else if(!hit && dirtymux_out)
		next_state = writebackss;
	cacheupdatess:
		next_state = physicalmemss;
	physicalmemss:
		next_state = idless;
	writebackss:
		if(physicalmem_response == 0)
			next_state = writebackss;
		else
			next_state = aloss;
	aloss:
		if(physicalmem_response == 0)
			next_state = aloss;
		else
			next_state = cacheupdatess;
	default: 
		next_state = idless;
endcase
end

always_ff @(posedge clk)
begin: next_state_assignment
begin : next_state_assignment
	state <= next_state;
end
end
endmodule



//////////////////////////////////////////////////////////
////////////////DCACHE MODULES NOW////////////////////////
//////////////////////////////////////////////////////////


//This module hold the cache controller whihc basically contains the state machine that is there for the cache

import LC_3B_2::*;
module CONTROLLER_DCache(
input clk,physicalmem_response, mem_write, mem_read, dirtymux_out, hit,
input LC_3B_word mem_addr, mem_wdata,
input cache_flow physicalmem_rdata,
input [1:0] mem_byte_enable,
input logic outputLRU, p0ANDo, DCache_enable,
output logic LRU_in, LRU_write, readphysicalmem, MUXRW_select,stbwritemux_select, physicalmemMXselect, writephysicalmem, mem_response,
output logic writedata0, writedata1, writetag0, writetag1, writevalid0, writevalid1, writedirty0, writedirty1, inputdirty0, inputdirty1);

enum int unsigned {idless, cacheupdatess, physicalmemss, writebackss, aloss} state, next_state;

always_comb
begin : state_actions
	LRU_in = 1'b0;
	inputdirty0 = 1'b0;
	writetag0 = 1'b0;
	writedata0 = 1'b0;
	writedirty0 = 1'b0;
	writevalid0 = 1'b0;
	inputdirty1 = 1'b0;
	writetag1 = 1'b0;
	writedata1 = 1'b0;
	writedirty1 = 1'b0;
	writevalid1 = 1'b0;
	writephysicalmem = 1'b0;
	mem_response = 1'b0;
	readphysicalmem = 1'b0;
	LRU_write = 1'b0;
	stbwritemux_select = 1'b0;
	physicalmemMXselect = 1'b0;
	MUXRW_select = 1'b0;

case(state)
idless: begin
LRU_write = 0;
if(mem_read && hit)
begin
	mem_response = 1;
	LRU_write = 1;
	if(hit&&p0ANDo)
		LRU_in = 1;
	else if(hit && !p0ANDo)
		LRU_in = 0;
end
else if((mem_write && hit && DCache_enable) && (mem_byte_enable != 2'b00))
begin
	LRU_write = 1;
	if((mem_byte_enable == 2'b10) || (mem_byte_enable == 2'b01))
		stbwritemux_select = 1;
	if(p0ANDo)
	begin
		inputdirty0 = 1;
		writedata0 = 1;
		writedirty0 = 1;
		writetag0 = 1;
		writevalid0 = 1;
		LRU_in = 1;
	end
	else 
	begin
		inputdirty1 = 1;
		writedata1 = 1;
		writedirty1 = 1;
		writetag1 = 1;
		writevalid1 = 1;
		LRU_in = 0;
	end
	mem_response = 1;
	MUXRW_select = 1;
end
end
cacheupdatess: begin
if(mem_read)
	LRU_write = 1;
	if(p0ANDo && hit)
		LRU_in = 1;
	else if(!p0ANDo && hit)
		LRU_in = 0;
end
physicalmemss: begin
if(mem_write)
begin
	if((mem_byte_enable == 2'b10) || (mem_byte_enable == 2'b01))
		stbwritemux_select = 1;
		MUXRW_select = 1;
	if(outputLRU == 0)
	begin
		inputdirty0 = 1;
		writedata0 = 1;
		writedirty0 = 1;
		writetag0 = 1;
		writevalid0 = 1;
		LRU_in = 1;
	end
	else 
	begin
		inputdirty1 = 1;
		writedata1 = 1;
		writedirty1 = 1;
		writetag1 = 1;
		writevalid1 = 1;
		LRU_in = 0;
	end
end
mem_response = 1;
end
writebackss: begin
writephysicalmem = 1;
physicalmemMXselect = 1;
end
aloss: begin
readphysicalmem = 1;
if(outputLRU == 0)
begin
	inputdirty0 = 0;
	writedata0 = 1;
	writedirty0 = 1;
	writetag0 = 1;
	writevalid0 = 1;
end
else 
begin
	inputdirty1 = 0;
	writedata1 = 1;
	writedirty1 = 1;
	writetag1 = 1;
	writevalid1 = 1;
end
end
default:;
endcase 
end
always_comb
begin : next_state_logic
next_state = state;
case(state)
	idless:
	if(hit && mem_read && DCache_enable)
		next_state = idless;
	else if(!hit && dirtymux_out && DCache_enable)
		next_state = writebackss;
	else if(!hit && !dirtymux_out && DCache_enable)
		next_state = aloss;
	cacheupdatess:
	next_state = physicalmemss;
	physicalmemss:
	next_state = idless;
	writebackss:
	if(physicalmem_response == 0)
		next_state = writebackss;
	else
		next_state = aloss;
	aloss:
	if(physicalmem_response == 0)
		next_state = aloss;
	else
		next_state = cacheupdatess;
	default: 
		next_state = idless;
   endcase
end

always_ff @(posedge clk)
begin: next_state_assignment
begin: next_state_assignment
	state <= next_state;
end
end
endmodule



//This is the top evel modle for the DCache

import LC_3B_2::*;
module L1_DCache(
input clk, DCache_enable, mem_read, mem_write,
input cache_flow physicalmem_rdata,
input LC_3B_word mem_addr, mem_wdata,
input LC_3B_mem_wmask mem_byte_enable,
input logic physicalmem_response,
output mem_response,
output cache_flow physicalmem_wdata,
output LC_3B_word mem_rdata, physicalmem_addr,
output logic hit, readphysicalmem, writephysicalmem);

logic LRU_in, LRU_write, outputLRU;
logic inputdirty0, writedata0, writedirty0, writevalid0, writetag0, inputdirty1, writedata1,writedirty1, writevalid1, writetag1;
logic p0ANDo, dirtymux_out, physicalmemMXselect, stbwritemux_select, MUXRW_select;

CONTROLLER_DCache CONTROLLER_DCache(
.clk, .hit, .DCache_enable, .p0ANDo, .mem_addr, .mem_byte_enable, .mem_write, .mem_read, .mem_wdata, .mem_response,
.LRU_in, .outputLRU, .LRU_write, .readphysicalmem, .writephysicalmem, .physicalmem_rdata, .physicalmem_response,
.physicalmemMXselect, .dirtymux_out, .MUXRW_select, .stbwritemux_select, .inputdirty0, .writedata0, .writetag0, .writedirty0, .writevalid0,
.inputdirty1, .writedata1, .writetag1, .writedirty1, .writevalid1);

DatapathCache  DatapathCache(
.clk, .hit, .p0ANDo, .physicalmem_rdata, .MUXRW_select, .dirtymux_out, .physicalmem_addr, .physicalmem_wdata, .stbwritemux_select,
.physicalmemMXselect, .LRU_in, .LRU_write, .outputLRU, .mem_addr, .mem_byte_enable, .mem_wdata, .mem_rdata, .inputdirty0, .writedata0,
.writetag0, .writedirty0, .writevalid0, .inputdirty1, .writedata1, .writetag1, .writedirty1, .writevalid1);
endmodule



//////////////////////////////////////////////////////////
////////////////ICACHE MODULES NOW////////////////////////
//////////////////////////////////////////////////////////


//This module hold the cache controller whihc basically contains the state machine that is there for the cache

import LC_3B_2::*;

module CONTROLLER_ICache(
input clk, mem_read, mem_write, physicalmem_response,
input cache_flow physicalmem_rdata,
input LC_3B_word mem_addr, mem_wdata,
input [1:0] mem_byte_enable,
input logic hit, outputLRU, p0ANDo, TRUE_BR, dirtymux_out,
output logic LRU_in, LRU_write, mem_response, MUXRW_select, idle_state,
output logic stbwritemux_select, readphysicalmem, writephysicalmem, physicalmemMXselect,
output logic inputdirty0, writedata0, writetag0, writedirty0, writevalid0,
output logic inputdirty1, writedata1, writetag1, writedirty1, writevalid1);

enum int unsigned {idless, cacheupdatess, physicalmemss, writebackss, aloss} state, next_state;

always_comb
begin : state_actions
	idle_state = 1'b0;
	LRU_in = 1'b0;
	LRU_write = 1'b0;
	inputdirty0 = 1'b0;
	writetag0 = 1'b0;
	writedata0 = 1'b0;
	writedirty0 = 1'b0;
	writevalid0 = 1'b0;
	inputdirty1 = 1'b0;
	writetag1 = 1'b0;
	writedata1 = 1'b0;
	writedirty1 = 1'b0;
	writevalid1 = 1'b0;
	mem_response = 1'b0;
	readphysicalmem = 1'b0;
	writephysicalmem = 1'b0;
	MUXRW_select = 1'b0;
	physicalmemMXselect = 1'b0;
	stbwritemux_select = 1'b0;
   case(state)
   idless: begin
		LRU_write = 0;
		idle_state = 1;
		if(mem_read && hit)
		begin
			mem_response = 1;
			LRU_write = 1;
			if(p0ANDo && hit)
				LRU_in = 1;
			else if(!p0ANDo && hit)
				LRU_in = 0;
		end
		else if((mem_write && hit) && (mem_byte_enable != 2'b00))
		begin
			LRU_write = 1;
			if(p0ANDo)
			begin
				inputdirty0 = 1;
				writedata0 = 1;
				writetag0 = 1;
				writedirty0 = 1;
				writevalid0 = 1;
				LRU_in = 1;
			end
			else 
			begin
				inputdirty1 = 1;
				writedata1 = 1;
				writetag1 = 1;
				writedirty1 = 1;
				writevalid1 = 1;
				LRU_in = 0;
			end
			mem_response = 1;
			MUXRW_select = 1;
		end
		end
		cacheupdatess: begin
		if(mem_read)
			LRU_write = 1;
			if(p0ANDo && hit)
				LRU_in = 1;
			else if(!p0ANDo && hit)
				LRU_in = 0;
		end
		physicalmemss: begin
		if(mem_write)
		begin
			if((mem_byte_enable == 2'b10) | (mem_byte_enable == 2'b01))
				stbwritemux_select = 1;
				MUXRW_select = 1;
				if(outputLRU == 0)
				begin
					inputdirty0 = 1;
					writedata0 = 1;
					writetag0 = 1;
					writedirty0 = 1;
					writevalid0 = 1;
					LRU_in = 1;
				end
				else 
				begin
					inputdirty1 = 1;
					writedata1 = 1;
					writetag1 = 1;
					writedirty1 = 1;
					writevalid1 = 1;
					LRU_in = 0;
				end
		end
		mem_response = 1;
		end
		writebackss: begin
			writephysicalmem = 1;
			physicalmemMXselect = 1;
		end
		aloss: begin
			readphysicalmem = 1;
			if(outputLRU == 0)
			begin
				inputdirty0 = 0;
				writedata0 = 1;
				writetag0 = 1;
				writedirty0 = 1;
				writevalid0 = 1;
			end
			else 
			begin
				inputdirty1 = 0;
				writedata1 = 1;
				writetag1 = 1;
				writedirty1 = 1;
				writevalid1 = 1;
			end
		end
		default:;
endcase 
end
always_comb
begin : next_state_logic
next_state = state;
case(state)
	idless:
	if(hit && mem_read)
		next_state = idless;
	else if(!hit && !dirtymux_out && !TRUE_BR)
		next_state = aloss;
	else if(!hit && dirtymux_out)
		next_state = writebackss;
	cacheupdatess:
		next_state = physicalmemss;
		physicalmemss:
		next_state = idless;
	writebackss:
		if(physicalmem_response == 1)
			next_state = aloss;
		else
			next_state = writebackss;
	aloss:
		if(physicalmem_response == 1)
			next_state = cacheupdatess;
		else
			next_state = aloss;
	default: 
			next_state = idless;
endcase
end

always_ff @(posedge clk)
begin: next_state_assignment
begin: next_state_assignment
	state <= next_state;
end
end
endmodule



//This is the top evel modle for L1ICache
import LC_3B_2::*;
module L1_ICache(
input clk, TRUE_BR, mem_read, mem_write,
input cache_flow physicalmem_rdata,
input LC_3B_mem_wmask mem_byte_enable,
input LC_3B_word mem_addr, mem_wdata, 
input logic physicalmem_response,
output mem_response,
output cache_flow physicalmem_wdata,
output LC_3B_word mem_rdata, physicalmem_addr,
output logic idle_state, readphysicalmem, writephysicalmem);

logic hit, p0ANDo, dirtymux_out, stbwritemux_select, MUXRW_select, LRU_in, LRU_write, outputLRU, physicalmemMXselect;
logic inputdirty0, writedata0, writetag0, writedirty0, writevalid0;
logic inputdirty1, writedata1, writetag1, writedirty1, writevalid1;

CONTROLLER_ICache CONTROLLER_ICache(
.clk, .hit, .p0ANDo, .TRUE_BR, .mem_addr, .mem_read, .mem_write, .mem_wdata, .mem_byte_enable, .LRU_in, .outputLRU, .LRU_write,
.mem_response, .MUXRW_select, .stbwritemux_select, .physicalmemMXselect, .physicalmem_rdata, .dirtymux_out, .physicalmem_response,
.idle_state,.readphysicalmem, .writephysicalmem, .inputdirty0, .writedata0, .writetag0, .writedirty0, .writevalid0, .inputdirty1, .writedata1,
.writetag1, .writedirty1, .writevalid1);

DatapathCache  DatapathCache(
.clk, .hit, .p0ANDo, .dirtymux_out, .mem_addr, .mem_rdata, .mem_wdata, .mem_byte_enable, .LRU_in, .outputLRU, .LRU_write,
.physicalmem_addr, .physicalmem_rdata, .physicalmem_wdata, .physicalmemMXselect, .MUXRW_select, .stbwritemux_select,
.inputdirty0, .writedata0, .writetag0, .writedirty0, .writevalid0, .inputdirty1, .writedata1, .writetag1, .writedirty1, .writevalid1);
endmodule
