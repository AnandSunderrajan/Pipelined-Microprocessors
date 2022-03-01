//his module is the controller for the arbiter it hold the control logic for it
module arbiter_control(
input clk,
input logic l2hit, l2_mem_response, internal_mem_read, internal_mem_write, DC_mem_read, DC_mem_write,
output logic memaddrmux_select, readsignalmux_select, writesignalmux_select, memrdatademux_select, memwdatamux_select, byteenablemux_select, memresponsemux_select);

enum int unsigned {idless, s_imiss, s_dmiss, s_bothmiss} state, next_state;
always_comb
begin : state_actions
memaddrmux_select = 1'b0;
memrdatademux_select = 1'b0;
memwdatamux_select = 1'b0;
memresponsemux_select = 1'b0;
readsignalmux_select = 1'b0;
writesignalmux_select = 1'b0;
byteenablemux_select = 1'b0;
case(state)
	idless:begin
	end
	s_imiss:begin
		memaddrmux_select = 1'b1;
		readsignalmux_select = 1'b1;
		writesignalmux_select = 1'b1;
		memrdatademux_select = 1'b1;
		memwdatamux_select = 1'b1;
		memresponsemux_select = 1'b1;
		byteenablemux_select = 1'b1;
	end
	s_dmiss:begin
	end
	s_bothmiss:begin
	end
   default:;
endcase 
end

always_comb
begin : next_state_logic
next_state = state;
case(state)
idless:begin
	if((DC_mem_read || DC_mem_write)&&(internal_mem_read || internal_mem_write))
		next_state = s_bothmiss;
	else if(internal_mem_read || internal_mem_write)
		next_state = s_imiss;
	else if((DC_mem_read || DC_mem_write)&&(!l2hit))
		next_state = s_dmiss;
	else 
		next_state = idless;
end
s_imiss:begin
	if((l2_mem_response == 1)&&(!(DC_mem_read || DC_mem_write)))
		next_state = idless;
	else if((l2_mem_response == 1)&&(DC_mem_read || DC_mem_write))
		next_state = s_dmiss;
	else 
		next_state = s_imiss;
end
s_dmiss:begin
	if(l2_mem_response == 1)
		next_state = idless;
	else
		next_state = s_dmiss;
end
s_bothmiss:begin
	if(l2_mem_response == 1)
		next_state = s_imiss;
	else
		next_state = s_bothmiss;
end
default: next_state = idless;
endcase
end

always_ff @(posedge clk)
begin: next_state_assignment
    /* Assignment of next state on clock edge */
     begin : next_state_assignment
         state <= next_state;
     end
end

endmodule


//This module is the top level module for the arbitr

import LC_3B_2::*;
module arbiter(
input clk,l2_mem_response,
input LC_3B_word DC_mem_addr,
input LC_3B_word internal_mem_addr,
input cache_flow internal_mem_wdata,
input cache_flow DC_mem_wdata, 
input cache_flow l2_mem_rdata,
input logic l2hit, internal_mem_read, internal_mem_write, DC_mem_read, DC_mem_write,
input logic [1:0] internal_mem_byte_enable, DC_mem_byte_enable,
output LC_3B_word arbiter_mem_addr,
output cache_flow arbiter_internal_mem_rdata,
output cache_flow arbiter_DC_mem_rdata,
output cache_flow arbiter_mem_wdata,
output logic arbiter_mem_read, arbiter_mem_write, arbiter_DC_mem_response, arbiter_internal_mem_response,
output logic[1:0] arbiter_physicalmem_byte_enable);

logic memaddrmux_select;
logic memresponsemux_select;
logic readsignalmux_select;
logic writesignalmux_select;
logic memrdatademux_select;
logic memwdatamux_select;
logic byteenablemux_select;

arbiter_control arbiter_control(
.clk, .l2hit(l2hit), .l2_mem_response(l2_mem_response),.readsignalmux_select(readsignalmux_select),.writesignalmux_select(writesignalmux_select),
.internal_mem_read(internal_mem_read),.internal_mem_write(internal_mem_write), .DC_mem_read(DC_mem_read), .DC_mem_write(DC_mem_write),
.memaddrmux_select(memaddrmux_select), .memwdatamux_select(memwdatamux_select), .memrdatademux_select(memrdatademux_select),
.memresponsemux_select(memresponsemux_select), .byteenablemux_select(byteenablemux_select));

datapath_arbiter datapath_arbiter(
.clk, .internal_mem_addr(internal_mem_addr), .internal_mem_read(internal_mem_read), .internal_mem_write(internal_mem_write),
.internal_mem_wdata(internal_mem_wdata), .DC_mem_addr(DC_mem_addr), .DC_mem_read(DC_mem_read), .DC_mem_write(DC_mem_write), .DC_mem_wdata(DC_mem_wdata), 
.memaddrmux_select(memaddrmux_select), .memwdatamux_select(memwdatamux_select), .memresponsemux_select(memresponsemux_select), .memrdatademux_select(memrdatademux_select),
.byteenablemux_select(byteenablemux_select), .l2_mem_rdata(l2_mem_rdata), .l2_mem_response(l2_mem_response), .readsignalmux_select(readsignalmux_select),
.writesignalmux_select(writesignalmux_select), .DC_mem_byte_enable(DC_mem_byte_enable), .internal_mem_byte_enable(internal_mem_byte_enable),
.arbiter_internal_mem_response(arbiter_internal_mem_response), .arbiter_DC_mem_response(arbiter_DC_mem_response),
.arbiter_internal_mem_rdata(arbiter_internal_mem_rdata), .arbiter_mem_read(arbiter_mem_read), .arbiter_mem_write(arbiter_mem_write),
.arbiter_DC_mem_rdata(arbiter_DC_mem_rdata), .arbiter_mem_wdata(arbiter_mem_wdata), .arbiter_physicalmem_byte_enable(arbiter_physicalmem_byte_enable),
.arbiter_mem_addr(arbiter_mem_addr));
endmodule
