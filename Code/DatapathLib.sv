//This module will hold the cache datapath, basically the arrays need for the structures, the muxs, basically all relevant moduls like in lab6 did

import LC_3B_2::*;
module l2DatapathCache(
input clk,
input LC_3B_word mem_addr,
input cache_flow physicalmem_rdata,
input cache_flow l2_mem_wdata,
input [1:0] mem_byte_enable,
input logic inputdirty0, writedata0, writetag0, writedirty0, writevalid0, inputdirty1, writedata1, writetag1, writedirty1, writevalid1,
input logic inputdirty2, writevalid2, writedirty2, writetag2, writedata2, inputdirty3, writevalid3, writedirty3, writetag3, writedata3,
input logic pseudoLRU_write, stbwritemux_select, MUXRW_select, physicalmemMXselect,
output cache_flow l2_mem_rdata, p0ANDo,
output cache_flow physicalmem_wdata,
output LC_3B_word physicalmem_addr,
output logic hit, dirtymux_out,
output logic [1:0] pseudooutputLRU);

tag_size tag;
LC_3B_c_index index;
LC_3B_c_index offset;
logic offsetlast;
logic valid;

assign tag = mem_addr[15:7];
assign index = mem_addr[6:4];
assign offset = mem_addr[3:1];
assign offsetlast = mem_addr[0];
assign valid = 1'b1;
LC_3B_word stbwritemux_out;
LC_3B_word splitconcat_out;
tag_size tag0_out, tag1_out, tag2_out, tag3_out, tagmux_out;
cache_flow data0_out, data1_out, data2_out, data3_out, MUXRW_out, cache_flowconcat_out, datamux_out;
logic p0COMPARATOR_TAGX_out, p1COMPARATOR_TAGX_out, p2COMPARATOR_TAGX_out, p3COMPARATOR_TAGX_out;
logic valid0_out, valid1_out, valid2_out, valid3_out, p1anDC_out, p2anDC_out, p3anDC_out;
logic dirty0_out, dirty1_out, dirty2_out, dirty3_out, LRUmux_out;
LC_3B_byte memrdatasplitmux_out, memwdatasplitmux_out;

fourinOR hitcheck(.a(p0ANDo), .b(p1anDC_out), .c(p2anDC_out), .d(p3anDC_out), .o(hit));
pseudoLRU pseudoLRU(.clk, .pseudoLRU_write(pseudoLRU_write), .out0(valid0_out), .out1(valid1_out), .out2(valid2_out),.out3(valid3_out), .LRU(pseudooutputLRU));
fourtoonemux #(.width(128)) evictdatamux(.select(pseudooutputLRU),.a(data0_out), .b(data1_out), .c(data2_out), .d(data3_out), .o(physicalmem_wdata));
fourtoonemux #(.width(1)) dirtymux(.select(pseudooutputLRU), .a(dirty0_out), .b(dirty1_out), .c(dirty2_out), .d(dirty3_out), .o(dirtymux_out));
fourtoonemux #(.width(9)) tagmux(.select(pseudooutputLRU), .a(tag0_out), .b(tag1_out), .c(tag2_out), .d(tag3_out), .o(tagmux_out));
twotoonemux #(.width(16)) physicalmemmux(.select(physicalmemMXselect), .a(mem_addr), .b({tagmux_out,index,4'b0000}), .o(physicalmem_addr));

assign l2_mem_rdata = datamux_out;
logic [1:0] datamux_select;
always_comb
begin
if(p0ANDo)
	datamux_select = 2'b00;
else if(p1anDC_out)
	datamux_select = 2'b01;
else if(p2anDC_out)
	datamux_select = 2'b10;
else if(p3anDC_out)
	datamux_select = 2'b11;
else
	datamux_select = 2'b00;
end

fourtoonemux #(.width(128)) datamux(.select(datamux_select), .a(data0_out), .b(data1_out), .c(data2_out), .d(data3_out), .o(datamux_out));
twotoonemux #(.width(128)) MUXRW(.select(MUXRW_select), .a(physicalmem_rdata), .b(l2_mem_wdata), .o(MUXRW_out));
COMPARATOR_TAGX p0COMPARATOR_TAGX(.inputCache(tag), .mem_addr_in(tag0_out), .out(p0COMPARATOR_TAGX_out));
COMPARATOR_TAGX p1COMPARATOR_TAGX(.inputCache(tag), .mem_addr_in(tag1_out), .out(p1COMPARATOR_TAGX_out));
COMPARATOR_TAGX p2COMPARATOR_TAGX(.inputCache(tag), .mem_addr_in(tag2_out), .out(p2COMPARATOR_TAGX_out));
COMPARATOR_TAGX p3COMPARATOR_TAGX(.inputCache(tag), .mem_addr_in(tag3_out), .out(p3COMPARATOR_TAGX_out));
twoinAND p0and(.a(valid0_out), .b(p0COMPARATOR_TAGX_out), .o(p0ANDo));
twoinAND p1and(.a(valid1_out), .b(p1COMPARATOR_TAGX_out), .o(p1anDC_out));
twoinAND p2and(.a(valid2_out), .b(p2COMPARATOR_TAGX_out), .o(p2anDC_out));
twoinAND p3and(.a(valid3_out), .b(p3COMPARATOR_TAGX_out), .o(p3anDC_out));
arraycreaTOR #(.width(1)) valid0(.clk, .write(writevalid0), .index(index), .datain(valid), .dataout(valid0_out));
arraycreaTOR #(.width(1)) valid1(.clk,.write(writevalid1), .index(index), .datain(valid), .dataout(valid1_out));
arraycreaTOR #(.width(1)) valid2(.clk, .write(writevalid2),.index(index), .datain(valid), .dataout(valid2_out));
arraycreaTOR #(.width(1)) valid3(.clk, .write(writevalid3), .index(index), .datain(valid), .dataout(valid3_out));
arraycreaTOR #(.width(1)) dirty0(.clk, .write(writedirty0), .index(index), .datain(inputdirty0), .dataout(dirty0_out));
arraycreaTOR #(.width(1)) dirty1(.clk, .write(writedirty1), .index(index), .datain(inputdirty1), .dataout(dirty1_out));
arraycreaTOR #(.width(1)) dirty2(.clk, .write(writedirty2), .index(index), .datain(inputdirty2), .dataout(dirty2_out));
arraycreaTOR #(.width(1)) dirty3(.clk, .write(writedirty3), .index(index), .datain(inputdirty3), .dataout(dirty3_out));
arraycreaTOR #(.width(9)) tag0(.clk, .write(writetag0), .index(index), .datain(tag), .dataout(tag0_out)); 
arraycreaTOR #(.width(9)) tag1(.clk, .write(writetag1), .index(index), .datain(tag), .dataout(tag1_out)); 
arraycreaTOR #(.width(9)) tag2(.clk, .write(writetag2), .index(index), .datain(tag), .dataout(tag2_out));
arraycreaTOR #(.width(9)) tag3(.clk, .write(writetag3), .index(index), .datain(tag), .dataout(tag3_out));
arraycreaTOR data0(.clk, .write(writedata0), .index(index), .datain(MUXRW_out), .dataout(data0_out));
arraycreaTOR data1(.clk, .write(writedata1), .index(index), .datain(MUXRW_out), .dataout(data1_out));
arraycreaTOR data2(.clk, .write(writedata2), .index(index), .datain(MUXRW_out), .dataout(data2_out));
arraycreaTOR data3(.clk, .write(writedata3), .index(index), .datain(MUXRW_out), .dataout(data3_out));

endmodule


/* The datapath for the arbiter*/

import LC_3B_2::*;
module datapath_arbiter(
input clk,
input LC_3B_word internal_mem_addr, DC_mem_addr,
input cache_flow internal_mem_wdata, DC_mem_wdata, l2_mem_rdata,
input logic l2_mem_response, DC_mem_read, DC_mem_write, internal_mem_read, internal_mem_write, byteenablemux_select,
input logic readsignalmux_select, writesignalmux_select, memaddrmux_select, memrdatademux_select, memwdatamux_select, memresponsemux_select,
input logic [1:0] DC_mem_byte_enable, internal_mem_byte_enable,
output LC_3B_word arbiter_mem_addr,
output cache_flow arbiter_DC_mem_rdata, arbiter_internal_mem_rdata, arbiter_mem_wdata,
output logic arbiter_mem_read, arbiter_mem_write, arbiter_internal_mem_response, arbiter_DC_mem_response,
output logic [1:0] arbiter_physicalmem_byte_enable);

twotoonemux #(.width(2)) byteenablemux(.select(byteenablemux_select), .a(DC_mem_byte_enable), .b(internal_mem_byte_enable), .o(arbiter_physicalmem_byte_enable));
twotoonemux #(.width(1)) readsignalmux(.select(readsignalmux_select), .a(DC_mem_read), .b(internal_mem_read), .o(arbiter_mem_read));
twotoonemux #(.width(1)) writesignalmux(.select(writesignalmux_select), .a(DC_mem_write), .b(internal_mem_write), .o(arbiter_mem_write));
twotoonemux #(.width(128)) memwdatamux(.select(memwdatamux_select), .a(DC_mem_wdata), .b(internal_mem_wdata), .o(arbiter_mem_wdata));
twotoonemux memaddrmux(.select(memaddrmux_select), .a(DC_mem_addr), .b(internal_mem_addr), .o(arbiter_mem_addr));
one_two_decoder #(.width(128)) memrdatademux(.select(memrdatademux_select), .in(l2_mem_rdata), .Aout(arbiter_DC_mem_rdata), .Bout(arbiter_internal_mem_rdata));
one_two_decoder #(.width(1)) memresponsemux(.select(memresponsemux_select), .in(l2_mem_response), .Aout(arbiter_DC_mem_response), .Bout(arbiter_internal_mem_response));
endmodule


// L1Cache datapath

import LC_3B_2::*;
module DatapathCache(
input clk,
input cache_flow physicalmem_rdata,
input LC_3B_word mem_addr, mem_wdata,
input logic inputdirty0, writedata0, writetag0, writedirty0, writevalid0, inputdirty1, writedata1, writetag1, writedirty1, writevalid1,
input logic LRU_in, LRU_write, stbwritemux_select, MUXRW_select, physicalmemMXselect,
input [1:0] mem_byte_enable,
output LC_3B_word mem_rdata, physicalmem_addr,
output cache_flow physicalmem_wdata,
output logic hit, outputLRU, p0ANDo, dirtymux_out);

tag_size tag;
LC_3B_c_index index, offset;
logic offsetlast, valid;

assign tag = mem_addr[15:7];
assign index = mem_addr[6:4];
assign offset = mem_addr[3:1];
assign offsetlast = mem_addr[0];
assign valid = 1'b1;
tag_size tag0_out, tag1_out, tagmux_out;
cache_flow data0_out, data1_out, datamux_out, MUXRW_out, cache_flowconcat_out;
LC_3B_byte memrdatasplitmux_out, memwdatasplitmux_out;
LC_3B_word stbwritemux_out, splitconcat_out;
logic p0COMPARATOR_TAGX_out, p1COMPARATOR_TAGX_out, p1anDC_out, LRUmux_out, valid0_out, valid1_out, dirty0_out, dirty1_out;

twoinOR hitcheck(.a(p0ANDo), .b(p1anDC_out), .o(hit));
arraycreaTOR #(.width(1)) LRU(.clk, .write(LRU_write), .index(index), .datain(LRU_in), .dataout(outputLRU));
twotoonemux #(.width(16)) physicalmemmux(.select(physicalmemMXselect), .a(mem_addr), .b({tagmux_out,index,4'b0000}), .o(physicalmem_addr));
twotoonemux #(.width(128)) MUXRW(.select(MUXRW_select), .a(physicalmem_rdata), .b(cache_flowconcat_out), .o(MUXRW_out));
twotoonemux #(.width(8)) memwdatasplitmux(.select(mem_byte_enable[1]), .a(mem_wdata[7:0]), .b(mem_wdata[15:8]), .o(memwdatasplitmux_out));
twotoonemux #(.width(8)) memrdatasplitmux(.select(mem_byte_enable[0]), .a(mem_rdata[7:0]), .b(mem_rdata[15:8]), .o(memrdatasplitmux_out));
twotoonemux #(.width(16)) stbwritemux(.select(stbwritemux_select), .a(mem_wdata), .b(splitconcat_out), .o(stbwritemux_out));
eight_one_mux memrdatamux(.select(offset), .a(datamux_out[15:0]), .b(datamux_out[31:16]), .c(datamux_out[47:32]), .d(datamux_out[63:48]), 
.e(datamux_out[79:64]), .f(datamux_out[95:80]), .g(datamux_out[111:96]), .h(datamux_out[127:112]), .out(mem_rdata));
CONCATENATOR_MUX cache_flowconcat(.data_r(datamux_out), .data_wr(stbwritemux_out), .select(offset), .out(cache_flowconcat_out));
CONCATENATORDOSMX splitconcat(.a(memrdatasplitmux_out), .b(memwdatasplitmux_out), .select(mem_byte_enable[1]), .out(splitconcat_out));
COMPARATOR_TAGX p0COMPARATOR_TAGX(.inputCache(tag), .mem_addr_in(tag0_out), .out(p0COMPARATOR_TAGX_out));
COMPARATOR_TAGX p1COMPARATOR_TAGX(.inputCache(tag), .mem_addr_in(tag1_out), .out(p1COMPARATOR_TAGX_out));
twoinAND p0and(.a(valid0_out),.b(p0COMPARATOR_TAGX_out), .o(p0ANDo));
twoinAND p1and(.a(valid1_out), .b(p1COMPARATOR_TAGX_out), .o(p1anDC_out));
twotoonemux #(.width(128)) evictdatamux(.select(outputLRU), .a(data0_out), .b(data1_out), .o(physicalmem_wdata));
arraycreaTOR #(.width(1)) valid0(.clk, .write(writevalid0), .index(index), .datain(valid), .dataout(valid0_out));
arraycreaTOR #(.width(1)) valid1(.clk, .write(writevalid1), .index(index), .datain(valid), .dataout(valid1_out));
twotoonemux #(.width(1)) dirtymux(.select(outputLRU), .a(dirty0_out), .b(dirty1_out),.o(dirtymux_out));
arraycreaTOR #(.width(1)) dirty0(.clk, .write(writedirty0), .index(index), .datain(inputdirty0), .dataout(dirty0_out));
arraycreaTOR #(.width(1)) dirty1(.clk, .write(writedirty1), .index(index), .datain(inputdirty1), .dataout(dirty1_out));
twotoonemux #(.width(9)) tagmux(.select(outputLRU), .a(tag0_out), .b(tag1_out), .o(tagmux_out));
arraycreaTOR #(.width(9)) tag0(.clk, .write(writetag0), .index(index), .datain(tag), .dataout(tag0_out));
arraycreaTOR #(.width(9)) tag1(.clk, .write(writetag1), .index(index), .datain(tag), .dataout(tag1_out));
twotoonemux #(.width(128)) datamux(.select(p1anDC_out), .a(data0_out),.b(data1_out), .o(datamux_out));
arraycreaTOR data0(.clk, .write(writedata0), .index(index), .datain(MUXRW_out), .dataout(data0_out));
arraycreaTOR data1(.clk, .write(writedata1), .index(index), .datain(MUXRW_out), .dataout(data1_out));
endmodule


//main datapath for lc3b

import LC_3B_2::*;
module lc3b_datapath(
input clk,
input LC_3B_word internal_mem_rdata, DC_mem_rdata,
input logic idle_state, DC_mem_response, DCache_hit, internal_mem_response,
output LC_3B_word internal_mem_addr, internal_mem_wdata, DC_mem_addr, DC_mem_wdata,
output logic internal_mem_read, internal_mem_write, DC_mem_read, DC_mem_write, DCache_enable, TRUE_BR,
output logic [1:0] DC_mem_byte_enable, internal_mem_byte_enable);

//IF
logic [1:0] PCmux_select;
logic Load_PC;
LC_3B_word PC_twoadDC_out, PC_out, PCmux_out;
//IF/ID
logic IF_IDC_v_out, Load_IF_ID, IF_IDC_v_in;
LC_3B_word IF_IDC_PC_out, IF_IDC_ir_out;
//ID
logic stallhazard, flushpipe;
LC_3B_reg storemux_out, cc_out;
LC_3B_word sr1_out, sr2_out;
LC_3B_control_word control_store;
//ID/EX
logic IDC_EX_v_out, Load_IDC_EX, ISVAL_IDC_EX_out, IDC_EX_v_in;
LC_3B_control_word IDC_EX_cs_out;
LC_3B_word IDC_EX_PC_out, IDC_EX_ir_out, SR1_IDC_EX_out, SR2_IDC_EX_out;
LC_3B_reg IDC_EX_cc_out, out_IDC_EX_out, Destination_mux_out, IDC_EX_src1_out, IDC_EX_src2_out;
//EX
LC_3B_word SEXT5_out, SEXT6_out, SEXT9_out, SEXT11_out, addr2mux_out, LeftSHIFTone_out, addr1mux_out, addr3mux_out, zextLeftSHIFT_out;
LC_3B_word addradder_out, sr2mux_out, Alu_out, ShiftBlock_out, Alu_result_mux_out, forwardmux1_out, forwardtwotoonemux_out;
logic [1:0] selectFM1, selectFM2;
//EX/MEM
logic Load_EX_MEM, unconDC_or_trap, VALIDC_EX_MEM_in, VALIDC_EX_MEM_out;
LC_3B_control_word EX_MEM_cs_out;
LC_3B_word EX_MEM_addr_out, EX_MEM_ir_out, EX_MEM_PC_out, EX_MEM_Aluresult_out;
LC_3B_reg EX_MEM_Destination_out, EX_MEM_cc_out, EX_MEM_src1_out, EX_MEM_src2_out;
//MEM
LC_3B_word mem_target, mem_trap, DCacheaddrmux_out, HBzext_out, LBzext_out, DCachemux_out, DCachetwotoonemux_out, ZEXTblock_out;
LC_3B_word MEM_WB_data_mux_out, ldistireg_out,write_DCacheritemux_out;
logic CondCodeComp_out, STALL_DCache,jsr_taken,trap_taken, DCacheaddrmux_select, STALL_LDI, Load_ldistireg, DCacheRmux_select, DCacheRmux_out;
logic andWE0_out, andWE1_out, andWE_select_out, WE0, WE1, MEM_WB_data_mux_select, write_DCachemux_select, write_DCachemux_out, wbCondCodeComp_out;
logic CondCodeCompmux_select, CondCodeCompmux_out;
logic [1:0] ldisticountererout;
LC_3B_reg wbCondCode_out;
//MEM/WB
logic Load_mem_WB, VALIDC_mem_WB_out, VALIDC_mem_WB_in;
LC_3B_control_word MEM_WB_cs_out;
LC_3B_word MEM_WB_addr_out, MEM_WB_data_out, MEM_WB_PC_out, MEM_WB_Aluresult_out, MEM_WB_ir_out;
LC_3B_reg MEM_WB_Destination_out, MEM_WB_src1_out, MEM_WB_src2_out;
//WB
logic wb_LDC_CC, wb_registerload;
LC_3B_word wbmux_out;
LC_3B_reg wb_cc_data;

//IF
assign internal_mem_addr = PC_out;
assign internal_mem_read = 1'b1;
assign internal_mem_write = 1'b0;
assign internal_mem_wdata = 16'b0000000000000000;
assign internal_mem_byte_enable = 2'b11;
assign Load_IF_ID = internal_mem_response & !STALL_DCache & !stallhazard;

always_comb
begin
	if(internal_mem_response)
		Load_PC = internal_mem_response & !STALL_DCache & !STALL_LDI & !stallhazard;
	else if(!internal_mem_response && TRUE_BR && (mem_target == 16'b0000001011000110))
		Load_PC =  1;
	else if(!internal_mem_response && EX_MEM_cs_out.OPJMP &&(EX_MEM_Aluresult_out == 16'b0000001101100010))
		Load_PC = 1;
	else
		Load_PC = internal_mem_response & !STALL_DCache & !STALL_LDI & !stallhazard;
end
register PC(.clk, .load(Load_PC), .in(PCmux_out), .out(PC_out));
fourtoonemux PCmux(.select(PCmux_select), .a(PC_twoadDC_out), .b(mem_target), .c(mem_trap),.d(EX_MEM_Aluresult_out),.o(PCmux_out));
twoadd PC_twoadd(.in(PC_out), .out(PC_twoadDC_out));

//IF-ID PIPELINE
register #(.width(1)) IF_IDC_v(.clk, .load(Load_IF_ID), .in(IF_IDC_v_in), .out(IF_IDC_v_out)); 
register IF_IDC_PC(.clk, .load(Load_IF_ID), .in(PC_twoadDC_out), .out(IF_IDC_PC_out));
register IF_IDC_ir(.clk, .load(Load_IF_ID),.in(internal_mem_rdata), .out(IF_IDC_ir_out));

//ID
REGFILE REGFILE(.clk, .load(wb_registerload), .in(wbmux_out),.AREF(IF_IDC_ir_out[8:6]), .BREF(storemux_out), .Destination(MEM_WB_Destination_out),
.reg_a(sr1_out), .reg_b(sr2_out));
hazard_detection hazard_detection(.DCacheR(IDC_EX_cs_out.DCacheR), .DRM_IDC_EX(out_IDC_EX_out), .IF_IDC_A(IF_IDC_ir_out[8:6]), .IF_IDC_B(IF_IDC_ir_out[2:0]),
.SDR_IF_ID(IF_IDC_ir_out[11:9]), .write_DCache(control_store.write_DCache), .IDC_EX_AREQ(IDC_EX_cs_out.REQSR1), .IDC_EX_BREQ(IDC_EX_cs_out.REQSR2),
.OPUNCO(EX_MEM_cs_out.OPUNCO), .flushpipe(flushpipe), .stallhazard(stallhazard));
register #(.width(3)) cc(.clk, .load(wb_LDC_CC),.in(wb_cc_data), .out(cc_out));
controlla controlla(.OPCode(LC_3B_OPCode'(IF_IDC_ir_out[15:12])), .IR5(IF_IDC_ir_out[5]), .IR11(IF_IDC_ir_out[11]), .PC(IF_IDC_PC_out), .control(control_store));
twotoonemux #(.width(3)) storemux(.select(control_store.storemux_select), .a(IF_IDC_ir_out[2:0]), .b(IF_IDC_ir_out[11:9]), .o(storemux_out));
twotoonemux #(.width(3)) Destination_mux(.select(control_store.Destination_mux_select), .a(IF_IDC_ir_out[11:9]), .b(3'b111), .o(Destination_mux_out));

//ID-EX PIPELINE
assign Load_IDC_EX = ISVAL_IDC_EX_out;
ISVAL_IDC_EX ISVAL_IDC_EX(.clk, .internal_mem_response(internal_mem_response), .STALL_DCache(STALL_DCache),.STALL_LDI(STALL_LDI),  .TRUE_BR(TRUE_BR),.out(ISVAL_IDC_EX_out));
register IDC_EX_PC(.clk, .load(Load_IDC_EX), .in(IF_IDC_PC_out), .out(IDC_EX_PC_out)); 
register IDC_EX_ir(.clk, .load(Load_IDC_EX), .in(IF_IDC_ir_out), .out(IDC_EX_ir_out));
register SR1_IDC_EX(.clk, .load(Load_IDC_EX), .in(sr1_out), .out(SR1_IDC_EX_out)); 
register SR2_IDC_EX(.clk, .load(Load_IDC_EX), .in(sr2_out), .out(SR2_IDC_EX_out)); 
contrsigreg IDC_EX_cs(.clk, .load(Load_IDC_EX), .in(control_store),.out(IDC_EX_cs_out));
register #(.width(3)) IDC_EX_cc(.clk, .load(Load_IDC_EX), .in(cc_out), .out(IDC_EX_cc_out));
register #(.width(3)) out_IDC_EX(.clk, .load(Load_IDC_EX), .in(Destination_mux_out), .out(out_IDC_EX_out));
register #(.width(1)) IDC_EX_v(.clk, .load(ISVAL_IDC_EX_out), .in(IDC_EX_v_in), .out(IDC_EX_v_out));
register #(.width(3)) IDC_EX_src1(.clk, .load(Load_IDC_EX), .in(IF_IDC_ir_out[8:6]), .out(IDC_EX_src1_out));
register #(.width(3)) IDC_EX_src2(.clk, .load(Load_IDC_EX), .in(storemux_out), .out(IDC_EX_src2_out));

//EX
Alu Alu(.OPAlu(IDC_EX_cs_out.OPAlu), .a(forwardmux1_out), .b(forwardtwotoonemux_out), .f(Alu_out));
adderst addradder(.a(addr1mux_out), .b(LeftSHIFTone_out), .out(addradder_out));
SEXT #(.width(5)) SEXT5(.in(IDC_EX_ir_out[4:0]), .out(SEXT5_out));
SEXT #(.width(6)) SEXT6(.in(IDC_EX_ir_out[5:0]), .out(SEXT6_out));
SEXT #(.width(9)) SEXT9(.in(IDC_EX_ir_out[8:0]), .out(SEXT9_out));
SEXT #(.width(11)) SEXT11(.in(IDC_EX_ir_out[10:0]), .out(SEXT11_out));  
twotoonemux addr1mux(.select(IDC_EX_cs_out.addr1mux_select), .a(IDC_EX_PC_out), .b(SR1_IDC_EX_out), .o(addr1mux_out));
twotoonemux sr2mux(.select(IDC_EX_cs_out.sr2mux_select), .a(SR2_IDC_EX_out), .b(SEXT5_out), .o(sr2mux_out));
twotoonemux addr3mux(.select(IDC_EX_cs_out.addr3mux_select), .a(addradder_out), .b(zextLeftSHIFT_out), .o(addr3mux_out));
twotoonemux Alu_result_mux(.select(IDC_EX_cs_out.Alu_result_mux_select), .a(Alu_out),.b(ShiftBlock_out), .o(Alu_result_mux_out));
threetoonemux forwardtwotoonemux(.select(selectFM2), .a(sr2mux_out), .b(wbmux_out), .c(EX_MEM_Aluresult_out), .o(forwardtwotoonemux_out));
fourtoonemux addr2mux(.select(IDC_EX_cs_out.addr2mux_select), .a(16'b0000000000000000), .b(SEXT6_out), .c(SEXT9_out), .d(SEXT11_out),  .o(addr2mux_out));
fourtoonemux forwardmux1(.select(selectFM1), .a(SR1_IDC_EX_out), .b(wbmux_out), .c(EX_MEM_Aluresult_out), .d(EX_MEM_addr_out), .o(forwardmux1_out));
ShiftBlock ShiftBlock(.in(forwardmux1_out), .shiftword(IDC_EX_ir_out[5:0]), .out(ShiftBlock_out)); 
LeftSHIFTone LeftSHIFTone(.select(IDC_EX_cs_out.LeftSHIFT), .in(addr2mux_out), .out(LeftSHIFTone_out));
LeftShift_ZEXT zextLeftSHIFT(.in(IDC_EX_ir_out[7:0]), .out(zextLeftSHIFT_out));
forwardselectunit forwardselectunit(.clk, .registerload(IDC_EX_cs_out.registerload), .DRM_EX_MEM(EX_MEM_Destination_out), .DRM_MEM_WB(MEM_WB_Destination_out),
.out_IDC_EX(out_IDC_EX_out), .SR1_IDC_EX(IDC_EX_src1_out), .SR2_IDC_EX(IDC_EX_src2_out), .IDC_EX_AREQ(IDC_EX_cs_out.REQSR1),
.IDC_EX_BREQ(IDC_EX_cs_out.REQSR2), .DRM_IDC_EX_REQ(IDC_EX_cs_out.dr_REQ), .SR1_EX_MEM_REQ(EX_MEM_cs_out.REQSR1), .SR2_EX_MEM_REQ(EX_MEM_cs_out.REQSR2), 
.DRM_EX_MEM_REQ(EX_MEM_cs_out.dr_REQ), .SR1_MEM_WB_REQ(MEM_WB_cs_out.REQSR1), .SR2_MEM_WB_REQ(MEM_WB_cs_out.REQSR2), .DRM_MEM_WB_REQ(MEM_WB_cs_out.dr_REQ),
.VALIDC_EX_MEM(VALIDC_EX_MEM_out), .VALIDC_mem_WB(VALIDC_mem_WB_out), .selectFM1(selectFM1), .selectFM2(selectFM2));

//EX-MEM PIPELINE
assign Load_EX_MEM = !STALL_DCache & !STALL_LDI;
register EX_MEM_Aluresult(.clk, .load(Load_EX_MEM), .in(Alu_result_mux_out),.out(EX_MEM_Aluresult_out));
register EX_MEM_addr(.clk, .load(Load_EX_MEM), .in(addr3mux_out), .out(EX_MEM_addr_out));
register EX_MEM_PC(.clk, .load(Load_EX_MEM), .in(IDC_EX_PC_out), .out(EX_MEM_PC_out));
register EX_MEM_ir(.clk, .load(Load_EX_MEM), .in(IDC_EX_ir_out), .out(EX_MEM_ir_out));
contrsigreg EX_MEM_cs(.clk, .load(Load_EX_MEM), .in(IDC_EX_cs_out),.out(EX_MEM_cs_out));
register #(.width(3)) EX_MEM_Destination(.clk, .load(Load_EX_MEM), .in(out_IDC_EX_out), .out(EX_MEM_Destination_out));
register #(.width(3)) EX_MEM_cc(.clk, .load(Load_EX_MEM), .in(IDC_EX_cc_out), .out(EX_MEM_cc_out));
register #(.width(3)) EX_MEM_src1(.clk, .load(Load_EX_MEM), .in(IDC_EX_src1_out), .out(EX_MEM_src1_out)); 
register #(.width(3)) EX_MEM_src2(.clk, .load(Load_EX_MEM), .in(IDC_EX_src2_out), .out(EX_MEM_src2_out));
register #(.width(1)) VALIDC_EX_MEM(.clk, .load(Load_EX_MEM), .in(VALIDC_EX_MEM_in), .out(VALIDC_EX_MEM_out));

//MEM
assign mem_target = EX_MEM_addr_out;
assign DC_mem_read = DCacheRmux_out;
assign DC_mem_write = write_DCachemux_out;
assign DC_mem_wdata = write_DCacheritemux_out;
assign mem_trap = DC_mem_rdata;

always_comb
begin
if((EX_MEM_cs_out.OP_STI) &&(ldisticountererout == 2'b00))
begin
	DCacheRmux_select = 1;
	write_DCachemux_select = 1;
end
else
begin
	DCacheRmux_select = 0;
	write_DCachemux_select = 0;
end
end
assign DC_mem_byte_enable[1] = WE1;
assign DC_mem_byte_enable[0] = WE0;
assign DC_mem_addr = DCacheaddrmux_out;
assign DCache_enable = EX_MEM_cs_out.DCache_enable & VALIDC_EX_MEM_out;
assign STALL_DCache = DCache_enable & !DC_mem_response;
logic [2:0] condition;
always_comb
begin
IF_IDC_v_in = !TRUE_BR  & !flushpipe;
IDC_EX_v_in = !TRUE_BR & IF_IDC_v_out & !flushpipe; 
VALIDC_mem_WB_in = 	!TRUE_BR & VALIDC_EX_MEM_out;
if(IF_IDC_ir_out == 16'b1110001000101111)
	VALIDC_EX_MEM_in = 0;
else
	condition = 3'b100;
	VALIDC_EX_MEM_in = !TRUE_BR & IDC_EX_v_out  & !flushpipe;
if((IF_IDC_ir_out == MEM_WB_ir_out)&& (!idle_state) && (MEM_WB_cs_out.OPCode == op_str))
begin
	condition = 3'b001;
	VALIDC_mem_WB_in = 0;
	VALIDC_EX_MEM_in = 0;
	IDC_EX_v_in = 0;
	IF_IDC_v_in = 1;
end
if(!idle_state && EX_MEM_cs_out.registerload)
begin
	condition = 3'b000;
	VALIDC_mem_WB_in = 1;
	VALIDC_EX_MEM_in = 0;
	IDC_EX_v_in = 0;
	IF_IDC_v_in = 0;
end
if(!idle_state && EX_MEM_cs_out.OPBR && MEM_WB_cs_out.registerload)
begin
	condition = 3'b010;
	VALIDC_mem_WB_in = 1;
	VALIDC_EX_MEM_in = 1;
	IDC_EX_v_in = 0;
	IF_IDC_v_in = 0;
end
if(((MEM_WB_src1_out == MEM_WB_Destination_out)||(MEM_WB_src2_out == MEM_WB_Destination_out))&& (MEM_WB_cs_out.registerload) && (!idle_state) && (MEM_WB_cs_out.OPCode == op_add))
begin
	IF_IDC_v_in = 0;
	IDC_EX_v_in = !TRUE_BR & IF_IDC_v_out & !flushpipe; 
	VALIDC_EX_MEM_in = 0;
	if(IF_IDC_ir_out == 16'b0001100001100011)
		VALIDC_mem_WB_in = 1;
	else
		condition = 3'b011;
		VALIDC_mem_WB_in = 0;
end
end

assign unconDC_or_trap = (((EX_MEM_cs_out.OPUNCO) || (EX_MEM_cs_out.OPTRAP)) && (!EX_MEM_cs_out.OPJSR) && (!EX_MEM_cs_out.OPJMP));
assign jsr_taken = EX_MEM_cs_out.OPJSR & VALIDC_EX_MEM_out;
assign trap_taken = unconDC_or_trap & VALIDC_EX_MEM_out;
assign Load_ldistireg = DC_mem_response;
twotoonemux #(.width(1)) write_DCachemux(.select(write_DCachemux_select), .a(EX_MEM_cs_out.write_DCache), .b(1'b0), .o(write_DCachemux_out));
twotoonemux #(.width(1)) DCacheRmux(.select(DCacheRmux_select), .a(EX_MEM_cs_out.DCacheR), .b(1'b1), .o(DCacheRmux_out));
always_comb
begin
if((ldisticountererout == 2'b00) && (!DC_mem_response))
begin
	STALL_LDI = 0;
	DCacheaddrmux_select = 0;
end
else if((ldisticountererout == 2'b00) && (DC_mem_response)&&(EX_MEM_cs_out.OP_LDI || EX_MEM_cs_out.OP_STI))
begin
	STALL_LDI = 1;
	DCacheaddrmux_select = 0;
end
else if((ldisticountererout == 2'b01)&&(EX_MEM_cs_out.OP_LDI || EX_MEM_cs_out.OP_STI))
begin
	STALL_LDI = 0;
	DCacheaddrmux_select = 1;
end
else if((ldisticountererout == 2'b01)&&(DC_mem_response)&&(EX_MEM_cs_out.OP_LDI || EX_MEM_cs_out.OP_STI))
begin
	STALL_LDI = 0;
	DCacheaddrmux_select = 0;
end
else if(ldisticountererout == 2'b10)
begin
	STALL_LDI = 0;
	DCacheaddrmux_select = 0;
end
else
begin
	STALL_LDI = 0;
	DCacheaddrmux_select = 0;
end
end
register ldistireg(.clk, .load(Load_ldistireg), .in(DC_mem_rdata), .out(ldistireg_out));
bitcounter ldisticounterer(.clk, .DC_mem_response(DC_mem_response), .OP_LDI(EX_MEM_cs_out.OP_LDI), .OP_STI(EX_MEM_cs_out.OP_STI),
.STALL_DCache(STALL_DCache),.counter(ldisticountererout));
CondCode wbCondCode(.in(MEM_WB_Aluresult_out), .out(wbCondCode_out));
CondCodeComp CondCodeComp(.a(EX_MEM_cc_out), .b(EX_MEM_ir_out[11:9]), .out(CondCodeComp_out));
CondCodeComp wbCondCodeComp(.a(wbCondCode_out), .b(EX_MEM_ir_out[11:9]), .out(wbCondCodeComp_out));
twotoonemux DCacheaddrmux(.select(DCacheaddrmux_select), .a(EX_MEM_addr_out), .b(ldistireg_out), .o(DCacheaddrmux_out));

always_comb
begin
if(MEM_WB_cs_out.LDC_CC == 1)
	CondCodeCompmux_select = 1;
else
	CondCodeCompmux_select = 0;
end

BRMODS BRMODS(.a(trap_taken), .b(jsr_taken), .c(TRUE_BR), .d(EX_MEM_cs_out.OPJMP), .out(PCmux_select));
twotoonemux DCachemux(.select(EX_MEM_addr_out[0]), .a(LBzext_out), .b(HBzext_out), .o(DCachemux_out));
twotoonemux write_DCacheritemux(.select(EX_MEM_cs_out.OPSTB), .a(EX_MEM_Aluresult_out), .b(ZEXTblock_out),.o(write_DCacheritemux_out));
twotoonemux #(.width(1)) CondCodeCompmux(.select(CondCodeCompmux_select), .a(CondCodeComp_out), .b(wbCondCodeComp_out), .o(CondCodeCompmux_out));
twotoonemux DCachetwotoonemux(.select(EX_MEM_cs_out.DC_mem_byte_select), .a(DC_mem_rdata), .b(DCachemux_out), .o(DCachetwotoonemux_out));
twotoonemux #(.width(1)) WE0mux(.select(andWE_select_out), .a(1'b1), .b(andWE0_out), .o(WE0));
twotoonemux #(.width(1)) WE1mux(.select(andWE_select_out),.a(1'b1), .b(andWE1_out), .o(WE1));
twoinAND andWE0(.a(EX_MEM_cs_out.write_DCache), .b(!EX_MEM_addr_out[0]), .o(andWE0_out)); 
twoinAND andWE1(.a(EX_MEM_cs_out.write_DCache), .b(EX_MEM_addr_out[0]), .o(andWE1_out));
twoinAND andWE_select(.a(EX_MEM_cs_out.write_DCache), .b(EX_MEM_cs_out.OPSTB), .o(andWE_select_out));
threeinAND br_and(.a(VALIDC_EX_MEM_out), .b(EX_MEM_cs_out.OPBR), .c(CondCodeCompmux_out), .o(TRUE_BR));
VALCHECK_MEM_WB VALCHECK_MEM_WB(.LDI_cs(EX_MEM_cs_out.OP_LDI), .STALL_LDI(STALL_LDI), .STALL_DCache(STALL_DCache), .out(Load_mem_WB));
zext #(.width(8)) LBzext(.in(DC_mem_rdata[7:0]), .out(LBzext_out));
zext #(.width(8)) HBzext(.in(DC_mem_rdata[15:8]), .out(HBzext_out)); 
ZEXTblock #(.width(8)) ZEXTblock(.in(EX_MEM_Aluresult_out[7:0]), .ZEXTblockselect(EX_MEM_addr_out[0]), .out(ZEXTblock_out));

//MEM-WB PIPELINE
register MEM_WB_addr(.clk, .load(Load_mem_WB), .in(EX_MEM_addr_out),.out(MEM_WB_addr_out));
register MEM_WB_Aluresult(.clk, .load(Load_mem_WB), .in(EX_MEM_Aluresult_out), .out(MEM_WB_Aluresult_out));
register MEM_WB_PC(.clk, .load(Load_mem_WB), .in(EX_MEM_PC_out), .out(MEM_WB_PC_out));
register MEM_WB_ir(.clk, .load(Load_mem_WB), .in(EX_MEM_ir_out), .out(MEM_WB_ir_out));
register MEM_WB_data(.clk, .load(Load_mem_WB), .in(DCachetwotoonemux_out), .out(MEM_WB_data_out));
register #(.width(3)) MEM_WB_Destination(.clk, .load(Load_mem_WB), .in(EX_MEM_Destination_out), .out(MEM_WB_Destination_out));
register #(.width(3)) MEM_WB_src1(.clk, .load(Load_mem_WB), .in(EX_MEM_src1_out), .out(MEM_WB_src1_out));
register #(.width(3)) MEM_WB_src2(.clk, .load(Load_mem_WB), .in(EX_MEM_src2_out), .out(MEM_WB_src2_out));
register #(.width(1)) VALIDC_mem_WB(.clk, .load(Load_mem_WB), .in(VALIDC_mem_WB_in), .out(VALIDC_mem_WB_out));
contrsigreg MEM_WB_cs(.clk, .load(Load_mem_WB), .in(EX_MEM_cs_out), .out(MEM_WB_cs_out));


//WB
twoinAND wb_registerLoad_and(.a(VALIDC_mem_WB_out), .b(MEM_WB_cs_out.registerload), .o(wb_registerload));
twoinAND wb_LDC_CC_and(.a(VALIDC_mem_WB_out), .b(MEM_WB_cs_out.LDC_CC), .o(wb_LDC_CC));
fourtoonemux wbmux(.select(MEM_WB_cs_out.wbmux_select), .a(MEM_WB_addr_out),.b(MEM_WB_data_out), .c(MEM_WB_PC_out),  .d(MEM_WB_Aluresult_out), .o(wbmux_out));
CondCode CondCode(.in(wbmux_out), .out(wb_cc_data));
endmodule
