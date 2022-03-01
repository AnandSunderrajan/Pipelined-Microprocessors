import LC_3B_2::*;

module projtoplevel(
input clk,
input cache_flow physicalmem_mem_rdata,
input logic physicalmem_mem_response,
output logic physicalmem_mem_read, physicalmem_mem_write,
output logic [1:0] physicalmem_byte_enable,
output cache_flow physicalmem_mem_wdata,
output LC_3B_word physicalmem_mem_addr);
LC_3B_word arbiter_mem_addr;
LC_3B_word internal_mem_rdata;
LC_3B_word internal_mem_addr;
LC_3B_word internal_mem_wdata;
LC_3B_word IC_physicalmem_addr;
cache_flow IC_physicalmem_wdata;
cache_flow IC_physicalmem_rdata;
LC_3B_word DC_mem_addr;
LC_3B_word DC_mem_wdata;
LC_3B_word DC_mem_rdata;
LC_3B_word DC_physicalmem_addr;
cache_flow DC_physicalmem_wdata;
cache_flow DC_physicalmem_rdata;
cache_flow arbiter_internal_mem_rdata;
cache_flow arbiter_DC_mem_rdata;
cache_flow arbiter_mem_wdata;
cache_flow l2_mem_rdata;
logic internal_mem_read, internal_mem_write, l2_mem_response, l2hit, internal_mem_response, DC_mem_read, DC_mem_write, DC_mem_response;
logic DCache_enable, IC_readphysicalmem, IC_writephysicalmem, IC_physicalmem_response, TRUE_BR, idle_state, DC_readphysicalmem;
logic DC_writephysicalmem, DC_physicalmem_response, DCache_hit, arbiter_internal_mem_response, arbiter_DC_mem_response, arbiter_mem_write, arbiter_mem_read;
logic [1:0] DC_mem_byte_enable, internal_mem_byte_enable, IC_physicalmem_byte_enable, DC_physicalmem_byte_enable, arbiter_physicalmem_byte_enable;

assign physicalmem_byte_enable = arbiter_physicalmem_byte_enable;


//Arbiter wrappa
arbiter arbiter(
.clk,
.internal_mem_addr(IC_physicalmem_addr),
.internal_mem_read(IC_readphysicalmem),
.internal_mem_write(IC_writephysicalmem),
.internal_mem_wdata(IC_physicalmem_wdata),
.internal_mem_byte_enable(internal_mem_byte_enable),
.DC_mem_addr(DC_physicalmem_addr),
.DC_mem_read(DC_readphysicalmem),
.DC_mem_write(DC_writephysicalmem),
.DC_mem_wdata(DC_physicalmem_wdata),
.DC_mem_byte_enable(DC_mem_byte_enable),
.l2hit(l2hit),
.l2_mem_rdata(l2_mem_rdata),
.l2_mem_response(l2_mem_response),
.arbiter_mem_addr(arbiter_mem_addr),
.arbiter_mem_read(arbiter_mem_read),
.arbiter_mem_write(arbiter_mem_write),
.arbiter_mem_wdata(arbiter_mem_wdata),
.arbiter_internal_mem_response(arbiter_internal_mem_response),
.arbiter_DC_mem_response(arbiter_DC_mem_response),
.arbiter_DC_mem_rdata(arbiter_DC_mem_rdata),
.arbiter_internal_mem_rdata(arbiter_internal_mem_rdata),
.arbiter_physicalmem_byte_enable(arbiter_physicalmem_byte_enable));


// L1 I Cache Wrappa
L1_ICache L1_ICache(
.clk,
.idle_state(idle_state),
.TRUE_BR(TRUE_BR),
.readphysicalmem(IC_readphysicalmem),
.writephysicalmem(IC_writephysicalmem),
.mem_addr(internal_mem_addr),
.mem_read(internal_mem_read),
.mem_write(internal_mem_write),
.mem_rdata(internal_mem_rdata),
.mem_wdata(internal_mem_wdata),
.mem_byte_enable(internal_mem_byte_enable),
.physicalmem_addr(IC_physicalmem_addr),
.physicalmem_rdata(arbiter_internal_mem_rdata),
.physicalmem_wdata(IC_physicalmem_wdata),
.physicalmem_response(arbiter_internal_mem_response),
.mem_response(internal_mem_response));


//L1 D cache wrappa
L1_DCache L1_DCache(
.clk,
.hit(DCache_hit),
.mem_addr(DC_mem_addr),
.mem_read(DC_mem_read),
.mem_write(DC_mem_write),
.mem_rdata(DC_mem_rdata),
.mem_wdata(DC_mem_wdata),
.mem_response(DC_mem_response),
.mem_byte_enable(DC_mem_byte_enable),
.DCache_enable(DCache_enable),
.physicalmem_addr(DC_physicalmem_addr),
.physicalmem_rdata(arbiter_DC_mem_rdata),
.physicalmem_wdata(DC_physicalmem_wdata),
.physicalmem_response(arbiter_DC_mem_response),
.readphysicalmem(DC_readphysicalmem),
.writephysicalmem(DC_writephysicalmem));


//L2Cache
L2Cache L2Cache(
.clk,
.physicalmem_addr(physicalmem_mem_addr),
.physicalmem_rdata(physicalmem_mem_rdata),
.physicalmem_wdata(physicalmem_mem_wdata),
.physicalmem_response(physicalmem_mem_response),
.l2hit(l2hit),
.l2_mem_byte_enable(arbiter_physicalmem_byte_enable),
.l2_mem_addr(arbiter_mem_addr),
.l2_mem_read(arbiter_mem_read),
.l2_mem_write(arbiter_mem_write),
.l2_mem_rdata(l2_mem_rdata),
.l2_mem_wdata(arbiter_mem_wdata),
.l2_mem_response(l2_mem_response),
.readphysicalmem(physicalmem_mem_read),
.writephysicalmem(physicalmem_mem_write));

//LC3B datapath
lc3b_datapath lc3b_datapath(
.clk,
.idle_state(idle_state),
.TRUE_BR(TRUE_BR),
.internal_mem_addr(internal_mem_addr),
.internal_mem_response(internal_mem_response),
.internal_mem_read(internal_mem_read),
.internal_mem_write(internal_mem_write),
.internal_mem_rdata(internal_mem_rdata),
.internal_mem_wdata(internal_mem_wdata),
.internal_mem_byte_enable(internal_mem_byte_enable),
.DC_mem_addr(DC_mem_addr),
.DC_mem_response(DC_mem_response),
.DC_mem_read(DC_mem_read),
.DC_mem_write(DC_mem_write),
.DC_mem_rdata(DC_mem_rdata),
.DC_mem_wdata(DC_mem_wdata),
.DC_mem_byte_enable(DC_mem_byte_enable),
.DCache_enable(DCache_enable),
.DCache_hit(DCache_hit));

endmodule
