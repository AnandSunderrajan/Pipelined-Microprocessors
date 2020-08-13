module testbench;

timeunit 1ns;
timeprecision 1ns;

logic clk;
logic physicalmem_mem_response, physicalmem_mem_read, physicalmem_mem_write;
logic [127:0] physicalmem_mem_rdata, physicalmem_mem_wdata;
logic [15:0] physicalmem_mem_addr;
logic [1:0] physicalmem_byte_enable;

initial clk = 0;
always #5 clk = ~clk;

projtoplevel lcthreeeeeeee(
.clk,
.physicalmem_mem_response(physicalmem_mem_response),
.physicalmem_mem_rdata(physicalmem_mem_rdata),
.physicalmem_mem_read(physicalmem_mem_read),
.physicalmem_mem_write(physicalmem_mem_write),
.physicalmem_byte_enable(physicalmem_byte_enable),
.physicalmem_mem_wdata(physicalmem_mem_wdata),
.physicalmem_mem_addr(physicalmem_mem_addr));


endmodule
