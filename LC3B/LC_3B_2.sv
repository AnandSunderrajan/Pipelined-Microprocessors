package LC_3B_2;
typedef logic [127:0] cache_flow;
typedef logic [8:0] tag_size;
typedef logic [15:0] LC_3B_word;
typedef logic  [7:0] LC_3B_byte;
typedef logic  [10:0] LC_3B_offset11;
typedef logic  [8:0] LC_3B_offset9;
typedef logic  [5:0] LC_3B_offset6;
typedef logic  [4:0] LC_3B_imm5;
typedef logic  [3:0] LC_3B_imm4;
typedef logic  [1:0] LC_3B_mem_wmask;
typedef logic [2:0] LC_3B_c_index;
typedef logic  [2:0] LC_3B_reg;
typedef logic  [2:0] LC_3B_nzp;


typedef enum bit [3:0]{
op_br   = 4'b0000,
op_add  = 4'b0001,
op_ldb  = 4'b0010,
op_stb  = 4'b0011,
op_jsr  = 4'b0100,
op_and  = 4'b0101,
op_ldr  = 4'b0110,
op_str  = 4'b0111,
op_rti  = 4'b1000,
op_not  = 4'b1001,
op_ldi  = 4'b1010,
op_sti  = 4'b1011,
op_jmp  = 4'b1100,
op_shf  = 4'b1101,
op_lea  = 4'b1110,
op_trap = 4'b1111} LC_3B_OPCode;

typedef enum bit [3:0]{
Alu_add,
Alu_and,
Alu_not,
Alu_passa,
Alu_passb,
Alu_shiftleft,
Alu_shiftrile,
Alu_signedshiftright} LC_3B_OPAlu;


typedef struct packed{
LC_3B_word PC;
LC_3B_OPCode OPCode;
LC_3B_OPAlu OPAlu;
logic OPTRAP, OPBR, OPJSR, OPJMP, OPUNCO, OPLDB, OPSTB, OP_LDI, OP_STI, LDC_CC, registerload, REQSR1, REQSR2, LeftSHIFT, dr_REQ;
logic DCacheR, write_DCache, DCache_enable, DC_mem_byte_select, Alu_result_mux_select, addr1mux_select, addr3mux_select;
logic Destination_mux_select, sr2mux_select, storemux_select;
logic [1:0] addr2mux_select;
logic [1:0] wbmux_select;}LC_3B_control_word;

endpackage
