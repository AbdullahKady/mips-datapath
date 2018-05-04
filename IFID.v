module IFID (
  Instruction_OUT,
  PC_OUT,
  Instruction_IN,
  PC_IN,
  clk
);

  output reg [31:0] Instruction_OUT,PC_OUT;
  input [31:0] Instruction_IN,PC_IN;
  input clk;
  always @(posedge clk) begin
    Instruction_OUT <= Instruction_IN;
    PC_OUT <= PC_IN;
  end
endmodule

module EXMEM (
  regWriteFLAG_IN,
  memToRegFLAG_IN,
  branchFLAG_IN,
  memReadFLAG_IN,
  memWriteFLAG_IN,
  PC_ADDR_IN,
  zeroFLAG_IN,
  ALU_RESULT_IN,
  READ_DATA_2_IN,
  writeREGaddress_IN,

  regWriteFLAG_OUT,
  memToRegFLAG_OUT,
  branchFLAG_OUT,
  memReadFLAG_OUT,
  memWriteFLAG_OUT,
  PC_ADDR_OUT,
  zeroFLAG_OUT,
  ALU_RESULT_OUT,
  READ_DATA_2_OUT,
  writeREGaddress_OUT,
  clk
);

  input [31:0] PC_ADDR_IN,ALU_RESULT_IN,READ_DATA_2_IN;
  input regWriteFLAG_IN,memToRegFLAG_IN,branchFLAG_IN,memWriteFLAG_IN,zeroFLAG_IN;
  input [1:0] memReadFLAG_IN;
  input [4:0] writeREGaddress_IN;
  input clk;

  output [31:0] PC_ADDR_OUT,ALU_RESULT_OUT,READ_DATA_2_OUT;
  output regWriteFLAG_OUT,memToRegFLAG_OUT,branchFLAG_OUT,memWriteFLAG_OUT,zeroFLAG_OUT;
  output [1:0] memReadFLAG_OUT;
  output [4:0] writeREGaddress_OUT;


  always @(posedge clk) begin
    regWriteFLAG_OUT <= regWriteFLAG_IN;
    memToRegFLAG_OUT <= memToRegFLAG_IN;
    branchFLAG_OUT <= branchFLAG_IN;
    memReadFLAG_OUT <= memReadFLAG_IN;
    memWriteFLAG_OUT <= memWriteFLAG_IN;
    PC_ADDR_OUT <= PC_ADDR_IN;
    zeroFLAG_OUT <= zeroFLAG_IN;
    ALU_RESULT_OUT <= ALU_RESULT_IN;
    READ_DATA_2_OUT <= READ_DATA_2_IN;
    writeREGaddress_OUT <= writeREGaddress_IN;
  end
endmodule


module MEMWB(
  regWriteFLAG_IN,
  memToRegFLAG_IN,
);