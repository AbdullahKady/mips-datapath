module IDEX(
    regDestFLAG_IN,
    branchFLAG_IN,
    memReadFLAG_IN,
    memToRegFLAG_IN,
    ALU_SELECTION_IN,
    memWriteFLAG_IN,
    aluSrcFLAG_IN,
    regWriteFLAG_IN,
    IFID_PC_IN,
    REG_READ_1_IN,
    REG_READ_2_IN,
    SIGN_EXTEND_IN,
    INS_20_16_IN,
    INS_15_11_IN,
    
    regDestFLAG_OUT,
    branchFLAG_OUT,
    memReadFLAG_OUT,
    memToRegFLAG_OUT,
    ALU_SELECTION_OUT,
    memWriteFLAG_OUT,
    aluSrcFLAG_OUT,
    regWriteFLAG_OUT,
    IFID_PC_OUT,
    REG_READ_1_OUT,
    REG_READ_2_OUT,
    SIGN_EXTEND_OUT,
    INS_20_16_OUT,
    INS_15_11_OUT,
    clk
);

always @(posedge clk) begin
  
   regDestFLAG_OUT <= regDestFLAG_IN ;
   branchFLAG_OUT <= branchFLAG_IN ;
   memReadFLAG_OUT <= memReadFLAG_IN ;
   memToRegFLAG_OUT <= memToRegFLAG_IN ;
   ALU_SELECTION_OUT <= ALU_SELECTION_IN ;
   memWriteFLAG_OUT <= memWriteFLAG_IN ;
   aluSrcFLAG_OUT <= aluSrcFLAG_IN ;
   regWriteFLAG_OUT <= regWriteFLAG_IN ;
   IFID_PC_OUT <= IFID_PC_IN ;
   REG_READ_1_OUT <= REG_READ_1_IN ;
   REG_READ_2_OUT <= REG_READ_2_IN ;
   SIGN_EXTEND_OUT <= SIGN_EXTEND_IN ;
   INS_20_16_OUT <= INS_20_16_IN ;
   INS_15_11_OUT <= INS_15_11_IN ;

end

endmodule


