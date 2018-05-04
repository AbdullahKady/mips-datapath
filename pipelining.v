module MEMWBRegister(
	input clk, WB, 
	input [4:0] INS,
	input [31:0] ALU, DATA_MEM,
	output reg WB_out,
	output reg [4:0] INS_out,
	output reg [31:0] ALU_out, DATA_MEM_out
	);

always @ (posedge clk) begin
 WB_out=WB;

	 INS_out=INS;
	 ALU_out=ALU;
	 DATA_MEM_out=DATA_MEM ;
end

endmodule



module exMem(
	input clk, WB, M,
	input [4:0] mux,
	input [31:0] ALU,ADD, ReadData2,
	output reg WB_out, M_out,
	output reg [4:0] mux_out,
	output reg [31:0] ALU_out, ReadData2_out,ADD_out
	);

always @ (posedge clk) begin
 WB_out=WB;
	 M_out=M;
	 mux_out=mux;
	 ALU_out=ALU;
	 ReadData2_out=ReadData2 ;
	 ADD_out=ADD;
end

endmodule

module idex(
	input clk, WB, M,EX,
	input [4:0] inst_20,inst_15,
	input [31:0] ALU,ADD,ReadData1, ReadData2,SIGNEXT,
	output reg WB_out, M_out,EX_out,
	output reg [4:0] inst_20_out,inst_15_out,
	output reg [31:0] ReadData1_out, ReadData2_out,SIGNEXT_out
	);

always @ (posedge clk) begin
 WB_out=WB;
 M_out=M;
 EX_out=EX;
	 inst_20_out=inst_20;inst_15_out=inst_15;
	 
	 ReadData1_out=ReadData1 ;
	 ReadData2_out=ReadData2 ;
	 SIGNEXT_out=SIGNEXT;
	
end

endmodule

module ifid(
	input clk,
	input [31:0] INST, ADD,
	output reg [31:0] INST_out, ADD_out
	);

always @ (posedge clk) begin
 INST_out=INST_out;
 ADD_out=ADD;
	
end


endmodule





