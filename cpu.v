module CPU(clk, outputTEST_PC, outputTEST_ALU, outputTEST_REG_READ1, outputTEST_REG_READ2, TERMINATED);
  output wire[31:0] outputTEST_PC;  
	output wire[31:0] outputTEST_ALU;
  output wire[31:0] outputTEST_REG_READ1;
  output wire[31:0] outputTEST_REG_READ2;		
  output wire[2:0] outputTEST_SEL;					
	output reg TERMINATED;
	
  assign outputTEST_PC = PC;		
  assign outputTEST_ALU = ALU_result;
  assign outputTEST_REG_READ1 = registerFileReadData_1;
  assign outputTEST_REG_READ2 = registerFileReadData_2;



	///////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////
  input clk;
  reg [31:0] PC; 

	//PROGRAM COUNTER
	wire [31:0] newPC;
  wire [31:0] PC_PLUS_4; 
  wire [31:0] PC_ADDER;
  wire [31:0] currentInstruction;
	wire branchMUX;
	//PROGRAM COUNTER

  //CONTROL SIGNALS
  wire regDestFLAG;
  wire branchFLAG;
  wire[1:0] memReadFLAG;
  wire memToRegFLAG;
  wire[2:0] ALU_SELECTION;
  wire memWriteFLAG;
  wire aluSrcFLAG;
  wire regWriteFLAG;
  //CONTROL SIGNALS
  
  //REGISTER FILE
  wire[4:0] IDEXMEM_REGWRITE;
  wire[31:0] registerFileReadData_1;
  wire[31:0] registerFileReadData_2;  
  wire[31:0] registerFileWriteData;  
  //REGISTER FILE

  //ALU
  wire[31:0] ALU_result;
  wire zeroFLAG;
  wire[31:0] aluInputData_2;
	wire[31:0] immediateValueExtended;
  //ALU

  //DATA MEMORY
  wire[31:0] dataMemoryOut;
  //DATA MEMORY

  always @(posedge clk) begin
    PC <= newPC;
		//MAYBE LATER ON ADD INSTRUCTION FOR TERMINATION
		if(PC > 100)
			TERMINATED <= 1;
  end
  //Sign extended immediate value (the address).
  assign immediateValueExtended = {{16{INS_OUT_IFID[15]}}, INS_OUT_IFID[15:0]};
  
  assign PC_PLUS_4 = PC + 4;
  assign PC_ADDER = IDEX_IFID_PC_OUT + (IDEX_SIGN_EXTEND_OUT<<2);

  assign branchMUX = EXEM_branchFLAG_OUT & EXEM_zeroFLAG_OUT;

  //PC select mux 
  assign newPC = (branchMUX) ? EXEM_PC_ADDR_OUT : PC_PLUS_4;
  //Write register MUX
  assign IDEXMEM_REGWRITE = (IDEX_regDestFLAG_OUT) ? IDEX_INS_15_11_OUT : IDEX_INS_20_16_OUT;
  //ALU 2nd input MUX
  assign aluInputData_2 = (IDEX_aluSrcFLAG_OUT) ? IDEX_SIGN_EXTEND_OUT : IDEX_REG_READ_2_OUT;
  //Write register file MUX
  assign registerFileWriteData = (MEMWB_memToRegFLAG_OUT) ? MEMWB_dataMemoryOut : MEMWB_ALU_RESULT_OUT;

  InstructionMemory instMemory(
    PC,
    currentInstruction
  );

  ControlUnit ctrlUnit(
    INS_OUT_IFID[31:26],
    INS_OUT_IFID[5:0],
    regDestFLAG,
    branchFLAG,
    memReadFLAG,
    memToRegFLAG,
    ALU_SELECTION,
    memWriteFLAG,
    aluSrcFLAG,
    regWriteFLAG
  );

	//TODO MEMWB
  RegisterFile regFile(
    INS_OUT_IFID[25:21],
    INS_OUT_IFID[20:16],    
    MEMWB_writeREGaddress_OUT,
    registerFileReadData_1,
    registerFileReadData_2,
    registerFileWriteData,
    MEMWB_REGWRITE_FLAG,
    clk
  );

  ALU alu(
    ALU_result,
    zeroFLAG,
    IDEX_REG_READ_1_OUT,
    aluInputData_2,
    IDEX_ALU_SELECTION_OUT,
		IDEX_INS_10_6_OUT_SHAMT
  );

  DataMemory dataMemory(
    EXEM_ALU_RESULT_OUT,
    EXEM_READ_DATA_2_OUT,
    EXEM_memWriteFLAG_OUT,
    EXEM_memReadFLAG_OUT,
    dataMemoryOut,
    clk
  );

	wire [31:0] INS_OUT_IFID, PC_OUT_IFID;
	IFID PIPE_IFID(
		INS_OUT_IFID,
		PC_OUT_IFID,
		currentInstruction,
		PC_PLUS_4,
		clk
	);

	wire IDEX_regDestFLAG_OUT,IDEX_branchFLAG_OUT,IDEX_memToRegFLAG_OUT,IDEX_memWriteFLAG_OUT,IDEX_aluSrcFLAG_OUT,IDEX_regWriteFLAG_OUT;
	wire [31:0] IDEX_IFID_PC_OUT,IDEX_REG_READ_1_OUT,IDEX_REG_READ_2_OUT,IDEX_SIGN_EXTEND_OUT;
	wire [4:0] IDEX_INS_20_16_OUT,IDEX_INS_15_11_OUT, IDEX_INS_10_6_OUT_SHAMT;
	wire [2:0] IDEX_ALU_SELECTION_OUT;
	wire [1:0] IDEX_memReadFLAG_OUT;
  
	IDEX PIPE_IDEX(
		regDestFLAG,
    branchFLAG,
    memReadFLAG,
    memToRegFLAG,
    ALU_SELECTION,
    memWriteFLAG,
    aluSrcFLAG,
    regWriteFLAG,
		PC_OUT_IFID,
    registerFileReadData_1,
    registerFileReadData_2,
    immediateValueExtended,
    INS_OUT_IFID[20-16],
    INS_OUT_IFID[15-11],
    INS_OUT_IFID[10-6],		
    
    IDEX_regDestFLAG_OUT,
    IDEX_branchFLAG_OUT,
    IDEX_memReadFLAG_OUT,
    IDEX_memToRegFLAG_OUT,
    IDEX_ALU_SELECTION_OUT,
    IDEX_memWriteFLAG_OUT,
    IDEX_aluSrcFLAG_OUT,
    IDEX_regWriteFLAG_OUT,
    IDEX_IFID_PC_OUT,
    IDEX_REG_READ_1_OUT,
    IDEX_REG_READ_2_OUT,
    IDEX_SIGN_EXTEND_OUT,
    IDEX_INS_20_16_OUT,
    IDEX_INS_15_11_OUT,
    IDEX_INS_10_6_OUT_SHAMT,		
    clk
	);


	wire EXEM_regWriteFLAG_OUT, EXEM_memToRegFLAG_OUT,EXEM_branchFLAG_OUT,EXEM_memWriteFLAG_OUT, EXEM_zeroFLAG_OUT;
	wire [1:0] EXEM_memReadFLAG_OUT;
	wire [31:0] EXEM_PC_ADDR_OUT, EXEM_ALU_RESULT_OUT,EXEM_READ_DATA_2_OUT;
	EXMEM PIPE_EXMEM(
		IDEX_regWriteFLAG_OUT,
  	IDEX_memToRegFLAG_OUT,
  	IDEX_branchFLAG_OUT,
  	IDEX_memReadFLAG_OUT,
  	IDEX_memWriteFLAG_OUT,
  	PC_ADDER,
  	zeroFLAG,
  	ALU_result,
  	IDEX_REG_READ_2_OUT,
  	IDEXMEM_REGWRITE,

  	EXEM_regWriteFLAG_OUT,
  	EXEM_memToRegFLAG_OUT,
  	EXEM_branchFLAG_OUT,
  	EXEM_memReadFLAG_OUT,
  	EXEM_memWriteFLAG_OUT,
  	EXEM_PC_ADDR_OUT,
  	EXEM_zeroFLAG_OUT,
  	EXEM_ALU_RESULT_OUT,
  	EXEM_READ_DATA_2_OUT,
  	EXEM_writeREGaddress_OUT,
  	clk
	);

	wire MEMWB_REGWRITE_FLAG,MEMWB_memToRegFLAG_OUT;
	wire [31:0] MEMWB_dataMemoryOut,MEMWB_ALU_RESULT_OUT;
	wire [4:0] MEMWB_writeREGaddress_OUT;
	MEMWB PIPE_MEMWB(
		EXEM_regWriteFLAG_OUT,
  	EXEM_memToRegFLAG_OUT,
  	dataMemoryOut,
  	EXEM_ALU_RESULT_OUT,
  	EXEM_writeREGaddress_OUT,
	
  	MEMWB_REGWRITE_FLAG,
  	MEMWB_memToRegFLAG_OUT,
  	MEMWB_dataMemoryOut,
  	MEMWB_ALU_RESULT_OUT,
  	MEMWB_writeREGaddress_OUT,
  	clk
	);


endmodule


////////////////////////////////////////////////////////////////////////

module MEMWB(
  regWriteFLAG_IN,
  memToRegFLAG_IN,
  dataMemoryOut_IN,
  ALU_RESULT_IN,
  writeREGaddress_IN,

  regWriteFLAG_OUT,
  memToRegFLAG_OUT,
  dataMemoryOut_OUT,
  ALU_RESULT_OUT,
  writeREGaddress_OUT,
  clk
);

  input regWriteFLAG_IN,memToRegFLAG_IN;
  input [31:0]dataMemoryOut_IN,ALU_RESULT_IN;
  input [4:0] writeREGaddress_IN;

  output reg  regWriteFLAG_OUT,memToRegFLAG_OUT;
  output reg [31:0]dataMemoryOut_OUT,ALU_RESULT_OUT;
  output reg [4:0] writeREGaddress_OUT;

  input clk;


  always @(posedge clk) begin
    regWriteFLAG_OUT <= regWriteFLAG_IN;
    memToRegFLAG_OUT <= memToRegFLAG_IN;
    dataMemoryOut_OUT <= dataMemoryOut_IN;
    ALU_RESULT_OUT <= ALU_RESULT_IN;
    writeREGaddress_OUT <= writeREGaddress_IN;
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
    INS_10_6_IN,
		
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
    INS_10_6_OUT,		
    clk
);

	output reg regDestFLAG_OUT,branchFLAG_OUT,memToRegFLAG_OUT,memWriteFLAG_OUT,aluSrcFLAG_OUT,regWriteFLAG_OUT;
	output reg [31:0] IFID_PC_OUT,REG_READ_1_OUT,REG_READ_2_OUT,SIGN_EXTEND_OUT;
	output reg [4:0] INS_20_16_OUT,INS_15_11_OUT, INS_10_6_OUT;
	output reg [2:0] ALU_SELECTION_OUT;
	output reg [1:0] memReadFLAG_OUT;

	input regDestFLAG_IN,branchFLAG_IN,memToRegFLAG_IN,memWriteFLAG_IN,aluSrcFLAG_IN,regWriteFLAG_IN;
	input [31:0] IFID_PC_IN,REG_READ_1_IN,REG_READ_2_IN,SIGN_EXTEND_IN;
	input [4:0] INS_20_16_IN,INS_15_11_IN, INS_10_6_IN;
	input [2:0] ALU_SELECTION_IN;
	input [1:0] memReadFLAG_IN;
	input clk;



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
   INS_10_6_OUT <= INS_10_6_IN ; 
 
end

endmodule


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


module InstructionMemory(
		readAddress,
		instruction
	);
	input [31:0] readAddress;
	output [31:0] instruction;
	reg [7:0] mem [511:0];
	
	//INITIAL PROGRAM
	initial 
	begin
		// PROGRAM TRYING TO MANIPULATE THE ZERO REGISTER
		// {mem[0],mem[1],mem[2],mem[3]} = 32'b00100000000000000000000001100100; //ADDI $zero $zero 100
		// {mem[4],mem[5],mem[6],mem[7]} = 32'b00100010001100010000001100110011; //ADDI s1 s1 819
		// {mem[8],mem[9],mem[10],mem[11]} = 32'b00000000000100011000100000100000; //ADD s1 zero  s1 
		// PROGRAM TRYING TO MANIPULATE THE ZERO REGISTER


		//PROGRAM UTILIZING IMMEDIATE VALUES, DATA MEMORY, AND TYPICAL ADD/SUB
		// {mem[0],mem[1],mem[2],mem[3]} = 32'b00100010000100000000010100000000; //addi s0,s0,1280
		// {mem[4],mem[5],mem[6],mem[7]} = 32'b00100010001100010000001000100010; //addi s1,s1,546
		// {mem[8],mem[9],mem[10],mem[11]} = 32'b00000010000100011011100000100000; //add s7, s0,s1
		// {mem[12],mem[13],mem[14],mem[15]} = 32'b10101101001101110000000000000000; //sw $s7, 0x0($t1)
		// {mem[16],mem[17],mem[18],mem[19]} = 32'b10001101001101010000000000000000; //lw $s5, 0x0($t1)
		// {mem[20],mem[21],mem[22],mem[23]} = 32'b00000010101100001010100000100010; //sub s5,s5,s0
		//PROGRAM UTILIZING IMMEDIATE VALUES, DATA MEMORY, AND TYPICAL ADD/SUB
		

		//BOILERPLATE PROGRAM TO SAVE YOU THE TIME OF WRITING MEMORY ADDRESSES :D
		{mem[0],mem[1],mem[2],mem[3]} = 32'b00100010000100000001000100010001; //addi s0 s0 x1111 
		{mem[4],mem[5],mem[6],mem[7]} = 32'b00000000000100001000000010000010; //srl s0,s0,0x2 
		{mem[8],mem[9],mem[10],mem[11]} = 32'b10101101000100000000000000000000; //sw s0, t0
		// {mem[12],mem[13],mem[14],mem[15]} = 32'b10001101001101010000000000000000; 
		// {mem[16],mem[17],mem[18],mem[19]} = 32'b00000010000101011001000000100010; 
		// {mem[20],mem[21],mem[22],mem[23]} = 32'b00100010001100010000001000000000;
		// {mem[24],mem[25],mem[26],mem[27]} = 32'b00100010001100010000001000000000;
		// {mem[28],mem[29],mem[30],mem[31]} = 32'b00100010001100010000001000000000;
		// {mem[32],mem[33],mem[34],mem[35]} = 32'b00100010001100010000001000000000;
		// {mem[36],mem[37],mem[38],mem[39]} = 32'b00100010001100010000001000000000;
		// {mem[40],mem[41],mem[42],mem[43]} = 32'b00100010001100010000001000000000;
		// {mem[44],mem[45],mem[46],mem[47]} = 32'b00100010001100010000001000000000;
		// {mem[48],mem[49],mem[50],mem[51]} = 32'b00100010001100010000001000000000;
		// {mem[52],mem[53],mem[54],mem[55]} = 32'b00100010001100010000001000000000;
		// {mem[56],mem[57],mem[58],mem[59]} = 32'b00100010001100010000001000000000;
		// {mem[60],mem[61],mem[62],mem[63]} = 32'b00100010001100010000001000000000;

	end
	//INITIAL PROGRAM
	assign instruction = {
		mem[readAddress],
		mem[readAddress+1],
		mem[readAddress+2],
		mem[readAddress+3]
	};
endmodule


module ControlUnit(OPCODE, FUNC,regdist,branch,memread,memtoreg,ALU_SELECTION,memwrite,alusrc,regwrite);
	input	[5:0]	OPCODE, FUNC;
	output reg	[2:0]	ALU_SELECTION;	
	output reg regdist,branch,memtoreg,memwrite,alusrc,regwrite; 
	output reg[1:0] memread;
				
	always	@	(OPCODE) begin
		case	(OPCODE)
			6'b000000 : begin //R TYPE
				regdist= 1;
				branch = 0;
				memread= 0;
				memtoreg = 0;
				memwrite = 0;
				alusrc 	 = 0;
				regwrite = 1;
				case (FUNC)
					6'b100000: ALU_SELECTION = 0; 
					6'b100010: ALU_SELECTION = 1; 
					6'b000000: ALU_SELECTION = 2;
					6'b000010: ALU_SELECTION = 3;
					6'b100100: ALU_SELECTION = 4;
					6'b100101: ALU_SELECTION = 5;
					6'b101010: ALU_SELECTION = 6;
					6'b101011: ALU_SELECTION = 7;
				endcase
			end			
			6'b100011: begin //LW
				regdist = 0;
				branch = 0;
				memread = 1;
				memtoreg = 1;
				memwrite = 0;
				alusrc = 1;
				regwrite = 1;
				ALU_SELECTION  = 0;
			end
			6'b101011: begin //SW
				regdist = 1'bx;
				branch = 0;
				memread = 0;
				memtoreg = 1'bx;
				memwrite = 1;
				alusrc = 1;
				regwrite = 0;
				ALU_SELECTION  = 0;
			end
			6'b000100: begin	//BEQ
				ALU_SELECTION  = 1'bx;
				regdist = 1'bx;
				branch = 1;
				memread = 0;
				memtoreg = 1'bx;
				memwrite = 0;
				alusrc = 0;
				regwrite = 0;
			end
			6'b100001: begin	//LH
				regdist = 0;
				branch = 0;
				memread = 2;
				memtoreg = 1;
				memwrite = 0;
				alusrc = 1;
				regwrite = 1;
				ALU_SELECTION  = 0;
			end
			6'b100101: begin	//LHU
				regdist = 0;
				branch = 0;
				memread = 3;
				memtoreg = 1;
				memwrite = 0;
				alusrc = 1;
				regwrite = 1;
				ALU_SELECTION  = 0;
			end

			
			//Immediate Function
			default: begin
				regdist     = 1'b0;
				alusrc      = 1'b1;
				memtoreg    = 1'b0;
				regwrite    = 1'b1;
				memwrite    = 1'b0;
				memread     = 1'b0;
				case(OPCODE)
					6'b001000: ALU_SELECTION = 0; //ADDI
					6'b001100: ALU_SELECTION = 4; //ANDI
					6'b001101: ALU_SELECTION = 5; //ORI
				endcase
			end
		endcase
	end
endmodule

module RegisterFile (readAddress_1, readAddress_2, writeAddress, outputData_1, outputData_2, writeInputData, regWrite, clk);

	input [4:0] readAddress_1, readAddress_2, writeAddress;
	input [31:0] writeInputData;
	input clk, reset, regWrite;
	output [31:0] outputData_1, outputData_2;

	reg [31:0] Regfile [31:0];
	integer k; //Just a counter for intitializing all registers
	
	assign outputData_1 = Regfile[readAddress_1];
	assign outputData_2 = Regfile[readAddress_2];


  initial
  begin
		for (k=0; k<32; k=k+1)
		begin
			Regfile[k] = 32'b0;
		end
	end 


	always @(posedge clk)
	begin
	  if (regWrite)
    begin
      if(writeAddress !== 0) Regfile[writeAddress] = writeInputData; 
    end
	end

endmodule

module	ALU	(OUT,	ZeroFlag,	input1_unsigned,	input2_unsigned,ALU_SELECTION, SHIFT_AMOUNT);
				
	input	[31:0]	input1_unsigned,	input2_unsigned;
	input	[2:0]	ALU_SELECTION;
	input [4:0] SHIFT_AMOUNT;
	output	reg	[31:0]	OUT;
	output	ZeroFlag;
	integer input1_signed,input2_signed;
				

	// BEQ 
	assign ZeroFlag = (input1_unsigned == input2_unsigned);

	// ALL OTHER CASES
	always	@	(*)
	begin
		//SIGNED VALUES 
		input1_signed = input1_unsigned;
		input2_signed = input2_unsigned;

		case	(ALU_SELECTION)
			0	:	OUT	=	input1_signed	+	input2_signed; //ADD
			1	:	OUT	=	input1_signed	- input2_signed; //SUB  
			2	:	OUT	=	input2_unsigned	<< SHIFT_AMOUNT; //SLL 
			3	:	OUT	=	input2_unsigned >> SHIFT_AMOUNT; //SRL 
			4	:	OUT	=	input1_unsigned	&	input2_unsigned; //AND
			5	:	OUT	=	input1_unsigned	|	input2_unsigned; //OR
			6 : OUT = input1_signed < input2_signed;  //Compare signed
			7 : OUT = input1_unsigned < input2_unsigned; //Compare unsigned
		endcase
	end
endmodule

module DataMemory(
		address,
		writeData,
		memWrite,
		memRead,
		dataOut,
		clk
	);
	input [31:0] address, writeData;
	input memWrite,clk;
	input [1:0] memRead;
	output reg [31:0] dataOut;
	reg [7:0] mem [1023:0];

	//INITIALIZE ANY DATA HERE. The data segment in assembler
	// initial 
	// begin 
	// 	mem[0] = 8'b00100010;
	// 	mem[1] = 8'b01110010;
	// 	mem[2] = 8'b11110000;
	// 	mem[3] = 8'b00000100;
	// 	mem[4] = 8'b11111010;
	// 	mem[5] = 8'b00100010;
	// 	mem[6] = 8'b00111010;
	// 	mem[7] = 8'b11011010;
	// end

	always @ (posedge clk)
	begin
		if (memWrite)
		begin
			mem[address] = writeData[31:24];
			mem[address+1] = writeData[23:16];
			mem[address+2] = writeData[15:8];
			mem[address+3] = writeData[7:0];
		end
	end
	
	always @(memRead, address)begin
    if(memRead !== 2'b00)begin
			case(memRead)
				//Load word
				1	:	dataOut = {mem[address],mem[address+1],mem[address+2],mem[address+3]};
				//Load half word
				2	:	dataOut = {
						{16{mem[address][7]}},
						{mem[address],mem[address+1]}
					};
				//Load half word unsigned
				3	:	dataOut = {
					{16'b0000000000000000},
					{mem[address],mem[address+1]}
				};
			endcase
		end
  end
	
endmodule

module testBench();
  reg clk;
  wire[31:0] OUTPUT_PC;
	wire[31:0] OUTPUT_ALU;
	wire[31:0] OUTPUT_REG1;
	wire[31:0] OUTPUT_REG2; 
	wire TERMINATED;
  CPU cpu(clk,OUTPUT_PC, OUTPUT_ALU,OUTPUT_REG1,OUTPUT_REG2, TERMINATED);
  integer k;
  initial	begin
    cpu.PC = 0;
    cpu.TERMINATED = 0;		
		clk	=	0;
		forever	begin
			#10	clk	=	~clk;
		end
	end

  always@(posedge clk) begin
		if(TERMINATED) begin
			$display("========================================================= SYSTEM TERMINATED =========================================================");	
			$display("========================================================= REGISTER FILE DUMP =========================================================");				
			for (k=0; k<32; k=k+1)
			begin
				$display("\t\t\t\t\t\t\t\t\t\t\t\tRegister #%d => %d",k,cpu.regFile.Regfile[k]);	
			end
			$display("========================================================= DATA MEMORY DUMP =========================================================");
			for (k=0; k<200; k=k+1)
			begin
				$display("\t\t\t\t\t\t\t\t\t\t\t\tMemory Cell #%d => %d",k,cpu.dataMemory.mem[k]);	
			end				
			$finish;
		end
		$display("%t => Current PC %d || Current ALU result %d || Read register1: %d || Read register2: %d",	$time,	OUTPUT_PC, OUTPUT_ALU, OUTPUT_REG1, OUTPUT_REG2);
	end
endmodule