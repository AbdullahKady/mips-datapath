module CPU(clk, outputTEST_PC, outputTEST_ALU, outputTEST_REG_READ1, outputTEST_REG_READ2);
  output wire[31:0] outputTEST_PC;  
	output wire[31:0] outputTEST_ALU;
  output wire[31:0] outputTEST_REG_READ1;
  output wire[31:0] outputTEST_REG_READ2;			
		
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
  wire[4:0] writeRegisterAddress;
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
  end
  //Sign extended immediate value (the address).
  assign immediateValueExtended = {{16{currentInstruction[15]}}, currentInstruction[15:0]};
  
  assign PC_PLUS_4 = PC + 4;
  assign PC_ADDER = PC_PLUS_4 + (immediateValueExtended<<2);

  assign branchMUX = branchFLAG & zeroFLAG;

  //PC select mux 
  assign newPC = (branchMUX) ? PC_ADDER : PC_PLUS_4;
  //Write register MUX
  assign writeRegisterAddress = (regDestFLAG) ? currentInstruction[15:11] : currentInstruction[20:16];
  //ALU 2nd input MUX
  assign aluInputData_2 = (aluSrcFLAG) ? immediateValueExtended : registerFileReadData_2;
  //Write register file MUX
  assign registerFileWriteData = (memToRegFLAG) ? dataMemoryOut : ALU_result;

  InstructionMemory instMemory(
    PC,
    currentInstruction
  );

  ControlUnit ctrlUnit(
    currentInstruction[31:26],
    currentInstruction[5:0],
    regDestFLAG,
    branchFLAG,
    memReadFLAG,
    memToRegFLAG,
    ALU_SELECTION,
    memWriteFLAG,
    aluSrcFLAG,
    regWriteFLAG
  );

  RegisterFile regFile(
    currentInstruction[25:21],
    currentInstruction[20:16],    
    writeRegisterAddress,
    registerFileReadData_1,
    registerFileReadData_2,
    registerFileWriteData,
    regWriteFLAG,
    clk
  );

  ALU alu(
    ALU_result,
    zeroFLAG,
    registerFileReadData_1,
    aluInputData_2,
    ALU_SELECTION,
    currentInstruction[10:6]
  );

  DataMemory dataMemory(
    ALU_result,
    registerFileReadData_2,
    memWriteFLAG,
    memReadFLAG,
    dataMemoryOut,
    clk
  );

  
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
		// {mem[0],mem[1],mem[2],mem[3]} = 32'b00100010000100000000000111110100; 
		// {mem[4],mem[5],mem[6],mem[7]} = 32'b10101101001100000000000000000000; 
		// {mem[8],mem[9],mem[10],mem[11]} = 32'b00100010010100100000000011001000; 
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
			2	:	OUT	=	input1_unsigned	<< SHIFT_AMOUNT; //SLL 
			3	:	OUT	=	input1_unsigned >> SHIFT_AMOUNT; //SRL 
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

	//INITIALIZE ANY DATA HERE
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
						{16{mem[address][0]}},
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
  CPU cpu(clk,OUTPUT_PC, OUTPUT_ALU,OUTPUT_REG1,OUTPUT_REG2);
  
  initial	begin
    cpu.PC = 0;
		clk	=	0;
		forever	begin
			#10	clk	=	~clk;
		end
	end

  always@(posedge clk) begin
		$display("%t => Current PC %d || Current ALU result %d || Read register1: %d || Read register2: %d",	$time,	OUTPUT_PC, OUTPUT_ALU, OUTPUT_REG1, OUTPUT_REG2);
	end
endmodule