module CPU(clk, outputTEST);
  output wire[31:0] outputTEST;
  assign outputTEST = PC;
  input clk;
  reg [31:0] PC; 

	wire [31:0] newPC;
  wire [31:0] PC_PLUS_4; 
  wire [31:0] next_PC;
  wire [31:0] add_PC_addr;
  wire [31:0] currentInstruction;
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
		mem[0] = 8'b00000010;
		mem[1] = 8'b01010000;
		mem[2] = 8'b10001000;
		mem[3] = 8'b00100000;
		mem[4] = 8'b00000010;
		mem[5] = 8'b01010000;
		mem[6] = 8'b10001000;
		mem[7] = 8'b00100000;
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
		Regfile[0] = 32'b0;
		Regfile[1] = 32'b0;  
		Regfile[2] = 65;
		Regfile[3] = 100;
		Regfile[4] = 150;
		for (k=5; k<32; k=k+1)
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

	//TESTING ONLY
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
	//TESTING ONLY

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
  wire[31:0] OUTPUT; 
  CPU cpu(clk,OUTPUT);
  
  initial	begin
    cpu.PC = 0;
		clk	=	0;
		forever	begin
			#10	clk	=	~clk;
		end
	end

  always@(posedge clk) begin
		$display("%t => Current PC %d",	$time,	OUTPUT);
	end
endmodule