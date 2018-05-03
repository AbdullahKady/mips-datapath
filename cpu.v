module CPU(clk)
  reg [31:0] PC; 
  initial begin
    PC = 0;
  end
  wire [31:0] PC_PLUS_4; 
  wire [31:0] next_PC;
  wire [31:0] add_PC_addr;
  wire [31:0] currentInstruction;
  //CONTROL SIGNALS
  wire regDestFLAG;
  wire branchFLAG;
  wire memReadFLAG;
  wire memToRegFLAG;
  wire ALU_SELECTION;
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

  always @(posedge clk) begin
    PC <= newPC;
  end
  //Sign extended immediate value (the address).
  assign immediateValueExtended = {{16{currentInstruction[15]}}, currentInstruction[15:0]};
  
  assign PC_PLUS_4 = PC + 4;
  assign PC_ADDER = PC_PLUS_4 + (immediateValueExtended<<2);

  assign branchMUX = branchFLAG & ZF;

  //PC select mux 
  assign newPC = (branchMUX) ? PC_ADDER : PC_PLUS_4;
  // Write register MUX
  assign writeRegisterAddress = (regDestFLAG) ? currentInstruction[15:11] : currentInstruction[20:16];



  IntstructionMemory instMemory(
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

  

endmodule




module IntstructionMemory(
		readAddress,
		instruction
	);
	input [31:0] readAddress;
	output [31:0] instruction;
	reg [7:0] mem [511:0];
	
	//INITIAL PROGRAM
	initial 
	begin
		mem[0] = 8'b00100010;
		mem[1] = 8'b01110010;
		mem[2] = 8'b11110000;
		mem[3] = 8'b00000100;
		mem[4] = 8'b11111010;
		mem[5] = 8'b10100010;
		mem[6] = 8'b00111011;
		mem[7] = 8'b11011010;
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

// module testBench()
  // reg clk;
  // initial	begin
		// clk	=	0;
		// forever	begin
			// #10	clk	=	~clk;
		// end
	// end
// 
  // CPU cpu(clk);
// endmodule