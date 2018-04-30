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


module	instructionTest();
	reg [31:0] readAddress;
	wire [31:0] instruction;
	IntstructionMemory dmtest(
		readAddress,
		instruction
	);
	
	initial begin
		readAddress <= 0;
		$monitor("%t => %b",	$time,	instruction);
		#5 readAddress <= 3;
		#5 readAddress <= 1;
		#5 readAddress <= 2;
		#5 $finish;
	end

endmodule