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
	reg [7:0] mem [511:0];

	//TESTING ONLY
	initial 
	begin 
		mem[0] = 8'b00100010;
		mem[1] = 8'b01110010;
		mem[2] = 8'b11110000;
		mem[3] = 8'b00000100;
		mem[4] = 8'b11111010;
		mem[5] = 8'b00100010;
		mem[6] = 8'b00111010;
		mem[7] = 8'b11011010;
	end
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
	
	always @(memRead, address)
    begin
        if(memRead !== 2'b00)
		begin
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


module	datamemtest();
	reg [31:0] address, writeData;
	reg memWrite, clk;
	reg [1:0] memRead;
	wire [31:0] dataOut;
	DataMemory dmtest(
		address,
		writeData,
		memWrite,
		memRead,
		dataOut,
		clk
	);

	initial	begin
		clk	=	0;
		forever	begin
			#5	clk	=	~clk;
		end
	end

	initial begin
		memRead	<=	2'b00; memWrite	<=	1'b0; address <= 0;
		$monitor("%t => %b",	$time,	dataOut);
		#5 memRead <= 1;
		#5 memRead <= 2;
		#5 memRead <= 3;	
		#5 memRead <= 0;
		#5 $finish;
	end

endmodule