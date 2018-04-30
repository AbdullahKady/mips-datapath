module Register_File (readAddress_1, readAddress_2, writeAddress, outputData_1, outputData_2, writeInputData, regWrite, clk);

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

module	test_Register();
  reg	clk;
  reg	[4:0]	read_reg_1,	read_reg_2,	write_reg;
  reg	[31:0]	write_data;
  reg	regWrite;
  wire	[31:0]	read_data_1,	read_data_2;


  Register_File regFile(
    read_reg_1,
    read_reg_2,
    write_reg,
    read_data_1,
    read_data_2,
    write_data,
    regWrite,
    clk
  );

  initial begin 
    clk = 0;
    forever begin
    #5 clk = ~clk;
    end
  end


	initial
	begin
		#5	read_reg_1	<=	2'b0;
		$monitor("%t %d	%b",	$time,	read_data_1,	clk);
		#5	write_reg	<=	2'b00;			 regWrite	<=	1;	write_data<=32'd55;
		#5	regWrite	=	1;
		#5	read_reg_1	<=	2'b01;
		#5	write_reg	<=	2'b01;			 regWrite	<=	1;	write_data<=32'd55;		
		#5	$finish;
	end

endmodule