module SignExtender16to32(inputData, outputData);
  
  input[15:0] inputData;
  output[31:0] outputData;
  reg [31:0] outputData;
  
  always@(inputData)
    begin
      
      outputData[15:0]  = inputData[15:0];
      outputData[31:16] = {16{inputData[15]}};
      
    end
endmodule

/*module	signextest();
				reg	[15:0]	in;
				
				
			
				wire [31:0]out;
				initial	begin
							in=26;
								
								#10	$display	("%d",	out);
							
								
								
									//$display	("%d",	OUT2);	
				end
				always@(in);
				SignExtender16to32	ctest(in,out);
endmodule*/
