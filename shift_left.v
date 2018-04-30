module leftshift(in,out);
  
  input [31:0]in;
  output reg [31:0]out;
  
  
  always@(in)
    begin
      
      out=in<<2;
  
    end
    
endmodule 
