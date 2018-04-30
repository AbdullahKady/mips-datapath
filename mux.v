module	MUX	(OUT,In1,In2,op);
				
				input	[31:0]	In1,In2;
				input [1:0] op;
				output	reg	[31:0]	OUT;	
				
				always	@	(In1,In2,op)
				
				begin
				case	(op)
						0	:	OUT	=	In1;
						1	:	OUT	=	In2;
						
					endcase
			
				end
endmodule

/*module	mux();
				reg	[31:0]	a,b;
				reg[1:0] op;
				wire	[31:0]	OUT;
				initial	begin
								a	=	1;
				b=3;op=0;
								
								#10	$display	("%d",	OUT);
							
								
								#10	op=1;
								#10	$display	("%d",	OUT);	
				end
				always@(a,b,op);
				MUX	muxtest(OUT,a,b,op);
endmodule
*/
