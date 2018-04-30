module	ADD1	(OUT,In1,In2);
				
				input	[31:0]	In1;
				input	[31:0]	In2;
				output	reg	[31:0]	OUT;	
				
				always	@	(In1,In2)
				
				begin
			OUT=In1+In2;
				end
endmodule

module	add();
				reg	[31:0]	a;
				reg	[31:0]	b;
				wire	[31:0]	OUT;
				initial	begin
								a	=	1;
				b=3;
								
								#10	$display	("%d",	OUT);
							
								
								#10	a	=	10;b=6;
								#10	$display	("%d",	OUT);	
				end
				always@(a);
				ADD1	addTest(OUT,a,b);
endmodule