module	ALU	(OUT,	ZeroFlag,	In1,	In2,	ALUOP);
				
				input	[31:0]	In1,	In2;
				input	[2:0]	ALUOP;
				output	reg	[31:0]	OUT;	
				output	reg	ZeroFlag;
				
				always	@	(In1,	In2,	ALUOP)
				begin
					if	(In1	==	In2)
								ZeroFlag	=	1;
				else
								ZeroFlag	=	0;
				end
				always	@	(In1,	In2,	ALUOP)
				begin
					case	(ALUOP)
						0	:	OUT	=	In1	+	In2;
						1	:	OUT	=	In1	- In2;
						2	:	OUT	=	In1	&	In2;
						3	:	OUT	=	In1	&	In2;
						4	:	OUT	=	In1	|	In2;
						5	:	OUT	=	In1	<	In2;
					endcase
				end
endmodule
/*module	ALU_Test();
				reg	[31:0]	a,	b;
				reg	[2:0]	select;
				wire	[31:0]	OUT;
				wire	zeroflag;
				initial	begin
								a	=	1;
								b	=	2;
								select	=	0;
								#10	$display	("%d",	OUT);	$display	("%d",	zeroflag);
								#10	a	=	2;	b	=	2;
								#10	$display	("%d",	OUT);	$display	("%d",	zeroflag);
								#10	a	=	10;	b	=	20;
								#10	$display	("%d",	OUT);	$display	("%d",	zeroflag);
				end
				always@(a,	b,	select);
				ALU	aluTest(OUT,	zeroflag,	a,	b,	select);
endmodule*/
