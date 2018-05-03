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
