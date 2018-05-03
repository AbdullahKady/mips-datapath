module control(OPCODE, FUNC,regdist,branch,memread,memtoreg,ALU_SELECTION,memwrite,alusrc,regwrite);
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

module	controltest();
				reg	[5:0]	OPCODE;
				
				
				wire	regdist,jump,branch,memread,memtoreg,memwrite,alusrc,regwrite;
				
				wire [1:0]ALU_SELECTION;
				initial	begin
							OPCODE=2'h23;
								
							#10	$display	("%d",	regdist);
							#10	$display	("%d",	regwrite);
							#10	$display	("%d",	alusrc);
							#10	$display	("%d",branch);	
								
									//$display	("%d",	OUT2);	
				end
				always@(OPCODE);
				control	contest(OPCODE,regdist,jump,branch,memread,memtoreg,ALU_SELECTION,memwrite,alusrc,regwrite);
endmodule
