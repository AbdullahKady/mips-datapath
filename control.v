module control(OPCODE,regdist,jump,branch,memread,memtoreg,aluop,memwrite,alusrc,regwrite);
	input	[5:0]	OPCODE;
	output	reg	[1:0]	aluop;	
	output	reg regdist,branch,memread,memtoreg,memwrite,alusrc,regwrite; 
				
	always	@	(OPCODE) begin
		case	(OPCODE)
			6'b000000 : begin //R TYPE
				aluop  = 2'b10; //

				regdist = 1;
				branch = 0;
				memread = 0;
				memtoreg = 0;
				memwrite = 0;
				alusrc = 0;
				regwrite = 1;
			end
			/*6'b001000: begin //ADDI
				regdist = 0;
				branch = 0;
				memread = 0;
				memtoreg = 0;
				memwrite = 0;
				alusrc = 1;
				regwrite = 1;
			end*/
			6'b100011: begin //LW
				regdist = 0;
				branch = 0;
				memread = 1;
				memtoreg = 1;
				memwrite = 0;
				alusrc = 1;
				regwrite = 1;
				
				aluop  = 2'b00;
			end
			6'b101011: begin //SW
				regdist = 1'bx;
				branch = 0;
				memread = 0;
				memtoreg = 1'bx;
				memwrite = 1;
				alusrc = 1;
				regwrite = 0;
				
				aluop  = 2'b00;

			end
			//BEQ
			6'b000100: begin	// beq
				aluop  = 2'b01;
				
				regdist = 1'bx;
				branch = 1;//
				memread = 0;//
				memtoreg = 1'bx;//
				memwrite = 0;//
				alusrc = 0;//
				regwrite = 0;//
			end
			
			//Immediate Function
			default: begin
				regdist      = 1'b0;
				alusrc      = 1'b1;
				memtoreg    = 1'b0;
				regwrite    = 1'b1;
				memread     = 1'b0;
				memwrite    = 1'b0;
				/*case (opcode) still needd to know more about aluop
					ADDI:   ALUOp = ADD;
					ANDI:   ALUOp = AND;
					ORI:    ALUOp = OR;
					XORI:   ALUOp = XOR;
					SLTI:   ALUOp = SLT;
					SLTIU:  ALUOp = SLTU;
				endcase*/
			end

			

		
			
			
		endcase

	end
endmodule

module	controltest();
				reg	[5:0]	OPCODE;
				
				
				wire	regdist,jump,branch,memread,memtoreg,memwrite,alusrc,regwrite;
				
				wire [1:0]aluop;
				initial	begin
							OPCODE=2'h23;
								
								#10	$display	("%d",	regdist);
							#10	$display	("%d",	regwrite);
							#10	$display	("%d",	alusrc);
							#10	$display	("%d",branch);	
								
									//$display	("%d",	OUT2);	
				end
				always@(OPCODE);
				control	contest(OPCODE,regdist,jump,branch,memread,memtoreg,aluop,memwrite,alusrc,regwrite);
endmodule
