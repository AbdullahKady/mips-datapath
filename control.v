module control(instruction,regdist,jump,branch,memread,memtoreg,aluop,memwrite,alusrc,regwrite);
input	[5:0]	instruction;
				output	reg	[1:0]	aluop;	
				output	reg regdist,jump,branch,memread,memtoreg,memwrite,alusrc,regwrite; 
				
				always	@	(instruction)
				begin
				if(instruction==1'h8)//addi
regdist=0;jump=0;branch=0;memread=0;memtoreg=0;memwrite=0;alusrc=1;regwrite=1;
					if(instruction==2'h23 )//lw
		regdist=0;		jump=0;branch=0;memread=1;memtoreg=1;memwrite=0;	alusrc=1;regwrite=1;
						if(instruction==2'h2B )//lw
				jump=0;branch=0;memread=0;memwrite=1;alusrc=1;regwrite=1;
					
				end
endmodule

module	controltest();
				reg	[5:0]	instruction;
				
				
				wire	regdist,jump,branch,memread,memtoreg,memwrite,alusrc,regwrite;
				
				wire [1:0]aluop;
				initial	begin
							instruction=2'h23;
								
								#10	$display	("%d",	regdist);
							#10	$display	("%d",	regwrite);
							#10	$display	("%d",	alusrc);
							#10	$display	("%d",branch);	
								
									//$display	("%d",	OUT2);	
				end
				always@(instruction);
				control	contest(instruction,regdist,jump,branch,memread,memtoreg,aluop,memwrite,alusrc,regwrite);
endmodule
