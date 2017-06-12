//--------------------------------------------------------------------
//					ALU  	(Not Present)
//--------------------------------------------------------------------
module arm_alu (output reg [31:0] result, output reg N_flag, Z_flag, C_flag, V_flag, input [3:0] op_code, input [31:0] dataA, dataB, input carry);
reg [31:0] temp;
always @ (*)
case (op_code)
		4'b0000: //[AND] Logical AND
		begin
		result = dataA & dataB;
		C_flag = carry;
		update_N_Z_flags();
		end
		4'b0001: //[EOR] Logical Exclusive OR This is a Logical Exclusive OR!!
		begin
		result = (dataA & ~dataB) | (~dataA & dataB);
		C_flag = carry;
		update_N_Z_flags();
		end
		4'b0010: //[SUB] Subtract
		begin
		{C_flag, result} = dataA - dataB;
		if((dataA[31] == 0 && dataB[31] == 1 && result[31] == 1) || (dataA[31] == 1 && dataB[31] == 0 && result[31] == 0))
		V_flag = 1;
		else
		V_flag = 0;
		update_N_Z_flags();
		end
		4'b0011: //[RSB] Reverse Subtract
		begin
		{C_flag, result} = dataB - dataA;
		if((dataB[31] == 0 && dataA[31] == 1 && result[31] == 1) || (dataB[31] == 1 && dataA[31] == 0 && result[31] == 0))
		V_flag = 1;
		else
		V_flag = 0;
		update_N_Z_flags();
		end
		4'b0100: //[ADD] Add
		begin
		{C_flag, result} = dataA + dataB;
		if((dataA[31] == 0 && dataB[31] == 0 && result[31] == 1) || (dataA[31] == 1 && dataB[31] == 1 && result[31] == 0))
		V_flag = 1;
		else
		V_flag = 0;
		update_N_Z_flags();
		end
		4'b0101: //[ADC] Add with Carry
		begin
		{C_flag, result} = dataA + dataB + carry;
		if((dataA[31] == 0 && dataB[31] == 0 && result[31] == 1) || (dataA[31] == 1 && dataB[31] == 1 && result[31] == 0))
		V_flag = 1;
		else
		V_flag = 0;
		update_N_Z_flags();
		end
		4'b0110: //[SBC] Subtract with Carry
		begin
		{C_flag, result} = dataA - dataB + carry;
		if((dataA[31] == 0 && dataB[31] == 1 && result[31] == 1) || (dataA[31] == 1 && dataB[31] == 0 && result[31] == 0))
		V_flag = 1;
		else
		V_flag = 0;
		update_N_Z_flags();
		end
		4'b0111: //[RSC] Reverse Subtract with Carry
		begin
		{C_flag, result} = dataB - dataA + carry;
		if((dataB[31] == 0 && dataA[31] == 1 && result[31] == 1) || (dataB[31] == 1 && dataA[31] == 0 && result[31] == 0))
		V_flag = 1;
		else
		V_flag = 0;
		update_N_Z_flags();
		end
		4'b1000: //[TST] Test
		begin
		temp = dataA & dataB;
		C_flag = carry;
		N_flag = temp[31];
		if(temp == 0) 
		Z_flag = 1;
		else 
		Z_flag = 0;
		end
		4'b1001: //[TEQ] Test Equivalence
		begin
		temp = (dataA & ~dataB) | (~dataA & dataB);
		C_flag = carry;
		N_flag = temp[31];
		if(temp == 0) 
		Z_flag = 1;
		else 
		Z_flag = 0;
		end
		4'b1010: //[CMP] Compare
		begin
		{C_flag, temp} = dataA - dataB;
		N_flag = temp[31];
		if(temp == 0) 
		Z_flag = 1;
		else 
		Z_flag = 0;
		end
		4'b1011: //[CMN] Compare Negated
		begin
		{C_flag, temp} = dataA + dataB;
		N_flag = temp[31];
		if(temp == 0) 
		Z_flag = 1;
		else 
		Z_flag = 0;
		end
		4'b1100: //[ORR] Logical OR
		begin
		result = dataA | dataB;
		C_flag = carry;
		update_N_Z_flags();
		end
		4'b1101: //[MOV] Move
		begin
		result = dataB;
		C_flag = carry;
		update_N_Z_flags();
		end
		4'b1110: //[BIC] Bit Clear
		begin
		result = dataA & ~dataB;
		C_flag = carry;
		update_N_Z_flags();
		end
		4'b1111: //[MVN] Move Not
		begin	
		result = ~dataB;
		C_flag = carry;
		update_N_Z_flags();
		end
		/*5'b10000: //Add 4 PC
			begin
				result = dataB + 4;
				end*/
				endcase

	task update_N_Z_flags; //Update the N & Z flags
	begin
	N_flag = result[31];
	if(result == 0) 
	Z_flag = 1;
	else 
	Z_flag = 0;
	end
	endtask

	endmodule


//--------------------------------------------------------------------
//					Branch Extender
//--------------------------------------------------------------------
module branch_extender(output reg [31:0] final_offset, input [23:0] offset);

reg [25:0] offset1;

always @ (*)

begin

offset1=offset;

offset1=offset1<<2;

final_offset= $signed(offset1);

end

endmodule




//--------------------------------------------------------------------
//					Control Unit (Possibly Hardwired)
//--------------------------------------------------------------------
module control_unit(output reg [44:0] cu_out, input clk, reset, mfc, input [31:0] cu_in, input [3:0] flags);

//Wires connected to other components

wire im_out, Mh_out, enc_en, le, inv_out;
wire [1:0] next_state_out;
wire [5:0] enc_out, inc_out, Mj_out, Mk_out, incR_out;
wire [54:0] rom_out;
reg [31:0] tempIR;
parameter ONE = 6'b000001;
parameter ZERO = 6'b000000;


wire [54:0] tempPipe;
reg [5:0] nxSt;

//Components Created

control_register pipe_reg (tempPipe, rom_out, clk);

input_manager input_man (im_out, flags, cu_in [31:28]);

mux_2to1_1bit MuxH (Mh_out, tempPipe[15], im_out, mfc);

mux_4to1_6bits MuxJ (Mj_out, next_state_out, enc_out, ONE, tempPipe[8:3], incR_out);

mux_2to1_6bits MuxK (Mk_out, reset, ZERO, Mj_out);

encoder enc (enc_out, tempIR, clk);

microstore rom (rom_out, Mk_out);

incrementer inc (inc_out, Mk_out);

incrementer_register incReg (incR_out, inc_out, clk);

next_state_select next_state (next_state_out [0], next_state_out [1], inv_out, tempPipe[2:0]);

Inverter inverter (inv_out, Mh_out, tempPipe[54]);



always @ (*)
begin	
//cu_out <= {tempPipe[53:9]};
cu_out <= {tempPipe[46:19], tempPipe[54:50], tempPipe[18:9], tempPipe[49:48]};
tempIR <= cu_in;	
end

endmodule

//--------------------------------------------------------------------
//					DATA PATH
//--------------------------------------------------------------------
module DataPath(output reg mfc, output reg [31:0] IR_Out, StaReg_Out, input [3:0] cu_opcode, write_cu, readA_cu, readB_cu, input [1:0] Ma, dataType, imme_SEL, Mg,
	input RFen, Mb, Mc, Md, Me, Mf, Mo, Ms, Ml, MAREn, MDREn, IREn, SHFen, SignExtEn, SignExtEn2, r_w, MemEN, dwp, mfa, clk, clr, StatusRegEn_cu);

//Wires connected to other components

wire [31:0] RFOut_A, RFOut_B, BranchExt_Out, SignExt_Out, Ma_Out, Mb_Out, Mc_Out, Md_Out, Me_Out, 
SignExt_Out2, ALU_Out, Shf_Out, MDR_Out, MAR_Out, MemOut, Flags;

wire [3:0] write, readA, readB, opcode_in, Ml_out;
reg carry;
wire StatusRegEn;

//assign write = wireIR[15:12];
assign write = Ml_out;

//Temp variables

wire [31:0] temp4, tempStaReg;
wire [31:0] wireIR;
wire tempMfc;


assign temp4 = 32'h00000004;

always @ (*)
begin
IR_Out = wireIR;
StaReg_Out = tempStaReg;
mfc = tempMfc;
carry = StaReg.Y[2];
end

//Components created

register_file regf(RFOut_A, RFOut_B, ALU_Out, readA, readB, write, RFen, clr, clk);

mux_4to1 MuxA (Ma_Out, Ma, RFOut_B, MDR_Out, Mb_Out, temp4);

mux_2to1 MuxB (Mb_Out, Mb, BranchExt_Out, SignExt_Out);

mux_2to1 MuxC (Mc_Out, Mc, ALU_Out, MemOut);

mux_2to1 MuxD (Md_Out, Md, Me_Out, RFOut_A);

mux_2to1 MuxE (Me_Out, Me, wireIR, wireIR);

mux_2to1_4bits MuxF (readA, Mf, readA_cu, wireIR[19:16]);

mux_4to1_4bits MuxG (readB, Mg, readB_cu, wireIR[3:0], wireIR[15:12], wireIR[19:16]);

mux_2to1_4bits MuxL (Ml_out, Ml, write_cu, wireIR[15:12]);

mux_2to1_4bits MuxO (opcode_in, Mo, cu_opcode, wireIR[24:21]);

mux_2to1_1bit MuxS (StatusRegEn, Ms, StatusRegEn_cu, ~wireIR[20]);

arm_alu alu (ALU_Out, Flags [0], Flags [1], Flags[2], Flags [3], opcode_in, RFOut_A, Shf_Out, carry);

branch_extender branchExt (BranchExt_Out, wireIR[23:0]);

Imme_Sign_Extension signExt (SignExt_Out, wireIR, SignExtEn, imme_SEL);

Sign_Extension2 signExt2 (SignExt_Out2, Mc_Out, dataType, SignExtEn2);

Shifter shift (Shf_Out, Flags [2], Ma_Out, wireIR[6:5], Md_Out, SHFen);

register_32_bits InsReg (wireIR, MemOut, IREn, clr, clk);

register_32_bits StaReg (tempStaReg, Flags, StatusRegEn, clr, clk);

register_32_bits Mar (MAR_Out, ALU_Out, MAREn, clr, clk);

register_32_bits Mdr (MDR_Out, SignExt_Out2, MDREn, clr, clk);

Ram ram (MDR_Out, MemEN, r_w, mfa, dataType, dwp, MAR_Out [7:0], tempMfc, MemOut);

endmodule


//--------------------------------------------------------------------
//					Decoder 16 to 1
//--------------------------------------------------------------------
module decoder (output reg [15:0] out, input [3:0] in, input enable);

always @ (in, enable)

begin

if(in == 4'b0000) begin
	out = 16'b0000000000000001;
end 

else if(in == 4'b0010) begin
	out = 16'b0000000000000100;
end 

else if(in == 4'b0011) begin
	out = 16'b0000000000001000;;
end 

else if(in == 4'b0100) begin
	out = 16'b0000000000010000;
end 

else if(in == 4'b0101) begin
	out = 16'b0000000000100000;
end 

else if(in == 4'b0110) begin
	out = 16'b0000000001000000;;
end 

else if(in == 4'b0111) begin
	out = 16'b0000000010000000;
end 

else if(in == 4'b1000) begin
	out = 16'b0000000100000000;
end 

else if(in == 4'b1001) begin
	out = 16'b0000001000000000;
end 

else if(in == 4'b1010) begin
	out = 16'b0000010000000000;
end 

else if(in == 4'b1011) begin
	out = 16'b0000100000000000;
end 

else if(in == 4'b1100) begin
	out = 16'b0001000000000000;
end 

else if(in == 4'b1101) begin
	out = 16'b0010000000000000;
end 

else if(in == 4'b1110) begin
	out = 16'b0100000000000000;
end 

else if(in == 4'b1111) begin
	out = 16'b1000000000000000;
end 


if(!enable)

begin

out = 16'h0000;

end

end

endmodule


//--------------------------------------------------------------------
//					Encoder
//--------------------------------------------------------------------
module encoder (output reg [5:0] enc_out, input [31:0] enc_in, input clk);

reg [31:0] tempEnc_in;

initial begin

assign tempEnc_in = enc_in;

end

always @ (enc_in)
begin

	//5 = Register Shifter Operand
	if (tempEnc_in[27:25] == 3'b000 && tempEnc_in[4] == 1'b0 && tempEnc_in[11:7] == 5'b00000) begin
		enc_out = 6'b000101; 
	end

	//7 = Shift by Immediate Shifter Operand
	else if (tempEnc_in[27:25] == 3'b000 && tempEnc_in[4] == 1'b0 && tempEnc_in[11:7] != 5'b00000) begin 
		enc_out = 6'b000111; 
	end

	//41 = Immediate Post-Indexed Load MISCELLANEOUS
	if (tempEnc_in[27:25] == 3'b000 && tempEnc_in[4] == 1'b1 && tempEnc_in[24] == 1'b0 && tempEnc_in[20] == 1'b1) begin
  		enc_out = 6'b101001; 
  	end

	//37 = Immediate Post-Indexed Store MISCELLANEOUS
  	if (tempEnc_in[27:25] == 3'b000 && tempEnc_in[4] == 1'b1 && tempEnc_in[24] == 1'b0 && tempEnc_in[20] != 1'b1) begin
		enc_out = 6'b100101; 
	end

	//39 = Immediate Offset Load MISCELLANEOUS
	if (tempEnc_in[27:25] == 3'b000 && tempEnc_in[4] == 1'b1 && tempEnc_in[24] != 1'b0 && tempEnc_in[21] == 1'b0 && tempEnc_in[20] == 1'b1 ) begin
		enc_out = 6'b100111; 
	end

	//35 = Immediate Offset Store MISCELLANEOUS
	if (tempEnc_in[27:25] == 3'b000 && tempEnc_in[4] == 1'b1 && tempEnc_in[24] != 1'b0 && tempEnc_in[21] == 1'b0 && tempEnc_in[20] != 1'b1 ) begin
		enc_out = 6'b100011; 
	end


	//40 = Immediate Pre-Indexed Load MISCELLANEOUS
	if (tempEnc_in[27:25] == 3'b000 && tempEnc_in[4] == 1'b1 && tempEnc_in[24] != 1'b0 && tempEnc_in[21] == 1'b1 && tempEnc_in[20] == 1'b1 ) begin
		enc_out = 6'b101000; 
	end

	//36 = Immediate Pre-Indexed Store MISCELLANEOUS
	if (tempEnc_in[27:25] == 3'b000 && tempEnc_in[4] == 1'b1 && tempEnc_in[24] != 1'b0 && tempEnc_in[21] == 1'b1 && tempEnc_in[20] != 1'b1 ) begin
		enc_out = 6'b100100; 
	end

	//6 = 32-bit Immediate Shifter Operand
	if (tempEnc_in[27:25] == 3'b001 ) begin
		enc_out = 6'b000110; 
	end

	//24 = Immediate Post-Indexed Load
	if (tempEnc_in[27:25] == 3'b010 && tempEnc_in[24] == 1'b0 && tempEnc_in[20] == 1'b1 ) begin
		enc_out = 6'b011000; 
	end

	//16 = Immediate Post-Indexed Store
	if (tempEnc_in[27:25] == 3'b010 && tempEnc_in[24] == 1'b0 && tempEnc_in[20] != 1'b1 ) begin
		enc_out = 6'b010000; 
	end

	//20 = Immediate Offset Load
	if (tempEnc_in[27:25] == 3'b010 && tempEnc_in[24] != 1'b0 && tempEnc_in[21] == 1'b0  && tempEnc_in[20] == 1'b1) begin
		enc_out = 6'b010100; 
	end

	//20 = Immediate Offset Load
	if (tempEnc_in[27:25] == 3'b010 && tempEnc_in[24] != 1'b0 && tempEnc_in[21] == 1'b0  && tempEnc_in[20] != 1'b1) begin
		enc_out = 6'b010100; 
	end

	//22 = Immediate Pre-Indexed Load
	if (tempEnc_in[27:25] == 3'b010 && tempEnc_in[24] != 1'b0 && tempEnc_in[21] == 1'b1  && tempEnc_in[20] == 1'b1) begin
		enc_out = 6'b010110; 
	end

	//14 = Immediate Pre-Indexed Store
	if (tempEnc_in[27:25] == 3'b010 && tempEnc_in[24] != 1'b0 && tempEnc_in[21] == 1'b1  && tempEnc_in[20] != 1'b1) begin
		enc_out = 6'b001110; 
	end

	//26 = Register Post-Indexed Load
	if (tempEnc_in[27:25] == 3'b011 && tempEnc_in[24] == 1'b0 &&  tempEnc_in[20] == 1'b1) begin
		enc_out = 6'b011010; 
	end

	//18 = Register Post-Indexed Store
	if (tempEnc_in[27:25] == 3'b011 && tempEnc_in[24] == 1'b0 &&  tempEnc_in[20] != 1'b1) begin
		enc_out = 6'b010010; 
	end

	//21 = Register Offset Load
	if (tempEnc_in[27:25] == 3'b011 && tempEnc_in[24] != 1'b0 &&  tempEnc_in[21] == 1'b0 && tempEnc_in[20] == 1'b1) begin
		enc_out = 6'b010101; 
	end

	//13 = Register Offset Store
	if (tempEnc_in[27:25] == 3'b011 && tempEnc_in[24] != 1'b0 &&  tempEnc_in[21] == 1'b0 && tempEnc_in[20] != 1'b1) begin
		enc_out = 6'b001101; 
	end

	//23 = Register Pre-Indexed Load
	if (tempEnc_in[27:25] == 3'b011 && tempEnc_in[24] != 1'b0 &&  tempEnc_in[21] == 1'b1 && tempEnc_in[20] == 1'b1) begin	
		enc_out = 6'b010111; 
	end

	//15 = Register Pre-Indexed Store
	if (tempEnc_in[27:25] == 3'b011 && tempEnc_in[24] != 1'b0 &&  tempEnc_in[21] == 1'b1 && tempEnc_in[20] != 1'b1) begin
		enc_out = 6'b001111; 
	end

	//43 = Branch
	if (tempEnc_in[27:25] == 3'b101 && tempEnc_in[24] == 1'b0) begin
		enc_out = 6'b101011; 
	end

	//44 = Branch with Link
	if (tempEnc_in[27:25] == 3'b101 && tempEnc_in[24] != 1'b0) begin
		enc_out	= 6'b101100; 
	end

	if (tempEnc_in == 31'h00000000) begin
		enc_out=6'b000000;
	end

end
endmodule


//--------------------------------------------------------------------
//					Sign Extension
//--------------------------------------------------------------------
module Imme_Sign_Extension (output reg [31:0] Y, input [31:0] In, input signEN, input [1:0] imme_SEL);

always @ (*)

begin
if (!signEN)
begin
case (imme_SEL)
2'b00:
begin
Y[7:0] = In[7:0];
Y[31:8] = {24{In[7]}};
end
2'b01:
begin
Y[11:0] = In[11:0];
Y[31:12] = {20{In[7]}};
end
2'b10:
begin
Y[23:0] = In[23:0];
Y[31:24] = {8{In[7]}};
end
2'b11:
begin
Y[7:0] = {In[11:8], In[3:0]};
Y[31:8] = {24{In[7]}};
end
endcase
end

else
begin
case (imme_SEL)
2'b00:
begin
Y[7:0] = In[7:0];
Y[31:8] = 24'b000000000000000000000000;
end
2'b01:
begin
Y[11:0] = In[11:0];
Y[31:12] = 20'b00000000000000000000;
end
2'b10:
begin
Y[23:0] = In[23:0];
Y[31:24] = 8'b00000000;
end
2'b11:
begin
Y[7:0] = {In[11:8], In[3:0]};
Y[31:8] = 24'b000000000000000000000000;
end
endcase
end
end	

endmodule

//--------------------------------------------------------------------
//					Incrementer
//--------------------------------------------------------------------
module incrementer (output reg [5:0] inc_out, input [5:0] inc_in);


initial inc_out = 6'b000000;

always @ (*)
inc_out = inc_in + 1'b1;

endmodule

//--------------------------------------------------------------------
//					Incrementer Register
//--------------------------------------------------------------------
module incrementer_register (output reg [5:0] out, input [5:0] in, input clk);

always @(posedge clk)	
out <= in;	

endmodule



//-----------------------------------------------------------------
//					INPUT Signals MANAGER
//-----------------------------------------------------------------

module input_manager (output reg out, input [3:0] Flags, cond_Code);

/*
Flags[0] = Negative Flag
Flags[1] = Zero Flag
Flags[2] = Carry Flag
Flags[3] = Overflow Flag
*/
always @ (*)
begin

//EQ - Equal - Flags tested: Z=1
if (cond_Code == 4'b0000) begin
	out = Flags[1];
end

//NE - Not equal - Flags tested: Z=1
else if (cond_Code == 4'b0001) begin
	out = ~Flags[1];
end

//CS/HS - Unsigned higher or same - Flags tested: C=1
else if (cond_Code == 4'b0010) begin
	out = ~Flags[2];
end

//CC/LO - Unsigned lower - Flags tested: C=0
else if (cond_Code == 4'b0011) begin
	out = ~Flags[2];
end

//MI - Minus = Flags tested: N=1
else if (cond_Code == 4'b0100) begin
	out = Flags[0];
end

//PL - Positive or Zero - Flags tested: N=0
else if (cond_Code == 4'b0101) begin
	out = ~Flags[0];
end

//VS - Overflow - Flags tested: V=1
else if (cond_Code == 4'b0110) begin
	out = Flags[3];
end

//VC - No overflow - Flags tested: V=0
else if (cond_Code == 4'b0111) begin
	out = ~Flags[3];
end

//HI - Unsigned higher - C=1 & Z=0
else if (cond_Code == 4'b1000) begin
	out = Flags[2] & ~Flags[1];
end

//LS - Unsigned lower or same - Flags tested: C=0 or Z=1
else if (cond_Code == 4'b1001) begin
	out = ~Flags[2] | Flags[1];
end

//GE - Greater or equal - N=V
else if (cond_Code == 4'b1010) begin
	out = ~(Flags[0] ^ Flags[3]);
end

//LT - Less than - Flags tested: N!=V
else if (cond_Code == 4'b1011) begin
	out = Flags[0] ^ Flags[3];
end

//GT - Greater than - Flags tested: Z=0 & N=V
else if (cond_Code == 4'b1100) begin
	out = ~(Flags[0] ^ Flags[3]) & (~Flags[1]);
end

//LE - Less than or equal - Flags tested: Z=1 or N=!V
else if (cond_Code == 4'b1101) begin
	out = (Flags[0] ^ Flags[3]) | (Flags[1]);
end

//AL - Always
else if (cond_Code == 4'b1110) begin
	out = 1'b1;
end


end 

endmodule



/********************************************************************************/
/*					                 Inverter                                   */
//Input
//in: lo que sale del MUX 2x1
//invtr: inv

//Output
//out: (STS)
/********************************************************************************/

module Inverter (output reg out, input in, invtr);

always @(*)
begin
if(invtr == 1'b0)
out = in;
else
out = ~in;
end

endmodule

//-------------------------------------------------------------
//					Micro Processor ARM
//-------------------------------------------------------------
module microprocessor();

wire mfc; 
wire [31:0] IR_Out, StaReg_Out; 
wire [44:0] cu_out;
reg clk, clr;

//Module Instantiation
DataPath datapath (.mfc(mfc), .IR_Out(IR_Out), .StaReg_Out(StaReg_Out), .cu_opcode(cu_out[30:27]), .write_cu(cu_out[42:39]), .readA_cu(cu_out[38:35]), .readB_cu(cu_out[34:31]), 
	.Ma(cu_out[26:25]), .dataType(cu_out[9:8]), .imme_SEL(cu_out[1:0]), .RFen(cu_out[44]), .Mb(cu_out[20]), .Mc(cu_out[19]), .Md(cu_out[18]), .Me(cu_out[17]), .Mf(cu_out[16]), 
	.Mg(cu_out[15:14]), .Mo(cu_out[12]), .Ms(cu_out[22]), .Ml(cu_out[13]), .MAREn(cu_out[4]), .MDREn(cu_out[3]), .IREn(cu_out[5]), .SHFen(cu_out[24]), .SignExtEn(cu_out[21]), 
	.SignExtEn2(cu_out[2]), .r_w(cu_out[11]), .MemEN(cu_out[10]), .dwp(cu_out[7]), .mfa(cu_out[6]), .clk(clk), .clr(clr), .StatusRegEn_cu(cu_out[23]));

control_unit cu (.cu_out(cu_out), .clk(clk), .reset(clr), .mfc(mfc), .cu_in(IR_Out), .flags(StaReg_Out [3:0]));

//Memory Variables
reg [31:0] dat[0:155];
parameter WORD = 2'b10;
parameter WRITE = 1'b1;
parameter ENABLE = 1'b0;
reg [31:0] temp_data_in;
reg [7:0] temp_addr;

initial #5000 $finish;

initial $readmemb("testcode_arm1.txt", dat);

reg [8:0] i; // loop index

initial begin

//----------------------------------------------------------------------
//					FILL RAM MEMORY
//----------------------------------------------------------------------
for(i=9'h000;i<9'h030;i=i+9'h004)
begin
temp_data_in[31:24] <= dat[i[7:0]]; 
temp_data_in[23:16] <= dat[i[7:0]+1]; 
temp_data_in[15:8] <= dat[i[7:0]+2]; 
		temp_data_in[7:0] <= dat[i[7:0]+3]; //setting up data
		#5;
		datapath.ram.ram[i[7:0]] <= temp_data_in[31:24]; 
		datapath.ram.ram[i[7:0]+1]<= temp_data_in[23:16]; 
		datapath.ram.ram[i[7:0]+2] <= temp_data_in[15:8]; 
		datapath.ram.ram[i[7:0]+3] <= temp_data_in[7:0];
		#5;
		end 
		
		for(i=9'h000;i<9'h030;i=i+9'h004)
		begin
		$write ("WORD DATA at %d: %h", i, datapath.ram.ram[i[7:0]]);
		$write ("%h", datapath.ram.ram[i[7:0]+1]);
		$write ("%h", datapath.ram.ram[i[7:0]+2] ); 
		$display ("%h", datapath.ram.ram[i[7:0]+3]);
		end

		clr = 1'b0;
		#5 clk = 1'b0;
		#5 clk = 1'b1;
		#5 clk = 1'b0;
		#5 clk = 1'b1;
		clr = 1'b1;
		repeat (500) #5 clk = ~clk;

		#10 $display("\nTesting content of Registers:\nR0 = %h\nR1 = %h\nR2 = %h\nR3 = %h\nR4 = %h\nR5 = %h\nR6 = %h\nR7 = %h\nR8 = %h\nR9 = %h\nR10 = %h\nR11 = %h\nR12 = %h\nR13 = %h\nR14 = %h\nR15 = %h\n", datapath.regf.register0.Y, datapath.regf.register1.Y, datapath.regf.register2.Y, datapath.regf.register3.Y, datapath.regf.register4.Y, datapath.regf.register5.Y, datapath.regf.register6.Y, datapath.regf.register7.Y, datapath.regf.register8.Y, datapath.regf.register9.Y, datapath.regf.register10.Y, datapath.regf.register11.Y, datapath.regf.register12.Y, datapath.regf.register13.Y, datapath.regf.register14.Y, datapath.regf.register15.Y);

		for(i=9'h000;i<9'h030;i=i+9'h004)
		begin
		$write ("WORD DATA at %d: %h", i, datapath.ram.ram[i[7:0]]);
		$write ("%h", datapath.ram.ram[i[7:0]+1]);
		$write ("%h", datapath.ram.ram[i[7:0]+2] ); 
		$display ("%h", datapath.ram.ram[i[7:0]+3]);
		end

		end

		initial begin

		/*$monitor("R10: %h\nR5: %h\nR3: %h\nR2: %h\nR1: %h\nR0: %h\nR15: %h\nRead A: %b\nRead B: %b\nR15 Enable: %d\nRF Enable: %d\nIR: %h\nIR Enable: %d\nMDR: %h\nMAR: %h\nMar enable: %b\nRAM out: %h\nMFC: %b\nPipeToNext: %b\nCond S: %b\nInv Out: %b\nFlags: %b\nCond Code: %b\nIM Out: %b\nM0: %b\nM1: %b\nEncode OUT: %d\nData A: %h\nData B: %h\nALU out: %h\nOpcode: %b\nMJ Out: %d\nMK Sel: %b\nMK Out: %d\nBranch In: %h\nBranch Out: %h\nMuxA Sel: %b\nMuxA Out: %h\nMuxB Out: %h\nMuxB Sel: %b\nTime: %d\nNext State: %d\n--------------------------------------", datapath.regf.register10.Y, datapath.regf.register5.Y, datapath.regf.register3.Y, datapath.regf.register2.Y, datapath.regf.register1.Y, datapath.regf.register0.Y, datapath.regf.register15.Y, datapath.regf.readA, datapath.regf.readB, datapath.regf.register15.LE, datapath.regf.enable, datapath.InsReg.Y, datapath.InsReg.LE, datapath.Mdr.Y, datapath.Mar.Y, datapath.Mar.LE, datapath.ram.data_out, datapath.ram.mfc, cu.next_state.pipeline, cu.next_state.cond_S, cu.inverter.out, cu.input_man.Flags, cu.input_man.cond_Code, cu.input_man.out, cu.next_state.M0, cu.next_state.M1, cu.enc.enc_out, datapath.regf.outA, datapath.alu.dataB, datapath.alu.result, datapath.alu.op_code, cu.MuxJ.Y, cu.MuxK.S, cu.MuxK.Y, datapath.branchExt.offset, datapath.branchExt.final_offset, datapath.MuxA.S, datapath.MuxA.Y, datapath.MuxB.Y, datapath.MuxB.S, $time, cu.rom.index);*/

		$monitor("MAR: %0d\n\n--------------\n\nState: %d", datapath.Mar.Y, cu.rom.index);
	//$monitor("MAR: %0d\n Flags (ALU): %b %b %b %b\n Flags (SR): %b\n Flags (CU): %b\n--------------\n", datapath.Mar.Y, datapath.alu.N_flag, datapath.alu.Z_flag, datapath.alu.C_flag, datapath.alu.V_flag, datapath.StaReg.Y[3:0], cu.flags);

	end

	endmodule


//--------------------------------------------------------------------
//					MICROSTORE (ROM)
//--------------------------------------------------------------------

module microstore (output reg[54:0] out, input [5:0] index);

initial begin
	//index =  6'b000001;
	end

	always @(index)
	begin
	case(index)
		6'b000000:  out = 55'b0001000010000000000000110100110101000110001111000001010;
		6'b000001:  out = 55'b1001000011000011111111110100110101000110001011000010011;
		6'b000010:  out = 55'b0000000001111111111111010011110101000010001111000010011;
		6'b000011:  out = 55'b0000000111111111111111110111110101000010010101000011101;
		6'b000100:  out = 55'b0000000011111111111111110111110101000010001111000001000;
		6'b000101:  out = 55'b1011100001000000000000000000101101000110001111000001001;
		6'b000110:  out = 55'b1011100001000000000000000010101101000110001111000001001;
		6'b000111:  out = 55'b1011100001000000000000000010101100010110001111000001001;
		6'b001000:  out = 55'b1011001111000000000000110101110101000000011101001000101;
		6'b001001:  out = 55'b1011001001000000000000110101110100000110001111000001001;
		6'b001010:  out = 55'b1101001011000000000000110100110100000110001101001011011;
		6'b001011:  out = 55'b1101001111000000000000110100110100001000001111001011111;
		6'b001100:  out = 55'b1011001011000000000000010010110010000110001011001010010;
		6'b001101:  out = 55'b1011001011000000000000010000110100000110001011001010010;
		6'b001110:  out = 55'b1011001001000000000000010010110100000110001011001010010;
		6'b001111:  out = 55'b1011001001000000000000010000110100000110001011001010010;
		6'b010000:  out = 55'b1011001011000000000000110100110100000110001011010001011;
		6'b010001:  out = 55'b1011001001000000000000010010110100000110001111001010010;
		6'b010010:  out = 55'b1011001011000000000000110100110100000110001011010011011;
		6'b010011:  out = 55'b1011001001000000000000010000110100000110001111001010010;
		6'b010100:  out = 55'b1011001011000000000000010010110010000110001011001000010;
		6'b010101:  out = 55'b1011001011000000000000010000110100000110001011001000010;
		6'b010110:  out = 55'b1011001001000000000000010010110100000110001011001000010;
		6'b010111:  out = 55'b1011001001000000000000010000110100000110001011001000010;
		6'b011000:  out = 55'b1011001011000000000000110100110100000110001011011001011;
		6'b011001:  out = 55'b1011001001000000000000010010110100000110001111001000010;
		6'b011010:  out = 55'b1011001011000000000000110100110100000110001011011011011;
		6'b011011:  out = 55'b1011001001000000000000010000110100000110001111001000010;
		6'b011100:  out = 55'b1011011111000000000000110100110101000000011100011100101;
		6'b011101:  out = 55'b1011011001000000000000110101110100000110001111000001001;
		6'b011110:  out = 55'b1101001011000000000000110100110100000110001100011111011;
		6'b011111:  out = 55'b1101001111000000000000110100110100001000001111011111111;
		6'b100000:  out = 55'b1011011011000000000000010010110000000110001011011110010;
		6'b100001:  out = 55'b1011011001000000000000010010110100000110001011011110010;
		6'b100010:  out = 55'b1011001011000000000000110100110100000110001011100011011;
		6'b100011:  out = 55'b1011001001000000000000010010110100000110001111011110010;
		6'b100100:  out = 55'b1011011011000000000000010010110000000110001011011100010;
		6'b100101:  out = 55'b1011011001000000000000010010110100000110001011011100010;
		6'b100110:  out = 55'b1011011011000000000000110100110100000110001011100111011;
		6'b100111:  out = 55'b1011011001000000000000010010110100000110001111011100010;
	endcase
	end

	endmodule


	/********************************************************************************/
	/*					                    Mux 1                                   */
//Provee el input del inverter
//Input
//S: (MFA... control register bit 13)
//DO: im_out (output del input manager)
//D1: mfc 


//Output
//Y: (lo que entra al inverter)
/********************************************************************************/
module mux_2to1_1bit (output reg Y, input S,
	input D0, input D1);

always @ (S,D0,D1)

case (S)

1'b0:   Y = D0;
1'b1:   Y = D1;

endcase

endmodule


//Multiplexer 4 bits 2 to 1
module mux_2to1_4bits (output reg [3:0] Y, input S,
	input [3:0] D0, input [3:0] D1);

always @ (S,D0,D1)

case (S)
1'b0:   Y = D0;
1'b1:   Y = D1;
endcase

endmodule


//--------------------------------------------------------------------
//					Multiplexer 4 bits 2 to 1
//--------------------------------------------------------------------
module mux_4to1_4bits (output reg [3:0] Y, input [1:0] S,
	input [3:0] D0, input [3:0] D1, input [3:0] D2, input [3:0] D3);

always @ (S,D0,D1)

case (S)
2'b00:   Y = D0;
2'b01:   Y = D1;
2'b10:	 Y = D2;
2'b11:	 Y = D3;
endcase

endmodule

//Multiplexer 5 bits 4 to 1

module mux_4to1_5bits (output reg [4:0] Y, input [1:0] S,
	input [4:0] D0, input [4:0] D1, input [4:0] D2, input [4:0] D3);

always @ (S,D0,D1,D2,D3)

case (S)

2'b00:   Y = D0;
2'b01:   Y = D1;
2'b10:   Y = D2;
2'b11:   Y = D3;

endcase

endmodule

//--------------------------------------------------------------------
//					Multiplexer 5 bits 4 to 1
//--------------------------------------------------------------------
module mux_2to1_5bits (output reg [4:0] Y, input S,
	input [4:0] D0, input [4:0] D1);

always @ (S,D0,D1)

case (S)

1'b0:   Y = D0;
1'b1:   Y = D1;

endcase

endmodule


/********************************************************************************/
/*										Mux 2                                   */
//Input
//S: (2 bits M0 M1, output next state selector)
//DO: (output encoder)
//D1: (NextSt1) (output control register [8:3])
//D2: 1 (hardcoded #1)
//D3: (output incrementer)

//Output


/********************************************************************************/
module mux_4to1_6bits (output reg [5:0] Y, input [1:0] S,
	input [5:0] D0, input [5:0] D1, input [5:0] D2, input [5:0] D3);

always @ (S,D0,D1,D2,D3)

case (S)

2'b00:   Y = D0;
2'b01:   Y = D1;
2'b10:   Y = D2;
2'b11:   Y = D3;

endcase

endmodule

/********************************************************************************/
/*					                   Mux 3                                    */
//Input
//S: RESET
//D0: (hardcoded ZERO)
//D1: (output MUX 2)

//Output
//Y: Entra al incrementer y al microstore
/********************************************************************************/
module mux_2to1_6bits (output reg [5:0] Y, input S,
	input [5:0] D0, input [5:0] D1);

always @ (S,D0,D1)

case (S)

1'b0:   Y = D0;
1'b1:   Y = D1;

endcase

endmodule

//--------------------------------------------------------------------
//					Multiplexer 32 bits 16 to 1
//--------------------------------------------------------------------

module mux_16to1 (output reg [31:0] Y, input [3:0] S, input [31:0] D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12, D13, D14, D15);

always @ (S,D0,D1,D2,D3,D4,D5,D6,D7,D8,D9,D10,D11,D12,D13,D14,D15)

case (S)

4'b0000: Y = D0;
4'b0001: Y = D1;
4'b0010: Y = D2;
4'b0011: Y = D3;
4'b0100: Y = D4;
4'b0101: Y = D5;
4'b0110: Y = D6;
4'b0111: Y = D7;
4'b1000: Y = D8;
4'b1001: Y = D9;
4'b1010: Y = D10;
4'b1011: Y = D11;
4'b1100: Y = D12;
4'b1101: Y = D13;
4'b1110: Y = D14;
4'b1111: Y = D15;

endcase

endmodule


//--------------------------------------------------------------------
//					Multiplexer 32 bits 4 to 1
//--------------------------------------------------------------------


module mux_4to1 (output reg [31:0] Y, input [1:0] S,
	input [31:0] D0, input [31:0] D1, input [31:0] D2, input [31:0] D3);

always @ (S,D0,D1,D2,D3)

case (S)

2'b00:   Y = D0;
2'b01:   Y = D1;
2'b10:   Y = D2;
2'b11:   Y = D3;

endcase

endmodule

//Multiplexer 32 bits 2 to 1

module mux_2to1 (output reg [31:0] Y, input S,
	input [31:0] D0, input [31:0] D1);

always @ (S,D0,D1)

case (S)

1'b0:   Y = D0;
1'b1:   Y = D1;

endcase

endmodule

/********************************************************************************/
/*                                   Next State                                 */

//Input
//cond_S: Output of previous inverter (STS)
//pipeline: N0 - N2 (NxStaSel) 

//Output
//M0 (M0)
//M1 (M0)
/********************************************************************************/
module next_state_select (output reg M0, output reg M1, input cond_S, input [2:0] pipeline);

initial begin
M0 = 1'b1;
M1 = 1'b0;
end


always @ (*)
case (pipeline)

	3'b000: //Encoder
	begin
	M0 = 1'b0;
	M1 = 1'b0;
	end
	
	3'b001: //Fetch
	begin
	M0 = 1'b1;
	M1 = 1'b0;
	end
	
	3'b010: //Pipeline
	begin
	M0 = 1'b0;
	M1 = 1'b1;
	end
	
	3'b011: //Incrementer 
	begin
	M0 = 1'b1;
	M1 = 1'b1;
	end
	
	3'b100:
	begin
	if(cond_S == 1'b1)
	begin
	M0 = 1'b0;
	M1 = 1'b1;
	end
	else
	begin
	M0 = 1'b0;
	M1 = 1'b0;
	end 
	end	
	
	3'b101:
	begin
	if(cond_S == 1'b1)
	begin
	M0 = 1'b0;
	M1 = 1'b1;
	end
	else
	begin
	M0 = 1'b1;
	M1 = 1'b1;
	end
	end
	
	3'b110:
	begin
	if(cond_S == 1'b1)
	begin
	M0 = 1'b0;
	M1 = 1'b0;
	end
	else
	begin
	M0 = 1'b1;
	M1 = 1'b1;
	end
	end
	
	3'b111:
	begin
	if(cond_S == 1'b1)
	begin
	M0 = 1'b0;
	M1 = 1'b1;
	end
	else
	begin
	M0 = 1'b1;
	M1 = 1'b0;
	end
	end
	endcase

	endmodule


//--------------------------------------------------------
//				PIPELINE REGISTER
//--------------------------------------------------------

module control_register (output reg [54:0] out, input [54:0] pipeline_in, input clk);

always @(posedge clk)	
out <= pipeline_in;	

endmodule


//--------------------------------------------------------
//				RAM 256
//--------------------------------------------------------
module Ram (
	input [31:0] input_data   , 
	input enable,
	input read_write , 
	input memory_active,
	input [1:0] word_type,
	input double_div, 
	input [7:0] address,
	output reg memory_complete,
	output reg [31:0] output_data         
	);


reg [7:0] ram[0:255];

reg [7:0] temp_address;

// definitions
parameter BYTE = 2'b00;
parameter HALF_WORD = 2'b01;
parameter WORD = 2'b10;
parameter DOUBLE_WORD = 2'b11;
parameter WRITE = 1'b1;
parameter READ = 1'b0;
parameter ENABLE = 1'b0;



always@(memory_active)
begin



if(enable == 0) begin

case(word_type)

BYTE:
begin
	if(read_write==WRITE)
		begin
			ram[address] <= input_data[7:0];
		end

		output_data <= {{24{1'b0}}, ram[address]};
end

HALF_WORD:
begin
	temp_address[7:1] <= address[7:1];
	temp_address[0] <= 1'b0;
	#5;

	if(read_write==WRITE)
	begin
		ram[address] <= input_data[15:8];
		ram[address+1] <= input_data[7:0];
		#3;
	end

	output_data <= {{16{1'b0}}, ram[address], ram[address+1] };
	#3;
end

WORD:
begin
	temp_address[7:2] <= address[7:2];
	temp_address[1:0] <= 2'b00;
	#1;

	if(read_write==WRITE)
	begin
		ram[temp_address] <= input_data[31:24];
		ram[temp_address+1] <= input_data[23:16];
		ram[temp_address+2] <= input_data[15:8];
		ram[temp_address+3] <= input_data[7:0];
		#1;
	end 

	output_data[31:24] <= ram[temp_address];
	output_data[23:16] <= ram[temp_address+1];
	output_data[15:8] <= ram[temp_address+2];
	output_data[7:0] <= ram[temp_address+3];
	#1;
end

DOUBLE_WORD:
begin
	temp_address[7:3] <= address[7:3];
	temp_address[2:0] <= 3'b000;
	#5;

	if(read_write==WRITE)
	begin
		if(double_div)
		begin
			ram[temp_address] <= input_data[31:24];
			ram[temp_address+1] <= input_data[23:16];
			ram[temp_address+2] <= input_data[15:8];
			ram[temp_address+3] <= input_data[7:0];
		end
		else
		begin
			ram[temp_address+4] <= input_data[31:24];
			ram[temp_address+5] <= input_data[23:16];
			ram[temp_address+6] <= input_data[15:8];
			ram[temp_address+7] <= input_data[7:0];
		end
		#3;
	end 

	if(double_div)
	begin
		output_data[31:24] <= ram[temp_address];
		output_data[23:16] <= ram[temp_address+1];
		output_data[15:8] <= ram[temp_address+2];
		output_data[7:0] <= ram[temp_address+3];
	end
	else
	begin
		output_data[31:24] <= ram[temp_address+4];
		output_data[23:16] <= ram[temp_address+5];
		output_data[15:8] <= ram[temp_address+6];
		output_data[7:0] <= ram[temp_address+7];
	end
	#3;
	end
endcase

#5 memory_complete <= 1'b1;
end
end

always@(memory_active)
begin
if (memory_active == 1'b1)
memory_complete <= 1'b0;
end

endmodule



//--------------------------------------------------------
//				Register FIle
//--------------------------------------------------------
module register_file (output reg [31:0] outA, outB, input [31:0] dataIn, input [3:0] readA, readB, writeA, input enable, clr, clk);

reg decoder_enable;

wire [31:0] temp;
wire [31:0] temp1;
wire [15:0] decoder_out;
wire [31:0] register_out0, register_out1, register_out2, register_out3, register_out4, register_out5, register_out6, register_out7,
register_out8, register_out9, register_out10, register_out11, register_out12, register_out13, register_out14, register_out15;

always @ (enable)
begin 

if(!enable)

begin

decoder_enable = 1'b1;

end

else

begin

decoder_enable = 1'b0;

end

end

always @ (*)
begin
outA = temp;
outB = temp1;
end


decoder dec (decoder_out, writeA, decoder_enable);

register_32_bits register0 (register_out0, dataIn, ~decoder_out[0], clr, clk);
register_32_bits register1 (register_out1, dataIn, ~decoder_out[1], clr, clk);
register_32_bits register2 (register_out2, dataIn, ~decoder_out[2], clr, clk);
register_32_bits register3 (register_out3, dataIn, ~decoder_out[3], clr, clk);
register_32_bits register4 (register_out4, dataIn, ~decoder_out[4], clr, clk);
register_32_bits register5 (register_out5, dataIn, ~decoder_out[5], clr, clk);
register_32_bits register6 (register_out6, dataIn, ~decoder_out[6], clr, clk);
register_32_bits register7 (register_out7, dataIn, ~decoder_out[7], clr, clk);
register_32_bits register8 (register_out8, dataIn, ~decoder_out[8], clr, clk);
register_32_bits register9 (register_out9, dataIn, ~decoder_out[9], clr, clk);
register_32_bits register10 (register_out10, dataIn, ~decoder_out[10], clr, clk);
register_32_bits register11 (register_out11, dataIn, ~decoder_out[11], clr, clk);
register_32_bits register12 (register_out12, dataIn, ~decoder_out[12], clr, clk);
register_32_bits register13 (register_out13, dataIn, ~decoder_out[13], clr, clk);
register_32_bits register14 (register_out14, dataIn, ~decoder_out[14], clr, clk);
register_32_bits register15 (register_out15, dataIn, ~decoder_out[15], clr, clk);





mux_16to1 muxA (temp, readA, register_out0, register_out1, register_out2, register_out3, register_out4, register_out5, register_out6, register_out7,
	register_out8, register_out9, register_out10, register_out11, register_out12, register_out13, register_out14, register_out15);

mux_16to1 muxB (temp1, readB, register_out0, register_out1, register_out2, register_out3, register_out4, register_out5, register_out6, register_out7,
	register_out8, register_out9, register_out10, register_out11, register_out12, register_out13, register_out14, register_out15);

endmodule


//--------------------------------------------------------------------
//					Register of 32 bits
//--------------------------------------------------------------------
module register_32_bits (output reg [31:0] Y, input [31:0] I,
	input LE, CLR, CLK);

always @ (posedge CLK, negedge CLR)

if (!CLR) 

    Y <= 32'h00000000; //Clear es active low, 0 = clear y 1 = no esta clear

    else if ((LE == 0) && (CLK == 1)) 

    Y = I;

    endmodule 



//--------------------------------------------------------------------
//					Shifter Module
//--------------------------------------------------------------------

module Shifter (output reg [31:0] result, output reg C_flag, input [31:0] data, input [1:0] shift, input [31:0] shiftNum , input enable);

//Immediate shifter: shifter_operand = data LSR (2*shiftNum)

reg [31:0] tempNum;
reg [31:0] tempData;
reg [31:0] temp;
integer i;

always @ (data, posedge enable, shift, C_flag)

//Active-low
if (enable==1'b0)
begin

tempNum = shiftNum;
tempData = data;
temp = data;

case(shift)

2'b00: 
				//[LSL] 
				begin
				for(i = 0; i < tempNum; i = i+1)
				begin
				C_flag = tempData[31:31];
				tempData= tempData<<1;
				end
				end

				2'b01:
				//LSR
				begin
				for(i = 0; i < tempNum; i = i+1)
				begin
				C_flag = tempData[0];
				tempData= tempData>>1;
				end
				end

				2'b10:
				//ASR
				begin
				for(i = 0; i < tempNum; i = i+1)
				begin
				tempData= tempData>>1;
				tempData = {data[31:31], tempData[30:0]};
				end
				end

				2'b11:
				//ROR
				begin
				for(i = 0; i < tempNum; i = i+1)
				begin
				temp = tempData;
				tempData= tempData>>1;
				tempData = {temp[0:0], tempData[30:0]};
				end
				end

				endcase
				result = tempData;
				end
				else begin
				result = data;
				end

				endmodule


//--------------------------------------------------------------------
//					Sign Extension for Memory Data
//--------------------------------------------------------------------

module Sign_Extension2 (output reg [31:0] Y, input [31:0] In, input [1:0] Data_Type, input enable);


always @ (In, Data_Type)

if (!enable)

begin

case(Data_Type)

      		    2'b00: //Byte

      		    begin 

      		    assign Y[7:0] = In[7:0];
      		    assign Y[31:8] = {24{In[7]}};

      		    end

         		2'b01: //Halfword

         		begin

         		assign Y[15:0] = In[15:0];
         		assign Y[31:16] = {16{In[15]}};

         		end

         		2'b10: //Word

         		begin

         		assign Y = In;

         		end


         		2'b11: //Doubleword

         		begin

         		assign Y = In;

         		end
         		endcase

         		end

         		else

         		assign Y = In;

         		endmodule





