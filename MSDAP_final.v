`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
//
//
// Design Name:     MSDAP
// Project Name:    EEDG 6306 ASIC design Final Project 
// Target Devices:  ASIC
// Submitted by:    Saumya Shah,Rutvikk Kharod
//
//
//////////////////////////////////////////////////////////////////////////////////



/* Top Module */


module msdap(InputL, OutputL, InputR, Sclk, Dclk, Start,
			Reset_n, Frame, InReady, OutReady, OutputR);
			
input InputR, InputL, Sclk, Dclk, Start, Reset_n, Frame;
output InReady;
output OutReady;
output reg [39:0]OutputR;
output reg [39:0]OutputL;

//variables to send to adder
//wire [0:39] sign_extendL, sign_extendR;

//Variables for S2P module
wire data_in_ready;
wire s2p_clr;
wire s2p_en;

//Variables for Main Controller
//wire memCount_flag;
wire Sclk_in, Dclk_in, Frame_in;

//Variables for RJ memory
wire rj_clr;
wire [3:0] rjL_write_val;
wire [3:0] rjR_write_val;
wire [3:0] rjL_rd_addr;
wire [3:0] rjR_rd_addr;
wire rj_write_en;
wire [0 : 15] rjL_data;
wire [0 : 15] rjR_data;

//Variables for Coeff Memory
wire h_clr;
wire h_write_en;
wire [8:0] hL_write_val;
wire [8:0] hR_write_val;
wire [8:0] hL_rd_addr;
wire [8:0] hR_rd_addr;
wire [0 : 15] hL_data;
wire [0 : 15] hR_data; 

//Variables for Data Memory
wire x_clr;
wire [8:0] x_count_send;
wire x_write_en;
wire [7:0] xL_write_val;
wire [7:0] xR_write_val;
wire [7:0] xL_rd_addr;
wire [7:0] xR_rd_addr;
wire [0 : 15] dataL;
wire [0 : 15] dataR;
wire [0 : 15] xL_data;
wire [0 : 15] xR_data;
wire zeroL_flag, zeroR_flag;
wire outReadyL, outReadyR;

//Variables for ALU Controller
wire alu_en, alu_clr;
wire add_subL, add_subR;
wire shift_enL, shift_enR;
wire shift_addL, shift_addR;
wire self_addL, self_addR;

//Variables for Adder
wire [0:39] xL_add_in, xR_add_in;
wire [0:39] shifted_inL, shifted_inR;
wire adderL_clear, adderR_clear;

//Variables for Shifter
wire [0:39] shift_inL, shift_inR;
wire shiftL_clear, shiftR_clear;

//Variables for P2S module
wire enable, clr;
wire out_flag;


//Module Instantiations
main_controller msdap_main(.Sclk(Sclk), .Dclk(Dclk), 
			.Start(Start), .Reset_n(Reset_n), .Frame(Frame), .InReady(InReady),
			.data_in_ready(data_in_ready), .s2p_clr(s2p_clr), .s2p_en(s2p_en),
			.rj_clr(rj_clr), .rjL_write_val(rjL_write_val), .rjR_write_val(rjR_write_val), .rj_write_en(rj_write_en),
			.h_clr(h_clr), .h_write_en(h_write_en), .hL_write_val(hL_write_val), .hR_write_val(hR_write_val), .x_clr(x_clr),
			.x_count_send(x_count_send), .x_write_en(x_write_en), .xL_write_val(xL_write_val), .xR_write_val(xR_write_val),
			.zeroL_flag(zeroL_flag), .zeroR_flag(zeroR_flag), .alu_en(alu_en), .alu_clr(alu_clr), .outReadyL(outReadyL), .outReadyR(outReadyR),
			.enable(enable), .clr(clr), .out_flag(out_flag),
			.Sclk_out(Sclk_in), .Dclk_out(Dclk_in), .Frame_out(Frame_in));

s2p s2p_module (.Dclk(Dclk_in), .s2p_en(s2p_en),
				.s2p_clr(s2p_clr), .inputL(InputL), 
				.inputR(InputR), .dataL_in(dataL),
				.dataR_in(dataR), .data_in_ready(data_in_ready));
				
rj_memory rjL (.Sclk(Sclk_in), .write_en(rj_write_en), .write(rjL_write_val),
				 .data_in(dataL), .read(rjL_rd_addr),
				.rj_data(rjL_data), .rj_clr(rj_clr));
				
rj_memory rjR (.Sclk(Sclk_in), .write_en(rj_write_en), .write(rjR_write_val),
				 .data_in(dataR), .read(rjR_rd_addr),
				.rj_data(rjR_data), .rj_clr(rj_clr));
				
hcoeff_memory hL (.Sclk(Sclk_in), .write_en(h_write_en), .write(hL_write_val),
				 .data_in(dataL), .read(hL_rd_addr),
				.h_data(hL_data), .h_clr(h_clr));
				
hcoeff_memory hR (.Sclk(Sclk_in), .write_en(h_write_en), .write(hR_write_val),
				 .data_in(dataR), .read(hR_rd_addr),
				.h_data(hR_data), .h_clr(h_clr));
				
x_memory xL (.Sclk(Sclk_in), .write_en(x_write_en), .write(xL_write_val),
			 .data_in(dataL), .sleep_flag(zeroL_flag),
			.read(xL_rd_addr), .x_data(xL_data), .x_clr(x_clr));
			
x_memory xR (.Sclk(Sclk_in), .write_en(x_write_en), .write(xR_write_val),
			 .data_in(dataR), .sleep_flag(zeroR_flag),
			.read(xR_rd_addr), .x_data(xR_data), .x_clr(x_clr));
			
alu_controller alu (.Sclk(Sclk_in), .alu_en(alu_en), .indicate_L(add_subL), 
			.indicate_R(add_subR), .shift_enL(shift_enL), .shift_enR(shift_enR),
			.alu_clr(alu_clr), .rjL_in(rjL_data), .rjR_in(rjR_data), 
			.rjL_val(rjL_rd_addr), .rjR_val(rjR_rd_addr),.hL_in(hL_data), 
			.hR_in(hR_data), .hL_val(hL_rd_addr), 
			.hR_val(hR_rd_addr), .shifted_add_L(shift_addL), .self_add_L(self_addL), 
			.shifted_add_R(shift_addR), .self_add_R(self_addR), .xL_val(xL_rd_addr), 
			.xR_val(xR_rd_addr), .outReadyL(outReadyL), .outReadyR(outReadyR), 
			.addL_clr(adderL_clear), .addR_clr(adderR_clear), 
			.shiftL_clr(shiftL_clear), .shiftR_clr(shiftR_clear), 
			.xinp_data(x_count_send));
			
sign_extender sL (.xin_data(xL_data), .xout_data(xL_add_in));

sign_extender sR (.xin_data(xR_data), .xout_data(xR_add_in));

adder add_L(.Sclk(Sclk_in), .add_sub_flag(add_subL), .self_add(self_addL), .shifted_add(shift_addL), 
			.clr(adderL_clear), .x1(xL_add_in), .x2(shifted_inL), .y(shift_inL));
			
adder add_R(.Sclk(Sclk_in), .add_sub_flag(add_subR), .self_add(self_addR), .shifted_add(shift_addR), 
			.clr(adderR_clear), .x1(xR_add_in), .x2(shifted_inR), .y(shift_inR));

shifter shift_L(.clr(shiftL_clear), .shift_in(shift_inL), .shift_out(shifted_inL), .shift_en(shift_enL));

shifter shift_R(.clr(shiftR_clear), .shift_in(shift_inR), .shift_out(shifted_inR), .shift_en(shift_enR));

final_output op_module (.Sclk(Sclk_in), .frame(Frame_in), .enable(enable), .out_flag(out_flag),
				.clr(clr), .channelL_in(shifted_inL), .channelR_in(shifted_inR),
				.outputL(OutputL), .outputR(OutputR), .outReady(OutReady));
				
endmodule






/* Serial to Parallel module */

module s2p(Dclk, s2p_en, s2p_clr, inputL, inputR, dataL_in, dataR_in, data_in_ready);
input Dclk, s2p_en, s2p_clr, inputL, inputR;
output reg data_in_ready;
output reg [0:15] dataL_in, dataR_in;

reg [4:0] count;

always@(negedge Dclk or posedge s2p_clr)
begin
	if(s2p_clr)
	begin
		data_in_ready = 1'b0;
		count = 5'b0;
		dataL_in = 16'b0;
		dataR_in = 16'b0;
	end
	else if(s2p_en)
	begin
		if(count < 5'd16)
		begin
			data_in_ready = 1'b0;
			dataL_in[count] = inputL;
			dataR_in[count] = inputR;
			count = count + 5'd1;
		end
		else
		begin
			data_in_ready = 1'b1;
			count = 5'b0;
		end
	end
end
endmodule





/* Rj memory module */

module rj_memory(Sclk, write_en, write, data_in, read, rj_data, rj_clr);
input write_en, rj_clr, Sclk;
input [3:0] write, read;
input [7:0] data_in;
output [7:0] rj_data;
//reg [4:0] i = 5'd0;


reg [0:15] rj_mem [15:0];

R_MEM R_MEM_1(
		.RW0_addr(write),
                .RW0_clk(Sclk),
		.RW0_wdata(data_in),
		.RW0_rdata(rj_data),
		.RW0_en(write_en),
		.RW0_wmode(rj_clk)
);

endmodule


/* Coefficient memory module */

module hcoeff_memory(Sclk, write_en, write, data_in, read, h_data, h_clr);
input write_en, h_clr, Sclk;
input [8:0] write, read;
input [8:0] data_in;
output [8:0] h_data;

//reg [9:0] i = 10'd0;

reg [8:0] h_mem [511:0];
CO_MEM CO_MEM_1(.RW0_addr(write),
				.RW0_clk(Sclk),
				.RW0_wdata(h_data),
				.RW0_rdata(data_in),
				.RW0_en(rd_en),
				.RW0_wmode(write_en));


endmodule


/* Input memory module */

module x_memory(Sclk, write_en, write, data_in, sleep_flag, read, x_data, x_clr);
input write_en, x_clr, Sclk;
input [7:0] write, read;
input [15:0] data_in;
output [15:0] x_data;
output reg sleep_flag;

//reg [8:0] i = 9'd0;
reg [0:15] x_mem [255:0];
reg [9:0] zero_count;

assign read_addr = read;

DATA_MEM DATA_MEM_1(
		.RW0_addr(write),
		.RW0_clk(Sclk),
		.RW0_wdata(data_in),
		.RW0_rdata(x_data),
		.RW0_en(write_en),
		.RW0_wmode(x_clr)
);
endmodule

/* ALU Controller */

module alu_controller(Sclk, alu_en, indicate_L, indicate_R, shift_enL, shift_enR, alu_clr, rjL_in, rjR_in, rjL_val, rjR_val,
						hL_in, hR_in, hL_val, hR_val, shifted_add_L, 
						self_add_L, shifted_add_R, self_add_R, xL_val, xR_val,
						outReadyL, outReadyR, addL_clr, addR_clr, 
						shiftL_clr, shiftR_clr, xinp_data);

input Sclk, alu_en, alu_clr;
//input memCount_flag;
input [0:15] hL_in, hR_in, rjL_in, rjR_in;
input [8:0] xinp_data;
output reg [3:0] rjL_val, rjR_val;
output reg [8:0] hL_val, hR_val;
output reg [7:0] xL_val, xR_val;
output reg indicate_L, shifted_add_L, self_add_L;
output reg indicate_R, shifted_add_R, self_add_R;
output reg shift_enL, shift_enR;
output reg outReadyL, outReadyR;
output reg addL_clr, addR_clr, shiftL_clr, shiftR_clr;
						
reg [15:0] iL, jL, kL, pL, x_nL, nL;
reg [15:0] iR, jR, kR, pR, x_nR, nR;
reg [2:0] compStateL, compStateR;
reg [39:0] uL, uR;
reg [15:0] h1L, h1R;
reg [15:0] signL, signR;
reg [8:0] memCount;
reg flag_outL, flag_outR;
reg [3:0] cnt_L, cnt_R;

always@(negedge Sclk or posedge alu_clr)
begin

	if(alu_clr)
	begin
		rjL_val = 0; rjR_val = 0; hL_val = 0; hR_val = 0;
		
	end

	else if(alu_en)
	begin
		if(!flag_outL)
		begin
			case(compStateL)
			3'b001:
			begin
				if((kL < 16'd16) && (pL < 16'd16))
				begin
					rjL_val = pL;
				end
			end
			
			3'b010:
			begin
				if(iL < jL)
				begin
					hL_val = iL;
				end
			end
			endcase
		end
		
		if(!flag_outR)
		begin	
			case(compStateR)
			3'b001:
			begin
				if((kR < 16'd16) && (pR < 16'd16))
				begin
					rjR_val = pR;
				end
			end
			
			3'b010:
			begin
				if(iR < jR)
				begin
					hR_val = iR;
				end
			end
			endcase
		end
	end
end



always@(posedge Sclk or posedge alu_clr)
begin

	if(alu_clr)
	begin
		//rjL_val = 0; rjR_val = 0; hL_val = 0; hR_val = 0;
		xL_val = 0; xR_val = 0;
		indicate_L = 0; shifted_add_L = 0; self_add_L = 0;
		indicate_R = 0; shifted_add_R = 0; self_add_R = 0;
		shift_enL = 0; shift_enR = 0;
		outReadyL = 0; outReadyR = 0;
		shiftL_clr = 1; shiftR_clr = 1;
		addL_clr = 1; addR_clr = 1;
	
		iL = 0; jL = 0; kL = 0; pL = 0; x_nL = 0; nL = 0;
		iR = 0; jR = 0; kR = 0; pR = 0; x_nR = 0; nR = 0;
		compStateL = 0; compStateR = 0;
		uL = 0; uR = 0;
		h1L = 0; h1R = 0;
		signL = 0; signR = 0;
		memCount = 0;
		flag_outL = 0; flag_outR = 0;
		cnt_L = 0; cnt_R = 0;
		
	end
	
	else if(alu_en)
	begin
	
		if((xinp_data > x_nL) || (xinp_data > x_nR))
		begin
			x_nL = xinp_data;
			x_nR = xinp_data;
		end
		
		if(!flag_outL)
		begin
		case(compStateL)
		3'b000:
		begin
			//yL[nL] = 0;
			outReadyL = 1'b0;
			iL = 0;
			jL = 0;
			kL = 0;
			pL = 0;
			shifted_add_L = 1'b0;
			self_add_L = 1'b0;
			shift_enL = 1'b0;
			if(cnt_L > 4'd2)
			begin
				shiftL_clr = 1; 
				addL_clr = 1;
				compStateL = 3'b001;
			end
			cnt_L = cnt_L + 1;
		end
					
		3'b001:
		begin
			cnt_L = 0;
			shiftL_clr = 0; 
			addL_clr = 0; 
			shifted_add_L = 1'b0;
			self_add_L = 1'b0;
			shift_enL = 1'b0;
			
			if((kL < 16'd16) && (pL < 16'd16))
			begin
				//rjL_val = pL;
				uL = 0;
				//rjL_int = rjL_in; 
				jL = jL + rjL_in;
				compStateL = 3'b010;
			end
			else
			begin
			
				nL = nL + 1;
				x_nL = x_nL + 1;
				outReadyL = 1'b1;
				compStateL = 3'b000;
			end
		end
					
		3'b010:
		begin
		
			shifted_add_L = 1'b0;
			self_add_L = 1'b0;
			shift_enL = 1'b0;
			if(iL < jL)
			begin
				//hL_val = iL;
				h1L = hL_in;
				signL = h1L & 16'h0100;
				h1L = h1L & 16'h00FF;
			
				if(((memCount + x_nL) > h1L) || ((memCount + x_nL) == h1L))
				begin
					if(signL == 16'h0100)
					begin
						indicate_L = 1'b1;
						shifted_add_L = 1'b0;
						self_add_L = 1'b1;
						shift_enL = 1'b0;
						if(x_nL < h1L)
						begin
							xL_val = (memCount + x_nL) + (~(h1L) + 1);
							
						end
						else
						begin
							xL_val = x_nL + (~(h1L) + 1);
						end
					end
					else
					begin
						indicate_L = 1'b0;
						shifted_add_L = 1'b0;
						self_add_L = 1'b1;
						shift_enL = 1'b0;
						if(x_nL < h1L)
						begin
							xL_val = (memCount + x_nL) + (~(h1L) + 1);
						end
						else
						begin
							xL_val = x_nL + (~(h1L) + 1);
						end
					end
				end
				iL = iL + 1;
				compStateL = 3'b010;
			end
			else
			begin
				compStateL = 3'b011;
			end
		end
					
		3'b011:
		begin
			shifted_add_L = 1'b1;
			self_add_L = 1'b0;
			shift_enL = 1'b0;
			compStateL = 3'b100;
		end
					
		3'b100:
		begin
			shifted_add_L = 1'b0;
			self_add_L = 1'b0;
			shift_enL = 1'b1;
			
			pL = pL + 1;
			kL = kL + 1;
			compStateL = 3'b001;
		end
					
		endcase
		end
		
	
	if(!flag_outR)
	begin	
	case(compStateR)
		3'b000:
		begin
			outReadyR = 1'b0;
			iR = 0;
			jR = 0;
			kR = 0;
			pR = 0;
			shifted_add_R = 1'b0;
			self_add_R = 1'b0;
			shift_enR = 1'b0;
			if(cnt_R > 4'd2)
			begin
				shiftR_clr = 1; 
				addR_clr = 1;
				compStateR = 3'b001;
			end
			cnt_R = cnt_R + 1;
		end
					
		3'b001:
		begin
			cnt_R = 0;
			shiftR_clr = 0; 
			addR_clr = 0;
			shifted_add_R = 1'b0;
			self_add_R = 1'b0;
			shift_enR = 1'b0;
			
			if((kR < 16'd16) && (pR < 16'd16))
			begin
				uR = 0;
				jR = jR + rjR_in;
				compStateR = 3'b010;
			end
			else
			begin
				
				nR = nR + 1;
				x_nR = x_nR + 1;
				outReadyR = 1'b1;
				compStateR = 3'b000;
			end
		end
					
		3'b010:
		begin
		
			shifted_add_R = 1'b0;
			self_add_R = 1'b0;
			shift_enR = 1'b0;
			if(iR < jR)
			begin
				h1R = hR_in;
				signR = h1R & 16'h0100;
				h1R = h1R & 16'h00FF;
			
				if(((memCount + x_nR) > h1R) || ((memCount + x_nR) == h1R))
				begin
					if(signR == 16'h0100)
					begin
						indicate_R = 1'b1;
						shifted_add_R = 1'b0;
						self_add_R = 1'b1;
						shift_enR = 1'b0;
						if(x_nR < h1R)
						begin
							xR_val = (memCount + x_nR) + (~(h1R) + 1);
						end
						else
						begin
							xR_val = x_nR + (~(h1R) + 1);
						end
					end
					else
					begin
						indicate_R = 1'b0;
						shifted_add_R = 1'b0;
						self_add_R = 1'b1;
						shift_enR = 1'b0;
						if(x_nR < h1R)
						begin
							xR_val = (memCount + x_nR) + (~(h1R) + 1);
						end
						else
						begin
							xR_val = x_nR + (~(h1R) + 1);
						end
					end
				end
				iR = iR + 1;
				compStateR = 3'b010;
			end
			else
			begin
				compStateR = 3'b011;
			end
		end
					
		3'b011:
		begin
			shifted_add_R = 1'b1;
			self_add_R = 1'b0;
			shift_enR = 1'b0;
			compStateR = 3'b100;
		end
					
		3'b100:
		begin
			shifted_add_R = 1'b0;
			self_add_R = 1'b0;
			shift_enR = 1'b1;
			
			pR = pR + 1;
			kR = kR + 1;
			compStateR = 3'b001;
		end
					
		endcase
		end
		
		if(outReadyL)
			flag_outL = 1;
			
		if(outReadyR)
			flag_outR = 1;
		
		if(flag_outL && flag_outR)
		begin
			flag_outL = 0;
			flag_outR = 0;
		end
		
		

	if((x_nL == 16'd256) && (x_nR == 16'd256))
	begin
		x_nL = 0;
		x_nR = 0;
		memCount = 9'd256;
	end
	end
end
endmodule

/* Sign Extender Module */

module sign_extender(xin_data, xout_data);
input [0:15] xin_data;
output [0:39] xout_data;

assign xout_data = (xin_data[0]) ? {8'hFF, xin_data, 16'h0000} : {8'h00, xin_data, 16'h0000};

endmodule


/* Adder Module */


module adder (Sclk, add_sub_flag, self_add, shifted_add, clr, x1, x2, y);
input Sclk, add_sub_flag, self_add, shifted_add;
input clr;
input [0:39] x1, x2;
output reg [0:39] y;
reg [0:39] temp;

always@(posedge Sclk)
begin
	if(clr)
	begin
		temp  = 40'b0;
		y = 40'b0;
	end
	
	else if(self_add && (add_sub_flag == 0))
	begin
		temp = temp + x1;
	end
	
	else if(self_add && add_sub_flag)
	begin
		temp = temp + (~(x1) + 1);
	end
	
	else if(shifted_add)
	begin
		y = temp + x2;
		temp = 0;
	end
	
end

endmodule 


/* Shifter */


module shifter(clr, shift_in, shift_out, shift_en);
input [0:39] shift_in;
input shift_en, clr;
output reg [0:39] shift_out;

reg sign;

always@(clr or shift_en)
begin
	if(clr)
	begin
		shift_out = 40'b0;
		sign = 0;
	end
	
	else
	begin
	if(shift_en)
	begin
		sign = shift_in[0];
		
		shift_out = {shift_in[0], shift_in[0:38]};
		
	end
	else
	begin
		shift_out = shift_out;
	end
	end
	
end
endmodule


/* output module */

module final_output(Sclk, frame, enable, clr, channelL_in, channelR_in, outputL, outputR, out_flag, outReady);

input Sclk, frame, clr;
input [0:39] channelL_in, channelR_in;
input enable;
output reg out_flag;
output reg [39:0]outputR;
output reg [39:0]outputL; 
output reg outReady;

reg frame_check;
reg [5:0] count;
//integer yout;

always@(negedge Sclk or posedge clr)
begin
	

	if(clr)
	begin
		//count = 6'b0;
		frame_check = 1'b0;
		outReady = 1'b0;
		out_flag = 1'b0;
	end
	
	
	
	else 
		begin
			if(enable)
			begin
			
				if(frame)
				//@(posedge frame)
				begin
					frame_check = 1'b1;
				end
				if(frame_check)
				begin
					outReady = 1'b1;
					outputL = channelL_in;
					outputR = channelR_in;
					//count = count + 1'b1;
				end
			
			out_flag = 1'b1;
			frame_check = 1'b0;
			
			end
	
			else
				begin
					outReady = 0;
					out_flag = 1'b0;
				end
	
		end
	
end

endmodule

module main_controller(Sclk, Dclk, 
			Start, Reset_n, Frame, InReady, 
			data_in_ready, s2p_clr, s2p_en,
			rj_clr, rjL_write_val, rjR_write_val, rj_write_en,
			h_clr, h_write_en, hL_write_val, hR_write_val, x_clr,
			x_count_send, x_write_en, xL_write_val, xR_write_val,
			zeroL_flag, zeroR_flag, alu_en, alu_clr, outReadyL, outReadyR,
			enable, clr, out_flag, Dclk_out, Sclk_out, Frame_out);
input Sclk, Dclk, Start, Reset_n, Frame;
output reg InReady;
output Dclk_out, Sclk_out, Frame_out;


reg [3:0] state;
reg data_stored;
reg [20:0] topCount;
reg h_flag, x_flag; 
reg setSleep;
reg [8:0] memCount;
reg memCount_flag;
reg setup, start_calc, start_calc_flag;
reg s1_flag, s5_flag;

//Variables for S2P module
input data_in_ready;
output reg s2p_clr;
output reg s2p_en;

//Variables for RJ memory
//wire rj_Sclk;
output reg rj_clr;
output reg [3:0] rjL_write_val;
output reg [3:0] rjR_write_val;
//wire [15:0] rjL_rd_addr;
//wire [15:0] rjR_rd_addr;

output reg rj_write_en;
reg [4:0] rj_count;
//wire [0 : 15] rjL_data;
//wire [0 : 15] rjR_data;

//Variables for Coeff Memory
output reg h_clr;
reg [9:0] h_count;
output reg h_write_en;
output reg [8:0] hL_write_val;
output reg [8:0] hR_write_val;

//Variables for Data Memory
output reg x_clr;
reg [8:0] x_count;
output reg [8:0] x_count_send;
output reg x_write_en;
output reg [7:0] xL_write_val;
output reg [7:0] xR_write_val;
input zeroL_flag, zeroR_flag;

//Variables for ALU Controller
output reg alu_en, alu_clr;
reg yReadyL, yReadyR;
input outReadyL, outReadyR;

//Variables for P2S module
output reg enable, clr;
input out_flag;

assign Dclk_out = Dclk;
assign Sclk_out = Sclk;
assign Frame_out = Frame;

always@(posedge Sclk or negedge Start or negedge Reset_n)
begin

	if(!Reset_n)
	begin
		state = 4'b0111;
		case(state)
		4'b0111:
		begin
			InReady = 0; 
			//OutReady = 0; OutputR = 0; OutputL = 0;
			data_stored = 0;
			setSleep = 0;
			rj_write_en = 0;
			h_write_en = 0; start_calc = 0;
			x_clr = 1'b1; x_count = 0; x_write_en = 0;
			rj_count = 0; rjL_write_val = 0; rjR_write_val = 0; 
			h_count = 0; hL_write_val = 0; hR_write_val = 0;
			memCount = 0; memCount_flag = 0;
			s2p_clr = 1; s2p_en = 0;
			xL_write_val = 0; xR_write_val = 0;
			alu_en = 0; alu_clr = 1;
			//outReadyL = 0; outReadyR = 0; 
			yReadyL = 0; yReadyR = 0; start_calc_flag = 0;
			//adderL_clear = 1; adderR_clear = 1;
			enable = 0; clr = 1; 
			x_count_send = 0;
			s5_flag = 1;
			//shift_clear = 1;
			//out_flag = 0;
			//zeroL_flag = 0; zeroR_flag = 0;			
			
		end
		endcase
	end
	
	else if(Start == 0)												//if start ==0 //start low
	begin
		state = 4'b0000;
		case(state)
		4'b0000:
		begin
			InReady = 0; 
			//OutReady = 0; OutputR = 0; OutputL = 0;
			data_stored = 0;
			topCount = 0;
			setSleep = 0; start_calc = 0;
			rj_clr = 1'b1; rj_write_en = 0;
			h_clr = 1'b1; h_write_en = 0;
			x_clr = 1'b1; x_count = 0; x_write_en = 0; x_flag = 0;
			rj_count = 0; rjL_write_val = 0; rjR_write_val = 0; 
			h_count = 0; hL_write_val = 0; hR_write_val = 0; h_flag = 0;
			memCount = 0; memCount_flag = 0;
			s2p_clr = 1; s2p_en = 0;
			xL_write_val = 0; xR_write_val = 0;
			alu_en = 0; alu_clr = 1; start_calc_flag = 0;
			//outReadyL = 0; outReadyR = 0; 
			yReadyL = 0; yReadyR = 0;
			//adderL_clear = 1; adderR_clear = 1;
			enable = 0; clr = 1;
			x_count_send = 0;
			s1_flag = 1;
			//shift_clear = 1;
			//out_flag = 0;
			//zeroL_flag = 0; zeroR_flag = 0;

			
		end
		endcase
	end
	
	else if(Start)												//start low (!Start)
	begin
		if(s1_flag)
		begin
			state = 4'b0001;
			s1_flag = 0;
		end
		
		if(s5_flag)
		begin
			state = 4'b0101;
			s5_flag = 0;
		end
		
		case(state)
		//wait to receive rj
		4'b0001:
		begin
			//adderL_clear = 0; adderR_clear = 0;
			//shift_clear = 0;
			data_stored = 1'b0;
			InReady = 1;
			if(Frame == 0)									//frame low
			begin
				state = 4'b0010;
			end
			
		end
		
		//Receiving Rj values from the controller
		4'b0010:
		begin
			s2p_clr = 1'b0;
			rj_clr = 1'b0;
			s2p_en = 1'b1;
			if(data_in_ready && !data_stored)
			begin
				if(rj_count < 5'd16)
				begin
					rj_write_en = 1'b1;
					rjL_write_val = rj_count;
					rjR_write_val = rj_count;
					rj_count = rj_count + 1;
					data_stored = 1'b1;
				end
				if(rj_count == 5'd16)
				begin
					//rj_write_en = 1'b0;
					state = 4'b0011;
					s2p_en = 1'b0;
				end
			end
			else if(!data_in_ready)
			begin
				data_stored = 1'b0;
				rj_write_en = 1'b0;
			end
			else
			begin
				rj_write_en = 1'b0;
			end
		end
		
		//wait to receive h
		4'b0011:
		begin
			rj_write_en = 1'b0;
			data_stored = 1'b0;
			InReady = 1;
			if(Frame == 0)						//frame low
			begin
				state = 4'b0100;
			end
		end

		//Receiving H values from the controller
		4'b0100:
		begin
			s2p_en = 1'b1;
			h_clr = 1'b0;
			if(!data_in_ready)
				h_flag = 1;
				
			if(data_in_ready && !data_stored)
			begin
				if((h_count < 10'd512) && (h_flag))
				begin
					h_write_en = 1'b1;
					hL_write_val = h_count;
					hR_write_val = h_count;
					h_count = h_count + 1;
					data_stored = 1'b1;
				end
				if(h_count == 10'd512)
				begin
					//h_write_en = 1'b0;
					state = 4'b0101;
					s2p_en = 1'b0;
				end
			end
			else if(!data_in_ready)
			begin
				data_stored = 1'b0;
				h_write_en = 1'b0;
			end
			else
			begin
				h_write_en = 1'b0;
			end
		end
		
		//wait to receive input
		4'b0101:
		begin
			//adderL_clear = 0; adderR_clear = 0;
			//shift_clear = 0;
			h_write_en = 1'b0;
			data_stored = 1'b0;
			InReady = 1;
			if(Frame == 0)							//frame low
			begin
				state = 4'b0110;
			end
			/*if(Reset_n == 0)
			begin
				state = 4'b0111;
			end*/
			
		end

		//Working mode-receives input data
		4'b0110:
		begin
			//x_count_send = 0;
			x_clr = 1'b0;
			s2p_clr = 1'b0;
			if(Reset_n == 0)
			begin
				state = 4'b0111;
			end
			
			if(!data_in_ready)
				x_flag = 1;
			
			if((x_count < 9'd256) && (topCount < 21'd7000))
				s2p_en = 1'b1;
				
			if(data_in_ready && !data_stored && (topCount < 21'd7000))
			begin
				if(x_count < 9'd256 && x_flag)
				begin
					x_write_en = 1'b1;
					xL_write_val = x_count;
					xR_write_val = x_count;
					x_count = x_count + 1;
					topCount = topCount + 1;
					data_stored = 1'b1;
				end
				if(x_count == 9'd256)
				begin
					x_count = 0;
					//x_write_en = 1'b0;
					state = 4'b0110;
					s2p_en = 1'b0;
					memCount = 9'd256;
					memCount_flag = 1'b1;
				end
			end
			else if(!data_in_ready)
			begin
				data_stored = 1'b0;
				x_write_en = 1'b0;
			end
			else if(topCount == 21'd7000)
			begin
				s2p_en = 0;
			end
			else
			begin
				x_write_en = 1'b0;
			end
			
			if(zeroL_flag && zeroR_flag)
			begin
				setSleep = 1;
				//state = 4'b1000;
			end
			
			
		//Calculation part
			if(data_stored && !setSleep)
			begin
				start_calc = 1;
				start_calc_flag = 1;
			end
			
			if(start_calc && !yReadyL && !yReadyR && !setSleep)
			begin
				alu_clr = 1'b0;
				alu_en = 1'b1;
			end
			else if(yReadyL && yReadyR && !setSleep)
			begin
				alu_en = 1'b0;
				start_calc = 0;
				//@(negedge data_stored) alu_en = 1'b1;
			end
			
			if(!alu_en && start_calc_flag && !Frame && !setSleep)
			begin
				start_calc = 1;
				start_calc_flag = 0;
			end
			
			if(outReadyL)
				yReadyL = 1'b1;
				
			 if(outReadyR)
				yReadyR = 1'b1;
			
			if(yReadyL && yReadyR)
			begin
				clr = 0;
				x_count_send = 0;
				enable = 1'b1;
				if(out_flag)
				begin
					enable = 1'b0;
					yReadyL = 1'b0;
					yReadyR = 1'b0;
					//out_flag = 1'b0;
				end
				if(out_flag && setSleep)
				begin
					state = 4'b1000;
					setSleep = 0;
					alu_en = 0;
					enable = 0;
				end
			end

			
			
		end

		//sleep mode-only receive input
		4'b1000:
		begin
			InReady = 1'b1;
			//OutReady = 1'b0;
			
			if(x_count < 9'd256 && (topCount < 21'd7000))
				s2p_en = 1'b1;
				
			if(data_in_ready && !data_stored && (topCount < 21'd7000))
			begin
				if(x_count < 9'd256)
				begin
					x_write_en = 1'b1;
					xL_write_val = x_count;
					xR_write_val = x_count;
					x_count = x_count + 1;
					topCount = topCount + 1;
					data_stored = 1'b1;
				end
				if(x_count == 9'd256)
				begin
					x_count = 0;
					memCount = 9'd256;
					x_write_en = 1'b0;
					state = 4'b0110;
					s2p_en = 1'b0;
				end
			end
			else if(!data_in_ready)
			begin
				data_stored = 1'b0;
				x_write_en = 1'b0;
			end
			else if(topCount == 21'd7000)
			begin
				s2p_en = 0;
			end
			else
			begin
				x_write_en = 1'b0;
			end
			
			if(zeroL_flag && zeroR_flag)
			begin
				state = 4'b1000;
			end
			else
			begin
				state = 4'b0110;
				x_count_send = x_count - 1;
			end
			
		end
		endcase
		
	end
end
				
endmodule









module CO_MEM(
  input  [8:0]  RW0_addr,
  input         RW0_clk,
  input  [8:0] RW0_wdata,
  output [8:0] RW0_rdata,
  input         RW0_en,
  input         RW0_wmode
);
  wire [11:0] mem_0_O,mem_1_O,mem_2_O,mem_3_O;
  wire mem_OEB;
  wire mem_WEB;
  wire mem_0_CSB, mem_1_CSB, mem_2_CSB, mem_3_CSB;
  
  SRAM1RW128x12 mem_0_0 (
    .A(RW0_addr[6:0]),
    .CE(RW0_clk),
    .I( {3'b000, RW0_wdata} ),
    .O(mem_0_O),
    .CSB(mem_0_CSB),
    .OEB(mem_OEB),
    .WEB(mem_WEB)
  );
  SRAM1RW128x12 mem_0_1 (
    .A(RW0_addr[6:0]),
    .CE(RW0_clk),
    .I( {3'b000, RW0_wdata} ),
    .O(mem_1_O),
    .CSB(mem_1_CSB),
    .OEB(mem_OEB),
    .WEB(mem_WEB)
  );
  SRAM1RW128x12 mem_0_2 (
    .A(RW0_addr[6:0]),
    .CE(RW0_clk),
    .I( {3'b000, RW0_wdata} ),
    .O(mem_2_O),
    .CSB(mem_2_CSB),
    .OEB(mem_OEB),
    .WEB(mem_WEB)
  );
  SRAM1RW128x12 mem_0_3 (
    .A(RW0_addr[6:0]),
    .CE(RW0_clk),
    .I( {3'b000, RW0_wdata} ),
    .O(mem_3_O),
    .CSB(mem_3_CSB),
    .OEB(mem_OEB),
    .WEB(mem_WEB)
  );
  assign RW0_rdata = {9{~mem_OEB}} & (({9{~mem_0_CSB}} & mem_0_O[8:0]) | ({9{~mem_1_CSB}} & mem_1_O[8:0]) | ({9{~mem_2_CSB}} & mem_2_O[8:0]) | ({9{~mem_3_CSB}} & mem_3_O[8:0]));
  assign mem_OEB = ~(RW0_en & ~RW0_wmode);
  assign mem_WEB = ~(RW0_en & RW0_wmode);
  assign mem_0_CSB = ~(RW0_addr[8:7] == 2'd0);
  assign mem_1_CSB = ~(RW0_addr[8:7] == 2'd1);
  assign mem_2_CSB = ~(RW0_addr[8:7] == 2'd2);
  assign mem_3_CSB = ~(RW0_addr[8:7] == 2'd3);
endmodule




module DATA_MEM(
  input  [7:0]  RW0_addr,
  input         RW0_clk,
  input  [15:0] RW0_wdata,
  output [15:0] RW0_rdata,
  input         RW0_en,
  input         RW0_wmode
);
  wire [7:0] mem_0_0_A;
  wire  mem_0_0_CE;
  wire [7:0] mem_0_0_I;
  wire [7:0] mem_0_0_O;
  wire  mem_0_0_CSB;
  wire  mem_0_0_OEB;
  wire  mem_0_0_WEB;
  wire [7:0] mem_0_1_A;
  wire  mem_0_1_CE;
  wire [7:0] mem_0_1_I;
  wire [7:0] mem_0_1_O;
  wire  mem_0_1_CSB;
  wire  mem_0_1_OEB;
  wire  mem_0_1_WEB;
  SRAM1RW256x8 mem_0_0 (
    .A(mem_0_0_A),
    .CE(mem_0_0_CE),
    .I(mem_0_0_I),
    .O(mem_0_0_O),
    .CSB(mem_0_0_CSB),
    .OEB(mem_0_0_OEB),
    .WEB(mem_0_0_WEB)
  );
  SRAM1RW256x8 mem_0_1 (
    .A(mem_0_1_A),
    .CE(mem_0_1_CE),
    .I(mem_0_1_I),
    .O(mem_0_1_O),
    .CSB(mem_0_1_CSB),
    .OEB(mem_0_1_OEB),
    .WEB(mem_0_1_WEB)
  );
  assign RW0_rdata = {mem_0_1_O,mem_0_0_O};
  assign mem_0_0_A = RW0_addr;
  assign mem_0_0_CE = RW0_clk;
  assign mem_0_0_I = RW0_wdata[7:0];
  assign mem_0_0_CSB = ~RW0_en;
  assign mem_0_0_OEB = ~(~RW0_wmode & RW0_en);
  assign mem_0_0_WEB = ~RW0_wmode;
  assign mem_0_1_A = RW0_addr;
  assign mem_0_1_CE = RW0_clk;
  assign mem_0_1_I = RW0_wdata[15:8];
  assign mem_0_1_CSB = ~RW0_en;
  assign mem_0_1_OEB = ~(~RW0_wmode & RW0_en);
  assign mem_0_1_WEB = ~RW0_wmode;
endmodule



module R_MEM(
  input  [3:0]  RW0_addr,
  input         RW0_clk,
  input  [7:0]  RW0_wdata,
  output [7:0]  RW0_rdata,
  input         RW0_en,
  input         RW0_wmode
);
  wire [3:0] mem_0_0_A1;
  wire  mem_0_0_CE1;
  wire [7:0] mem_0_0_I1;
  wire [7:0] mem_0_0_O1;
  wire  mem_0_0_CSB1;
  wire  mem_0_0_OEB1;
  wire  mem_0_0_WEB1;
  wire [3:0] mem_0_0_A2;
  wire  mem_0_0_CE2;
  wire [7:0] mem_0_0_I2;
  wire [7:0] mem_0_0_O2;
  wire  mem_0_0_CSB2;
  wire  mem_0_0_OEB2;
  wire  mem_0_0_WEB2;
  SRAM2RW16x8 mem_0_0 (
    .A1(mem_0_0_A1),
    .CE1(mem_0_0_CE1),
    .I1(mem_0_0_I1),
    .O1(mem_0_0_O1),
    .CSB1(mem_0_0_CSB1),
    .OEB1(mem_0_0_OEB1),
    .WEB1(mem_0_0_WEB1),
    .A2(mem_0_0_A2),
    .CE2(mem_0_0_CE2),
    .I2(mem_0_0_I2),
    .O2(mem_0_0_O2),
    .CSB2(mem_0_0_CSB2),
    .OEB2(mem_0_0_OEB2),
    .WEB2(mem_0_0_WEB2)
  );
  assign RW0_rdata = mem_0_0_O1;
  assign mem_0_0_A1 = RW0_addr;
  assign mem_0_0_CE1 = RW0_clk;
  assign mem_0_0_I1 = RW0_wdata[7:0];
  assign mem_0_0_CSB1 = ~RW0_en;
  assign mem_0_0_OEB1 = ~(~RW0_wmode & RW0_en);
  assign mem_0_0_WEB1 = ~RW0_wmode;
  assign mem_0_0_A2 = RW0_addr;
  assign mem_0_0_CE2 = 'b1;
  assign mem_0_0_I2 = RW0_wdata[7:0];
  assign mem_0_0_CSB2 = 'b1;
  assign mem_0_0_OEB2 = 'b1;
  assign mem_0_0_WEB2 = 'b1;
endmodule



