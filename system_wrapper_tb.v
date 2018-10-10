//Copyright 1986-2016 Xilinx, Inc. All Rights Reserved.
//--------------------------------------------------------------------------------
//Tool Version: Vivado v.2016.4 (win64) Build 1756540 Mon Jan 23 19:11:23 MST 2017
//Date        : Thu Mar 01 01:54:12 2018
//Host        : DESKTOP-DNBUTA0 running 64-bit major release  (build 9200)
//Command     : generate_target system_wrapper.bd
//Design      : system_wrapper
//Purpose     : IP block netlist
//--------------------------------------------------------------------------------
`timescale 1 ps / 1 ps
`include "processing_system7_bfm_v2_0_vl_rfs.v"
`define NULL 0


module system_wrapper_tb
(
	FIXED_IO_mio,
	FIXED_IO_ps_clk,
	FIXED_IO_ps_porb,
	FIXED_IO_ps_srstb,
	LED,
	IMG_FRAME_VAL,
	IMG_LINE_VAL,
	IMG_CLK_OUT,
	IMG_TRIG,
	IMG_MON,
	IMG_DOUT
);
inout [31:0]FIXED_IO_mio;
inout FIXED_IO_ps_clk;
inout FIXED_IO_ps_porb;
inout FIXED_IO_ps_srstb;

output [0:0]LED;
input IMG_FRAME_VAL;
input IMG_LINE_VAL;
input IMG_CLK_OUT;
output reg [2:0] IMG_TRIG = 0;
input [1:0] IMG_MON;
input [9:0] IMG_DOUT;

wire [31:0]FIXED_IO_mio;
wire FIXED_IO_ps_clk;
wire FIXED_IO_ps_porb;
wire FIXED_IO_ps_srstb;
reg signal_know_if_or_not = 0;

/***************  Block Memory Stuff *************/
//This memory will hold data produced from the image processing algrotihm and send to the PS ARM
wire [12:0]addr        ;//block RAM size =2^13 ; 
reg  [15:0]din         ;//each location holds 16 bit of the block RAM      
reg  we;				//Block RAM Write Enable 
/*************************************************/
wire clk;               //clock 
wire sample_clk;

wire [0:0]gpio_1_tri_o;
wire [0:0]gpio_2_tri_i;
wire [0:0]gpio_tri_o;

wire trigger;
reg  trigger_temp, trigger_temp2, frame_val_temp, frame_val_temp2;
reg  done=0, running=0, trigger_run=0;

wire [31:0] trigger_limitation;
reg  [31:0] cnt_trigger=0;
wire [9:0]  L1, L2, L3, L4;


//Clock generate part to generate clk = 72 Mhz, clk_shift_270 72Mhz, and 288 Mhz clock
clk_wiz_0 clk_wiz_0
(
	// Clock out ports
	.clk_out1(clk),     // output clk_out1
	.clk_out2(clk_shift_270),     // output clk_out2 //shift 270 for phase to latch data
	.clk_out3(sample_clk),     // output clk_out3
	// Clock in ports
	.clk_in1(IMG_CLK_OUT) // input clk_in1
);      



//Zynq processor system
system system_i
(
	.FIXED_IO_mio     (FIXED_IO_mio),
	.FIXED_IO_ps_clk  (FIXED_IO_ps_clk),
	.FIXED_IO_ps_porb (FIXED_IO_ps_porb),
	.FIXED_IO_ps_srstb(FIXED_IO_ps_srstb),
	//The control signal from zynq processor
	.GPIO_1_tri_o(trigger),//trigger signal to send trigger0 1 2 to camera
	.GPIO_2_tri_i(done),   //receive full of 1/8 image needed
	.GPIO_tri_o  (LED[0]), //control led 
	.GPIO_3_tri_o(trigger_limitation),//The clock cycle for the trigger0
	.GPIO_4_tri_o(rst),//reset fpga
	//The control signal to write into block ram
	.addra(addr),//address
	.clka (clk_shift_270),//clock choose to write in clk_shift_270
	.dina (din),//data in
	.wea  (we), //write enable
	//The signal to see in real time for debug
	.clk   (sample_clk),
	.probe0(IMG_FRAME_VAL),
	.probe1(IMG_LINE_VAL),
	.probe2(clk),
	.probe3(clk_shift_270),
	.probe4(din),
	.probe5(signal_know_if_or_not)
);
/***************  Testbench Stuff   
*
*
***********************************/
reg clock      ;
reg reset      ;
reg[9:0] inputPixel ;

assign      IMG_DOUT = inputPixel;
assign      rst      = reset;
assign      clk      = clock;


/****************
*
*
***************  Testbench Stuff   End*/

//Image dimensions 
parameter FRAME_WIDTH  = 1280;
parameter FRAME_HEIGHT = 1024;


/****************************************************************************************

 transferring pixels from Camrea to the ZYNQ
        ///////////////////////////////  Description  \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
//The memory will store each 7-consecutive rows from the image 
//in circular shift rolling way ; so after reading each row after the 7th \
//oldest row will be replaced by the new one l head and tail pointers will kept pointing to the start row 
//and the last row inside the circular memory 
 
//After each stream : store in the data memory 
//then apply the centroiding/median subtracting mixed algorithm 
//Then extract the stars 

 
*****************************************************************************************/
/*********** Prameters *********/
parameter   IMG_STREAM_PIECE = 8960;//Total number of pixels in each captured piece 
 
/*********** Registers *********/
/*********** Memory to hold the stream of images **********/
//Subdivided to save the utilization
reg   [9 :0] mem0[FRAME_WIDTH-1:0];//Data memory to hold the 7 rows of the image 
reg   [9 :0] mem1[FRAME_WIDTH-1:0];reg   [9 :0] mem2[FRAME_WIDTH-1:0];// 
reg   [9 :0] mem3[FRAME_WIDTH-1:0];reg   [9 :0] mem4[FRAME_WIDTH-1:0];//   
reg   [9 :0] mem5[FRAME_WIDTH-1:0];reg   [9 :0] mem6[FRAME_WIDTH-1:0];//  

//11 bit to be able to cover upto 1280 pixels 
//[now addr_DM is the column Number]
reg   [11:0]addr_DM   = 0;
//21-bit cover the whole image (only numbering purpose)
reg   [20:0]addr_Img  = 0;                
//will sent to the block RAM and to be consistent 
//with the maximum width x_weighted_sum of 15 bits width
reg   [14:0]rowNumber = 0; 
reg   [ 2:0] rowIndex  = 0; //rowNumber%7
//Flag to indicate that this is the first piece retirieved from the image  
reg    firstPartOfImage   =1;
/*********** Wires     *********/
//the Data memory input data will be the pixel out of the camera 
wire   transferFromCamera   ;
assign transferFromCamera   = running & IMG_FRAME_VAL & IMG_LINE_VAL ;
/*********** Process *********/
always @(posedge clk)//run at 72 MHz 
begin 
//At the reset , make the address of the data memory = 0
	if (rst) 
	  begin	
		//The address of the pixel inside the circular buffer [7 rows]
		addr_DM <= 0;
		//The actual address of the pixel in the original image
		addr_Img<= 0;
		firstPartOfImage   <= 1;
		rowNumber  <= 0; rowIndex   <=0;
        end
	else
	if (transferFromCamera)
		begin
		    if(addr_Img == (FRAME_HEIGHT*FRAME_WIDTH)) 
			    	begin 
  					    //Next time read the first part of the mew image
						firstPartOfImage <=1;
						rowNumber        <=0; rowIndex<=0; 
					end
	        else 
			   begin 
				 //If the last pixel in the first piece of the image 
			     if (addr_Img == IMG_STREAM_PIECE)
				   	//First part read is finished					   
					   firstPartOfImage   <= 0;			 					 	   
				   				
				//Adjust rowIndex = rowNumber%7 to update the proper memory segment
				 if(addr_DM == FRAME_WIDTH) 
				    begin 
					   rowNumber<= rowNumber+1;
					   addr_DM  <= 0;
					   //circular shift proper update of the row index 
					   if(rowIndex<6)     rowIndex<=rowIndex+1;
					   else               rowIndex<=0;          
					end
					
				if(rowIndex == 0)    mem0[addr_DM]   <= IMG_DOUT;
 		        else if(rowIndex==1) mem1[addr_DM]   <= IMG_DOUT;
				else if(rowIndex==2) mem2[addr_DM]   <= IMG_DOUT;				
			    else if(rowIndex==3) mem3[addr_DM]   <= IMG_DOUT;
				else if(rowIndex==4) mem4[addr_DM]   <= IMG_DOUT;
				else if(rowIndex==5) mem5[addr_DM]   <= IMG_DOUT;
				else if(rowIndex==6) mem6[addr_DM]   <= IMG_DOUT;
				//always update the current address inside the small memory 
				//and the actual address inside the whole image 
				addr_DM <= addr_DM  + 1; 
                addr_Img<= addr_Img + 1;					 
            
					 
				//Update the circular memory head pointer 
				//The constant multiplication by parameter is constant so won't use a multiplier in synthesis
				//it is just MUX and comparators 
				end
        end
end		
 
/****************************************************************************************
// Centroding  
 
*****************************************************************************************/
/*********** Registers *********/
//It is used as REG not PARAMETER as in future it would be customized basen on the image mean -standard deviation 
reg  [9:0]cutoff_Brightness = 100 ;
//Array store the number of the bright pixels 
//each location corresponds to different cutoff (powers of 2)
//i.e. first array element is the total number of pixels of value greater than 2^0
//i.e. second array element is the total number of pixels of value greater than 2^1 and so on
reg [20:0]brightCounts[9:0];//21 bits to account for the case if all pixels values >1 then we need to count 1310720 pixels needs 21 bits

reg [ 9:0] crntPix,center;
//Number of stars  [Bright areas]
reg [ 9:0] num_bright_areas = 0;
//Centroding 
//Sum need 14 bit as 2^10*2^4=2^14 [if all pixels in the background are fully bright]
reg [13:0]background_sum ;
// background_mean needs only 10 bits 
reg [ 9:0 ] background_mean;
reg [ 14:0] background_mean_21;
reg [ 15:0] background_mean_63 ;
// sum needs 15 bits as the maximum value for the centroid sum is  21*2^10 
reg [14:0]centroid_sum   ;
//x_weighted_sum needs 2^6*2^10 (63*max bright pixel value) as ( 63 = 3*1 + 5*2 + 5*3 + 5*4 + 3*5)
reg [15:0]x_weighted_sum ;
//y_weighted_sum needs 2^6*2^10 (63*max bright pixel value) as ( 63 = 3*1 + 5*2 + 5*3 + 5*4 + 3*5)
reg [15:0]y_weighted_sum ;
reg getaddrPhase=0;
/*********** Wires *********/
wire  [12:0]crntAddr   ;
/*********** assign wires *********/
assign  crntAddr = addr_DM - 1;
//Total number of items to be written in the block RAM are 5021
//1000 stars each star need 5 locations totalling 5000 locations:
//     1,2- (x,y coordinates of bright area start 
//     3-   centroid_sum
//     4,5- x_weighted_sum , y_weighted_sum
//and 20 locations for 10 brightCounts 
//         for number of bright pixels greater than some threshold 
//         20 locations as each count is 21 bit will be inserted in 2 locations 
//and number of stars needs one location 


reg [12:0] addr_write_to_Stars_blkRAM = -1; //13-bit can cover upto 8192 locations
assign   addr          = addr_write_to_Stars_blkRAM;
reg  update_stars;
//3 bit enough to hold index may extended from 0 to 5
//index used to specify which item is being sent to block memory now (x,y,x_weighted_sum,y_weighted_sum,centroid_sum)
reg [2:0]index;
//adjR6 holds the values for pixels below current row by 6 [in the bright area]
//adjR6 holds the values for pixels below current row by 5 [in the bright area] and so on 
//each of adjRi is an array of 7 elements each of 10-bits value 
reg [9:0]adjR6[0:6];reg [9:0]adjR5[0:6];
reg [9:0]adjR4[0:6];reg [9:0]adjR3[0:6];
reg [9:0]adjR2[0:6];reg [9:0]adjR1[0:6];
/*********** Variables *********/
integer i;
/*********** Process *********/
always @(posedge sample_clk)
begin
  if(rst)
    begin 
 		//Initalize the number of bright pixels in each category to zeros
		brightCounts[0] <= 0;brightCounts[1] <= 0;brightCounts[2] <= 0;
		brightCounts[3] <= 0;brightCounts[4] <= 0;brightCounts[5] <= 0;
		brightCounts[6] <= 0;brightCounts[7] <= 0;brightCounts[8] <= 0;
		brightCounts[9] <= 0;
        num_bright_areas <= 0;		
		addr_write_to_Stars_blkRAM <= 0;
 		getaddrPhase               <= 1;
		update_stars               <= 0;
		index                      <= 0;
	end	
  else  if (rowNumber>=6 & crntAddr>=6)// 
		//Begin the star extraction   
	    begin 
            // { Begin the extracting - updating the stars memory 							
			begin 
				//extract but not update the stars memory
			  if(!update_stars )
				 //{ Begin check if extracting but not updating stars memory				
			   begin  
				    //to reduce computational complexities 
					//it is enough at first to get the center pixel and current pixel
				if(rowIndex == 0)    begin crntPix <=mem0[crntAddr];center<= mem4[crntAddr-3]; end
				else if(rowIndex==1) begin crntPix <=mem1[crntAddr];center<= mem5[crntAddr-3]; end
				else if(rowIndex==2) begin crntPix <=mem2[crntAddr];center<= mem6[crntAddr-3];end 			
				else if(rowIndex==3) begin crntPix <=mem3[crntAddr];center<= mem0[crntAddr-3]; end
				else if(rowIndex==4) begin crntPix <=mem4[crntAddr];center<= mem1[crntAddr-3]; end
				else if(rowIndex==5) begin crntPix <=mem5[crntAddr];center<= mem2[crntAddr-3]; end 
				else if(rowIndex==6) begin crntPix <=mem6[crntAddr];center<= mem3[crntAddr-3]; end
				//Then update the total number of pixels that excced certain thresold of 2^i 
				if (crntPix>1)  brightCounts[0]<=brightCounts[0]+1;
				if (crntPix>2)  brightCounts[1]<=brightCounts[1]+1;
				if (crntPix>4)  brightCounts[2]<=brightCounts[2]+1;
				if (crntPix>8)  brightCounts[3]<=brightCounts[3]+1;
				if (crntPix>16) brightCounts[4]<=brightCounts[4]+1;
				if (crntPix>32) brightCounts[5]<=brightCounts[5]+1;
				if (crntPix>64) brightCounts[6]<=brightCounts[6]+1;
				if (crntPix>128)brightCounts[7]<=brightCounts[7]+1;
				if (crntPix>256)brightCounts[8]<=brightCounts[8]+1;
				if (crntPix>512)brightCounts[9]<=brightCounts[9]+1;
							  
				//{ begin if  the center value is greater than the cutoff for the brightness
				if(center > cutoff_Brightness)
				begin 
					//Now we need to get the whole bright area 
					//i.e. to fill the adjRi array elements 
					//As we are using a circular buffer 
					//So depending on the current row index 
					//we will extract the bright area 
				
					if(rowIndex == 0) 
					 for(i = 0; i < 7; i = i + 1)
					  begin 
					       adjR1[i] <= mem6[crntAddr-i];
					       adjR2[i] <= mem5[crntAddr-i];adjR3[i] <= mem4[crntAddr-i];
					       adjR4[i] <= mem3[crntAddr-i];adjR5[i] <= mem2[crntAddr-i];
					       adjR6[i] <= mem1[crntAddr-i]; 
					  end
					else if(rowIndex==1)
					      for(i = 0; i < 7; i = i + 1)					
					        begin adjR1[i] <= mem0[crntAddr-i];
					              adjR2[i] <= mem6[crntAddr-i];adjR3[i] <= mem5[crntAddr-i];
					              adjR4[i] <= mem4[crntAddr-i];adjR5[i] <= mem3[crntAddr-i];
					              adjR6[i] <= mem2[crntAddr-i]; 
				            end
					else if(rowIndex==2)
					      for(i = 0; i < 7; i = i + 1)
					        begin adjR1[i] <= mem1[crntAddr-i];
					              adjR2[i] <= mem0[crntAddr-i];adjR3[i] <= mem6[crntAddr-i];
					              adjR4[i] <= mem5[crntAddr-i];adjR5[i] <= mem4[crntAddr-i];
					              adjR6[i] <= mem3[crntAddr-i]; 
					        end
					else if(rowIndex==3)
						  for(i = 0; i < 7; i = i + 1)
					        begin adjR1[i] <= mem2[crntAddr-i];
					              adjR2[i] <= mem1[crntAddr-i];adjR3[i] <= mem0[crntAddr-i];
					              adjR4[i] <= mem6[crntAddr-i];adjR5[i] <= mem5[crntAddr-i];
					              adjR6[i] <= mem4[crntAddr-i]; 
					        end
					else if(rowIndex==4) 
					      for(i = 0; i < 7; i = i + 1)
					        begin adjR1[i] <= mem3[crntAddr-i];
					              adjR2[i] <= mem2[crntAddr-i];adjR3[i] <= mem1[crntAddr-i];
					              adjR4[i] <= mem0[crntAddr-i];adjR5[i] <= mem6[crntAddr-i];
					              adjR6[i] <= mem5[crntAddr-i]; 
					        end
					else if(rowIndex==5)
					      for(i = 0; i < 7; i = i + 1)					
					        begin  adjR1[i] <= mem4[crntAddr-i];
								   adjR2[i] <= mem3[crntAddr-i];adjR3[i] <= mem2[crntAddr-i];
					               adjR4[i] <= mem1[crntAddr-i];adjR5[i] <= mem0[crntAddr-i];
					               adjR6[i] <= mem6[crntAddr-i]; 
					        end
					else if(rowIndex==6)
						  for(i = 0; i < 7; i = i + 1)
					        begin adjR1[i] <= mem5[crntAddr-i];
								  adjR2[i] <= mem4[crntAddr-i];adjR3[i] <= mem3[crntAddr-i];
					              adjR4[i] <= mem2[crntAddr-i];adjR5[i] <= mem1[crntAddr-i];
					              adjR6[i] <= mem0[crntAddr-i]; 
							end
					//
					/*  above <=adjR4[3] ;//-4*FRAME_WIDTH-3
						below <=adjR2[3] ;//-2*FRAME_WIDTH-3
						right <=adjR3[4] ;//-3*FRAME_WIDTH-3-4
						left  <=adjR3[2] ;//-3*FRAME_WIDTH-3-2 */
					if( adjR4[3] >cutoff_Brightness | adjR2[3] >cutoff_Brightness | adjR3[4] > cutoff_Brightness | adjR3[2] > cutoff_Brightness)
						//if any of the adjacentes are bright then proceed 
						if( (center >= adjR3[2] & center >= adjR4[3] & center < adjR2[3] & center <adjR3[4] ))
							begin //{ //centroding 
									  	num_bright_areas       <= num_bright_areas+1;
 										//calculate the sum of pixels used for background subtraction
 																	
										background_sum <= mem0[crntAddr-2]+ mem0[crntAddr-3]+mem0[crntAddr-4];
										background_sum <= background_sum + adjR1[1]+adjR1[5];
									    background_sum <= background_sum + adjR2[0]+adjR2[6];
									    background_sum <= background_sum + adjR3[0]+adjR3[6];
									    background_sum <= background_sum + adjR4[0]+adjR4[6];
									    background_sum <= background_sum + adjR5[1]+adjR5[5];
									    background_sum <= background_sum + adjR6[2]+adjR6[3]+ adjR6[4];
 									    									    
										//calculate average of background pixels (dvision by 16 => shift left four bits) 
										background_mean = background_sum >> 4;
										
										//calculate sum of pixels used for centroiding
										centroid_sum <= adjR1[2]+ adjR1[3]+ adjR1[4];
										centroid_sum <= centroid_sum+adjR2[1]+adjR2[2]+ adjR2[3]+ adjR2[4]+adjR2[5];
										centroid_sum <= centroid_sum+adjR3[1]+adjR3[2]+ adjR3[3]+ adjR3[4]+adjR3[5];
										centroid_sum <= centroid_sum+adjR4[1]+adjR4[2]+ adjR4[3]+ adjR4[4]+adjR4[5];
										centroid_sum <= centroid_sum+adjR5[2]+adjR5[3]+ adjR5[4];
										
										// calculate weighted pixel sums in x dimension
										//any constant multiplication will be done using adders and shift units
										x_weighted_sum <= 						  (adjR1[2]+adjR1[2])+ ( adjR1[3]+(adjR1[3]+adjR1[3]))+ (adjR1[4]<<2);
										x_weighted_sum <= x_weighted_sum+adjR2[1]+(adjR2[2]+adjR2[2])+ ( adjR2[3]+(adjR2[3]+adjR2[3]))+ (adjR2[4]<<2)+(adjR2[5]+(adjR2[5]<<2));
										x_weighted_sum <= x_weighted_sum+adjR3[1]+(adjR3[2]+adjR3[2])+ ( adjR3[3]+(adjR3[3]+adjR3[3]))+ (adjR3[4]<<2)+(adjR3[5]+(adjR3[5]<<2));
										x_weighted_sum <= x_weighted_sum+adjR4[1]+(adjR4[2]+adjR4[2])+ ( adjR4[3]+(adjR4[3]+adjR4[3]))+ (adjR4[4]<<2)+(adjR4[5]+(adjR4[5]<<2));
										x_weighted_sum <= x_weighted_sum+		  (adjR5[2]+adjR5[2])+ ( adjR5[3]+(adjR5[3]+adjR5[3]))+ (adjR5[4]<<2);
										// calculate weighted pixel sums in y dimension
										y_weighted_sum <= (adjR1[2]+(adjR1[2] << 2))+ ( adjR1[3]+(adjR1[3] << 2))+ (adjR1[4]+(adjR1[4]<<2));
										y_weighted_sum <= y_weighted_sum+(adjR2[1]<<2)+(adjR2[2]<<2)+ ( adjR2[3]<<2)+ (adjR2[4]<<2)+(adjR2[5]<<2);
										y_weighted_sum <= y_weighted_sum+(adjR3[1]+(adjR3[1]+adjR3[1]))+(adjR3[2]+(adjR3[2]+adjR3[2]))+ ( adjR3[3]+(adjR3[3]+adjR3[3]))+ (adjR3[4]+(adjR3[4]+adjR3[4]))+(adjR3[5]+(adjR3[5]+adjR3[5]));
										y_weighted_sum <= y_weighted_sum+(adjR4[1]+adjR4[1])+(adjR4[2]+adjR4[2])+ (adjR4[3]+adjR4[3])+ (adjR4[4]+adjR4[4])+(adjR4[5]+adjR4[5]);
										y_weighted_sum <= y_weighted_sum+(adjR5[2]   )+(adjR5[3]   )+ (adjR5[4]   );
										//Subtracting from the background 
										//21 is the count of the pixels in the neighborhod of the centroid
										//21 multiplication will be done through shift and addition 
										background_mean_21 =(background_mean<<4)+(background_mean<<2)+background_mean;
										if(centroid_sum > background_mean_21)
											centroid_sum <= centroid_sum - background_mean_21;
										else 
											centroid_sum <= 0;
										//////////////////////////////////////////
										background_mean_63 =(background_mean<<6)- background_mean;
										if(x_weighted_sum > background_mean_63)
											x_weighted_sum <= x_weighted_sum - background_mean_63;
										else 
											x_weighted_sum <= 0;
										
										if(y_weighted_sum > background_mean_63)
											y_weighted_sum <= y_weighted_sum - background_mean_63;
										else 
											y_weighted_sum <= 0;
										//////////////////////////////////////////
										 update_stars <= 1;
									 //}centroding    
							end
								 //} end if  the center value is greater than the cutoff for the brightness
				end  
							  							 							                          							
					  //} End check if extracting but not updating stars mem0   							 
					  end  
				else //Updating the stars memory 
                  begin //{ Send the stars locations x,y com to the block RAM
					   if(getaddrPhase)
					    begin
						  we   <= 1;getaddrPhase <= 0;
						end 
					   else 
					    begin 
                             if(index== 0) //Send the row number of the bright area start to block RAM
							   begin 
							     addr_write_to_Stars_blkRAM <= addr_write_to_Stars_blkRAM + 1;
							     din <= rowNumber -6 ;//write the y of the bright area start 
							     index <= 1 ;
							   end 
							 else if(index == 1)//Send the column number of the bright area start to block RAM
							   begin 
							     addr_write_to_Stars_blkRAM <= addr_write_to_Stars_blkRAM + 1;
								 din  <= crntAddr -6 ;//write the x of the bright area start 
							     index <= 2 ;
							   end
							 else if(index == 2)//Send the y_weighted_sum of the star to block RAM 
							     begin 
								   addr_write_to_Stars_blkRAM <= addr_write_to_Stars_blkRAM + 1;
								   din <= y_weighted_sum ; 
							       index <= 3 ;
								 end 
								 
							 else if(index == 3)//Send the x_weighted_sum of the star to block RAM
							     begin 
								   addr_write_to_Stars_blkRAM <= addr_write_to_Stars_blkRAM + 1;
								   din <= x_weighted_sum  ;  
							       index <= 4 ;
								 end  
						     else if(index == 4)//Send the centroid_sum of the star to block RAM
							     begin 
								   addr_write_to_Stars_blkRAM <= addr_write_to_Stars_blkRAM + 1;
								   din <= centroid_sum  ; 
							       index <= 0 ;
								   update_stars <= 0;
								 end  
							   
						end 
						//} end Send the stars locations x,y com to the block RAM
					end  				   
		 
            end // }
        end 
 end 

//Define the control signal after receive trigger from zynq
always @(posedge clk)
begin
	//2 delay 1 clock cycle of trigger and IMG_FRAME_VAL
	trigger_temp    <= trigger;
	trigger_temp2   <= trigger_temp;
	frame_val_temp  <= IMG_FRAME_VAL;
	frame_val_temp2 <= frame_val_temp;
	//If see the rising_edge of trigger, assert trigger run, it will be deasserted when done
	if (rst)
		begin
			trigger_run <= 0;
		end
	else
	if (done)
		trigger_run <= 0;
	else   
	if (trigger && !trigger_temp2)
		begin
			trigger_run <= 1;
		end

	//If see the rising_edge of IMG_FRAME_VAL and in trigger run, assert running, it will be deasserted when done  
	//It mean, start to receive pixel from camera 
	if (rst)
		begin
			running <= 0;
		end
	else
	if (done)		running <= 0;
	else         
	if (trigger_run && IMG_FRAME_VAL && !frame_val_temp2)
		running <= 1;

	//Raise done when receive enough pixel of 1 block 1/8 image
	if (rst)
		begin
			done <= 0;
		end
	if (!trigger)
		done <= 0;
	else 
	if (we)
		begin
			if (addr == FRAME_WIDTH*FRAME_HEIGHT-1)
				begin
					done <= 1;
				end
		end

    //Control part to send trigger 0 1 2 to camera   
	if (rst)
		IMG_TRIG <= 0;
	else
	if  (trigger && !trigger_temp2)//See rising edge of trigger, raise 3 IMG_TRIG
		begin
			IMG_TRIG <= 7;
		end
	else     
	if (cnt_trigger == trigger_limitation-1)//counter = trigger_limitation-1, put low in  IMG_TRIG[0]
		IMG_TRIG[0] <= 0;
	else if (cnt_trigger == trigger_limitation/4-1)//counter = trigger_limitation/2-1, put low in  IMG_TRIG[1] (dual slope)
        IMG_TRIG[1] <= 0;
	//else if (cnt_trigger == trigger_limitation*3/4-1)//counter = trigger_limitation/4-1, put low in  IMG_TRIG[2] (triple slope)
    //else if (cnt_trigger == trigger_limitation-1000)//counter = trigger_limitation/4-1, put low in  IMG_TRIG[2] (triple slope)
    //else if (cnt_trigger == 2000)//counter = trigger_limitation/4-1, put low in  IMG_TRIG[2] (triple slope)
    else if (cnt_trigger ==  trigger_limitation/2-1)//counter = trigger_limitation/4-1, put low in  IMG_TRIG[2] (triple slope)
       	IMG_TRIG[2] <= 0;

    //We have counter when IMG_TRIG = 1, when counter reach trigger_limitation-1, come back to 0
	if (rst)
		cnt_trigger <= 0;
	else
	if (IMG_TRIG[0])
		begin
		    signal_know_if_or_not <= 1;
			if (cnt_trigger == trigger_limitation-1)
				cnt_trigger <= 0;
			else   
				cnt_trigger <= cnt_trigger + 1;
		end
end


//Calculate some debug signal, dont need to care
reg [31:0] test_frame=0, test_part=0;
reg cnt_frame=0, cnt_part=0, line_val_temp;
always @(posedge clk)
begin
	line_val_temp <= IMG_LINE_VAL;
	if (IMG_FRAME_VAL && !frame_val_temp)
		begin
			cnt_frame <= 1;
			test_frame <= 0;
		end
	else 
	if (!IMG_FRAME_VAL)
		begin
			cnt_frame <= 0;
		end
	else    
	if (cnt_frame)
		test_frame <= test_frame + 1;

	if (running && IMG_LINE_VAL && !line_val_temp)
		begin
			cnt_part <= 1;
			test_part <= 0;
		end
	else 		
	if (done)
		begin
			cnt_part <= 0;
		end
	else
	if (cnt_part)
		test_part <= test_part + 1;
end

/*****************************************************************************
****  Testbench Start 
****
******************************************************************************/
	//============================================================================
	 
	integer ifile1;
	integer ofile1,ofile2,ofile3;
	(* keep_hierarchy = "yes" *)
	
	initial
	begin
		//====================================================================
		ifile1  = $fopen("inputImage.txt","r");
		if (ifile1 == `NULL) begin
			$display("ifile1 handle was NULL [Input file doesn't exist]");
			$finish;
		end
		//====================================================================
		ofile1 = $fopen("outputImage.txt","w");	// The pixels read to confirm proper read [Debug]
		ofile2 = $fopen("starCoordinates.txt","w");	// output of pixels
		ofile3 = $fopen("totBrightness.txt","w");	// output of encoder
		//====================================================================
		 
		clock			    <=   0 ;//Initalize the clock to be zero  	 
		forever #1 clock	<= !clock;//after each time step toggle the clock
		//====================================================================
		reset 		<= 1;//Initalize the reset 
		#3 reset 	<= 0;//After three time steps deactivate the reset 
		//====================================================================
		
		#1310720  $stop;//Stop the simulation after 1280*1024 timesteps - we may need extra time steps after read the image 
		
	end
	
	always @(posedge clk) begin
		$fscanf(ifile1, "%d\n", inputPixel); 
		if (!$feof(ifile1)) begin
			//use captured_data as you would any other wire or reg value;
			$fwrite(ofile1,"%d\n",inputPixel);
	end
end

/*****************************************************************************
****  Testbench  End
****
******************************************************************************/

endmodule
