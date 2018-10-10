module Star_Extract_IP#(parameter FRAME_WIDTH  = 1280,parameter FRAME_HEIGHT = 1024)
(

    input rst,
	/********************************************************************
						Camera Interfacing I/O Ports
	*********************************************************************/
	/*The PYTHON 1300 CMOS output	configuration mode P2
	  10âˆ’bit parallel CMOS pixel data output 
	  â€˜frame validâ€™ and â€˜line validâ€™ CMOS output signals
	*/
	input IMG_CLK_OUT    ,//Input clock to the IP from the Camera [72 MHz at the P2-SN/SE Mode]
	//input clock = 72 MHz 
	//in 10-bit mode the data rate is 720Mbps 
	input [9:0] IMG_DOUT ,//The 10-bit pixel value 
	input IMG_FRAME_VAL  ,
	//Frame frame_valid asserted at the start of a new frame 
	//and remains asserted until the last line of the frame is completely transmitted.
	input IMG_LINE_VAL  , //line_valid iis asserted when the data channels contain valid pixel data.
	/*********************************************************************/
	
	/********************************************************************
						Block Memory Interfacing I/O Ports
	*********************************************************************/
	output reg   we  , //write enable of the block memory 
	reg    [ 9:0]din ,
	output [17:0]addr,
	output   sample_clk,//Output clock to the block RAM [4 times faster than input clock rate]

	/********************************************************************
						User Control/Status Interfacing I/O Ports
	*********************************************************************/
	
	output [0:0]LED 					,//LED output indicates finish of the algorithm execution for the current frame 
	output reg finish_star_extract_IP	,//Indicates the finish of the star extraction for the current frame    
	output reg [2:0] IMG_TRIG,
	input [1:0] IMG_MON 
	
    
); 

/********************************************************************
*********************************************************************
						Clock Management Block Instantiation
Purpose : Multiply the Camera output clock source by 4 
		  to provide faster clock for the algorithm run 
*********************************************************************
********************************************************************/
wire  clk;
wire  clk_shift_270;
wire locked;
reg running = 0;
clk_wiz_0 clk_wiz_0
(
	// Clock out ports
	.clk_out1(clk),     // output clk_out1
	.clk_out2(clk_shift_270),     // output clk_out2 //shift 270 for phase to latch data
	.clk_out3(sample_clk),     // output clk_out3
	// Clock in ports
	.clk_in1(IMG_CLK_OUT), // input clk_in1
	//Output indicates that the clocking is locked now and is stable to be used
	.locked(locked) 
	
);  
always @(posedge clk && locked)
begin 
	running = 1;
	//Ensure that positive edge of clk happens before the shifted version positive edge
	//to make the transfer from camera to register happens at first 
	//then transfer from register to memory happens next 
end 

reg   [31:0]frame_addr=0, start_addr=0;
wire  [31:0]end_addr;



reg   frame_val_temp, frame_val_temp2;
reg   done=0, trigger_run=0;
 
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
parameter    streamPieces    = 1018;//Scan the image Seven rows and roll [So we will capture 1024-7+1 pieces] 
parameter   IMG_STREAM_PIECE = 8960;//Total number of pixels in each captured piece 
parameter FRAMEWIDTH_2 =2560;
parameter FRAMEWIDTH_3 =3840,FRAMEWIDTH_4 =5120  ; 
parameter FRAMEWIDTH_5 =6400,FRAMEWIDTH_6 =7680   ; 
/*********** Registers *********/
reg   [9 :0] memory[IMG_STREAM_PIECE-1:0];//Data memory to hold the 7 rows of the image 
reg   [14:0]addr_DM   = 0;             //13 bit to be able to cover upto 8960 pixel [2^13=8192 and 8^14=16384] 
reg   [20:0]addr_Img  = 0;             //21-bit cover the whole image ( only numbering purpose )
reg   [14:0]rowNumber = 0; //will sent to the block RAM and to be consistent with the maximum width x_weighted_sum of 15 bits width
reg   [14:0]colNumber = 0; //Even that the col,row need only at max 11 bits to store the maximum 1280 (max col number)
reg   [14:0] headPointer = 0;
reg   [14:0]tailPointer  = 0;
reg   firstPartOfImage   = 1;//Flag to indicate that this is the first piece retirieved from the image 
/*********** Wires     *********/
wire   transferFromCamera   ;//the Data memory input data will be the pixel out of the camera 
assign transferFromCamera   =  running&IMG_FRAME_VAL & IMG_LINE_VAL ;
/*********** following registers & wires are needed for centroiding algorithm     ******/
/*********** but is calculated while transferring the previous pixel to save time *****/
reg [14:0]FRAME_WIDTH_1  ,FRAME_WIDTH_2  ,FRAME_WIDTH_3  ;
reg [14:0]FRAME_WIDTH_4  ,FRAME_WIDTH_5  ,FRAME_WIDTH_6  ; 

/*********** Process *********/
always @(posedge clk_shift_270 && locked )//run at 72 MHz 
begin 
	
	if (transferFromCamera)
		begin
		    if(addr_Img == (FRAME_HEIGHT*FRAME_WIDTH)-1)//Final pixel 
			    	  firstPartOfImage <=1;//Next time read the first part of the mew image 
	        else 
			   begin 
			   
			     if (addr_DM == IMG_STREAM_PIECE)//If the last pixel in the stream of the image 
			        begin 
						memory[0]          <= IMG_DOUT;					
         				addr_DM            <= 1;          //circular shift 
					    firstPartOfImage   <= 0;			 //First part read is finished	
				 	   
				    end
				 else 
				    begin 
						memory[addr_DM]   <= IMG_DOUT;
						addr_DM <= addr_DM  + 1;//Otherwise update the address 
					end 
					
				addr_Img <= addr_Img + 1;					 
                
                if(colNumber == FRAME_WIDTH-1)//make the next colNumber=0 if we reached the last column 
					 colNumber=0;
				else 
                     colNumber<= colNumber + 1;
					 
				//Update the circular memory head pointer 
				//The constant multiplication by parameter is constant so won't use a multiplier in synthesis
				//it is just MUX and comparators 
				if(!firstPartOfImage)
                    begin				
					/* for(i=1;i<7;i++)
						if(addr_DM== (FRAME_WIDTH*(i-1))
							begin 
							   headPointer <= FRAME_WIDTH*i;
							   tailPointer <= FRAME_WIDTH*(i-1); 
							   rowNumber   <= rowNumber+1;   
							end		 */				   
					//Unroll the for loop to avoid variable multiplication 
					if(addr_DM==IMG_STREAM_PIECE)
					 begin 
					 headPointer <= FRAME_WIDTH ;tailPointer <=0;rowNumber<= rowNumber+1;   
					 end	
					if(addr_DM==FRAME_WIDTH)
					 begin 
					 headPointer <= FRAMEWIDTH_2 ;tailPointer <=FRAME_WIDTH;rowNumber<= rowNumber+1;   
					 end		
					if(addr_DM==FRAMEWIDTH_2)
					 begin 
					 headPointer <= FRAMEWIDTH_3 ;tailPointer <=FRAMEWIDTH_2;rowNumber<= rowNumber+1;   
					 end
					if(addr_DM==FRAMEWIDTH_3)
					 begin 
					 headPointer <= FRAMEWIDTH_4 ;tailPointer <=FRAMEWIDTH_3;rowNumber<= rowNumber+1;   
					 end
					if(addr_DM==FRAMEWIDTH_4)
					 begin 
					 headPointer <= FRAMEWIDTH_5 ;tailPointer <=FRAMEWIDTH_4;rowNumber<= rowNumber+1;   
					 end 
					if(addr_DM==FRAMEWIDTH_5)
					 begin 
					 headPointer <= FRAMEWIDTH_6 ;tailPointer <=FRAMEWIDTH_5;rowNumber<= rowNumber+1;   
					 end
					if(addr_DM==FRAMEWIDTH_6)
					 begin 
					 headPointer <= 0            ;tailPointer <=FRAMEWIDTH_6;rowNumber<= rowNumber+1;   
					 end
                    end	
				else 
				begin 
						if(addr_DM==FRAME_WIDTH)
							rowNumber <= 1;						
						else 
						begin 
							if(addr_DM==FRAMEWIDTH_2)
								rowNumber <= 2;
							else 
							begin 
								if(addr_DM==FRAMEWIDTH_3)
									rowNumber <= 3;
								else
								begin
									if(addr_DM==FRAMEWIDTH_4)
										rowNumber <= 4;
									else
									begin
										
										if(addr_DM==FRAMEWIDTH_5)
											rowNumber <= 5;
										else
										begin 
											if(addr_DM==FRAMEWIDTH_6)
												rowNumber <= 6;
											else
											begin 
												if(addr_DM==IMG_STREAM_PIECE)
													rowNumber <= 7;
											end
										end 								
									end 
								end 
							end
						end 
			    end
			
				//////////////////////////////////////////////////
				////////// Neighbors pixels offsets calculation 
				////////// Prior to centroiding algorithm to save time 
				if (rowNumber>=6 & colNumber>=6)
				begin 					 		
					//for center 
					if(addr_DM<FRAMEWIDTH_3) 
								FRAME_WIDTH_3<=FRAMEWIDTH_3-IMG_STREAM_PIECE;
					else 
									FRAME_WIDTH_3 <=3840;		
					//for above 
					if(addr_DM<FRAMEWIDTH_4) 
							 		FRAME_WIDTH_4<=FRAMEWIDTH_4-IMG_STREAM_PIECE;	
					else 
									FRAME_WIDTH_4 <=5120;
					//for below 			
					if(addr_DM<FRAMEWIDTH_2) 
									FRAME_WIDTH_2<=FRAMEWIDTH_2-IMG_STREAM_PIECE;	
					else 
									FRAME_WIDTH_2 <=2560;
					//rest neighbors 
					if(addr_DM<FRAMEWIDTH_5) 
									FRAME_WIDTH_5<=FRAMEWIDTH_5-IMG_STREAM_PIECE;	
					else 
									FRAME_WIDTH_5 <=6400;
					
					if(addr_DM<FRAMEWIDTH_6) 
									FRAME_WIDTH_6<=FRAMEWIDTH_6-IMG_STREAM_PIECE;	
					else 
									FRAME_WIDTH_6 <=7680;
					
					if(addr_DM<FRAME_WIDTH) 
									FRAME_WIDTH_1<=FRAME_WIDTH-IMG_STREAM_PIECE;	
					else 
									FRAME_WIDTH_1 <=1280;
				end 
				
			end
		end	
end		
 
/****************************************************************************************
// Centroding  
*****************************************************************************************/

/*********** Registers *********/
reg  [9:0]cutoff_Brightness = 100 ;//It is used as REG not PARAMETER as in future it would be customized basen on the image mean -standard deviation 
reg  [9:0]brightCounts[9:0];//
reg [9:0] num_bright_areas = 0;//Number of stars  [Bright areas]
//Centroding 
reg [9:0] background_mean_21,background_mean_63 ;// background_mean needs only 10 bits 
reg getaddrPhase=0;


reg [17:0] addr_write_to_Stars_blkRAM = 0;
reg  update_stars;
reg [2:0]index;//3 bit enough to hold index may extended from 0 to 5
//index used to specify which item is being sent to block memory now (x,y,x_weighted_sum,y_weighted_sum,centroid_sum)
/************ wires ************/
//rather than make them as registers 
assign   addr          = addr_write_to_Stars_blkRAM;
                         
                        
                        
wire  [14:0]crnt_Star_addr   ;
assign  crnt_Star_addr = addr_DM;

wire  [9:0] center,above,below,right,left;
wire [14:0]aboveAddr = crnt_Star_addr-FRAME_WIDTH_4-3;
wire [14:0]belowAddr = crnt_Star_addr-FRAME_WIDTH_2-3;
wire [14:0]centerAddr = crnt_Star_addr-FRAME_WIDTH_2;
assign	center= memory[centerAddr];      //-3*FRAME_WIDTH-3
assign above  = memory[aboveAddr];//-4*FRAME_WIDTH-3
assign below  = memory[belowAddr];//-2*FRAME_WIDTH-3
assign left   = memory[crnt_Star_addr-FRAME_WIDTH_3-4];//-3*FRAME_WIDTH-3-1
assign right  = memory[crnt_Star_addr-FRAME_WIDTH_3-2];//-3*FRAME_WIDTH-3+1

/***********   background_sum wire to enhance the performance rather than register accumulator **********/
wire [13:0]background_sum ;//Sum need 14 bit as 2^10*2^4=2^14 [if all pixels in the background are fully bright]
//calculate the sum of pixels used for background subtraction
assign background_sum = memory[crnt_Star_addr-2]+ memory[crnt_Star_addr-3]+memory[crnt_Star_addr-4]
+ memory[crnt_Star_addr-FRAME_WIDTH_1-1]+memory[crnt_Star_addr-FRAME_WIDTH_1-5]
+ memory[crnt_Star_addr-FRAME_WIDTH_2]+memory[crnt_Star_addr-FRAME_WIDTH_2-6]
+ memory[crnt_Star_addr-FRAME_WIDTH_3]+memory[crnt_Star_addr-FRAME_WIDTH_3-6]
+ memory[crnt_Star_addr-FRAME_WIDTH_4]+memory[crnt_Star_addr-FRAME_WIDTH_4-6]
+ memory[crnt_Star_addr-FRAME_WIDTH_5-1]+memory[crnt_Star_addr-FRAME_WIDTH_5-5]
+  memory[crnt_Star_addr-FRAME_WIDTH_6-2]+ memory[crnt_Star_addr-FRAME_WIDTH_6-3]+ memory[crnt_Star_addr-FRAME_WIDTH_6-4];
/***********   centroid_sum wire to enhance the performance rather than register accumulator **********/
wire [14:0]centroid_sum   ;// sum needs 15 bits as the maximum value for the centroid sum is  21*2^10 
//calculate sum of pixels used for centroiding									
assign centroid_sum  = memory[crnt_Star_addr-FRAME_WIDTH_1-2]+ memory[crnt_Star_addr-FRAME_WIDTH_1-3]+ memory[crnt_Star_addr-FRAME_WIDTH_1-4]
+memory[crnt_Star_addr-FRAME_WIDTH_2-1]+memory[crnt_Star_addr-FRAME_WIDTH_2-2]+ memory[crnt_Star_addr-FRAME_WIDTH_2-3]+ memory[crnt_Star_addr-FRAME_WIDTH_2-4]+memory[crnt_Star_addr-FRAME_WIDTH_2-5]
+memory[crnt_Star_addr-FRAME_WIDTH_3-1]+memory[crnt_Star_addr-FRAME_WIDTH_3-2]+ memory[crnt_Star_addr-FRAME_WIDTH_3-3]+ memory[crnt_Star_addr-FRAME_WIDTH_3-4]+memory[crnt_Star_addr-FRAME_WIDTH_3-5]
+memory[crnt_Star_addr-FRAME_WIDTH_4-1]+memory[crnt_Star_addr-FRAME_WIDTH_4-2]+ memory[crnt_Star_addr-FRAME_WIDTH_4-3]+ memory[crnt_Star_addr-FRAME_WIDTH_4-4]+memory[crnt_Star_addr-FRAME_WIDTH_4-5]
+memory[crnt_Star_addr-FRAME_WIDTH_5-2]+ memory[crnt_Star_addr-FRAME_WIDTH_5-3]+ memory[crnt_Star_addr-FRAME_WIDTH_5-4];
/***********   centroid_sum wire to enhance the performance rather than register accumulator **********/
wire [15:0]x_weighted_sum ;//x_weighted_sum needs 2^6*2^10 (63*max bright pixel value) as ( 63 = 3*1 + 5*2 + 5*3 + 5*4 + 3*5)
wire [15:0]y_weighted_sum ;//y_weighted_sum needs 2^6*2^10 (63*max bright pixel value) as ( 63 = 3*1 + 5*2 + 5*3 + 5*4 + 3*5)
// calculate weighted pixel sums in x dimension
//any constant multiplication will be done using adders and shift units
assign x_weighted_sum  = (memory[crnt_Star_addr-FRAME_WIDTH_1-2] << 1)+ ( memory[crnt_Star_addr-FRAME_WIDTH_1-3]+(memory[crnt_Star_addr-FRAME_WIDTH_1-3] << 1))+ (memory[crnt_Star_addr-FRAME_WIDTH_1-4]<<2)
+memory[crnt_Star_addr-FRAME_WIDTH_2-1]+(memory[crnt_Star_addr-FRAME_WIDTH_2-2]<<1)+ ( memory[crnt_Star_addr-FRAME_WIDTH_2-3]+(memory[crnt_Star_addr-FRAME_WIDTH_2-3]<<1))+ (memory[crnt_Star_addr-FRAME_WIDTH_2-4]<<2)+(memory[crnt_Star_addr-FRAME_WIDTH_2-5]+(memory[crnt_Star_addr-FRAME_WIDTH_2-5]<<2))
+memory[crnt_Star_addr-FRAME_WIDTH_3-1]+(memory[crnt_Star_addr-FRAME_WIDTH_3-2]<<1)+ ( memory[crnt_Star_addr-FRAME_WIDTH_3-3]+(memory[crnt_Star_addr-FRAME_WIDTH_3-3]<<1))+ (memory[crnt_Star_addr-FRAME_WIDTH_3-4]<<2)+(memory[crnt_Star_addr-FRAME_WIDTH_3-5]+(memory[crnt_Star_addr-FRAME_WIDTH_3-5]<<2))
+memory[crnt_Star_addr-FRAME_WIDTH_4-1]+(memory[crnt_Star_addr-FRAME_WIDTH_4-2]<<1)+ ( memory[crnt_Star_addr-FRAME_WIDTH_4-3]+(memory[crnt_Star_addr-FRAME_WIDTH_4-3]<<1))+ (memory[crnt_Star_addr-FRAME_WIDTH_4-4]<<2)+(memory[crnt_Star_addr-FRAME_WIDTH_4-5]+(memory[crnt_Star_addr-FRAME_WIDTH_4-5]<<2))
+(memory[crnt_Star_addr-FRAME_WIDTH_5-2]<<2)+ (memory[crnt_Star_addr-FRAME_WIDTH_5-3]+(memory[crnt_Star_addr-FRAME_WIDTH_5-3]<<1))+ (memory[crnt_Star_addr-FRAME_WIDTH_5-4]<<2);
// calculate weighted pixel sums in y dimension
assign	y_weighted_sum  = (memory[crnt_Star_addr-FRAME_WIDTH_1-2]+(memory[crnt_Star_addr-FRAME_WIDTH_1-2] << 2))+ ( memory[crnt_Star_addr-FRAME_WIDTH_1-3]+(memory[crnt_Star_addr-FRAME_WIDTH_1-3] << 2))+ (memory[crnt_Star_addr-FRAME_WIDTH_1-4]+(memory[crnt_Star_addr-FRAME_WIDTH_1-4]<<2))
+(memory[crnt_Star_addr-FRAME_WIDTH_2-1]<<2)+(memory[crnt_Star_addr-FRAME_WIDTH_2-2]<<2)+ ( memory[crnt_Star_addr-FRAME_WIDTH_2-3]<<2)+ (memory[crnt_Star_addr-FRAME_WIDTH_2-4]<<2)+(memory[crnt_Star_addr-FRAME_WIDTH_2-5]<<2)
+(memory[crnt_Star_addr-FRAME_WIDTH_3-1]+(memory[crnt_Star_addr-FRAME_WIDTH_3-1]<<1))+(memory[crnt_Star_addr-FRAME_WIDTH_3-2]+(memory[crnt_Star_addr-FRAME_WIDTH_3-2]<<1))+ ( memory[crnt_Star_addr-FRAME_WIDTH_3-3]+(memory[crnt_Star_addr-FRAME_WIDTH_3-3]<<1))+ (memory[crnt_Star_addr-FRAME_WIDTH_3-4]+(memory[crnt_Star_addr-FRAME_WIDTH_3-4]<<1))+(memory[crnt_Star_addr-FRAME_WIDTH_3-5]+(memory[crnt_Star_addr-FRAME_WIDTH_3-5]<<1))
+(memory[crnt_Star_addr-FRAME_WIDTH_4-1]<<1)+(memory[crnt_Star_addr-FRAME_WIDTH_4-2]<<1)+ (memory[crnt_Star_addr-FRAME_WIDTH_4-3]<<1)+ (memory[crnt_Star_addr-FRAME_WIDTH_4-4]<<1)+(memory[crnt_Star_addr-FRAME_WIDTH_4-5]<<1)
+(memory[crnt_Star_addr-FRAME_WIDTH_5-2]   )+(memory[crnt_Star_addr-FRAME_WIDTH_5-3]   )+ (memory[crnt_Star_addr-FRAME_WIDTH_5-4]   );
										
										
										

always @(posedge sample_clk && locked)
begin
    if (rowNumber>=6 & colNumber>=6)//  
	    begin //Begin the star extraction 
            begin // { Begin the extracting - updating the stars memory 			
				if(!update_stars )//extract but not update the stars memory 
				 begin //{ Begin check if extracting but not updating stars memory 
				   
						  
							 if(memory[crnt_Star_addr ] > 1)
							   brightCounts[0]<=brightCounts[0]+1;
							 if (memory[crnt_Star_addr]>2)
							   brightCounts[1]<=brightCounts[1]+1;
							 if (memory[crnt_Star_addr]>4)
							   brightCounts[2]<=brightCounts[2]+1;
							 if (memory[crnt_Star_addr]>8)
							   brightCounts[3]<=brightCounts[3]+1;
							 if (memory[crnt_Star_addr]>16)
							   brightCounts[4]<=brightCounts[4]+1;
							 if (memory[crnt_Star_addr]>32)
							   brightCounts[5]<=brightCounts[5]+1;
							 if (memory[crnt_Star_addr]>64)
							   brightCounts[6]<=brightCounts[6]+1;
							 if (memory[crnt_Star_addr]>128)
							   brightCounts[7]<=brightCounts[7]+1;
							 if (memory[crnt_Star_addr]>256)
							   brightCounts[8]<=brightCounts[8]+1;
							 if (memory[crnt_Star_addr]>512)
							   brightCounts[9]<=brightCounts[9]+1;
							  
							 //Get the center pixel
							//Update the parameters for the circular shift 
							
							 
						
							 if( center > cutoff_Brightness)
							    begin //{ begin if  the center value is greater than the cutoff for the brightness
							      //The above center 
								  
							    if( above >cutoff_Brightness | below >cutoff_Brightness | right > cutoff_Brightness | left > cutoff_Brightness)
								  //if any of the adjacentes are bright then proceed 
								    if( (center >= left & center >= above & center < below & center <right ))
									  begin //{ //centroding 
									  	num_bright_areas       <= num_bright_areas+1;													 									    									    
										//calculate average of background pixels (dvision by 16 => shift left four bits) 
										//Subtracting from the background 
										//21 is the count of the pixels in the neighborhod of the centroid
										//21 multiplication will be done through shift and addition 
										
										/* Simplify the following : 
										background_mean = background_sum >> 4;																																								
										background_mean_21 =(background_mean<<4)+(background_mean<<2)+background_mean;
										into:*/
										background_mean_21 =( background_sum)+( background_sum >>2)  +( background_sum >>4);
										
										/* and Simplify the following : 
										background_mean = background_sum >> 4;																																								
										background_mean_63 =(background_mean<<6)- background_mean;
										into:*/
										background_mean_63 =( background_sum <<2)- ( background_sum>> 4);
										
										//////////////////////////////////////////
										 update_stars <= 1;
									   
								     end//}centroding
								 end  //} end if  the center value is greater than the cutoff for the brightness
							  							 							                          							
						//FRAME_WIDTH_3 =3840;//Reset FRAME_WIDTH_3
  						  							 
					  end //} End check if extracting but not updating stars memory 
				else //Updating the stars memory 
                    begin //{ Send the stars locations x,y com to the block RAM
					   if(getaddrPhase)
					    begin
						  we           <= 1;getaddrPhase <= 0;
						end 
					   else 
					    begin 
                             if(index== 0)
							   begin 
							     din <= rowNumber -6 ;//write the x of the bright area start 
							     addr_write_to_Stars_blkRAM <= addr_write_to_Stars_blkRAM + 1;
							     index <= 1 ;
							   end 
							 else if(index == 1)
							   begin 
							     din <= colNumber -6 ;//write the y of the bright area start 
							     addr_write_to_Stars_blkRAM <= addr_write_to_Stars_blkRAM + 1;
								 index <= 2 ;
							   end
							  else if(index == 2) 
							     begin 
									//Update and write the actual y_weighted_sum
									if(y_weighted_sum > background_mean_63)
											din <= y_weighted_sum - background_mean_63;
									else 
											din <= 0;
							         addr_write_to_Stars_blkRAM <= addr_write_to_Stars_blkRAM + 1;
								     index <= 3 ;
								 end 
								 
							  else if(index == 3)
							     begin
										//Update and write the actual x_weighted_sum
										if(x_weighted_sum > background_mean_63)
											din <= x_weighted_sum - background_mean_63;
										else 
											din <= 0;
										addr_write_to_Stars_blkRAM <= addr_write_to_Stars_blkRAM + 1;
										index <= 4 ;
								 end  
						      else if(index == 4)
							     begin 
								   din <= centroid_sum    ;//write the centroid_sum 
							       addr_write_to_Stars_blkRAM <= addr_write_to_Stars_blkRAM + 1;
								   index <= 0 ;
								   update_stars <= 0;
								   
								 end  
							   
						end 
					end  //} end Send the stars locations x,y com to the block RAM				   
		 
                end // }
            end 
  		
 end 

//all resets 
always @( posedge IMG_CLK_OUT)
begin 
	if (rst) //At the reset , make the address of the data memory = 0
	  begin	
	
	addr_DM <= 0;//The address of the pixel inside the circular buffer [7 rows]
		addr_Img<= 0;//The actual address of the pixel in the original image
		firstPartOfImage   <= 1;
		rowNumber  <= 0; colNumber   <= 0;
		headPointer<= 0; tailPointer <= FRAME_WIDTH_6;//Pointers to deal with the circular buffer
  
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
		FRAME_WIDTH_1 <=1280;
		FRAME_WIDTH_2 <=2560;
		FRAME_WIDTH_3 <=3840;
		FRAME_WIDTH_4 <=5120;
		FRAME_WIDTH_5 <=6400;
		FRAME_WIDTH_6 <=7680; 
		din           <=   0;
		background_mean_21<=0;
		background_mean_63<=0;
	end	
  
  
end

endmodule

