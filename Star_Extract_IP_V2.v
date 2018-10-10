module Star_Extract_IP#(parameter FRAME_WIDTH  = 1280,parameter FRAME_HEIGHT = 1024)
(

    input rst,
	/********************************************************************
						Camera Interfacing I/O Ports
	*********************************************************************/
	/*The PYTHON 1300 CMOS output	configuration mode P2
	  10-bit parallel CMOS pixel data output 
	  +frame valid  and line valid  CMOS output signals
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
	[Could be sent one by one to the ARM rather than store them in block RAM]
	*********************************************************************/
	output reg   we  , //write enable of the block memory 
	//RAM will store the row,col of the star,centroid sum,weighted x and weighted y coordinate sum
	//This needs 16 bit to be consistent with maximum width of weighted x,y sum 
	reg    [15:0]din , 
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
(* keep_hierarchy = "yes" *)
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
clk_wiz_0 Clk_Management_Unit1
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
	running <= 1;
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
(* ram_style = "distributed" *)  reg   [9 :0] memory0[FRAME_WIDTH-1:0];//Data memory to hold the 7 rows of the image 
(* ram_style = "distributed" *)  reg   [9 :0] memory1[FRAME_WIDTH-1:0];//Data memory to hold the 7 rows of the image 
(* ram_style = "distributed" *)  reg   [9 :0] memory2[FRAME_WIDTH-1:0];//Data memory to hold the 7 rows of the image 
(* ram_style = "distributed" *)  reg   [9 :0] memory3[FRAME_WIDTH-1:0];//Data memory to hold the 7 rows of the image 
(* ram_style = "distributed" *)  reg   [9 :0] memory4[FRAME_WIDTH-1:0];//Data memory to hold the 7 rows of the image 
(* ram_style = "distributed" *)  reg   [9 :0] memory5[FRAME_WIDTH-1:0];//Data memory to hold the 7 rows of the image 
(* ram_style = "distributed" *)  reg   [9 :0] memory6[FRAME_WIDTH-1:0];//Data memory to hold the 7 rows of the image 
reg [2:0]  memNum = 0;
reg [2:0]  Ind    = 0;
reg pix00,pix01,pix02,pix03,pix04,pix05,pix06;
reg pix10,pix11,pix12,pix13,pix14,pix15,pix16;
reg pix20,pix21,pix22,pix23,pix24,pix25,pix26;
reg pix30,pix31,pix32,pix33,pix34,pix35,pix36;
reg pix40,pix41,pix42,pix43,pix44,pix45,pix46;
reg pix50,pix51,pix52,pix53,pix54,pix55,pix56;
reg pix60,pix61,pix62,pix63,pix64,pix65,pix66;


reg   [11:0]addr_LM   = 0; //11-bit to be able to index upto 1280 pixel per line 
reg   [14:0]addr_DM   = 0; //14 bit to be able to cover upto 8960 pixel [2^13=8192 and 8^14=16384] 
reg   [20:0]addr_Img  = 0; //21-bit cover the whole image ( only numbering purpose )
reg   [14:0]rowNumber = 0; //will sent to the block RAM and to be consistent with the maximum width x_weighted_sum of 15 bits width
reg   [14:0]colNumber = 0; //Even that the col,row need only at max 11 bits to store the maximum 1280 (max col number)

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
	if (rst) //At the reset , make the address of the data memory = 0
	  begin	
		memNum        <=3'b0;
		Ind           <=3'b0;
		FRAME_WIDTH_1 <=1280;
		FRAME_WIDTH_2 <=2560;
		FRAME_WIDTH_3 <=3840;
		FRAME_WIDTH_4 <=5120;
		FRAME_WIDTH_5 <=6400;
		FRAME_WIDTH_6 <=7680; 
		addr_LM <= 0;//The address of the pixel inside the line memory 
	    addr_DM <= 0;//The address of the pixel inside the circular buffer [7 rows]
		addr_Img<= 0;//The actual address of the pixel in the original image
		firstPartOfImage   <= 1;
		rowNumber  <= 0; colNumber   <= 0;
	 end
	else 
	begin
	if (transferFromCamera)
		begin
		    if(addr_Img == (FRAME_HEIGHT*FRAME_WIDTH)-1)//Final pixel 
			    	  firstPartOfImage <=1;//Next time read the first part of the mew image 
	        else 
			   begin 
			   
			     if(addr_LM==FRAME_WIDTH-1)  
					addr_LM<=0;
				 else 
					addr_LM <= addr_LM+1;
					
			     if (addr_DM == IMG_STREAM_PIECE)//If the last pixel in the stream of the image 
			        begin 
						addr_DM             <= 1;          //circular shift 
					    firstPartOfImage    <= 0;			 //First part read is finished	
				 	   
				    end
				 else 
					addr_DM <= addr_DM  + 1;//Otherwise update the address 
				     
				if(memNum==0)memory0[addr_LM]   <= IMG_DOUT;
				if(memNum==1)memory1[addr_LM]   <= IMG_DOUT;
				if(memNum==2)memory2[addr_LM]   <= IMG_DOUT;
				if(memNum==3)memory3[addr_LM]   <= IMG_DOUT;
				if(memNum==4)memory4[addr_LM]   <= IMG_DOUT;
				if(memNum==5)memory5[addr_LM]   <= IMG_DOUT;
				if(memNum==6)memory6[addr_LM]   <= IMG_DOUT;
						
				
					
				addr_Img <= addr_Img + 1;					 
                
                if(colNumber == FRAME_WIDTH-1)//make the next colNumber=0 if we reached the last column 
					 colNumber=0;
				else 
                     colNumber<= colNumber + 1;
				if(Ind == 6)
					Ind=0;
				else 
					Ind<=Ind+1;
				//Update the circular memory head pointer 
				//The constant multiplication by parameter is constant so won't use a multiplier in synthesis
				//it is just MUX and comparators 
				    if(addr_DM==IMG_STREAM_PIECE)begin  rowNumber<= rowNumber+1;memNum<=0;   end	
					if(addr_DM==FRAME_WIDTH)	 begin  rowNumber<= rowNumber+1;memNum<=1;   end		
					if(addr_DM==FRAMEWIDTH_2)	 begin 	rowNumber<= rowNumber+1;memNum<=2;   end
					if(addr_DM==FRAMEWIDTH_3)	 begin 	rowNumber<= rowNumber+1;memNum<=3;   end
					if(addr_DM==FRAMEWIDTH_4)	 begin 	rowNumber<= rowNumber+1;memNum<=4;   end 
					if(addr_DM==FRAMEWIDTH_5)	 begin 	rowNumber<= rowNumber+1;memNum<=5;   end
					if(addr_DM==FRAMEWIDTH_6)	 begin 	rowNumber<= rowNumber+1;memNum<=6;   end
                     	
				//////////////////////////////////////////////////
				////////// Neighbors pixels offsets calculation 
				////////// Prior to centroiding algorithm to save time
				if(Ind==0) 
				begin 	pix66<=memory0[addr_LM];pix56<=memory1[addr_LM];pix46<=memory2[addr_LM];
						pix36<=memory3[addr_LM];pix26<=memory4[addr_LM];pix16<=memory5[addr_LM];
						pix06<=memory6[addr_LM];	
				end 
				if(Ind==1) 
				begin 	pix65<=memory0[addr_LM];pix55<=memory1[addr_LM];pix45<=memory2[addr_LM];
						pix35<=memory3[addr_LM];pix25<=memory4[addr_LM];pix15<=memory5[addr_LM];
						pix05<=memory6[addr_LM];	
				end 
				if(Ind==2) 
				begin 	pix64<=memory0[addr_LM];pix54<=memory1[addr_LM];pix44<=memory2[addr_LM];
						pix34<=memory3[addr_LM];pix24<=memory4[addr_LM];pix14<=memory5[addr_LM];
						pix04<=memory6[addr_LM];	
				end 
				if(Ind==3) 
				begin 	pix63<=memory0[addr_LM];pix53<=memory1[addr_LM];pix43<=memory2[addr_LM];
						pix33<=memory3[addr_LM];pix23<=memory4[addr_LM];pix13<=memory5[addr_LM];
						pix03<=memory6[addr_LM];	
				end 
				if(Ind==4) 
				begin 	pix62<=memory0[addr_LM];pix52<=memory1[addr_LM];pix42<=memory2[addr_LM];
						pix32<=memory3[addr_LM];pix22<=memory4[addr_LM];pix12<=memory5[addr_LM];
						pix02<=memory6[addr_LM];	
				end 
				if(Ind==5) 
				begin 	pix61<=memory0[addr_LM];pix51<=memory1[addr_LM];pix41<=memory2[addr_LM];
						pix31<=memory3[addr_LM];pix21<=memory4[addr_LM];pix11<=memory5[addr_LM];
						pix01<=memory6[addr_LM];	
				end 
				if(Ind==6) 
				begin 	pix60<=memory0[addr_LM];pix50<=memory1[addr_LM];pix40<=memory2[addr_LM];
						pix30<=memory3[addr_LM];pix20<=memory4[addr_LM];pix10<=memory5[addr_LM];
						pix00<=memory6[addr_LM];	
				end 
				
				
				
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
end
 
/****************************************************************************************
// Centroding  
*****************************************************************************************/

/*********** Registers *********/
reg  [9:0]cutoff_Brightness = 100 ;//It is used as REG not PARAMETER as in future it would be customized basen on the image mean -standard deviation 
reg  [9:0]brightCounts[9:0];//
reg [9:0] num_bright_areas = 0;//Number of stars  [Bright areas]
//Centroding 
reg [15:0] background_mean_21,background_mean_63 ;// background_mean needs only 10 bits 
reg [15:0]y_weighted_sum_REG,x_weighted_sum_REG,centroid_sum_REG;

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

reg  [9:0] center,above,below,right,left;
//debug purposes only to check that the addresses for the center,left,right,above and below are correct
reg  [14:0]aboveAddr,belowAddr,centerAddr,leftAddr,rightAddr;



/***********   background_sum wire to enhance the performance rather than register accumulator **********/
reg [13:0]background_sum ;//Sum need 14 bit as 2^10*2^4=2^14 [if all pixels in the background are fully bright]
//for debug purposes, calculate the background pixels addresses 
reg [14:0]bk02 ,bk03 ,bk04 ;
reg [14:0]bk11 ,bk15 ;
reg [14:0]bk20 ,bk26 ;
reg [14:0]bk30 ,bk36 ;
reg [14:0]bk40 ,bk46 ;
reg [14:0]bk51 ,bk55 ;
reg [14:0]bk62 ,bk63 ,bk64 ;

/***********   centroid_sum wire to enhance the performance rather than register accumulator **********/
reg [14:0]centroid_sum   ;// sum needs 15 bits as the maximum value for the centroid sum is  21*2^10 
reg [14:0]cen12,cen13,cen14;
reg [14:0]cen21,cen22,cen23,cen24,cen25;
reg [14:0]cen31,cen32,cen33,cen34,cen35;
reg [14:0]cen41,cen42,cen43,cen44,cen45;
reg [14:0]cen52,cen53,cen54;


/***********   centroid_sum wire to enhance the performance rather than register accumulator **********/
reg [15:0]x_weighted_sum ;//x_weighted_sum needs 2^6*2^10 (63*max bright pixel value) as ( 63 = 3*1 + 5*2 + 5*3 + 5*4 + 3*5)
reg [15:0]y_weighted_sum ;//y_weighted_sum needs 2^6*2^10 (63*max bright pixel value) as ( 63 = 3*1 + 5*2 + 5*3 + 5*4 + 3*5)


										
//debug for simulation purposes only :
/*
wire [9:0] m12=memory[cen12],m13= memory[cen13],m14= memory[cen14]
,m21=memory[cen21],m22=memory[cen22],m23=memory[cen23],m24=memory[cen24],m25=memory[cen25]
,m31=memory[cen31],m32=memory[cen32],m33=memory[cen33],m34=memory[cen34],m35=memory[cen35]
,m41=memory[cen41],m42=memory[cen42],m43=memory[cen43],m44=memory[cen44],m45=memory[cen45]
,m52=memory[cen52],m53=memory[cen53],m54=memory[cen54];
*/
reg [2:0] clockCount ;
																			
always @(posedge sample_clk && locked)
begin
	if(rst)
	begin 
	 //Initalize the number of bright pixels in each category to zeros
		brightCounts[0] <= 0;brightCounts[1] <= 0;brightCounts[2] <= 0;
		brightCounts[3] <= 0;brightCounts[4] <= 0;brightCounts[5] <= 0;
		brightCounts[6] <= 0;brightCounts[7] <= 0;brightCounts[8] <= 0;
		brightCounts[9] <= 0;
        num_bright_areas <= 0;		
		addr_write_to_Stars_blkRAM <= -1;
 		getaddrPhase               <= 1;
		update_stars               <= 0;
		index                      <= 0;
		clockCount    <=   0; 
		din           <=   0;
		background_mean_21<=0;
		background_mean_63<=0;
		y_weighted_sum_REG<=0;
		x_weighted_sum_REG<=0;
		centroid_sum_REG  <=0;
		pix00<=0,pix01<=0,pix02<=0,pix03<=0,pix04<=0,pix05<=0,pix06<=0;
		pix10<=0,pix11<=0,pix12<=0,pix13<=0,pix14<=0,pix15<=0,pix16<=0;
		pix20<=0,pix21<=0,pix22<=0,pix23<=0,pix24<=0,pix25<=0,pix26<=0;
		pix30<=0,pix31<=0,pix32<=0,pix33<=0,pix34<=0,pix35<=0,pix36<=0;
		pix40<=0,pix41<=0,pix42<=0,pix43<=0,pix44<=0,pix45<=0,pix46<=0;
		pix50<=0,pix51<=0,pix52<=0,pix53<=0,pix54<=0,pix55<=0,pix56<=0;
		pix60<=0,pix61<=0,pix62<=0,pix63<=0,pix64<=0,pix65<=0,pix66<=0;
	end	
	else
	begin
	
    if (rowNumber>=6 & colNumber>=6)//  
	    begin //Begin the star extraction 
             // { Begin the extracting - updating the stars memory 			
				if(clockCount==0)//extract but not update the stars memory 
				 begin // { Begin check if extracting but not updating stars memory 
				   
							 we           <= 0;
							 
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
							  

							//Debug purposes only 
							//Calculate the addresses of the center,left,right,above and below pixels 
							aboveAddr <= crnt_Star_addr-FRAME_WIDTH_4-3;
							belowAddr <= crnt_Star_addr-FRAME_WIDTH_2-3;
							centerAddr<= crnt_Star_addr-FRAME_WIDTH_3-3;
							leftAddr  <= crnt_Star_addr-FRAME_WIDTH_3-4;
							rightAddr <= crnt_Star_addr-FRAME_WIDTH_3-2;

							
							//Update the parameters for the circular shift 
							//Get the center,above,below,left and right pixels values 
							center <= memory[crnt_Star_addr-FRAME_WIDTH_3-3];//-3*FRAME_WIDTH-3
							above  <= memory[crnt_Star_addr-FRAME_WIDTH_4-3]; //-4*FRAME_WIDTH-3
							below  <= memory[crnt_Star_addr-FRAME_WIDTH_2-3]; //-2*FRAME_WIDTH-3
							left   <= memory[crnt_Star_addr-FRAME_WIDTH_3-4]; //-3*FRAME_WIDTH-3-1
							right  <= memory[crnt_Star_addr-FRAME_WIDTH_3-2]; //-3*FRAME_WIDTH-3+1
							//Start calculate the background_sum,centroid_sum,x_weighted_sum,y_weighted_sum
							
							//Debug purposes only
							//Background pixels addresses
							 bk02<=crnt_Star_addr-2              ;bk03<=crnt_Star_addr-3              ;bk04<=crnt_Star_addr-4 ;
							 bk11<=crnt_Star_addr-FRAME_WIDTH_1-1;bk15<=crnt_Star_addr-FRAME_WIDTH_1-5;
							 bk20<=crnt_Star_addr-FRAME_WIDTH_2  ;bk26<=crnt_Star_addr-FRAME_WIDTH_2-6;
							 bk30<=crnt_Star_addr-FRAME_WIDTH_3  ;bk36<=crnt_Star_addr-FRAME_WIDTH_3-6;
							 bk40<=crnt_Star_addr-FRAME_WIDTH_4  ;bk46<=crnt_Star_addr-FRAME_WIDTH_4-6;
							 bk51<=crnt_Star_addr-FRAME_WIDTH_5-1;bk55<=crnt_Star_addr-FRAME_WIDTH_5-5;
							 bk62<=crnt_Star_addr-FRAME_WIDTH_6-2;bk63<=crnt_Star_addr-FRAME_WIDTH_6-3;bk64<=crnt_Star_addr-FRAME_WIDTH_6-4;
                            //Centroid pixels addresses
							cen12<=crnt_Star_addr-FRAME_WIDTH_1-2;cen13<=crnt_Star_addr-FRAME_WIDTH_1-3;cen14<=crnt_Star_addr-FRAME_WIDTH_1-4;
							cen21<=crnt_Star_addr-FRAME_WIDTH_2-1;cen22<=crnt_Star_addr-FRAME_WIDTH_2-2;cen23<=crnt_Star_addr-FRAME_WIDTH_2-3;cen24<=crnt_Star_addr-FRAME_WIDTH_2-4;cen25<=crnt_Star_addr-FRAME_WIDTH_2-5;
							cen31<=crnt_Star_addr-FRAME_WIDTH_3-1;cen32<=crnt_Star_addr-FRAME_WIDTH_3-2;cen33<=crnt_Star_addr-FRAME_WIDTH_3-3;cen34<=crnt_Star_addr-FRAME_WIDTH_3-4;cen35<=crnt_Star_addr-FRAME_WIDTH_3-5;
							cen41<=crnt_Star_addr-FRAME_WIDTH_4-1;cen42<=crnt_Star_addr-FRAME_WIDTH_4-2;cen43<=crnt_Star_addr-FRAME_WIDTH_4-3;cen44<=crnt_Star_addr-FRAME_WIDTH_4-4;cen45<=crnt_Star_addr-FRAME_WIDTH_4-5;
							cen52<=crnt_Star_addr-FRAME_WIDTH_5-2;cen53<=crnt_Star_addr-FRAME_WIDTH_5-3;cen54<=crnt_Star_addr-FRAME_WIDTH_5-4;
							clockCount <= clockCount+1;
				  end // }
				 else
                 begin 				 
				   if(( ( center > cutoff_Brightness) && ( above >cutoff_Brightness | below >cutoff_Brightness | right > cutoff_Brightness | left > cutoff_Brightness) && (center >= left & center >= above & center > below & center > right ) ))
				    begin 
					  if(clockCount == 1)
						begin 
							//calculate the sum of pixels used for background subtraction
							background_sum <= memory[bk02]+ memory[bk03]+memory[bk04]
							+ memory[bk11]+memory[bk15]
							+ memory[bk20]+memory[bk26]
							+ memory[bk30]+memory[bk36]
							+ memory[bk40]+memory[bk46]
							+ memory[bk51]+memory[bk55]
							+ memory[bk62]+ memory[bk63]+ memory[bk64];

														
							//calculate sum of pixels used for centroiding									
							centroid_sum  <= memory[cen12]+ memory[cen13]+ memory[cen14]
							+memory[cen21]+memory[cen22]+ memory[cen23]+ memory[cen24]+memory[cen25]
							+memory[cen31]+memory[cen32]+ memory[cen33]+ memory[cen34]+memory[cen35]
							+memory[cen41]+memory[cen42]+ memory[cen43]+ memory[cen44]+memory[cen45]
							+memory[cen52]+ memory[cen53]+ memory[cen54];

							// calculate weighted pixel sums in x dimension
							//any constant multiplication will be done using adders and shift units
							/*
							Here we are applying the following mask:
												([[0, 0, 0, 0, 0, 0, 0],
												  [0, 0, 2, 3, 4, 0, 0],
												  [0, 1, 2, 3, 4, 5, 0],
												  [0, 1, 2, 3, 4, 5, 0],
												  [0, 1, 2, 3, 4, 5, 0],
												  [0, 0, 2, 3, 4, 0, 0],
												  [0, 0, 0, 0, 0, 0, 0]])
							*/					  
							x_weighted_sum  <= (memory[cen14] << 1)+ ( memory[cen13]+(memory[cen13] << 1))+ (memory[cen12]<<2)
							+memory[cen25]+(memory[cen24]<<1)+ ( memory[cen23]+(memory[cen23]<<1))+ (memory[cen22]<<2)+(memory[cen21]+(memory[cen21]<<2))
							+memory[cen35]+(memory[cen34]<<1)+ ( memory[cen33]+(memory[cen33]<<1))+ (memory[cen32]<<2)+(memory[cen31]+(memory[cen31]<<2))
							+memory[cen45]+(memory[cen44]<<1)+ ( memory[cen43]+(memory[cen43]<<1))+ (memory[cen42]<<2)+(memory[cen41]+(memory[cen41]<<2))
							+(memory[cen54]<<1)+ (memory[cen53]+(memory[cen53]<<1))+ (memory[cen52]<<2);
							// calculate weighted pixel sums in y dimension
							/*
							Here we are applying the following mask:
												([[0, 0, 0, 0, 0, 0, 0],
												  [0, 0, 1, 1, 1, 0, 0],
												  [0, 2, 2, 2, 2, 2, 0],
												  [0, 3, 3, 3, 3, 3, 0],
												  [0, 4, 4, 4, 4, 4, 0],
												  [0, 0, 5, 5, 5, 0, 0],
												  [0, 0, 0, 0, 0, 0, 0]])

							*/
							y_weighted_sum  <= (memory[cen12]+(memory[cen12] << 2))+ ( memory[cen13]+(memory[cen13] << 2))+ (memory[cen14]+(memory[cen14]<<2))
							+(memory[cen21]<<2)+(memory[cen22]<<2)+ ( memory[cen23]<<2)+ (memory[cen24]<<2)+(memory[cen25]<<2)
							+(memory[cen31]+(memory[cen31]<<1))+(memory[cen32]+(memory[cen32]<<1))+ ( memory[cen33]+(memory[cen33]<<1))+ (memory[cen34]+(memory[cen34]<<1))+(memory[cen35]+(memory[cen35]<<1))
							+(memory[cen41]<<1)+(memory[cen42]<<1)+ (memory[cen43]<<1)+ (memory[cen44]<<1)+(memory[cen45]<<1)
							+(memory[cen52]   )+(memory[cen53]   )+ (memory[cen54]   );

							clockCount <= clockCount+1;
							
				  end	
                      else 
						begin 
						if(clockCount == 2)
							begin 
						
							 // begin if  the center value is greater than the cutoff for the brightness
							      //The above center 
								  //and
							      //if any of the adjacentes are bright then proceed 
								
										num_bright_areas       <= num_bright_areas+1;													 									    									    
										//calculate average of background pixels (dvision by 16 => shift left four bits) 
										//Subtracting from the background 
										//21 is the count of the pixels in the neighborhod of the centroid
										//21 multiplication will be done through shift and addition 
										
										/* Simplify the following : 
										background_mean = background_sum >> 4;																																								
										background_mean_21 =(background_mean<<4)+(background_mean<<2)+background_mean;
										into:*/
										background_mean_21 <=( background_sum)+( background_sum >>2)  +( background_sum >>4);
										
										/* and Simplify the following : 
										background_mean = background_sum >> 4;																																								
										background_mean_63 =(background_mean<<6)- background_mean;
										into:*/
										background_mean_63 <=( background_sum <<2)- ( background_sum>> 4);
										
										//////////////////////////////////////////
										 clockCount <= clockCount + 1;
									   
								      
								  // end if  the center value is greater than the cutoff for the brightness
							  
							end		
					 	else 
							if(clockCount == 3)
							  begin 
								//Updating the stars memory 
								//{ Send the stars locations x,y com to the block RAM
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
											if(y_weighted_sum_REG > background_mean_63)
													din <= y_weighted_sum_REG - background_mean_63;
											else 
													din <= 0;
											 addr_write_to_Stars_blkRAM <= addr_write_to_Stars_blkRAM + 1;
											 index <= 3 ;
										 end 
										 
									  else if(index == 3)
										 begin
												//Update and write the actual x_weighted_sum
												if(x_weighted_sum_REG > background_mean_63)
													din <= x_weighted_sum_REG - background_mean_63;
												else 
													din <= 0;
												addr_write_to_Stars_blkRAM <= addr_write_to_Stars_blkRAM + 1;
												index <= 4 ;
										 end  
									  else if(index == 4)
										 begin 
											
											if(centroid_sum_REG > background_mean_21)
													din <= centroid_sum_REG - background_mean_21;
											else 
													din <= 0;
											 
										   addr_write_to_Stars_blkRAM <= addr_write_to_Stars_blkRAM + 1;
										   index <= 0 ;
										   update_stars <= 0;
										   clockCount <= 0;
										 end  
							   
								end 
					          end  //} end Send the stars locations x,y com to the block RAM				   
		 
                         end  
                     end
					 else 
						clockCount<=0;
  		
					end 
	 
            end
	end
end
endmodule

