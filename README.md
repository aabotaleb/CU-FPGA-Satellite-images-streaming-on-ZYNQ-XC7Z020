# CU-FPGA-Satellite-images-streaming-on-ZYNQ-XC7Z020
1-	Star_Extract_IP.v

The image processing “star extraction” via centroding module

2-	The clocking management IP [AXI4 (Clock wizard v5.1 provided by Vivado 2015.2)
Or (Clock wizard v5.3 provided by Vivado 2016.4)

The purpose is to multiply the input clock coming from the camera = 72MHz by 4 to obtain a faster clock being able to make the 
This IP provides many features including: 
a.	Frequency Synthesizer
This is the main unit doing the frequency generation of different frequencies.

b.	Phase Alignment

 1-	The image processing part
Part 1: [initialization]
	It is responsible for instantiating the clocking management unit. “The clocking wizard IP instance”
Part 2: [runs at the shifted input clock from the camera] 72MHz shifted by 270]
It is responsible for transferring pixels from the camera to the FPGA internal memory holding only the stream window of size = 7 rows.
	Part 3:	[runs at the faster clock 288 MHz]
		The star extraction and image processing stuff is done here.
2-	The clocking management part
3-	The test bench “Simulation Purposes Only”
