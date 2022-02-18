///////////////////////////////////////////////////////////////////////////////////
//
// Project		: Capacitated Vehicle Routing Problem on FPGA
// Function		: Testbench for the controller code 
// File			: controller_tb.v
// Created		: 02/08/2022
// Author		: Maximilian Heer		
// Notes		: Tests a simple run through all states for the controller-algorithm 
// Advisory 	: Nothing special 
//
///////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

module controller_tb; 
	
	// Clock and reset signals 
	reg clk; 
	reg rst; 
	
	// System under test 
	controller_test #(
		.pNumCities(16), 
		.pVehicleCapacity(10), 
		.pDepotXCoord(16), 
		.pDepotYCoord(16), 
		.pNumInitRuns(20), 
		.pNumRuns(68), 
		.pNumProcessingNodes(16)
	) dut_sim (
		.clk_i(clk), 
		.rst_i(rst)
	);
	
	// Generate clock cycle 
	always #1 clk = !clk; 
	
	// Generate test stimuli 
	initial 
		begin 
			clk <= 1'b0; 
			rst <= 1'b1; 
			#10
			rst <= 1'b0; 
		end 
	
endmodule 