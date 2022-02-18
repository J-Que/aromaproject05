///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Project		: Capacitated Vehicle Routing Problem on FPGA
// Function		: Controller for the complete algorithm, combines multiple processing nodes, memory ports and random number generators - test version for the whole algorithm, with a simulated reg file memory
// File			: controller.sv
// Created		: 12/04/2021
// Author 		: Maximilian Heer
// Notes 		: Features multiple processing nodes for parallel execution of multiple crossovers as well as memory ports and random number generators
// Advisory		: Try out various numbers of processing nodes and find out whether they fit the hardware on the FPGA
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
`default_nettype none

module controller_v2_test #(
	parameter pNumCities = 16, 
	parameter pVehicleCapacity = 10, 
	parameter pDepotXCoord = 16, 
	parameter pDepotYCoord = 16, 
	parameter pNumInitRuns = 1000, 
	parameter pNumRuns = 10000, 
	parameter pNumProcessingNodes = 16
)(
	// Generating module input for clk and rst to connect with hardware resources of the FPGA
	input wire logic clk_i, 
	input wire rst_i
); 

	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Definition of the localparams 
	//
	///////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Bit Width of the capacity requirement in each node 
	localparam lpCapBitWidth = 3; 

	// Bit Width of the IDs of the nodes 
	localparam lpIDBitWidth = 12; 

	// Bit Width of the coordinates of the nodes 
	localparam lpCoordBitWidth = 6; 

	// Bit Width of the maximum distance in the playfield 
	localparam lpDistMaxBitWidth = 13; 

	// Offset of cities for the selection of a random subtour from parent 1 to directly insert in the children 
	localparam lpRandomSubtourOffset = 5; 

	// Range of cities whose order is inverted for mutation 
	localparam lpMutationDistance = 6; 

	// Number of mutation processes that are executed for initialization of a new population in the beginning 
	localparam lpInitMutationProcesses = 25; 

	// Size of entire population 
	localparam lpSizeOfPopulation = 3*pNumCities; 

	// Bit Width of the number of a city within an individual 
	localparam lpCityNumBitWidth = $clog2(pNumCities - 1);

	// Bit Width of the number of an individual within the population 
	localparam lpIndividualNumBitWidth = $clog2(lpSizeOfPopulation - 1); 

	// Bit Width of the number of processing nodes in the design 
	localparam lpProcessingNumBitWidth = pNumProcessingNodes; 

	// Bit Width of the number of crossover runs in the algorithm
	localparam lpNumRunsBitWidth = $clog2(pNumRuns - 1); 
	
	// Bit Width for a summator for all priority values 
	localparam lpBitWidthPrioSummator = pNumProcessingNodes + $clog2(pNumProcessingNodes); 
	
	// Number of reading accesses required to completely read or write one individual from memory
	localparam lpNumOfMemAccesses = 5; 
	
	// Bitwidth of a counter for reading / writing accesses to the memory 
	localparam lpMemAccessesCounterBitWidth = $clog2(lpNumOfMemAccesses+1); 
	
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Additional local parameters just for testing purposes 
	//
	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// localparams for the coordinates 
	localparam coord3 = 6'b000011; 
	localparam coord2 = 6'b000010;
	localparam coord1 = 6'b000001;
	localparam coord18 = 6'b010010;
	localparam coord28 = 6'b011100;
	localparam coord9 = 6'b001001;
	localparam coord7 = 6'b000111;
	localparam coord13 = 6'b001101;
	localparam coord10 = 6'b001010;
	localparam coord24 = 6'b011000;
	localparam coord30 = 6'b011110;
	localparam coord14 = 6'b001110;
	localparam coord4 = 6'b000100;
	localparam coord16 = 6'b010000;
	localparam coord22 = 6'b010110;
	localparam coord20 = 6'b010100;
	localparam coord21 = 6'b010101; 
	localparam coord26 = 6'b011010; 
	localparam coord6 = 6'b000110;
	localparam coord29 = 6'b011101; 
	localparam coord17 = 6'b010001; 
	localparam coord32 = 6'b100000; 

	// localparams for the capacities
	localparam cap5 = 3'b101;
	localparam cap1 = 3'b001;
	localparam cap2 = 3'b010;
	localparam cap7 = 3'b111; 
	localparam cap3 = 3'b011; 
	localparam cap4 = 3'b100; 
	localparam cap6 = 3'b110; 

	// localparams for the parent fitness values 
	localparam parent_1_fitness = 16'b0111111111111111; 
	localparam parent_2_fitness = 16'b0111111111111111; 

	// localparams for the city nodes 
	localparam node_3_3_5 = {coord3, coord3, cap5};
	localparam node_18_2_5 = {coord18, coord2, cap5};
	localparam node_28_1_1 = {coord28, coord1, cap1};
	localparam node_9_7_2  = {coord9, coord7, cap2}; 
	localparam node_13_10_7 = {coord13, coord10, cap7};
	localparam node_24_10_1 = {coord24, coord10, cap1};
	localparam node_4_16_2 = {coord4, coord16, cap2}; 
	localparam node_30_14_1 = {coord30, coord14, cap1}; 
	localparam node_22_18_3 = {coord22, coord18, cap3};
	localparam node_28_20_4 = {coord28, coord20, cap4};
	localparam node_9_21_6 = {coord9, coord21, cap6};
	localparam node_14_24_3 = {coord14, coord24, cap3};
	localparam node_22_26_4 = {coord22, coord26, cap4};
	localparam node_6_29_7 = {coord6, coord29, cap7};
	localparam node_17_32_4 = {coord17, coord32, cap4};
	localparam node_29_29_3 = {coord29, coord29, cap3};


	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Definition of own datatypes 
	// 
	//////////////////////////////////////////////////////////////////////////////////////////////////////////// 

	// Definition of the number of a city within an individual 
	typedef logic [lpCityNumBitWidth-1:0] CityNumber; 

	// Definition of the number of an individual within the population 
	typedef logic [lpIndividualNumBitWidth-1:0] IndividualNumber; 

	// Definition of the two numbers of both parents in a processing node 
	typedef struct packed{
		IndividualNumber parent_1;
		IndividualNumber parent_2; 
	} DoubleIndividualNumbers; 

	// Definition of a 256-bit memory word for data exchange between memory ports and processing nodes 
	typedef logic [255:0] MemoryInterfaceWord; 

	// Definition of word for fitness value 
	typedef logic [lpDistMaxBitWidth-1:0] FitnessValueWord;

	// Definition of a counter for the processing nodes 
	typedef logic [lpProcessingNumBitWidth-1:0] ProcessingNodeCounter; 
	
	// Definition of a value word for marking the priority of a calculation 
	typedef logic [lpProcessingNumBitWidth:0] ProcessingPriority; 

	// Definition of a counter for the number of runs 
	typedef logic [lpNumRunsBitWidth-1:0] RunCounter; 

	// Definition of the states for the main FSM 
	typedef enum logic[3:0] {
		IDLE = 0, 
		WAIT_INIT_RUNS = 1, 
		WAIT_STANDARD_RUNS = 2, 
		WAIT_FOR_END = 3, 
		EVALUATE_BEST_SOLUTION = 4, 
		FINISHED = 5
	} MainFSMState;

	// Definition of the states for the processing node FSM 
	typedef enum logic[3:0] {
		IDLE_PN = 4'b0000, 
		START = 4'b0001, 
		LOAD_PARENT_1 = 4'b0010, 
		START_LOADING_PARENT_2 = 4'b0011, 
		LOAD_PARENT_2 = 4'b0100, 
		WAIT_PROCESSING = 4'b0101, 
		WB_OFFSPRING_1 = 4'b0110, 
		START_WB_OFFSPRING_2 = 4'b0111, 
		WB_OFFSPRING_2 = 4'b1000, 
		FINAL = 4'b1001, 
		LOAD_BEST_SOLUTION = 4'b1010
	} ProcessingNodeFSMState;
	
	// Definition of a summator word for the processing priorities 
	typedef logic [lpBitWidthPrioSummator-1:0] PrioSummator; 
	
	// Definition of a counter for memory accesses 
	typedef logic [lpMemAccessesCounterBitWidth-1:0] MemAccessCounter; 
	
	// Definition of a memory word 
	typedef logic [255:0] MemoryWord;


	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Definition of own signals and registers 
	//
	////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Raw outputs of the two random number generators for crossover and mutation 
	CityNumber raw_random_crossover_mutation_output_1; 
	CityNumber raw_random_crossover_mutation_output_2; 

	// Adapted random number for the selection of subtours for crossover 
	CityNumber crossover_starting_point_1; 
	CityNumber crossover_starting_point_2; 

	// Adapted random number for the selection of mutation parts 
	CityNumber mutation_starting_point_1;
	CityNumber mutation_starting_point_2; 

	// Raw output of the first random number generator for individual selection 
	IndividualNumber raw_individual_output_1; 
	
	// Raw output of the second random number generator for individual selection 
	IndividualNumber raw_individual_output_2; 

	// Adapted output of the random number generator for individual selection 
	IndividualNumber adapted_individual_output; 

	// Init mode signal for all processing nodes 
	logic init_mode[pNumProcessingNodes]; 

	// Exchange register with ports for start_parent_1
	logic start_parent_1_regfile[pNumProcessingNodes]; 

	// Exchange register with ports for start_parent_2 
	logic start_parent_2_regfile[pNumProcessingNodes]; 

	// Exchange register with ports for finished_parent_1_i
	logic finished_parent_1_regfile[pNumProcessingNodes]; 

	// Exchange register with ports for finished_parents_2_i
	logic finished_parent_2_regfile[pNumProcessingNodes]; 

	// Signal from main FSM to the node FSMs to start processing 
	logic start_processing[pNumProcessingNodes];

	// Signal from main FSM to the node FSMs to start the init mode for processing 
	logic processing_init[pNumProcessingNodes]; 

	// Input wires from memory ports to processing nodes 
	MemoryInterfaceWord pop_data_mem_to_pn[pNumProcessingNodes]; 

	// Output wires from processing nodes to memory ports 
	MemoryInterfaceWord pop_data_pn_to_mem[pNumProcessingNodes]; 

	// Transmission signals for child 1 from processing nodes 
	wire logic transmission_ind_1_out[pNumProcessingNodes]; 

	// Transmission signals for child 2 from processing nodes 
	wire logic transmission_ind_2_out[pNumProcessingNodes]; 

	// Signals for unchanged transmission
	wire logic unchanged_transmission_out[pNumProcessingNodes]; 

	// Signals that crossover is done 
	wire logic crossover_done_out[pNumProcessingNodes]; 

	// Signal to show which is the better fitness value 
	wire logic better_fitness_slot_out[pNumProcessingNodes]; 

	// Signals to show the better fitness value  
	FitnessValueWord better_fitness_value_out[pNumProcessingNodes]; 
	
	// Array that holds all processing priorities for all processingNodes 
	ProcessingPriority processing_priority[pNumProcessingNodes]; 
	
	// Summator signal for the single priorities of the processingNodes 
	PrioSummator priority_sum; 
	
	// Register file that counts all memory accesses for each processing node 
	MemAccessCounter mem_access_counter[pNumProcessingNodes]; 


	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Initialisation of two random number generators for selecting subtours for crossover and mutation 
	//
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// First random number generator 
	LFSR #(
			.pNumBits(lpCityNumBitWidth)
		)random_city_selector_1(
			.clk(clk_i), 
			.en_i(1'b 1), 
			.LFSR_Data_o(raw_random_crossover_mutation_output_1)
		);

	// Second random number generator 
	LFSR_changed #(
			.pNumBits(lpCityNumBitWidth)
		)random_city_selector_2(
			.clk(clk_i), 
			.en_i(1'b1), 
			.LFSR_Data_o(raw_random_crossover_mutation_output_2)
		);


	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Assignments and calculations of the random number outputs for crossover and mutation 
	//
	////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Assignment of the crossover starting points 
	assign crossover_starting_point_1 = (((raw_random_crossover_mutation_output_1+lpRandomSubtourOffset) < pNumCities) ? raw_random_crossover_mutation_output_1 : (raw_random_crossover_mutation_output_1-(raw_random_crossover_mutation_output_1+lpRandomSubtourOffset-pNumCities-1)));
	assign crossover_starting_point_2 = (((raw_random_crossover_mutation_output_2+lpRandomSubtourOffset) < pNumCities) ? raw_random_crossover_mutation_output_2 : (raw_random_crossover_mutation_output_2-(raw_random_crossover_mutation_output_2+lpRandomSubtourOffset-pNumCities-1)));

	// Assignments of the mutation starting points 
	assign mutation_starting_point_1 = (((raw_random_crossover_mutation_output_1+lpMutationDistance) < pNumCities) ? raw_random_crossover_mutation_output_1 : (raw_random_crossover_mutation_output_1-(raw_random_crossover_mutation_output_1+lpMutationDistance-pNumCities-1)));
	assign mutation_starting_point_2 = (((raw_random_crossover_mutation_output_2+lpMutationDistance) < pNumCities) ? raw_random_crossover_mutation_output_2 : (raw_random_crossover_mutation_output_2-(raw_random_crossover_mutation_output_2+lpMutationDistance-pNumCities-1)));


	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Initialisation of two random number generators for selecting individuals from the population as parents 
	//
	////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Selector for first parent 
	LFSR #(
			.pNumBits(lpIndividualNumBitWidth)
		)random_individual_selector_1(
			.clk(clk_i), 
			.en_i(!stop_LFSR_1), 
			.LFSR_Data_o(raw_individual_output_1)
		); 
		
	// Selector of second parent 
	LFSR_changed #(
			.pNumBits(lpIndividualNumBitWidth)
		)random_individual_selector_2(
			.clk(clk_i), 
			.en_i(1'b1), 
			.LFSR_Data_o(raw_individual_output_2)
		); 
		
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Additional signals for controlling the memory access 
	//
	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// Register file that holds all individuals currently in use 
	IndividualNumber ind_in_use [2*pNumProcessingNodes]; 
	
	// Signal that shows that chosen 1 is currently not in use 
	wire logic individual_1_not_in_use; 
	
	// Signal that shows that chosen 2 is currently not in use 
	wire logic individual_2_not_in_use; 
	
	// Signal that shows that both selected signals are not identical 
	wire logic individuals_not_identical; 
	
	// Multiple wire signale for storing whether chosen individuals are already in use 
	logic [2*pNumProcessingNodes-1 : 0] individual_1_comparison; 
	logic [2*pNumProcessingNodes-1 : 0] individual_2_comparison; 
	
	// Signals for calculated individual numbers (since random numbers could be higher than population size)
	IndividualNumber individual_proposed_1; 
	IndividualNumber individual_proposed_2; 
	
	// Signals for approving the proposed individuals 
	wire logic ind_1_approved; 
	wire logic ind_2_approved; 
	
	// Signal for approving the current round of signals 
	wire logic current_inds_approved; 
	
	// Signal for stopping first LFSR from time to time 
	wire logic stop_LFSR_1; 
	
	wire logic crossover_done; 
	

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Initialisation of the memory ports
	//
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// At this point: Initiazation of the registers for testing purposes: 
	MemoryWord pop_mem[2*lpSizeOfPopulation]; 



	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Initialisation of 16 Processing Nodes 
	//
	////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Parallel instantiation of up to 16 processing nodes (number can be chosen freely)
	genvar processingNodeCounter; 
	generate 
		for(processingNodeCounter = 0; processingNodeCounter < pNumProcessingNodes; processingNodeCounter++) begin: processing_node_generator
			ProcessingNode #(
				.pNumCities(pNumCities),
				.pCapBitWidth(lpCapBitWidth), 
				.pIDBitWidth(lpIDBitWidth), 
				.pCoordBitWidth(lpCoordBitWidth), 
				.pDistMaxBitWidth(lpDistMaxBitWidth), 
				.pVehicleCapacity(pVehicleCapacity), 
				.pRandomSubtourOffset(lpRandomSubtourOffset), 
				.pMutationDistance(lpMutationDistance), 
				.pInitMutationProcesses(lpInitMutationProcesses), 
				.pDepotXCoord(pDepotXCoord), 
				.pDepotYCoord(pDepotYCoord)
			) processing_node (
				.clk_i(clk_i),
				.rst_i(rst_i),
				.init_mode_i(init_mode[processingNodeCounter]), 
				.start_parent_1_i(start_parent_1_regfile[processingNodeCounter]), 
				.start_parent_2_i(start_parent_2_regfile[processingNodeCounter]), 
				.finished_parent_1_i(finished_parent_1_regfile[processingNodeCounter]), 
				.finished_parent_2_i(finished_parent_2_regfile[processingNodeCounter]),
				.pop_data_i(pop_data_mem_to_pn[processingNodeCounter]), 
				.pop_data_o(pop_data_pn_to_mem[processingNodeCounter]), 
				.transmission_ind_1_o(transmission_ind_1_out[processingNodeCounter]), 
				.transmission_ind_2_o(transmission_ind_2_out[processingNodeCounter]), 
				.unchanged_ind_o(unchanged_transmission_out[processingNodeCounter]), 
				.random_city_number_crossover_1_i(crossover_starting_point_1), 
				.random_city_number_crossover_2_i(crossover_starting_point_2),
				.random_city_number_mutation_1_i(mutation_starting_point_1), 
				.random_city_number_mutation_2_i(mutation_starting_point_2), 
				.crossover_done(crossover_done),
				.better_fitness_slot_o(better_fitness_slot_out[processingNodeCounter]), 
				.better_fitness_value_o(better_fitness_value_out[processingNodeCounter])
			);
		end 
	endgenerate


	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Initialisation of independent register files for storing relevant information from each processing node 
	//
	///////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Best list - storing the current best value from each processing node 
	FitnessValueWord best_known_solutions[pNumProcessingNodes];

	// Best list addresses - storing the addresses of the adresses of the best solutions 
	IndividualNumber best_known_solutions_addresses[pNumProcessingNodes];

	// Initial starting signals for the processingNode-functionalities 
	logic start_proeccesing[pNumProcessingNodes]; 

	// Final stop signals for the processingNode-functionalities
	logic stop_processing[pNumProcessingNodes]; 

	// Initial signal for running in init mode 
	logic process_init[pNumProcessingNodes]; 

	// Central counter for all runs 
	RunCounter run_counter; 
	
	// Run Counter grid 
	RunCounter run_counter_arr[pNumProcessingNodes]; 

	// Central counter for processing nodes 
	ProcessingNodeCounter processing_node_counter; 

	// register file for the FSMs of the processing nodes to store whether they have finished calculating 
	logic [pNumProcessingNodes-1:0] all_finished_reg_file; 

	// Combined signal to show that all processing nodes have finished calculation 
	wire logic all_finished; 

	// Signal to load the best solution into processing node 1 
	logic load_best_solution_processing_node_1; 

	// Search register for final best solution in the end 
	FitnessValueWord best_final_solution; 

	// Search register for the address for the final best solution in the end 
	IndividualNumber best_final_solution_address; 

	// State variable for the main FSM 
	MainFSMState state_main; 

	// State variable for the processing node FSM 
	ProcessingNodeFSMState state_pn[pNumProcessingNodes]; 


	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Assignment of common signals 
	// 
	////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Common finished signals
	assign all_finished = (all_finished_reg_file == 0); 


	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 
	// Central FSM - starts the initialization phase, counts the number of crossover operations and stops everything at the end 
	// 
	////////////////////////////////////////////////////////////////////////////////////////////////////////////

	always @ (posedge clk_i) begin 
		if(rst_i) begin 
			for(int nodeCounter = 0; nodeCounter < pNumProcessingNodes; nodeCounter++) begin 
				// resetting the actual memory registers 
				best_known_solutions[nodeCounter] <= 0; 
				best_known_solutions_addresses[nodeCounter] <= 0; 

				// resetting the registers that are used for communication with the processing nodes 
				init_mode[nodeCounter] <= 0; 
				start_parent_1_regfile[nodeCounter] <= 0; 
				start_parent_2_regfile[nodeCounter] <= 0; 
				finished_parent_1_regfile[nodeCounter] <= 0; 
				finished_parent_2_regfile[nodeCounter] <= 0; 
				start_processing[nodeCounter] <= 0; 
				stop_processing[nodeCounter] <= 0; 
				processing_init[nodeCounter] <= 0; 
			end 
			
			for(int populationCounter = 0; populationCounter < 2*lpSizeOfPopulation; populationCounter = populationCounter+2) begin 
				pop_mem[populationCounter] <= {parent_1_fitness, node_29_29_3, 1'b0, node_28_1_1, 1'b1, node_13_10_7, 1'b1, node_6_29_7, 1'b1, node_3_3_5, 1'b0, node_22_18_3, 1'b0, node_4_16_2, 1'b1, node_22_26_4, 1'b0, node_18_2_5, 1'b1, node_14_24_3, 1'b0, node_9_7_2, 1'b1, node_9_21_6, 1'b0, node_24_10_1, 1'b1, node_28_20_4, 1'b0, node_30_14_1, 1'b0};
				pop_mem[populationCounter+1] <= {node_17_32_4, 1'b1, 240'b0};
			end 
			
			state_main <= IDLE; 
			processing_node_counter <= 0; 
			load_best_solution_processing_node_1 <= 0; 
			best_final_solution <= 0; 
		end else begin 
			case(state_main) 
				IDLE: begin 
					start_processing[processing_node_counter] <= 1; 
					processing_init[processing_node_counter] <= 1; 
					processing_node_counter <= processing_node_counter + 1; 

					// Once all processing nodes are started, go to next state and wait for 
					if(processing_node_counter == pNumProcessingNodes) begin 
						state_main <= WAIT_INIT_RUNS; 
						processing_node_counter <= 0;
					end 

				end 

				WAIT_INIT_RUNS: begin 
					// If number of init runs is reached, leave the init mode and go to normal processing mode 
					if(run_counter == pNumInitRuns) begin 
						state_main <= WAIT_STANDARD_RUNS; 
						for(int nodeCounter = 0; nodeCounter < pNumProcessingNodes; nodeCounter++) begin 
							processing_init[nodeCounter] <= 0; 
						end 
					end 
				end 

				WAIT_STANDARD_RUNS: begin 
					// If number of overall runs is reached, finish the crossover process and go to final wait state 
					if(run_counter == pNumRuns) begin 
						state_main <= WAIT_FOR_END; 
						for(int nodeCounter = 0; nodeCounter < pNumProcessingNodes; nodeCounter++) begin 
							start_processing[nodeCounter] <= 0; 
						end 
					end 
				end 

				WAIT_FOR_END: begin 
					if(all_finished) begin 
						state_main <= EVALUATE_BEST_SOLUTION; 
						processing_node_counter <= 1; 
						best_final_solution <= best_known_solutions[0]; 
						best_final_solution_address <= best_known_solutions_addresses[0]; 
					end 
				end 

				EVALUATE_BEST_SOLUTION: begin 
					if(best_known_solutions[processing_node_counter] < best_final_solution) begin 
						best_final_solution <= best_known_solutions[processing_node_counter]; 
					end 

					if(processing_node_counter < pNumProcessingNodes - 1) begin 
						processing_node_counter <= processing_node_counter + 1; 
						state_main <= EVALUATE_BEST_SOLUTION; 
					end else begin 
						state_main <= FINISHED; 
						load_best_solution_processing_node_1 <= 1; 
					end 
				end 

				FINISHED: begin 
					state_main <= FINISHED; 
				end 

			endcase 
		end 
	end 
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Checking whether randomly chosen memory addresses can be used or are already in use, which would cause a block 
	//
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// First of all calculating regular individual numbers
	assign individual_proposed_1 = (raw_individual_output_1 > lpSizeOfPopulation - 1) ? (raw_individual_output_1 - lpSizeOfPopulation) : raw_individual_output_1; 
	assign individual_proposed_2 = (raw_individual_output_2 > lpSizeOfPopulation - 1) ? (raw_individual_output_2 - lpSizeOfPopulation) : raw_individual_output_2; 
	
	// Check wether the proposed individuals are identical 
	assign individuals_not_identical = (individual_proposed_1 != individual_proposed_2); 
	
	// Fill out the multi-bit signals for comparison of chosen individuals with those currently in use 
	always @ (*) begin 
		for(int individualCounter = 0; individualCounter < 2*pNumProcessingNodes; individualCounter++) begin 
			if(individual_proposed_1 == ind_in_use[individualCounter]) begin 
				individual_1_comparison[individualCounter] = 1; 
			end else begin 
				individual_1_comparison[individualCounter] = 0; 
			end 
			if(individual_proposed_2 == ind_in_use[individualCounter]) begin 
				individual_2_comparison[individualCounter] = 1; 
			end else begin 
				individual_2_comparison[individualCounter] = 0; 
			end 
		end 
	end 
	
	// Assign the signals for recognizing usage 
	assign individual_1_not_in_use = (individual_1_comparison == 0); 
	assign individual_2_not_in_use = (individual_2_comparison == 0); 
	
	// Final check for the proposed individuals 
	assign ind_1_approved = individual_1_not_in_use && individuals_not_identical; 
	assign ind_2_approved = individual_2_not_in_use && individuals_not_identical; 
	assign current_inds_approved = ind_1_approved && ind_2_approved; 
	
	assign stop_LFSR_1 = individual_1_not_in_use && !individuals_not_identical; 
	
	
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Granting access based on priority of the processing nodes
	//
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// Summing up the priority values in the priority register file for finding out who is allowed to start processing 
	always @ (*) begin 
		run_counter = 0;
		priority_sum = 0; 
		for(int prioCounter = 0; prioCounter < pNumProcessingNodes; prioCounter++) begin 
			priority_sum += processing_priority[prioCounter]; 
			run_counter += run_counter_arr[prioCounter]; 
		end 
	end 
	
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Calculating the all_finished signal 
	//
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	always @ (*) begin 
	   for(int finishedCounter = 0; finishedCounter < pNumProcessingNodes; finishedCounter++) begin 
		   all_finished_reg_file[finishedCounter] = (state_pn[finishedCounter] != IDLE_PN); 
	   end 
	end 
	

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Local FSMs for the various processing nodes 
	//
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////

	always @ (posedge clk_i) begin 
		for(int pn_counter = 0; pn_counter < pNumProcessingNodes; pn_counter++) begin 
			
			if(rst_i) begin 
				init_mode[pn_counter] <= 0; 
				start_parent_1_regfile[pn_counter] <= 0; 
				start_parent_2_regfile[pn_counter] <= 0; 
				finished_parent_1_regfile[pn_counter] <= 0; 
				finished_parent_2_regfile[pn_counter] <= 0;
				for(int nodeCounter = 0; nodeCounter < pNumProcessingNodes; nodeCounter++) begin 
					processing_priority[nodeCounter] <= 0; 
					state_pn[pn_counter] <= IDLE_PN; 
					run_counter_arr[pn_counter] <= 0; 
					pop_data_mem_to_pn[pn_counter] <= 0; 
				end 
				for(int indCounter = 0; indCounter < 2*pNumProcessingNodes - 1; indCounter++) begin 
				    ind_in_use[indCounter] <= 0; 
				end 

			end else begin 
				case(state_pn[pn_counter]) 
					IDLE_PN: begin 
						if(start_processing[pn_counter]) begin 
							state_pn[pn_counter] <= START; 
							processing_priority[pn_counter] <= 2**pn_counter; 

							// Checking for init-mode 
							if(processing_init[pn_counter]) begin 
								init_mode[pn_counter] <= 1; 
							end else begin 
								init_mode[pn_counter] <= 0; 
							end 
						end 
					end 

					START: begin 
						if((2**pn_counter > priority_sum/2) && current_inds_approved) begin 
							processing_priority[pn_counter] <= 0; 
							ind_in_use[2*pn_counter] <= individual_proposed_1; 
							ind_in_use[2*pn_counter + 1] <= individual_proposed_2;
							state_pn[pn_counter] <= LOAD_PARENT_1; 

							pop_data_mem_to_pn[pn_counter] <= pop_mem[2*individual_proposed_1]; 

							start_parent_1_regfile[pn_counter] <= 1; 
							mem_access_counter[pn_counter] <= 1;
						end 
					end 

					LOAD_PARENT_1: begin
						run_counter_arr[pn_counter] <= run_counter_arr[pn_counter] + 1; 
						if(mem_access_counter[pn_counter] < lpNumOfMemAccesses) begin 
							mem_access_counter[pn_counter] <= mem_access_counter[pn_counter] + 1; 

							pop_data_mem_to_pn[pn_counter] <= pop_mem[2*ind_in_use[2*pn_counter] + mem_access_counter[pn_counter]]; 

						end else begin 
							finished_parent_1_regfile[pn_counter] <= 1; 
							state_pn[pn_counter] <= START_LOADING_PARENT_2; 
							start_parent_1_regfile[pn_counter] <= 0;
						end 
					end 

					START_LOADING_PARENT_2: begin 
						state_pn[pn_counter] <= LOAD_PARENT_2; 
						finished_parent_1_regfile[pn_counter] <= 0; 

						pop_data_mem_to_pn[pn_counter] <= pop_mem[2*ind_in_use[2*pn_counter+1]];

						start_parent_2_regfile[pn_counter] <= 1; 
						mem_access_counter[pn_counter] <= 1;

					end

					LOAD_PARENT_2: begin 
						if(mem_access_counter[pn_counter] < lpNumOfMemAccesses) begin 
							mem_access_counter[pn_counter] <= mem_access_counter[pn_counter] + 1; 

							pop_data_mem_to_pn[pn_counter] <= pop_mem[2*ind_in_use[2*pn_counter+1] + mem_access_counter[pn_counter]];

						end else begin 
							finished_parent_2_regfile[pn_counter] <= 1; 
							state_pn[pn_counter] <= WAIT_PROCESSING; 
							start_parent_2_regfile[pn_counter] <= 0;
						end 
					end 

					WAIT_PROCESSING: begin 
						finished_parent_2_regfile[pn_counter] <= 0; 
						if(transmission_ind_1_out[pn_counter]) begin 
							if(unchanged_transmission_out[pn_counter]) begin 
								state_pn[pn_counter] <= START_WB_OFFSPRING_2; 
							end else begin 

								pop_mem[2*ind_in_use[2*pn_counter]] <= pop_data_pn_to_mem[pn_counter]; 

								state_pn[pn_counter] <= WB_OFFSPRING_1; 
								mem_access_counter[pn_counter] <= 1; 
							end 

							if(better_fitness_value_out[pn_counter] < best_known_solutions[pn_counter]) begin 
								best_known_solutions[pn_counter] <= better_fitness_value_out[pn_counter]; 
								if(better_fitness_slot_out[pn_counter] == 1'b0) begin 
									best_known_solutions_addresses[pn_counter] <= ind_in_use[2*pn_counter]; 
								end else begin 
									best_known_solutions_addresses[pn_counter] <= ind_in_use[2*pn_counter + 1]; 
								end 
							end 

						end 
					end 

					WB_OFFSPRING_1: begin 
						if(mem_access_counter[pn_counter] < lpNumOfMemAccesses) begin 
							mem_access_counter[pn_counter] <= mem_access_counter[pn_counter] + 1; 

							pop_mem[2*ind_in_use[2*pn_counter]+mem_access_counter[pn_counter]] <= pop_data_pn_to_mem[pn_counter];

						end else begin 
							finished_parent_1_regfile[pn_counter] <= 1; 
							state_pn[pn_counter] <= START_WB_OFFSPRING_2; 
							start_parent_1_regfile[pn_counter] <= 0;
						end
					end 

					START_WB_OFFSPRING_2: begin 
						finished_parent_1_regfile[pn_counter] <= 0; 
						if(transmission_ind_2_out[pn_counter]) begin 
							if(unchanged_transmission_out[pn_counter]) begin 
								state_pn[pn_counter] <= FINAL; 
							end else begin 

								pop_mem[2*ind_in_use[2*pn_counter+1]] <= pop_data_pn_to_mem[pn_counter]; 

								state_pn[pn_counter] <= WB_OFFSPRING_2; 
								mem_access_counter[pn_counter] <= 1; 
							end 
						end 
					end 

					WB_OFFSPRING_2: begin 
						if(mem_access_counter[pn_counter] < lpNumOfMemAccesses) begin 
							mem_access_counter[pn_counter] <= mem_access_counter[pn_counter] + 1; 

							pop_mem[2*ind_in_use[2*pn_counter+1]+mem_access_counter[pn_counter]] <= pop_data_pn_to_mem[pn_counter]; 

						end else begin 
							finished_parent_2_regfile[pn_counter] <= 1; 
							state_pn[pn_counter] <= FINAL; 
							start_parent_1_regfile[pn_counter] <= 0;
						end
					end 

					FINAL: begin 
						finished_parent_2_regfile[pn_counter] <= 0; 
						if(pn_counter == 0 && load_best_solution_processing_node_1) begin 
							state_pn[pn_counter] <= LOAD_BEST_SOLUTION; 
						end else begin 
							state_pn[pn_counter] <= IDLE_PN; 
						end 
					end 

					LOAD_BEST_SOLUTION: begin 


					end 
				endcase
			end 
		end
	end 
				
endmodule