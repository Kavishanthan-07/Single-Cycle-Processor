// ============================================================================
//  CO224 – Computer Architecture Laboratory
//  Lab 5 – Part 2 : Register File
//  Group 09
// ============================================================================

`timescale 1ns/100ps
// *********************************************************************
//  REGISTER FILE MODULE
// *********************************************************************

module reg_file(IN, OUT1, OUT2, INADDRESS, OUT1ADDRESS, OUT2ADDRESS, WRITE, CLK, RESET, BUSYWAIT);

    // -----------------------------------------------------------------
    // INPUTS
    // -----------------------------------------------------------------
    input signed [7:0] IN; // Data to write into the register file
    input [7:0] INADDRESS; // Register address to write to
    input [7:0] OUT1ADDRESS, OUT2ADDRESS; // Register address to read from
    input CLK; // Clock signal
    input WRITE; // Write enable signal
    input RESET; // Active-high reset signal
    input BUSYWAIT; // Memory/cache busywait signal


    // -----------------------------------------------------------------
    // OUTPUTS
    // -----------------------------------------------------------------
    output signed [7:0] OUT1, OUT2; // Data outputs from read registers
    
    // -----------------------------------------------------------------
    // REGISTER FILE DEFINITION
    // 8 general-purpose registers, each 8 bits wide
    // -----------------------------------------------------------------
    reg signed [7:0] regfile [0:7]; // 8 registers indexed from 0 to 7

    // -----------------------------------------------------------------
    // DEBUGGING OUTPUT: Register waveform dumping
    // This block creates a VCD file with register values for simulation
    // -----------------------------------------------------------------
    initial
    begin 
        $dumpfile("cpu_wavedata.vcd"); // VCD file for waveform viewing
        for(i = 0; i < 8; i = i + 1)
            $dumpvars(1, regfile[i]); // Track all registers in waveform
    end

    // -----------------------------------------------------------------
    // DEBUGGING DISPLAY: Console output on register value changes
    // This prints the register contents whenever they change
    // -----------------------------------------------------------------
    initial
	begin
		#5;
		$display("\n\t\t\t CHANGE OF REGISTER VALUES");
		$display("\t\ttime\treg0\treg1\treg2\treg3\treg4\treg5\treg6\treg7");
		$monitor($time, "\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d",regfile[0],regfile[1],regfile[2],regfile[3],regfile[4],regfile[5],regfile[6],regfile[7]);
	end

    // -----------------------------------------------------------------
    // ASYNCHRONOUS READ OPERATIONS
    // OUT1 and OUT2 provide values from specified registers
    // Reads happen asynchronously with a delay of 2ns
    // -----------------------------------------------------------------
    assign #2 OUT1 = regfile[OUT1ADDRESS];
    assign #2 OUT2 = regfile[OUT2ADDRESS];

    // -----------------------------------------------------------------
    // SYNCHRONOUS WRITE OPERATION
    // Writes occur on the rising edge of CLK when:
    // - WRITE = 1 (enabled)
    // - RESET = 0 (not in reset)
    // - BUSYWAIT = 0 (no memory/data stall)
    // Write delay is 1ns
    // -----------------------------------------------------------------
    always @(posedge CLK)
    begin
        if (WRITE && !RESET && !BUSYWAIT)
            #1 regfile[INADDRESS] = IN;  // Perform write to addressed register
    end
     
    // -----------------------------------------------------------------
    // SYNCHRONOUS RESET OPERATION
    // On rising CLK edge, if RESET is high, all registers are cleared to 0
    // Reset delay is 1ns
    // -----------------------------------------------------------------
    integer i;
    always @(posedge CLK)
    begin
        if (RESET)
            #1
            begin
                for(i = 0; i < 8; i = i + 1)
                    regfile[i] = 0;  // Reset all registers to 0
            end
    end

endmodule
