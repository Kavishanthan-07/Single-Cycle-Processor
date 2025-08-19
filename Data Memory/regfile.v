// ============================================================================
//  CO224 – Computer Architecture Laboratory
//  Lab 6 – Part 1: Register File
//  Group 09
// ----------------------------------------------------------------------------
//  File      : reg_file.v
//  Function  : 8-register × 8-bit register file with two asynchronous read
//               ports and one synchronous write port.
//               • READ  : dual-port, asynchronous, 2 time-unit propagation
//                          delay from address/data change to bus update.
//               • WRITE : edge-triggered on the rising edge of CLK,
//                          1 time-unit write delay when WRITE=1, RESET=0
//                          and BUSYWAIT=0.
//               • RESET : synchronous active-high. On the next rising edge
//                          of CLK all registers are cleared to 0 after a
//                          1 time-unit delay.
//               • BUSYWAIT : stalls writes when asserted (logic-high).
// ----------------------------------------------------------------------------
// ============================================================================

module reg_file (
    /*----------------------------------------------------------------------
     * 8-bit data input bus for write-port
     *--------------------------------------------------------------------*/
    input  signed [7:0] IN,

    /*----------------------------------------------------------------------
     * 3-bit register-select addresses
     *   – INADDRESS   : destination register for write-port
     *   – OUT1ADDRESS : source register for read-port 1
     *   – OUT2ADDRESS : source register for read-port 2
     *--------------------------------------------------------------------*/
    input  [2:0] INADDRESS,              // WRITE address (one hot not required)
    input  [2:0] OUT1ADDRESS,            // READ-1 address
    input  [2:0] OUT2ADDRESS,            // READ-2 address

    /*----------------------------------------------------------------------
     * Control and timing signals
     *--------------------------------------------------------------------*/
    input        WRITE,                  // 1 → enable write on rising CLK
    input        CLK,                    // system clock (positive-edge triggered)
    input        RESET,                  // synchronous active-high reset
    input        BUSYWAIT,               // high → stall writes (cache/memory busy)

    /*----------------------------------------------------------------------
     * 8-bit data output buses for asynchronous read-ports
     *--------------------------------------------------------------------*/
    output signed [7:0] OUT1,            // data read from OUT1ADDRESS
    output signed [7:0] OUT2             // data read from OUT2ADDRESS
);

    // ---------------------------------------------------------------------
    // Internal storage – 8 registers, each 8 bits wide.
    // Index range 0-7 corresponds to 3-bit address space.
    // ---------------------------------------------------------------------
    reg signed [7:0] regfile [0:7];

    // ---------------------------------------------------------------------
    // Simulation waveform dumping
    //   – Generates cpu_wavedata.vcd containing all eight registers.
    //   – Useful for GTKWave / ModelSim inspection of register contents.
    // ---------------------------------------------------------------------
    integer i;
    initial begin
        $dumpfile("cpu_wavedata.vcd");
        for (i = 0; i < 8; i = i + 1)
            $dumpvars(1, regfile[i]);
    end

    // ---------------------------------------------------------------------
    // Asynchronous READ path
    //   • Any change on OUT1ADDRESS, OUT2ADDRESS, or the addressed register
    //     contents propagates to OUT1/OUT2 after #2 time-units.
    //   • Until the delay elapses, OUT1/OUT2 retain their previous values
    //     (inertial delay semantics).
    // ---------------------------------------------------------------------
    assign #2 OUT1 = regfile[OUT1ADDRESS];
    assign #2 OUT2 = regfile[OUT2ADDRESS];

    // ---------------------------------------------------------------------
    // Synchronous WRITE path
    //   • Triggered on the rising edge of CLK.
    //   • Conditions to perform write:
    //        – WRITE asserted (logic-high)
    //        – RESET de-asserted (logic-low)
    //        – BUSYWAIT de-asserted (logic-low)
    //   • New data appears in destination register after #1 time-unit.
    // ---------------------------------------------------------------------
    always @(posedge CLK) begin
        if (WRITE && !RESET && !BUSYWAIT)
            #1 regfile[INADDRESS] = IN;
    end

    // ---------------------------------------------------------------------
    // Synchronous RESET
    //   • When RESET is asserted high, on the next rising edge of CLK all
    //     registers are cleared to 0 after #1 time-unit.
    //   • WRITE attempts during the same edge are ignored because RESET
    //     has priority (checked first in separate always block above).
    // ---------------------------------------------------------------------
    always @(posedge CLK) begin
        if (RESET)
            #1 begin
                for (i = 0; i < 8; i = i + 1)
                    regfile[i] = 0;
            end
    end

endmodule
