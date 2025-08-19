// ============================================================================
//  CO224 – Computer Architecture Laboratory
//  Lab 5 – Part 5 : Arithmetic-Logic Unit (ALU)
//  Group 09
// ----------------------------------------------------------------------------
//  File      : alu.v
//  Contents  : 8-bit ALU with eight separate functional units and an 8-to-1
//              result multiplexer.  A dedicated ZERO flag asserts when the
//              selected result equals 0.
//              - All functional units operate concurrently; the multiplexer
//                chooses the required output according to SELECT.
//              - All datapath delays are modelled with Verilog inertial
//                delays (#) to reflect realistic module latencies for
//                simulation.
//
//              SELECT encoding
//              ┌────┬─────────────┐
//              │000 │ Forward      │
//              │001 │ Add          │
//              │010 │ AND          │
//              │011 │ OR           │
//              │100 │ Multiply     │
//              │101 │ Shift Left   │
//              │110 │ Shift Right  │
//              │111 │ Rotate Right │
//              └────┴─────────────┘
// ============================================================================



// ============================================================================
//  Top-level ALU
// ============================================================================
module alu (
    /*----------------------------------------------------------------------
     * Operand inputs
     *--------------------------------------------------------------------*/
    input  signed [7:0] DATA1,           // 1st operand (arithmetic / logical)
    input  signed [7:0] DATA2,           // 2nd operand / shift amount / etc.

    /*----------------------------------------------------------------------
     * Operation selector
     *--------------------------------------------------------------------*/
    input        [2:0] SELECT,           // Chooses functional-unit result

    /*----------------------------------------------------------------------
     * Outputs
     *--------------------------------------------------------------------*/
    output signed [7:0] RESULT,          // Final ALU output (post-mux)
    output signed [7:0] forwout,         // Individual unit results – exposed
    output signed [7:0] addout,          //   mainly for waveform inspection
    output        [7:0] andout,
    output        [7:0] orout,
    output        [7:0] mulout,
    output        [7:0] sllout,
    output        [7:0] srout,
    output        [7:0] rorout,
    output              ZERO             // High -> RESULT == 0
);

    // ---------------------------------------------------------------------
    // Functional-unit instantiation
    // ---------------------------------------------------------------------
    forwardunit    f1  (DATA2,           forwout); // pass-through
    addunit        a1  (DATA1, DATA2,    addout);  // signed addition
    andunit        and1(DATA1, DATA2,    andout);  // bitwise AND
    orunit         or1 (DATA1, DATA2,    orout);   // bitwise OR
    mulunit        mul1(DATA1, DATA2,    mulout);  // shift-and-add multiply
    sr             sr1 (DATA1, DATA2,    srout);   // logical / arithmetic SR
    sl             sll1(DATA1, DATA2,    sllout);  // logical SL
    rotate_right   ror1(DATA1, DATA2,    rorout);  // barrel rotate right

    // ---------------------------------------------------------------------
    // Result multiplexer – selects one functional-unit output
    // ---------------------------------------------------------------------
    mux            m1  (forwout, addout, andout, orout,
                        mulout,  sllout, srout,  rorout,
                        SELECT,  RESULT);

    // ---------------------------------------------------------------------
    // ZERO flag generation
    //   Produces ’1’ only when every bit of RESULT is ’0’.
    //   Implemented with an 8-input AND gate fed by inverted RESULT bits.
    //   Net delay = logic delay of ‘and’ primitive (device-dependent).
    // ---------------------------------------------------------------------
    and (ZERO, ~RESULT[0], ~RESULT[1], ~RESULT[2], ~RESULT[3],~RESULT[4], ~RESULT[5], ~RESULT[6], ~RESULT[7]);

endmodule



// ============================================================================
//  Functional-unit definitions
// ============================================================================

// ---------------------------------------------------------
//  Pass-through (Forward) Unit
//    Latency #1 → matches single-cycle wire delay.
// ---------------------------------------------------------
module forwardunit (
    input  [7:0] DATA2,
    output signed [7:0] RESULT
);
    assign #1 RESULT = DATA2;
endmodule



// ---------------------------------------------------------
//  Adder Unit (signed addition)
//    Latency #2 – simple ripple adder model.
// ---------------------------------------------------------
module addunit (
    input  signed [7:0] DATA1, DATA2,
    output signed [7:0] RESULT
);
    assign #2 RESULT = DATA1 + DATA2;
endmodule



// ---------------------------------------------------------
//  Bitwise AND Unit
//    Latency #1
// ---------------------------------------------------------
module andunit (
    input  signed [7:0] DATA1, DATA2,
    output       [7:0] RESULT
);
    assign #1 RESULT = DATA1 & DATA2;
endmodule



// ---------------------------------------------------------
//  Bitwise OR Unit
//    Latency #1
// ---------------------------------------------------------
module orunit (
    input  signed [7:0] DATA1, DATA2,
    output       [7:0] RESULT
);
    assign #1 RESULT = DATA1 | DATA2;
endmodule



// ---------------------------------------------------------
//  Multiply Unit (shift-and-add implementation)
//    Latency #2 from last additive stage to RESULT.
//    * DATA2 acts as multiplier; each bit gates a shifted
//      copy of DATA1 which is then summed.
//    * Uses helper 2×1 multiplexers and shift-left blocks
//      to build the partial products.
// ---------------------------------------------------------
module mulunit (
    input  [7:0] DATA1, DATA2,
    output [7:0] RESULT
);
    /*  Internal signals  */
    wire [7:0] shift [0:7];              // eight shifted partial products
    wire [7:0] temp0, temp1, temp2, temp3,
               temp4, temp5, temp6, temp7;

    /*  Select DATA1 or 0 based on each multiplier bit  */
    shift_mux2x1 mm1(DATA1, 8'b0, temp0, DATA2[0]);
    shift_mux2x1 mm2(DATA1, 8'b0, temp1, DATA2[1]);
    shift_mux2x1 mm3(DATA1, 8'b0, temp2, DATA2[2]);
    shift_mux2x1 mm4(DATA1, 8'b0, temp3, DATA2[3]);
    shift_mux2x1 mm5(DATA1, 8'b0, temp4, DATA2[4]);
    shift_mux2x1 mm6(DATA1, 8'b0, temp5, DATA2[5]);
    shift_mux2x1 mm7(DATA1, 8'b0, temp6, DATA2[6]);
    shift_mux2x1 mm8(DATA1, 8'b0, temp7, DATA2[7]);

    /*  Appropriate left shifts (0-7 positions)  */
    sl sll2(temp0, 8'd0, shift[0]);      // shift 0
    sl sll3(temp1, 8'd1, shift[1]);      // shift 1
    sl sll4(temp2, 8'd2, shift[2]);      // shift 2
    sl sll5(temp3, 8'd3, shift[3]);      // shift 3
    sl sll6(temp4, 8'd4, shift[4]);      // shift 4
    sl sll7(temp5, 8'd5, shift[5]);      // shift 5
    sl sll8(temp6, 8'd6, shift[6]);      // shift 6
    sl sll9(temp7, 8'd7, shift[7]);      // shift 7

    /*  Sum all partial products (width truncation to 8 bits)  */
    assign #2 RESULT = shift[0] + shift[1] + shift[2] + shift[3] +
                       shift[4] + shift[5] + shift[6] + shift[7];
endmodule



// ---------------------------------------------------------
//  Logical Shift-Left Unit
//    Shifts DATA1 left by the amount (0-7) specified in
//    the three LSBs of DATA2. Unused bits in DATA2 are
//    ignored.  Latency #1.
// ---------------------------------------------------------
module sl (
    input  [7:0] DATA1, DATA2,
    output [7:0] RESULT
);
    /*  First-stage single-bit shift  */
    wire [7:0] shifted1 = {DATA1[6:0], 1'b0};
    wire [7:0] OUT1, OUT2, OUT4;

    /*  Barrel-style three-stage shift using 2×1 muxes  */
    shift_mux2x1 ml1( shifted1,
                      DATA1,             OUT1, DATA2[0]);      // <<1?
    shift_mux2x1 ml2({OUT1[5:0], 2'b0},
                      OUT1,              OUT2, DATA2[1]);      // <<2?
    shift_mux2x1 ml4({OUT2[3:0], 4'b0},
                      OUT2,              OUT4, DATA2[2]);      // <<4?

    assign #1 RESULT = OUT4;
endmodule



// ---------------------------------------------------------
//  Shift-Right Unit (logical or arithmetic)
//    The MSB of DATA2 decides the mode:
//      0 → Logical SR (fill with zeros)
//      1 → Arithmetic SR (sign-extend with original MSB)
//    Shift amount = DATA2[2:0].  Latency #1.
// ---------------------------------------------------------
module sr (
    input  [7:0] DATA1, DATA2,
    output [7:0] RESULT
);
    /*--------------------------------------------------
     * Determine mode and compute modified shift amount
     *  – For arithmetic SR, use 2's-complement of DATA2
     *    to prevent double-counting during left-barrel.
     *------------------------------------------------*/
    wire [7:0] comp        = ~DATA2 + 1;        // 2's-comp
    wire       msb;                             // fill-bit
    bit_mux2x1 muxmsb(DATA1[7], 1'b0, msb, DATA2[7]);

    wire [7:0] mod_data2;
    shift_mux2x1 mod(comp, DATA2, mod_data2, DATA2[7]);

    /*  Barrel shift right – three stages (1,2,4)   */
    wire [7:0] shifted1 = {msb, DATA1[7:1]};
    wire [7:0] OUT1, OUT2, OUT4;

    shift_mux2x1 ml1( shifted1,
                      DATA1,             OUT1, mod_data2[0]);  // >>1?
    shift_mux2x1 ml2({{2{msb}}, OUT1[7:2]},
                      OUT1,              OUT2, mod_data2[1]);  // >>2?
    shift_mux2x1 ml4({{4{msb}}, OUT2[7:4]},
                      OUT2,              OUT4, mod_data2[2]);  // >>4?

    assign #1 RESULT = OUT4;
endmodule



// ---------------------------------------------------------
//  Rotate-Right Unit
//    Rotates DATA1 right by DATA2[2:0] positions.
//    Latency #1.
// ---------------------------------------------------------
module rotate_right (
    input  [7:0] DATA1, DATA2,
    output [7:0] RESULT
);
    wire [7:0] rotate1 = {DATA1[0], DATA1[7:1]};
    wire [7:0] OUT1, OUT2, OUT4;

    shift_mux2x1 mrr1( rotate1,
                       DATA1,            OUT1, DATA2[0]);      // rot1?
    shift_mux2x1 mrr2({OUT1[1:0], OUT1[7:2]},
                       OUT1,            OUT2, DATA2[1]);       // rot2?
    shift_mux2x1 mrr4({OUT2[3:0], OUT2[7:4]},
                       OUT2,            OUT4, DATA2[2]);       // rot4?

    assign #1 RESULT = OUT4;
endmodule



// ============================================================================
//  Support modules (multiplexers)
// ============================================================================

// ---------------------------------------------------------
//  8-to-1 Multiplexer
//    Selects one of eight 8-bit inputs according to 3-bit
//    selector ‘sel’.  Combinational, zero delay (#0).
// ---------------------------------------------------------
module mux (
    input  signed [7:0] in0, in1, in2, in3,
                        in4, in5, in6, in7,
    input        [2:0]  sel,
    output reg   signed [7:0] result
);
    always @(*) begin
        case (sel)
            3'b000: result = in0;  // forward
            3'b001: result = in1;  // add
            3'b010: result = in2;  // and
            3'b011: result = in3;  // or
            3'b100: result = in4;  // mult
            3'b101: result = in5;  // sll
            3'b110: result = in6;  // srl
            3'b111: result = in7;  // ror
            default: result = -8'sd1; // indicates error
        endcase
    end
endmodule



// ---------------------------------------------------------
//  8-bit 2×1 Multiplexer
// ---------------------------------------------------------
module shift_mux2x1 (
    input  [7:0] in1, in2,
    input        control,               // 1 → select in1
    output reg [7:0] out
);
    always @(*) begin
        out = control ? in1 : in2;
    end
endmodule



// ---------------------------------------------------------
//  1-bit 2×1 Multiplexer
// ---------------------------------------------------------
module bit_mux2x1 (
    input  in1, in2,
    input  control,                     // 1 → select in1
    output reg out
);
    always @(*) begin
        out = control ? in1 : in2;
    end
endmodule
