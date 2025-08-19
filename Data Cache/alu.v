// ============================================================================
//  CO224 – Computer Architecture Laboratory
//  Lab 5 – Part 5 : ALU
//  Group 09
// ============================================================================

`timescale  1ns/100ps

// *********************************************************************
//  ALU MODULE
// *********************************************************************
module alu(DATA1, DATA2, RESULT, ZERO, SELECT);
    input signed [7:0] DATA1, DATA2; // ALU operands
    input [2:0] SELECT; // ALU operation select

    output signed [7:0] RESULT, forwout, addout, andout, orout, mulout, sllout, srout, rorout;
    output ZERO; // Zero flag for branch operations

    // Instantiate the individual functional units of the ALU
    forwardunit f1(DATA2, forwout); // Forward unit
    addunit a1(DATA1, DATA2, addout); // Adder unit
    andunit and1(DATA1, DATA2, andout); // AND unit
    orunit or1(DATA1, DATA2, orout); // OR unit
    mulunit mul1(DATA1, DATA2, mulout); // Multiplication unit
    sr sr1(DATA1, DATA2, srout); // Shift right unit
    sl sll1(DATA1, DATA2, sllout); // Shift left unit
    rotate_right ror1(DATA1, DATA2, rorout); // Rotate right unit

    // Instantiate the multiplexer to select the final result based on the SELECT input
    mux m1(forwout, addout, andout, orout, mulout, sllout, srout, rorout, SELECT, RESULT);

    // Compute ZERO flag (set if RESULT == 0)
    and (ZERO, ~RESULT[0], ~RESULT[1], ~RESULT[2], ~RESULT[3], ~RESULT[4], ~RESULT[5], ~RESULT[6], ~RESULT[7]);

endmodule

// *********************************************************************
//  ALU FUNCTION MODULES
// *********************************************************************

// FORWARD FUNCTION MODULE
module forwardunit(DATA2, RESULT);
    input [7:0] DATA2;
    output signed [7:0] RESULT;

    // Forward DATA2 to RESULT
    assign #1 RESULT = DATA2;
endmodule

// ADD FUNCTION MODULE
module addunit(DATA1, DATA2, RESULT);
    input signed [7:0] DATA1, DATA2;
    output signed [7:0] RESULT;

    // Perform the addition operation
    assign #2 RESULT = (DATA1 + DATA2);
endmodule

// AND FUNCTION MODULE
module andunit(DATA1, DATA2, RESULT);
    input signed [7:0] DATA1, DATA2;
    output [7:0] RESULT;

    // Perform the AND operation
    assign #1 RESULT = (DATA1 & DATA2);
endmodule

// OR FUNCTION MODULE
module orunit(DATA1, DATA2, RESULT);
    input signed [7:0] DATA1, DATA2;
    output [7:0] RESULT;

    // Perform the OR operation
    assign #1 RESULT = (DATA1 | DATA2);
endmodule

// MULTIPLICATION FUNCTION MODULE
module mulunit(DATA1, DATA2, RESULT);
    input [7:0] DATA1, DATA2;
    output [7:0] RESULT;

    // Internal wires to store shifted versions of DATA1
    wire [7:0] shift [0:7];
    wire [7:0] temp0, temp1, temp2, temp3, temp4, temp5, temp6, temp7;
    integer i;

    // Generate partial products using control bits
    shift_mux2x1 mm1(DATA1, 8'b00000000, temp0, DATA2[0]);
    shift_mux2x1 mm2(DATA1, 8'b00000000, temp1, DATA2[1]);
    shift_mux2x1 mm3(DATA1, 8'b00000000, temp2, DATA2[2]);
    shift_mux2x1 mm4(DATA1, 8'b00000000, temp3, DATA2[3]);
    shift_mux2x1 mm5(DATA1, 8'b00000000, temp4, DATA2[4]);
    shift_mux2x1 mm6(DATA1, 8'b00000000, temp5, DATA2[5]);
    shift_mux2x1 mm7(DATA1, 8'b00000000, temp6, DATA2[6]);
    shift_mux2x1 mm8(DATA1, 8'b00000000, temp7, DATA2[7]);

    // Shift each partial product
    sl sll2(temp0, 8'b00000000, shift[0]);
    sl sll3(temp1, 8'b00000001, shift[1]);
    sl sll4(temp2, 8'b00000010, shift[2]);
    sl sll5(temp3, 8'b00000011, shift[3]);
    sl sll6(temp4, 8'b00000100, shift[4]);
    sl sll7(temp5, 8'b00000101, shift[5]);
    sl sll8(temp6, 8'b00000110, shift[6]);
    sl sll9(temp7, 8'b00000111, shift[7]);

    // Add the shifted values together to get the final multiplication result
    assign #2 RESULT = shift[0] + shift[1] + shift[2] + shift[3] + shift[4] + shift[5] + shift[6] + shift[7];
endmodule

// SHIFT LEFT FUNCTION MODULE
module sl(DATA1, DATA2, RESULT);
    input [7:0] DATA1, DATA2;
    output [7:0] RESULT;

    // Internal wires to store intermediate shifted values
    wire [7:0] shifted1, OUT1, OUT2, OUT4;
    assign shifted1 = {DATA1[6:0], 1'b0};

    // Perform the shifting using multiplexers
    shift_mux2x1 ml1(shifted1, DATA1, OUT1, DATA2[0]);
    shift_mux2x1 ml2({OUT1[5:0], 2'b00}, OUT1, OUT2, DATA2[1]);
    shift_mux2x1 ml4({OUT2[3:0], 4'b0000}, OUT2, OUT4, DATA2[2]);

    // Output the final result of the shift operation
    assign #1 RESULT = OUT4;
endmodule

// SHIFT RIGHT FUNCTION MODULE
module sr(DATA1, DATA2, RESULT);
    input [7:0] DATA1, DATA2;
    output [7:0] RESULT;

    // Internal wires for 2's complement and shifted data
    wire [7:0] comp, mod_data2, shifted1, OUT1, OUT2, OUT4;
    wire msb;

    // Calculate 2's complement of DATA2 for arithmetic shift
    assign #1 comp = ~DATA2 + 1;

    // Select the MSB for arithmetic shift right
    bit_mux2x1 muxmsb(DATA1[7], 1'b0, msb, DATA2[7]);

    // Handle modified DATA2 for arithmetic shifts
    shift_mux2x1 mod(comp, DATA2, mod_data2, DATA2[7]);
    assign shifted1 = {msb, DATA1[7:1]};

    // Perform the right shift using multiplexers
    shift_mux2x1 ml1(shifted1, DATA1, OUT1, mod_data2[0]);
    shift_mux2x1 ml2({{2{msb}}, OUT1[7:2]}, OUT1, OUT2, mod_data2[1]);
    shift_mux2x1 ml4({{4{msb}}, OUT2[7:4]}, OUT2, OUT4, mod_data2[2]);

    // Output the final result after the shift
    assign #1 RESULT = OUT4;
endmodule

// ROTATE RIGHT FUNCTION MODULE
module rotate_right(DATA1, DATA2, RESULT);
    // Perform a rotate right operation on DATA1 based on the shift value in DATA2
    input [7:0] DATA1, DATA2;
    output [7:0] RESULT;

    // Internal wires to store rotated values
    wire [7:0] rotate1, OUT1, OUT2, OUT4;
    assign rotate1 = {DATA1[0], DATA1[7:1]};

    // Perform rotations using multiplexers
    shift_mux2x1 mrr1(rotate1, DATA1, OUT1, DATA2[0]);
    shift_mux2x1 mrr2({OUT1[1:0], OUT1[7:2]}, OUT1, OUT2, DATA2[1]);
    shift_mux2x1 mrr4({OUT2[3:0], OUT2[7:4]}, OUT2, OUT4, DATA2[2]);

    // Output the final rotated result
    assign #1 RESULT = OUT4;
endmodule

// *********************************************************************
//  MUX MODULES
// *********************************************************************

// 8-to-1 ALU result multiplexer
module mux(in0, in1, in2, in3, in4, in5, in6, in7, sel, result);
    input signed [7:0] in0, in1, in2, in3, in4, in5, in6, in7;
    input [2:0] sel;
    output reg signed [7:0] result;

    // Use a case statement to select the appropriate input based on the value of sel
    always @(in0, in1, in2, in3, in4, in5, in6, in7, sel)
    begin
        case (sel)
            3'b000: result = in0; // forward
            3'b001: result = in1; // add
            3'b010: result = in2; // and
            3'b011: result = in3; // or
            3'b100: result = in4; // mult
            3'b101: result = in5; // sll
            3'b110: result = in6; // srl
            3'b111: result = in7; // ror
            default: result = -1; // Invalid
        endcase
    end
endmodule

// 8-bit 2x1 multiplexer for shifts
module shift_mux2x1(in1, in2, out, control);
    input [7:0] in1, in2;
    input control;
    output reg [7:0] out;

    // Choose between in1 and in2 based on the value of control
    always @(control, in1, in2)
    begin
        if(control)
            out = in1;
        else
            out = in2;
    end
endmodule

// 1-bit 2x1 multiplexer for sign bit selection
module bit_mux2x1(in1, in2, out, control);
    input in1, in2;
    input control;
    output reg out;

    // Choose between in1 and in2 based on the value of control
    always @(control, in1, in2)
    begin
        if(control)
            out = in1;
        else
            out = in2;
    end
endmodule
