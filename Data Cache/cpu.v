// ============================================================================
//  CO224 – Computer Architecture Laboratory
//  Lab 6 – Part 2 : Simple CPU + Test-bench
//  Group 09
// ============================================================================

// *********************************************************************
//  TEST-BENCH : cpu_tb
// *********************************************************************
`timescale  1ns/100ps
module cpu_tb;

    // -------------------------
    // Stimulus / observation
    // -------------------------
    reg  CLK, RESET; // Clock & synchronous reset
    wire [31:0] PC; // Program-counter address
    wire [31:0] INSTRUCTION; // 32-bit instruction from instruction memory
    wire [7:0]  REGOUT1, RESULT, READDATA; // Debug signal outputs from CPU
    wire        WRITE, READ, BUSYWAIT; // CPU control signals
    wire        mem_read, mem_write; // Cache control signals
    wire        mem_busy; // Memory busy signal
    wire [5:0] mem_address; // Data memory address
    wire [31:0] mem_writedata, mem_readdata; // Data to/from memory

    // -------------------------
    // Instruction Memory
    // 1 k × 8-bit – initialised from ext. file “instr_mem.mem”
    // -------------------------
    reg [7:0] instr_mem [0:1023]; // 1KB instruction memory 

    initial 
    begin
        $readmemb("instr_mem.mem", instr_mem); // Load instructions
    end

    // Fetch instruction in little-endian format with #2 delay
    assign #2 INSTRUCTION = {instr_mem[PC+3], instr_mem[PC+2], instr_mem[PC+1], instr_mem[PC]};

    // -------------------------
    // Instantiate CPU, Cache and Memory
    // -------------------------
    cpu mycpu(PC, REGOUT1,RESULT,WRITE,READ,READDATA,BUSYWAIT,INSTRUCTION, CLK, RESET);
    
    dcache cache1(mem_read,mem_write,mem_address,mem_writedata,mem_readdata,mem_busy,BUSYWAIT,READDATA,REGOUT1,RESULT,READ,WRITE,CLK,RESET);

    data_memory mem1(CLK,RESET,mem_read,mem_write,mem_address,mem_writedata,mem_readdata,mem_busy);

    // -------------------------
    // Simulation control and waveform generation
    // -------------------------
    initial 
    begin
        $dumpfile("cpu_wavedata.vcd"); // Output VCD file
        $dumpvars(0, cpu_tb); // Dump all variables

        CLK   = 1'b0;
        RESET = 1'b1; // Assert reset
        #5 RESET = 1'b0; // Deassert reset after 5ns            

        #3000 $finish; // stop after 3000ns
    end

    // Clock generation: 8ns period (toggle every 4ns)
    always #4 CLK = ~CLK;

endmodule



// *********************************************************************
//  CPU CORE (Single-cycle with stall support)
// *********************************************************************
module cpu(PC, REGOUT1,RESULT,WRITE,READ,READDATA,BUSYWAIT,INSTRUCTION, CLK, RESET);

    // Inputs and outputs
    input              CLK, RESET;            // System timing
    input      [31:0]  INSTRUCTION;           // Fetched instruction
    output     [31:0]  PC_OUT;                
    output reg [31:0]  PC;                    

    // Register-file / ALU interface wires
    input      [7:0]   READDATA, WRITEDATA, INPUT2, READREG2, READREG1, WRITEREG;
    input      [7:0]   OPCODE;
    wire               ZERO;
    output     [7:0]   RESULT, REGOUT1, REGOUT2, temp2, temp3;

    // Memory-system handshake
    input              BUSYWAIT;
    output reg         READ, WRITE;
    reg WRITEENABLE;
    reg [2:0] temp_sel; // ALU control
    reg comp,imm,j,b,sra,write_mux_s;
    // -----------------------------------------------------------------
    // Program-counter & branch / jump wiring
    // -----------------------------------------------------------------
    wire        pc_j, pc_b;
    input signed [7:0] OFFSET;                // 8-bit immediate offset
    output     [31:0]  PC_JUMP;               // target address (PC+4+off)
    wire       [31:0]  PC_BRANCH, PC_NEXT;
    input      [31:0]  PC_UPDATE;             // PC+4

    // Arithmetic-shift specific
    wire [7:0] complement2, sra_result;

    // -----------------------------------------------------------------
    // Instruction-field split
    // -----------------------------------------------------------------
    assign {OPCODE, WRITEREG, READREG1, READREG2} = INSTRUCTION;

    // Clear memory control on each new instruction
    always @(PC) 
    begin
        READ  = 0;
        WRITE = 0;
    end

    // -----------------------------------------------------------------
    // PC generation logic
    // -----------------------------------------------------------------
    assign OFFSET = INSTRUCTION[23:16];

    pc_adder   add1 (PC, PC_UPDATE); // PC + 4
    jump_adder j1   (PC_UPDATE, OFFSET, PC_JUMP); // PC + offset
    mux2x1_32  m3   (PC_JUMP, PC_UPDATE, PC_BRANCH, pc_j); // Select jump
    mux2x1_32  m4   (PC_JUMP, PC_BRANCH, PC_NEXT,   pc_b); // Select branch
    mux2x1_32  m5   (PC, PC_NEXT, PC_OUT, BUSYWAIT); // Stall mux

    PC_update  pc1  (RESET, PC, PC_OUT, CLK); // PC update

    // always @(BUSYWAIT, PC_OUT) begin
    //     if (!BUSYWAIT)
    //         PC = PC_OUT;
    //     else
    //         PC = PC;
    // end

    // -----------------------------------------------------------------
    // Register file instantiation
    // -----------------------------------------------------------------
    reg_file reg1(WRITEDATA, REGOUT1, REGOUT2,WRITEREG,READREG1,READREG2, WRITEENABLE, CLK, RESET,BUSYWAIT);
    // -----------------------------------------------------------------
    // Control-unit decode table – unchanged bit-pattern literals
    // -----------------------------------------------------------------
    complement_module c1 (REGOUT2, complement2);

    always @(OPCODE, PC) begin
        case (OPCODE)
            8'b00000000: {WRITEENABLE,temp_sel,comp,imm,b,j,sra, READ,WRITE,write_mux_s} = #1 12'b1_000_0_1_0_0_0_0_0_0; // loadi
            8'b00000001: {WRITEENABLE,temp_sel,comp,imm,b,j,sra, READ,WRITE,write_mux_s} = #1 12'b1_000_0_0_0_0_0_0_0_0; // mov
            8'b00000010: {WRITEENABLE,temp_sel,comp,imm,b,j,sra, READ,WRITE,write_mux_s} = #1 12'b1_001_0_0_0_0_0_0_0_0; // add
            8'b00000011: {WRITEENABLE,temp_sel,comp,imm,b,j,sra, READ,WRITE,write_mux_s} = #1 12'b1_001_1_0_0_0_0_0_0_0; // sub
            8'b00000100: {WRITEENABLE,temp_sel,comp,imm,b,j,sra, READ,WRITE,write_mux_s} = #1 12'b1_010_0_0_0_0_0_0_0_0; // and
            8'b00000101: {WRITEENABLE,temp_sel,comp,imm,b,j,sra, READ,WRITE,write_mux_s} = #1 12'b1_011_0_0_0_0_0_0_0_0; // or
            8'b00000110: {WRITEENABLE,temp_sel,comp,imm,b,j,sra, READ,WRITE,write_mux_s} = #1 12'b0_000_0_0_0_1_0_0_0_0; // jump
            8'b00000111: {WRITEENABLE,temp_sel,comp,imm,b,j,sra, READ,WRITE,write_mux_s} = #1 12'b0_001_1_0_1_0_0_0_0_0; // beq
            8'b00001000: {WRITEENABLE,temp_sel,comp,imm,b,j,sra, READ,WRITE,write_mux_s} = #1 12'b0_001_1_0_1_1_0_0_0_0; // bne
            8'b00001001: {WRITEENABLE,temp_sel,comp,imm,b,j,sra, READ,WRITE,write_mux_s} = #1 12'b1_100_0_0_0_0_0_0_0_0; // mult
            8'b00001010: {WRITEENABLE,temp_sel,comp,imm,b,j,sra, READ,WRITE,write_mux_s} = #1 12'b1_101_0_1_0_0_0_0_0_0; // sll
            8'b00001011: {WRITEENABLE,temp_sel,comp,imm,b,j,sra, READ,WRITE,write_mux_s} = #1 12'b1_110_0_1_0_0_0_0_0_0; // srl
            8'b00001100: {WRITEENABLE,temp_sel,comp,imm,b,j,sra, READ,WRITE,write_mux_s} = #1 12'b1_110_0_1_0_0_1_0_0_0; // sra
            8'b00001101: {WRITEENABLE,temp_sel,comp,imm,b,j,sra, READ,WRITE,write_mux_s} = #1 12'b1_111_0_1_0_0_0_0_0_0; // ror
            8'b00001110: {WRITEENABLE,temp_sel,comp,imm,b,j,sra, READ,WRITE,write_mux_s} = #1 12'b1_000_0_0_0_0_0_1_0_1; // lwd
            8'b00001111: {WRITEENABLE,temp_sel,comp,imm,b,j,sra, READ,WRITE,write_mux_s} = #1 12'b1_000_0_1_0_0_0_1_0_1; // lwi
            8'b00010000: {WRITEENABLE,temp_sel,comp,imm,b,j,sra, READ,WRITE,write_mux_s} = #1 12'b0_000_0_0_0_0_0_0_1_0; // swd
            8'b00010001: {WRITEENABLE,temp_sel,comp,imm,b,j,sra, READ,WRITE,write_mux_s} = #1 12'b0_000_0_1_0_0_0_0_1_0; // swi
        endcase
    end

    // Operand-selection
    mux2x1 m1(complement2, REGOUT2, temp2, comp);
    mux2x1 m2(INSTRUCTION[7:0], temp2, temp3, imm);
    complement_module c2(temp3, sra_result);
    mux2x1 ma(sra_result, temp3, INPUT2, sra);

    // ALU instantiation
    alu alu1(REGOUT1, INPUT2, RESULT, ZERO, temp_sel);

    // Write-back selection
    mux2x1 write_mux(READDATA, RESULT, WRITEDATA, write_mux_s);

    // Branch & jump flag expressions
    assign pc_j = ~b && j;
    assign pc_b =  b && (j ^ ZERO);

endmodule



// *********************************************************************
//  SUPPORT MODULES
// *********************************************************************

// 8-bit 2×1 MUX
module mux2x1(in1, in2, out, control);
    input  [7:0] in1, in2;
    input        control;
    output reg [7:0] out;
    always @(control, in1, in2)
    begin
        if(control)
        begin
            out = in1;
        end
        else
        begin
            out = in2;
        end
    end
endmodule

// 32-bit 2×1 MUX
module mux2x1_32(in1, in2, out, control);
    input  [31:0] in1, in2;
    input         control;
    output reg [31:0] out;
    always @(control, in1, in2)
    begin
        if(control)
        begin
            out = in1;
        end
        else
        begin
            out = in2;
        end
    end
endmodule

// Synchronous PC register
module PC_update(RESET, PC_0, PC_NEXT, CLK);
    output reg [31:0] PC_0;
    input             CLK, RESET;
    input      [31:0] PC_NEXT;
    always @(posedge CLK)
        if (RESET) PC_0 = #1 0;
        else       PC_0 = #1 PC_NEXT;
endmodule

// PC + OFFSET adder (jump / branch)
module jump_adder(PC_4, OFFSET, PC_JUMP);
    input      [31:0] PC_4;
    output     [31:0] PC_JUMP;
    input signed [7:0] OFFSET;
    wire [31:0] temp;
    assign temp = {{22{OFFSET[7]}}, OFFSET[7:0], {2{1'b0}}}; // sign-extend & <<2
    assign #2 PC_JUMP = temp + PC_4;
endmodule

// PC + 4 adder
module pc_adder(PC_0, PC_1);
    input  [31:0] PC_0;
    output [31:0] PC_1;
    assign #1 PC_1 = PC_0 + 4;
endmodule

// 2's complement (8-bit)
module complement_module(DATA, COMPDATA);
    input  [7:0] DATA;
    output [7:0] COMPDATA;
    assign #1 COMPDATA = ~DATA + 1;
endmodule



// *********************************************************************
//  External units
// *********************************************************************
`include "alu.v"
`include "regfile.v"
`include "data_memory_2.v"
`include "dcacheFSM_skeleton.v"