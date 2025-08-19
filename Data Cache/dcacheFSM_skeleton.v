/*
Module  : Data Cache 
Author  : Isuru Nawinne, Kisaru Liyanage
Date    : 25/05/2020

Description	:

This file presents a skeleton implementation of the cache controller using a Finite State Machine model. Note that this code is not complete.
*/
`timescale 1ns/100ps
module dcache (mem_read,mem_write,mem_address,mem_writedata,mem_readdata,mem_busywait,busywait,READDATA,WRITEDATA,ADDRESS,read,write,clk,reset);

    
    /*
    Combinational part for indexing, tag comparison for hit deciding, etc.
    ...
    ...
    */
    
    //Input ports
    input clk,reset,read,write; // Control signals comin from CPU
    input [7:0] WRITEDATA, ADDRESS;
    input [31:0] mem_readdata;
    input mem_busywait;

    //Outputs ports
    output reg [0:31] mem_writedata;
    output reg busywait,mem_read,mem_write;
    output reg [7:0] READDATA;
    output reg [5:0] mem_address;

    reg valid_bit [0:7];
    reg dirty_bit [0:7];
    reg [2:0] tag_bits [0:7];
    reg [31:0] data [0:8];
    output [7:0] out;
    wire [2:0] tag,cache_tag,index;
    wire [1:0] offset;
    wire [31:0] data_block;
    wire dirty,valid,tag_comp; 
    reg hit;


    initial
    begin
        $dumpfile("cpu_wavedata.vcd");
        for (i = 0; i < 8 ;i++)
            $dumpvars(1, valid_bit[i], tag_bits[i], data[i], dirty_bit[i]);
    end

    always @(read,write)
    begin
        busywait = (read||write) ? 1 : 0;
    end

    assign {tag, index, offset} = ADDRESS;

    assign #1 {dirty, valid, cache_tag, data_block} = {dirty_bit[index], valid_bit[index], tag_bits[index], data[index]};

    mux4x1 mux1 (data_block[31:24],data_block[23:16],data_block[15:8],data_block[7:0],out,offset);

    assign tag_comp = ~(tag ^ cache_tag);
    always @(*)
        begin
            if((read||write))
            #0.9 hit = tag_comp && valid;
        end

    always @(hit, read, out)
        begin
            if(hit && read && !write)
            begin
                busywait = 0;
                READDATA = out;
            end
        end

    always @(posedge clk)
    begin
        if(write && hit && !read)
        begin
            busywait = 0;

            case(offset)
                2'b00: data[index][31:24] = #1 WRITEDATA;
                2'b01: data[index][23:16] = #1 WRITEDATA;
                2'b10: data[index][15:8]  = #1 WRITEDATA;
                2'b11: data[index][7:0]   = #1 WRITEDATA;
            endcase

            dirty_bit[index] = 1;
            valid_bit[index] = 1;
        end
    end
    /* Cache Controller FSM Start */

    parameter IDLE = 3'b000, MEM_READ = 3'b001, MEM_WRITE = 3'b010,CACHE_WRITE = 3'b011;
    reg [2:0] state, next_state;

    // combinational next state logic
    always @(*)
    begin
        case (state)
            IDLE:
                if ((read || write) && !dirty && !hit)  
                    next_state = MEM_READ;
                else if ((read || write) && dirty && !hit)
                    next_state = MEM_WRITE;
                else
                    next_state = IDLE;
            
            MEM_READ:
                if (!mem_busywait)
                    next_state = IDLE;
                else    
                    next_state = MEM_READ;
            
            MEM_WRITE:
                if ((read || write) && !mem_busywait)
                    next_state = MEM_READ;
                else
                    next_state = MEM_WRITE;
            
        endcase
    end

    // combinational output logic
    always @(*)
    begin
        case(state)
            IDLE:
            begin
                mem_read = 0;
                mem_write = 0;
                mem_address = 8'dx;
                mem_writedata = 8'dx;
                busywait = 0;
            end
         
            MEM_READ: 
            begin
                mem_read = 1;
                mem_write = 0;
                mem_address = {tag, index};
                mem_writedata = 32'dx;
                busywait = 1;
                #1
                if (mem_busywait == 0)
                begin
                    busywait = 1;
                    data[index] = {mem_readdata[7:0],mem_readdata[15:8],mem_readdata[23:16],mem_readdata[31:24]};
                    tag_bits[index] = tag;
                    dirty_bit[index] = 0;
                    valid_bit[index] = 1;
                end 
            end
            
            MEM_WRITE:
            begin
                mem_read = 0;
                mem_write = 1;
                mem_address = {cache_tag, index};
                mem_writedata = {data_block[7:0], data_block[15:8], data_block[23:16], data_block[31:24]};
                busywait = 1;
            end
        endcase
    end

    // sequential logic for state transitioning 
    always @(posedge clk, reset)
    begin
        if(reset)
            state = IDLE;
        else
            state = next_state;
    end

    /* Cache Controller FSM End */

integer i;
    always @(reset)
    begin
        if(reset)
        begin
            for (i = 0; i < 8; i++)
                begin
                    valid_bit[i] = 0;
                    dirty_bit[i] = 0;
                end
        end
    end
endmodule

module mux4x1(in0, in1, in2, in3, out, sel);
    input [7:0] in0,in1,in2,in3;
    output reg [7:0] out;
    input [1:0] sel;

    always @(in0, in1, in2, in3, sel) 
    begin
        case(sel)
            2'b00: #1 out = in0;    
            2'b01: #1 out = in1;    
            2'b10: #1 out = in2;    
            2'b11: #1 out = in3;
            default: #1 out = -1;
        endcase    
    end
endmodule