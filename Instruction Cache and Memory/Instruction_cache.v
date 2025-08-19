`timescale  1ns/100ps // Timescale directive: 1ns time unit and 100ps precision
// 10 bit address
// | tag || index | |offset|
// The address is split into three parts: tag (3 bits), index (3 bits), and offset (4 bits)

module instr_cache(clk,reset, address_in,read_inst_in, busywait_in, busywait_out, address_out, read_instr, read);

// Module declaration for the instruction cache
input clk, busywait_in, reset; // Input signals (clock, reset, busywait from instruction memory)
input [31:0] address_in; // 32-bit address input from the CPU
input [127:0] read_inst_in; // Instruction read from memory (128 bits, one cache block)
output reg [5:0] address_out; // 6-bit address output to memory
output reg [31:0] read_instr; // Instruction directed to the CPU
output reg busywait_out, read; // Output signals: busywait_out and read

// Internal wire and reg declarations
wire [9:0] temp_addr; // 10-bit address (from the lower 10 bits of the PC)
wire [2:0] tag, index, cache_tag; // Tag, index and cache_tag (3 bits each)
wire [3:0] offset; // Offset (4 bits)
reg valid_bit [7:0] ; // Valid bits for each cache line (8 entries)
reg [2:0] tag_bits [7:0]; // Tag bits stored for each cache line (3 bits per entry)
reg [127:0] instr [7:0];  // Cache array (8 blocks of 128 bits)
reg hit; // Cache hit flag
wire [127:0] data_block; // Cache data block (128 bits)
wire [31:0] out; // Instruction output selected by the mux
wire valid,tag_comp; // Valid bit and tag comparison result


// Initialization for waveform dumping
initial
    begin 
    $dumpfile("cpu_wavedata.vcd"); // Dump simulation data to the file
    for(i = 0; i < 8; i++) // Loop to dump the valid bits, tag bits, and instructions
        $dumpvars(1,valid_bit[i],tag_bits[i],instr[i]);
    end

// Extract the 10 least significant bits (LSBs) from the 32-bit PC address
assign temp_addr = address_in;

// Decode the 10-bit address into tag, index, and offset
assign {tag, index, offset} = temp_addr;

// Extract the data stored in the given cache index (with 1-cycle delay)
assign #1 {cache_tag, valid, data_block} = {tag_bits[index],valid_bit[index], instr[index]};

// Select the instruction based on the offset within the 128-bit block (1-cycle delay)
mux4x1_1 mux1 (data_block[31:0],data_block[63:32],data_block[95:64],data_block[127:96],out,offset);

// Compare the tags for cache hit (0.9-cycle delay)
assign #0.9 tag_comp = ~(tag ^ cache_tag) ;

// Determine whether there is a cache hit based on the tag comparison and valid bit
    always @(*)
    begin
        hit = tag_comp && valid;            
    end

// If there's a hit, output the instruction to the CPU
always @(hit,out)
    begin
        if(hit)
        begin
            busywait_out = 0;
            read_instr = out;
            end
    end

// Cache finite state machine (FSM)
parameter IDLE = 1'b0, MEM_READ = 1'b1;
reg  state, next_state;

// Next state assignment based on the current state and hit/miss condition
always @(*)
    begin
        case (state)
            IDLE:
                if (!hit)  
                    next_state = MEM_READ; // Transition to MEM_READ on cache miss
                else 
                    next_state = IDLE; // Stay in IDLE if it's a cache hit
                  
            MEM_READ:
           
                if ( !busywait_in)
                    next_state = IDLE; // Transition to IDLE when busywait_in is cleared
                else    
                    next_state = MEM_READ; // Stay in MEM_READ while waiting for memory
        endcase
    end

// Output logic based on the current state
always @(*)
begin
  case (state)
    
      IDLE: 
      begin
        read = 0;
        busywait_out=0;
        address_out = 6'dx; // Don't output address if in IDLE
        if(!hit)
        begin
          read = 1; // Read from memory if there's a miss
          address_out = {tag, index}; // Output the address (tag + index)
        end
      end

      MEM_READ:
      begin
        read = 1;
        busywait_out = 1; // Indicate that the system is waiting for memory
        address_out = {tag, index}; // Address output for memory read
        #1
        if (!busywait_in)
        begin
          // When memory read completes (busywait_in clears)
          instr[index] = read_inst_in; // Store the fetched instruction into the cache
          tag_bits[index] =tag; // Store the tag in the cache
          valid_bit[index]=1; // Set the valid bit for the cache line

        end
      end
  endcase
end

// State transition on the positive edge of the clock
always @(posedge clk)
    begin
        if(reset)
            state = IDLE; // Reset the state machine to IDLE
        else
            state = next_state; // Update to the next state
    end

// Reset the valid bits of the cache asynchronously on reset
integer i;
always @(posedge clk)
begin
    if (reset)
    begin
       // busywait =0;
    for (i = 0; i < 8; i= i + 1)
        begin
        valid_bit[i] = 0; // Reset all valid bits to 0
        end
    end
end

endmodule

// 4-to-1 multiplexer module to select the correct instruction based on the offset
module mux4x1_1(in0,in1,in2,in3,out,sel);
    input [31:0] in0,in1,in2,in3; // Inputs: 4 instruction segments
    output reg [31:0] out; // Output: selected instruction
    input [3:0] sel; // 4-bit selector (offset)

    always @(in0,in1,in2,in3,sel)
	begin
		case (sel)
			4'b0000:  #1 out = in0; // Select in0 if offset is 0000
			4'b0100:  #1 out = in1; // Select in0 if offset is 0100
			4'b1000:  #1 out = in2; // Select in0 if offset is 1000
			4'b1100:  #1 out = in3; // Select in0 if offset is 1100
			default:  #1 out = -1; // Default case (shouldn't happen)
			 
		endcase
	end
endmodule