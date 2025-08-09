/*  ------- Design of MOS6502 CPU in Verilog -------
    Date: 05/08/2025
    Day: Tuesday
    Version 1.0
    Revised: 0 (feel free to "++"" later)
*/

module mos6502(
    input wire clk,                 // CPU clock signal
    input wire reset,                 // Asynchronous reset signal --> active low
    output reg [15:0] add_bus,       // 16-bit address bus
    input wire [7:0] d_in,          // 8-bit data input bus (read from memory/peripherals)
    output reg [7:0] d_out,         // 8-bit data output bus (write to memory/peripherals)
    output reg write_en,            // Write enable signal (active low)
    input wire IRQ,                 // Interrupt request line (IRQ)
    input wire NMI,                 // Non-maskable interrupt line (NMI)
    input wire rdy                  // Ready signal. Pauses CPU when ready_signal=0
);

/* This is to understand the DIP layout of the CPU.

          .-------.
     GND | 1     40 | RES
     RDY | 2     39 | φ 2 out
  φ 1 out| 3     38 | S.O.
     IRQ | 4     37 | φ 0 in
      NC | 5     36 | NC
     NMI | 6     35 | NC
    SYNC | 7     34 | R/W
     VCC | 8     33 | D0
      A0 | 9     32 | D1
      A1 | 10    31 | D2
      A2 | 11    30 | D3
      A3 | 12    29 | D4
      A4 | 13    28 | D5
      A5 | 14    27 | D6
      A6 | 15    26 | D7
      A7 | 16    25 | A15
      A8 | 17    24 | A14
      A9 | 18    23 | A13
     A10 | 19    22 | A12
     A11 | 20    21 | GND
         `-------'

*/

// Internal Signal Declarations

reg [15:0] p_count; // Program Counter or Instruction Pointer
// Some operations need only a byte of the program counter, so we will use two 8-bit wires to access the high and low bytes
wire [7:0] pc_high = p_count[15:8]; // Program Counter High byte
wire [7:0] pc_low = p_count[7:0]; // Program Counter Low byte 

reg [7:0] instr_reg; // Instruction Register
  reg [7:0] instr_regHold; // Holds prefetched instruction temporarily -> IRHOLD
  reg instr_regHoldValid;  // Flag to indicate if instr_regHold is valid

    wire add_high = add_bus [15:8]; // High byte of address
    wire add_low = add_bus [7:0]; // Low byte of address

// Processor Status Register (P)
// Assigning initial values to the flags
/*
   7   6   5   4   3   2   1   0
   N | V | 1 | B | D | I | Z | C  Processor Status Register "P"
   ---------------------------------------------------------
   |   |   |   |   |   |   |   |
   |   |   |   |   |   |   |   +--> C (Carry Flag) = 1 if carry occurred, else 0
   |   |   |   |   |   |   +-----> Z (Zero Flag) = 1 if result is zero, else 0
   |   |   |   |   |   +-------> I (Interrupt Disable) = 1 to disable IRQ interrupts
   |   |   |   |   +---------> D (Decimal Mode) = 1 for BCD arithmetic
   |   |   |   +-----------> B (Break Command) = 1 if BRK, 0 if IRQ/NMI
   |   |   +-------------> 1 (Unused) = Always 1
   |   +---------------> V (Overflow Flag) = 1 if signed overflow
   +------------------> N (Negative Flag) = 1 if result is negative (MSB=1)
*/

reg N = 1'b0;    // Negative Flag
reg V = 1'b0;    // Overflow Flag
reg B = 1'b0;    // Break Command Flag
reg D = 1'b0;    // Decimal Mode Flag
reg I = 1'b0;    // Interrupt Disable Flag
reg Zero = 1'b0; // Zero Flag (if named simple Z, Verilog treats it as high impedance state))
reg C = 1'b0;    // Carry Flag

wire [7:0] alu_out; // ALU output register to temp store results of arithmetic/logic operations
reg [7:0] alu_in1, alu_in2;
reg alu_cin;
wire alu_cout;
// ALU Flahs 
wire alu_z;  // ALU Zero Flag
wire alu_n;  // ALU Negative Flag
wire alu_v;  // ALU Overflow Flag
wire alu_hc; // ALU Half Carry Flag --> BCD mode operation

// General Purpose Register File
reg [7:0] gpr [3:0]; // General Purpose Registers (GPRs) A, X, Y, SP

reg [7:0] d_inHold; // Hold data input temporarily
reg d_inHoldValid; // Flag to indicate if d_inHold is valid
wire [7:0] d_inMux; // Mux for data input selection

reg [1:0] reg_sel; // Register Select for GPRs
wire [7:0] selected_regOut = gpr[reg_sel]; // Selected GPR output

parameter select_A = 2'd0; // Select A Register
parameter select_X = 2'd1; // Select X Register
parameter select_Y = 2'd2; // Select Y Register
parameter select_SP = 2'd3; // Select Stack Pointer Register

// For debugging
`ifdef SIM 

    reg [7:0] acc, indX, indY, SP;

    always @(*) begin
        acc  = gpr[select_A];
        indX = gpr[select_X];
        indY = gpr[select_Y];
        SP   = gpr[select_SP];
    end

    initial begin
        $display("MOS6502 CPU Simulation Started");
        $monitor("Time: %0t, PC: %h, IR: %h, A: %h, X: %h, Y: %h, SP: %h, Process: %b", 
                 $time, p_count, instr_reg, acc, indX, indY, SP, {N, V, B, D, I, Zero, C});
    end
`endif


                                  
endmodule