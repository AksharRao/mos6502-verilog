/*  ------- Design of MOS6502 CPU in Verilog -------
    Date: 05/08/2025
    Day: Tuesday
    Version 1.0
    Revised: 0 (feel free to "++"" later)
*/

module mos6502(
    input wire clk,                 // CPU clock signal
    input wire res,                 // Asynchronous reset signal --> active low
    output reg [15:0] add_bus       // 16-bit address bus
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
reg [7:0] instr_reg; // Instruction Register

reg [7:0] add_high, add_low; // Address High and Low bytes
  assign add_high = add_bus [15:8]; // High byte of address
  assign add_low = add_bus [7:0]; // Low byte of address

// Processor Status Register (P)
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

reg N;    // Negative Flag
reg V;    // Overflow Flag
reg B;    // Break Command Flag
reg D;    // Decimal Mode Flag
reg I;    // Interrupt Disable Flag
reg Zero; // Zero Flag (if named simple Z, Verilog treats it as high impedance state))
reg C;    // Carry Flag

reg[7:0] alu_out; // ALU output register to temp store results of arithmetic/logic operations

// General Purpose Registers
reg [7:0] Acc; // Accumulator Register
reg [7:0] indX; // Index Register X (if named simple X, Verilog treats it as unknown signal)
reg [7:0] indY; // Index Register Y
reg [7:0] S_Ptr; // Stack Pointer Register

                                  
endmodule