/*  ------- Design of MOS6502 CPU in Verilog -------
    Date: 05/08/2025
    Day: Tuesday
    Version 1.0
    Revised: 0 (feel free to "++"" later)
*/

module cpu(
    input wire clk,                 // CPU clock signal
    input wire res,                 // Asynchronous reset signal --> active low
    output reg [15:0] add_bus       // 16-bit address bus
    input wire [7:0] d_in,          // 8-bit data input bus (read from memory/peripherals)
    output reg [7:0] d_out,         // 8-bit data output bus (write to memory/peripherals)
    output reg write_en,            // Write enable signal (active high)
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

endmodule