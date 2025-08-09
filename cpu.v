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

reg N;    // Negative Flag
reg V;    // Overflow Flag
reg B;    // Break Command Flag
reg D;    // Decimal Mode Flag
reg I;    // Interrupt Disable Flag
reg Zero; // Zero Flag (if named simple Z, Verilog treats it as high impedance state)
reg C;    // Carry Flag

reg [7:0] psr;

// Initialize individual flags
// REMEMBER: In Verilog, you cannot initialize a register with other registers or wires directly.
// You can only initialize with constants or in an initial block.
// So we will initialize them in an initial block instead of at declaration. In hardware terms, this initialization happens at power-up.
initial begin
    N = 1'b0;
    V = 1'b0;
    B = 1'b0;
    D = 1'b0;
    I = 1'b0;
    Zero = 1'b0;
    C = 1'b0;
end

// Keep PSR synchronized with individual flags
always @(*) begin
    psr = {N, V, 1'b1, B, D, I, Zero, C};
end // Processor Status Register (P) - 8 bits

// Initialize PSR in an initial block
initial begin
    psr = {1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0}; // Initial state of PSR
end



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

/*
 --> Microcode State Machine (Instruction Decoder/Sequencer)
 --> Each state represents a single clock cycle.
 */

reg [5:0] microcode_state; 

// Control Signals
reg pc_inc_en; // Enables PC increment (PC_inc)
reg [15:0] pc_nxt; // Intermediate value for next PC (PC_temp)

reg [1:0] src_regSel;  // to select dst reg
reg [1:0] dst_regSel;  // to select src reg

// Control and status flags for instruction decoding and execution
reg set_y;             // Use Y register for indexing (instead of X)
reg is_regLoad;        // True if current instruction loads a register (LDA/LDX/LDY)
reg is_incInstr;       // True if instruction is INC/INX/INY (increment operations)
reg is_rmw;            // True if instruction is read-modify-write (e.g., ASL, ROL, etc.)
reg is_loadOnly;       // True if instruction is a pure load (LDA/LDX/LDY)
reg is_storeInstr;     // True if instruction is a store (STA/STX/STY)
reg is_adc_sbc;        // True if instruction is ADC or SBC (add/subtract with carry)
reg is_compInstr;      // True if instruction is a compare (CMP/CPY/CPX)
reg is_shr;            // True if instruction is a shift or rotate (ASL/LSR/ROL/ROR)
reg is_rotateOnly;     // True if instruction is only a rotate (ROL/ROR)
reg is_branchBack;     // True if branch is backwards (negative offset)
reg branch_is_true;    // True if branch condition is met and should be taken
reg [2:0] branch_cond; // Encoded branch condition from instruction (e.g., zero, carry, etc.)
reg is_alu_shr_en;     // Enable ALU shift-right operation for this cycle
reg [3:0] main_aluOp;  // Main ALU operation code for the instruction
reg [3:0] current_aluOp; // ALU operation code for the current micro-operation
reg alu_bcd_en;        // Enable BCD (decimal) mode for ALU (used in ADC/SBC)
reg adj_bcd;           // True if ALU result needs BCD adjustment (for decimal mode)

// -- Special Instruction Flags
reg is_bit_ins;             // True if current instruction is BIT (bit_ins)
reg is_plp_ins;             // True if current instruction is PLP (pull processor status)
reg is_php_ins;             // True if current instruction is PHP (push processor status onto the stack)
reg is_clc_ins;             // True if current instruction is CLC (clear carry)
reg is_sec_ins;             // True if current instruction is SEC (set carry)
reg is_cld_ins;             // True if current instruction is CLD (clear decimal mode)
reg is_sed_ins;             // True if current instruction is SED (set decimal mode)
reg is_cli_ins;             // True if current instruction is CLI (clear interrupt disable)
reg is_sei_ins;             // True if current instruction is SEI (set interrupt disable)
reg is_clv_ins;             // True if current instruction is CLV (clear overflow)
reg is_brk_ins;             // True if current instruction is BRK (break)

reg is_reset_sequence;      // CPU begins normal execution after reset sequence is complete
                                  
endmodule