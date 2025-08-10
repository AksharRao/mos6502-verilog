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
// Miccrocode States --> Corresponds to 1 clock cycle

parameter
    // Common States
    DECODE = 6'd12, // IR has a valid opcode, it is now safe to decode it.
    FETCH = 6'd13, // Fetch next opcode from memory, perform previous ALU operation

    // Addressing Mode Related States
    ABS_LOW = 6'd0,  // Fetches the LSB of the address pointed to by the instruction pointer
    ABS_HIGH = 6'd1,  // Fetches the MSB of the address pointed to by the instruction pointer

    // Absolute X Addressing Mode
    ABS_XLOW = 6'd2,  // Fetches the LSB of the address pointed to by the instruction pointer + X
    ABS_XHIGH = 6'd3, // Fetches the MSB of the address pointed to by the instruction pointer + X
    ABS_Xwait = 6'd4, // Wait for ALU to complete operation (if needed for page cross or RMW)
    
    // Zero Page Addressing Mode
    ZEROPAGE_FETCH = 6'd47, // Fetch the single zero-page operand (one byte) from memory at PC
    ZEROPAGE_X_FETCH = 6'd48, // Fetch the zero-page address with X offset
    ZEROPAGE_X_LOAD = 6'd49, // Load data from zero-page onto the memory at the address pointed to by the zero-page address + X

    // Indirect Addressing Mode
       // (ZP), X Indexed Indirect Addressing Mode
       // first compute ZP + X to get a pointer in zero page; 
       // that pointer contains a two-byte address (LSB @ pointer, MSB @ pointer+1). Then read the data at that address.

    ZPX_FetchZP = 6'd14, // (ZP,X) - fetch ZP address, and send to ALU (+X)
    ZPX_FetchLSB = 6'd15, // (ZP,X) - fetch LSB at ZP+X, calculate ZP+X+1
    ZPX_FetchMSB = 6'd16, // (ZP,X) - fetch MSB at ZP+X+1
    ZPX_FetchData = 6'd17, // (ZP,X) - fetch data

       // (ZP), Y Indirect Indexed Addressing Mode
       // read a pointer from zero page at zp, form a 16-bit address from the two bytes there, then add Y to that 16-bit address (possible page carry). 
       // This causes an extra cycle if Y causes page crossing.

    ZPY_FetchZP = 6'd18, // (ZP),Y - fetch ZP address, and send ZP to ALU (+1)
    ZPY_FetchLSBAddY = 6'd19, // (ZP),Y - fetch at ZP+1, and send LSB to ALU (+Y)
    ZPY_FetchDataAddMSB = 6'd20, // (ZP),Y - fetch data, and send MSB to ALU (+Carry)
    ZPY_FetchDataPageCross = 6'd21, // (ZP),Y - fetch data (if page boundary crossed)

    // Read/Modify/Write (RMW) Specific States
    READ = 6'd35, // Read from memory for RMW (INC, DEC, shift)
    WRITE = 6'd46, // Write to memory for RMW

    // Register-to-Register Transfer / Implied Addressing
    REG2REG = 6'd36, // Read register for reg-reg transfers (e.g., TAX, INX)

    // Jump Instructions
        // Jump Absolute (JMP)
    JMP_ABS_LOW = 6'd22, // JMP - fetch PCL and hold
    JMP_ABS_HIGH = 6'd23, // JMP - fetch PCH
        // Jump Indirect (JMP IND)
    JMP_IND_LOW = 6'd24, // fetch low byte of indirect address
    JMP_IND_HIGH = 6'd25, // fetch high byte of indirect address


    // Subroutine Call (JSR)
    JSR_PushReturnHigh = 6'd26, // Push the high byte of the return address to the stack
    JSR_PushReturnLow = 6'd27, // Push the low byte of the return address to the stack
    JSR_UpdateSP = 6'd28, // Update the stack pointer (SP) after pushing return address
    JSR_FetchTargetHigh = 6'd29, // tells you you’re grabbing the high byte of the target address you’re jumping to.

    // General purpose Stack operations --> where stack is used to save the contents of the register temporarily so that it could be accessed later
    // Stack Pull (PLP/PLA)
    PULL_PREFETCH_NXTOP = 6'd30, // prefetching next opcode and storing it in instr_regHold for future use
    PULL_READ_STACK = 6'd31, // PLP/PLA - fetch data from stack, write S
    PULL_LOAD_IR = 6'd32, // PLP/PLA - prefetch op, but don't increment PC

    // Stack Push (PHP/PHA)
    PUSH_to_ALU = 6'd33, 
    PUSH_TO_STACK = 6'd34, 


    // Return From Interrupt (RTI)
    /* these are the micro-states for RTI (Return from Interrupt), 
    which basically restores the CPU exactly to the state it had before the interrupt happened. */
    
    INC_SP = 6'd37, // Increment SP by 1 to point to the saved status register (P)
    READ_P_FROM_STACK = 6'd38, // Read that status register value from $0100 + SP and load it into P (restoring all flags)
    PULL_PC_LOW = 6'd39, // Increment SP, read the saved PCL from the stack, load into PC low byte
    PULL_PC_HIGH_0 = 6'd40, // Increment SP again, begin fetching PCH from stack (first cycle)
    PULL_PC_HIGH_1 = 6'd41, // Complete PCH read, load into PC high byte, and resume execution at that address.

    // Return From Subroutine (RTS)
    RTS_INC_SP = 6'd42, // Increment SP to point to the saved PCL (low byte of return address).
    RTS_READ_FROM_STACK = 6'd43, // Read PCL from stack into a temporary location in register.
    RTS_WRITE_PCL_READ_PCH = 6'd44, // Send the PCL to the ALU so it can be loaded into PC later, increment SP again, and read PCH (high byte) from stack.
    RTS_LoadPC_INC = 6'd45, // RTS - load PC and increment

    // Branch Instructions
    /* During a conditional branch instruction, the 6502 CPU first fetches the 8-bit signed offset from the instruction and adds it to the low byte of the Program Counter (`PC`). 
    If the branch condition is met and the addition of the offset causes a page crossing (i.e., the high byte of the address changes), 
    the CPU takes an additional clock cycle to adjust the high byte of the Program Counter. 
    This process is managed by a sequence of microcode states:
    `STATE_BRANCH_FETCH_OFFSET` fetches the offset and begins the low-byte addition;
    `STATE_BRANCH_CALCULATE_TARGET` calculates the new high byte of the address to check for a page boundary crossing; 
    and `STATE_BRANCH_PAGE_CROSS_ADJUST` handles the extra clock cycle required if a page boundary is crossed. 
    This ensures the Program Counter is correctly updated to the new target address, whether the branch is forward or backward. */

    BRANCH_FETCH_OFFSET = 6'd5,  // Branch - fetch offset and send to ALU (+PC[7:0])
    BRANCH_CALC_TARGET = 6'd6,  // Branch - fetch opcode, and send PC[15:8] to ALU (page boundary check)
    BRANCH_PAGE_CROSS_ADJUST = 6'd7,  // Branch - fetch opcode (if page boundary crossed)

    // Break/Interrupt Sequence
    
    /* manage the CPU's response to an interrupt or a BRK instruction. In this sequence, the high and low bytes of the Program Counter (PC),
     followed by the Processor Status Register (P), are pushed onto the stack to save the CPU's current state. 
     During these pushes, the Stack Pointer (SP) is decremented. Finally, the CPU fetches the address of the interrupt service routine from 
     a specific vector in memory and loads it into the PC, allowing it to jump to the interrupt handler. 
     This process ensures the CPU can return to its previous state after the interrupt is handled. */

    BRK_PUSH_PCHi = 6'd8,  // BRK/IRQ - push PCH, send S to ALU (-1)
    BRK_PUSH_PCLo = 6'd9,  // BRK/IRQ - push PCL, send S to ALU (-1)
    BRK_PUSH_P_FLAG = 6'd10, // BRK/IRQ - push P, send S to ALU (-1)
    BRK_WRITE_SP_FETCH_VECTOR = 6'd11; // BRK/IRQ - write S, and fetch @ fffe (or ffca for NMI, fffe for IRQ/BRK)

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

// ALU Operation Parameters
parameter
        alu_op_or  = 4'b1100,
        alu_op_and = 4'b1101,
        alu_op_eor = 4'b1110,
        alu_op_add = 4'b0011,
        alu_op_sub = 4'b0111,
        op_rot_left = 4'b1011,
        op_passA   = 4'b1111;


                                  
endmodule