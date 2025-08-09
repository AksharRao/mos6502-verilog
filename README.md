# MOS6502 CPU Implementation in Verilog

This project implements the MOS6502 processor in Verilog HDL. The MOS6502 is an 8-bit microprocessor that was widely used in many early home computers and gaming systems, including the Nintendo Entertainment System (NES), Apple II, and Commodore 64.

## Project Structure

- `cpu.v` - Main CPU implementation file containing the core processor logic
- `alu.v` - Arithmetic Logic Unit implementation
- `cpu_exe` - Compiled simulation executable

## Features

- 8-bit data bus
- 16-bit address bus
- Basic instruction set implementation
- ALU operations

## Prerequisites

To compile and simulate this project, you need:

- Icarus Verilog (`iverilog`)
- GTKWave (optional, for waveform viewing)

## Building and Running

To compile the Verilog implementation:

```bash
iverilog -DSIM -o cpu_exe cpu.v
```

## Development Status

This is a work in progress implementation of the MOS6502 processor in Verilog. Currently implements the basic CPU structure and ALU operations.

## Contributing

Contributions to improve the implementation or add missing features are welcome. Please feel free to submit pull requests or open issues for bugs and feature requests.

## License

This project is open source and available under the MIT License.

## References

- MOS 6502 Datasheet
- [MOS Technology 6502 Wikipedia](https://en.wikipedia.org/wiki/MOS_Technology_6502)