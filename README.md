# Mini Stereo Digital Audio Processor

## Overview
This project involves the design and development of a mini stereo digital audio processor. The processor is capable of performing audio signal processing with a focus on low power consumption.

## Project Description
The Mini Stereo Digital Audio Processor project aims to create a robust digital audio processing unit using RTL design principles. The design includes several key components such as an Arithmetic Logic Unit (ALU), Serial-In-Parallel-Out (SIPO) and Parallel-In-Serial-Out (PISO) shift registers, a controller, and a memory unit. 

The core architecture features a custom two-channel Finite Impulse Response (FIR) filter tailored for efficient and low power audio signal processing. The design process involved synthesizing the design using Design Vision and verifying its functionality using ModelSim.

## Key Features
- **RTL Design Components:**
  - ALU
  - SIPO and PISO Shift Registers
  - Controller
  - Memory Unit
- **Custom Two-Channel FIR Filter:**
  - Designed for low power audio signal processing
- **Synthesis and Verification:**
  - Synthesized using Design Vision
  - Verified using ModelSim

## Technical Details
### Tools and Technologies
- **Languages:** Verilog
- **Synthesis Tool:** Design Vision, Genus
- **Simulation Tool:** ModelSim
- **Place n Route Tool:** Innovus

### Architecture
- **ALU:** Handles arithmetic and logic operations.
- **Shift Registers:** SIPO and PISO registers manage serial and parallel data transfer.
- **Controller and Memory Unit:** Coordinate the operation and store data respectively.
- **Custom FIR Filter:** Processes audio signals efficiently with low power consumption.

## Acknowledgments
This project was developed as part of the coursework at The University of Texas at Dallas, under the guidance and support of the faculty in the Electrical Engineering department for course EE 6306 ASIC Design

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.


---

*This project was created as a part of academic coursework and is intended for educational purposes.*
