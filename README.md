# Industrial Fault Detection System (IFDS)

## Overview

The Industrial Fault Detection System (IFDS) is an embedded monitoring and diagnostics platform designed for real-time industrial equipment health monitoring. It implements a comprehensive fault detection solution using CAN FD (Flexible Data-rate) protocol for high-speed communication and multi-sensor data acquisition.

---

## CANH7 - STM32H7 CAN FD Module

### Project Description

The **CANH7** folder contains the STM32H7 microcontroller-based firmware for the IFDS project. This module serves as the primary communication hub for the system, implementing:

- **CAN FD (FDCAN) Communication**: High-speed CAN FD protocol on FDCAN2 peripheral
- **CANopen Protocol**: Implements CANopen SDO (Service Data Object) communication for robust message handling
- **Multi-sensor Data Handling**: Processes and transmits sensor data including:
  - Temperature measurements
  - Vibration analysis with FFT
  - Pressure monitoring
  - Flow rate measurements
  - Current and voltage monitoring
- **UART Communication**: Dual UART interfaces for debugging and external communication
- **Real-time Data Packaging**: 156-byte packet structure for efficient data transmission

### Hardware Platform

- **Microcontroller**: STM32H7A3ZITxQ (ARM Cortex-M7 @ 400 MHz)
- **Development Board**: NUCLEO-H7A3ZI-Q STM32H7 Nucleo board
- **Memory**: 
  - Flash: 192 KB (configurable)
  - RAM: 192 KB total (includes SRAM1, SRAM2, SRAM3)

### Project Structure

```
CANH7/
├── Core/
│   ├── Inc/              # Header files
│   │   ├── main.h
│   │   ├── stm32h7xx_hal_conf.h
│   │   ├── stm32h7xx_it.h
│   │   └── stm32h7xx_nucleo_conf.h
│   ├── Src/              # Source files
│   │   ├── main.c        # Main application code
│   │   ├── stm32h7xx_hal_msp.c
│   │   ├── stm32h7xx_it.c
│   │   ├── syscalls.c
│   │   ├── sysmem.c
│   │   └── system_stm32h7xx.c
│   └── Startup/
│       └── startup_stm32h7a3zitxq.s
├── Drivers/              # HAL drivers and CMSIS
│   ├── BSP/             # Board Support Package
│   ├── CMSIS/           # Cortex Microcontroller Software Interface Standard
│   └── STM32H7xx_HAL_Driver/
├── Debug/               # Build artifacts and generated files
├── CANH7.ioc           # STM32CubeMX project configuration
├── CANH7 Debug.launch  # Debug launch configuration
└── STM32H7A3ZITXQ_FLASH.ld  # Linker script for Flash
```

---

## Pin Configuration & Connections

### FDCAN2 Interface (CAN FD)

| Pin | Port | Function | Mode | Notes |
|-----|------|----------|------|-------|
| PB12 | GPIO_B | FDCAN2_RX | Alternate Function | CAN FD Reception |
| PB13 | GPIO_B | FDCAN2_TX | Alternate Function | CAN FD Transmission |

**Connection Details**:
- These pins are connected to the CAN transceiver for proper CAN FD communication
- Standard CAN FD bus with 120Ω termination resistors required at both ends

### UART Interfaces

#### USART3 (Primary Debug/Output)
- **TX**: PD8 (STLINK_TX)
- **RX**: PD9 (STLINK_RX)
- **Baud Rate**: 115,200 bps
- **Configuration**: 8N1 (8 data bits, No parity, 1 stop bit)
- **Purpose**: Primary debug and printf output via ST-Link virtual COM port

#### USART2 (Secondary Communication)
- **Baud Rate**: 115,200 bps
- **Configuration**: 8N1
- **Purpose**: External sensor or device communication

### LED Indicators

| LED | Color | GPIO | Function |
|-----|-------|------|----------|
| LD1 | Green | PB0 | Status indicator |
| LD2 | Yellow | PB14 | Activity indicator |
| LD3 | Red | PB5 | Error indicator |

### Push Button

| Button | GPIO | Function |
|--------|------|----------|
| USER | PC13 | User interrupt input |

### Oscillator Pins

| Pin | Function |
|-----|----------|
| PC14 (OSC32_IN) | 32 kHz external oscillator input |
| PC15 (OSC32_OUT) | 32 kHz external oscillator output |

---

## FDCAN (CAN FD) Configuration

### Baud Rate Configuration

The FDCAN2 peripheral is configured for **320 kbps nominal bitrate** with the following parameters:

#### Nominal Phase (Arbitration Phase)

| Parameter | Value | Description |
|-----------|-------|-------------|
| **Nominal Prescaler** | 1 | Clock division factor |
| **Nominal TimeSeg1** | 86 | Time segment 1 (propagation + phase1) |
| **Nominal TimeSeg2** | 13 | Time segment 2 (phase2) |
| **Nominal SyncJumpWidth** | 13 | Maximum re-synchronization jump |
| **Calculated Bitrate** | 320 kbps | Standard CAN bitrate |
| **Time Quanta** | 31.25 ns | Time quantum resolution |
| **Bit Time** | 3125 ns | Total bit time (9.75 µs) |

#### Data Phase (FD Phase - faster transmission)

| Parameter | Value | Description |
|-----------|-------|-------------|
| **Data Prescaler** | 2 | Faster clock division |
| **Data TimeSeg1** | 12 | Phase 1 segment for data phase |
| **Data TimeSeg2** | 12 | Phase 2 segment for data phase |
| **Data SyncJumpWidth** | 12 | Re-synchronization width |

### Frame Configuration

| Setting | Value | Description |
|---------|-------|-------------|
| **Frame Format** | FDCAN_FRAME_FD_BRS | CAN FD with Bit Rate Switching |
| **Mode** | FDCAN_MODE_NORMAL | Normal operation mode |
| **Auto-Retransmission** | ENABLED | Automatic retry on transmission failure |
| **Transmit Pause** | DISABLED | No pause between frames |

### Message RAM Configuration

| Component | Count | Size | Purpose |
|-----------|-------|------|---------|
| **Standard ID Filters** | 1 | - | Filter standard CAN IDs |
| **Extended ID Filters** | 0 | - | No extended ID support |
| **RX FIFO0 Elements** | 1 | 8 bytes | Reception FIFO queue |
| **RX FIFO1 Elements** | 0 | - | Not used |
| **RX Buffer Elements** | 0 | 12 bytes | Dedicated RX buffers |
| **TX FIFO Queue Elements** | 1 | 8 bytes | Transmission FIFO |
| **TX Event Elements** | 0 | - | Not used |

### Filter Configuration

```c
// Standard ID Reception Filter to RX FIFO0
IdType:        FDCAN_STANDARD_ID
FilterIndex:   0
FilterConfig:  FDCAN_FILTER_TO_RXBUFFER
FilterID1:     0x601 (Node 1 ID)
RxBufferIndex: 0
```

### Interrupt Configuration

| Interrupt | Priority | Subpriority | Enabled |
|-----------|----------|-------------|---------|
| FDCAN2_IT0_IRQn | 0 | 0 | Yes |
| FDCAN2_IT1_IRQn | 0 | 0 | Yes |

Handles RX FIFO0 new message interrupts for real-time message processing.

---

## Data Packet Structure

The system transmits a 156-byte data packet with the following structure:

```c
#define PACKET_SIZE    156
#define PAYLOAD_SIZE   152
#define START_MARKER_1 0xAA
#define START_MARKER_2 0x55
#define END_MARKER_1   0x55
#define END_MARKER_2   0xAA

typedef struct {
  float Temperature;        // 4 bytes
  float Vibration;          // 4 bytes
  float Pressure;           // 4 bytes
  float Flow_Rate;          // 4 bytes
  float Current;            // 4 bytes
  float Voltage;            // 4 bytes
  float FFT_Temperature[8]; // 32 bytes
  float FFT_Vibration[8];   // 32 bytes
  float FFT_Pressure[8];    // 32 bytes
  float FFT_Flow[8];        // 32 bytes
} DataPacket;  // Total: 152 bytes

// Full Packet Layout:
// [START_MARKER_1(0xAA)] [START_MARKER_2(0x55)] [PAYLOAD(152)] [END_MARKER_1(0x55)] [END_MARKER_2(0xAA)]
// Byte 0                  Byte 1                  Bytes 2-153     Byte 154            Byte 155
```

---

## CANopen Implementation

The firmware implements CANopen protocol for SDO (Service Data Object) communication:

### Object Dictionary (OD) Indices

| Index | Name | Type | Access | Purpose |
|-------|------|------|--------|---------|
| 0x2000 | OD_TEMP_INDEX | float | Read | Temperature data |
| 0x2001 | OD_VIB_INDEX | float | Read | Vibration data |
| 0x2002 | OD_PRESS_INDEX | float | Read | Pressure data |
| 0x2003 | OD_FLOW_INDEX | float | Read | Flow rate data |
| 0x2004 | OD_CURRENT_INDEX | float | Read | Current data |
| 0x2005 | OD_VOLTAGE_INDEX | float | Read | Voltage data |
| 0x2010 | OD_FFT_TEMP_INDEX | float[] | Read | FFT temperature spectrum |
| 0x2011 | OD_FFT_VIB_INDEX | float[] | Read | FFT vibration spectrum |
| 0x2012 | OD_FFT_PRESS_INDEX | float[] | Read | FFT pressure spectrum |
| 0x2013 | OD_FFT_FLOW_INDEX | float[] | Read | FFT flow spectrum |

### SDO Protocol

- **Expedited Transfer**: 4-byte values (floats)
- **Segmented Transfer**: Array data (FFT data)
- **COB-ID Mapping**: 0x580 + Node_ID for responses
- **Supported Commands**: Upload (read), array chunking

---

## How to Build and Run

### Prerequisites

- **STM32CubeIDE** (latest version recommended)
- **ST-Link/V2 Debugger** (for programming and debugging)
- **USB Cable** (USB-A to Micro-USB for the Nucleo board)

### Build Instructions

1. **Open the Project**:
   ```bash
   # Navigate to the CANH7 directory
   cd CANH7
   ```

2. **Using STM32CubeIDE**:
   - Open STM32CubeIDE
   - Import the existing project: `File → Open Projects from File System`
   - Navigate to `/workspaces/Industrial-Fault-Detection-System-IFDS-/CANH7`
   - Click Finish

3. **Build the Project**:
   - Right-click on the project → `Build Project`
   - Or use keyboard shortcut: `Ctrl+B`
   - Output: `Debug/CANH7.elf`

### Programming the Board

1. **Connect the Hardware**:
   - Connect the Nucleo board via USB to your development machine
   - Connect CAN transceiver to PB12 (RX) and PB13 (TX) pins

2. **Flash the Firmware**:
   - Right-click project → `Run As → STM32 C/C++ Application`
   - Or use the Debug toolbar button
   - The ST-Link will program the device automatically

3. **Verify Programming**:
   - Once complete, the debug console will show successful programming
   - The board LEDs should initialize

### Debugging

- **Launch Debug Session**: `Debug → Debug As → STM32 C/C++ Application`
- **Debug Configuration**: `CANH7 Debug.launch` (pre-configured)
- **Serial Console**: Open serial terminal at `/dev/ttyACM0` (Linux) or `COM<X>` (Windows)
  - Baud: 115,200 bps
  - Monitor UART3 (PD8/PD9) for debug output

### Running Without Debugging

- Build the project: `Ctrl+B`
- Click Run button in toolbar
- Board will start executing automatically

---

## Configuration and Customization

### STM32CubeMX Configuration

The project uses STM32CubeMX device configuration (`.ioc` file):

1. **Open Configuration**:
   - Right-click `CANH7.ioc` → `Open with STM32CubeMX`

2. **Main Modifications**:
   - **Clock Configuration**: System clock set to 400 MHz (via HSI PLL)
   - **FDCAN2 Timing**: Modify NominalPrescaler, TimeSeg1/2 for different baud rates
   - **UART Baud Rates**: Adjustable in USART2/USART3 settings
   - **Pin Assignment**: FDCAN on PB12/PB13, UART on PD8/PD9

3. **Generate Code**:
   - `Project → Generate Code`
   - STM32CubeMX will regenerate HAL initialization code

### Adjusting CAN FD Baud Rate

To change the CAN FD bitrate:

1. Edit `CANH7.ioc` in STM32CubeMX
2. Navigate to: Peripherals → FDCAN2
3. Adjust:
   - `NominalPrescaler`: Affects base clock division
   - `NominalTimeSeg1/2`: Affects phase segments
   - Use the calculator to verify final bitrate
4. Generate code and rebuild

### Modifying Data Packet Structure

Edit `Core/Src/main.c` around line 35-50:

```c
typedef struct {
  float Temperature;
  float Vibration;
  // Add new fields here
} DataPacket;
```

Remember to keep `#pragma pack(push, 1)` for byte-aligned structure.

---

## Communication Protocols

### CAN FD Protocol Stack

```
Application Layer (CANopen SDO)
        ↓
FDCAN Driver (STM32 HAL)
        ↓
FDCAN2 Peripheral
        ↓
CAN Transceiver IC
        ↓
CAN Bus (PB12, PB13)
```

### Message Format

**CAN FD Standard Frame**:
- **ID Type**: Standard (11-bit)
- **Frame Format**: CAN FD with BRS (Bit Rate Switching)
- **Data Length**: Up to 64 bytes (segmented for larger payloads)
- **Bit Rate**: 320 kbps (nominal) with higher data phase

### Interrupt Handling

- **RX FIFO0 Interrupt**: Triggers on new message in reception FIFO
- **Callback Function**: `HAL_FDCAN_RxFifo0Callback()` in `stm32h7xx_it.c`

---

## Troubleshooting

### CAN FD Communication Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| No messages received | Filter not configured | Check FilterID1 = 0x601 in configuration |
| Transmission timeout | Transceiver not connected | Verify PB12/PB13 are connected to CAN transceiver |
| Baud rate mismatch | Timing parameters incorrect | Recalculate prescalers using STM32CubeMX |
| Silent abort | Auto-retransmission loop | Check CAN bus termination (120Ω resistors) |

### UART Output Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| No serial output | Wrong baud rate | Verify 115,200 bps in terminal |
| Garbled text | Clock misconfiguration | Check SystemClock_Config() |
| Data missing | FIFO overflow | Reduce output frequency |

### Programming Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| ST-Link not detected | USB driver missing | Install STM32CubeIDE ST-Link drivers |
| Flash write error | Memory access conflict | Clear project and rebuild |

---

## Additional Resources

- [STM32H7A3 Datasheet](https://www.st.com/resource/en/datasheet/stm32h7a3zi.pdf)
- [STM32CubeH7 HAL Documentation](https://github.com/STMicroelectronics/STM32CubeH7)
- [CANopen Protocol Specification](https://www.can-cia.org/can-knowledge/canopen/)
- [Nucleo-H7A3ZI Board Guide](https://www.st.com/resource/en/user_manual/um2581-nucleo-h7a3zi-q-board-user-manual-stmicroelectronics.pdf)

---

## License

This project is part of the Industrial Fault Detection System (IFDS). Please refer to the project root for licensing information.