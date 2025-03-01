# STM32F072 TSC LED Demo

This project demonstrates the use of the STM32F072’s Touch Sensing Controller (TSC) in a baremetal environment to drive a linear touch sensor on the STM32F072 Discovery kit. The sensor data is processed in an interrupt-driven manner, and specific LEDs are illuminated based on which electrode group is touched.

## Introduction
The STM32F072 TSC LED Demo is designed to illustrate capacitive touch sensing using the STM32F072 Discovery board. In this demo, the board’s touch sensor is divided into three electrode groups. Each group is monitored by the TSC, and based on predefined thresholds, a specific LED is turned on:
- **Red LED** (Group 1)
- **Orange LED** (Group 2)
- **Green LED** (Group 3)
- **Blue LED** is used as a fallback when no touch is detected.

## How It Works
1. **Peripheral Initialization:**
   - **USART1** is set up for serial communication and debugging.
   - **GPIOs** for the TSC electrodes and user LEDs are configured.
   - The **TSC peripheral** is initialized according to RM0091 guidelines.
   
2. **Touch Sensing:**
   - The TSC continuously acquires raw capacitance values from three electrode groups.
   - When an acquisition cycle completes, an interrupt is generated (EOAF).
   - The `TSC_IRQHandler` captures the sensor data and sets a flag indicating new data is available.

3. **LED Update:**
   - In the main loop, once new sensor data is available, the `update_leds` function compares each sensor group’s raw value against a threshold.
   - Depending on which threshold is met, the corresponding LED is lit.
   - If no group meets its threshold, the blue LED is used to indicate no touch.

Below is a short version of the "Build and Flash" section for your README in GitHub Markdown:

## Build and Flash

### Prerequisites
- **ARM GCC Toolchain** (`arm-none-eabi-gcc`, `arm-none-eabi-objcopy`)
- **GNU Make**
- **Flashing Tools:** Use either [J-Link](https://www.segger.com/downloads/jlink/) or [ST-Link](https://github.com/texane/stlink) (e.g., `st-flash`)

### Build
Run:
```bash
make
```
This compiles `main.c` into `firmware.elf` and then converts it to `firmware.bin` using optimized flags (e.g., `-Os`, `-ffunction-sections`).

### Flash
- **J-Link:**  
  ```bash
  make jflash
  ```
- **ST-Link:**  
  ```bash
  make stflash
  ```
These Targets erase the flash, write `firmware.bin` to `0x08000000`, and reset the device.

## Code Overview

### TSC Initialization (`TSC_init`)
**General Approach:**  
When programming the Touch Sensing Controller (TSC) on an STM32 device, you typically follow these steps:
1. **Enable Clocks:** Activate the necessary clocks for the TSC peripheral and GPIO ports.
2. **Configure GPIOs:** Set the pins connected to the touch electrodes (typically in alternate function mode with no pull-ups/downs) to ensure proper capacitive sensing.
3. **Program TSC Registers:**  
   - Set the prescaler to obtain the desired fPGCLK (e.g., fHCLK/32).  
   - Define pulse durations (typically 2 × tPGCLK) for both the high and low phases.
   - Set the maximum count value (MCV) to define the acquisition window (e.g., 16383 pulses).
   - Enable the TSC by setting the TSCE bit.
4. **Enable Interrupts:** Configure the TSC to generate an End-of-Acquisition interrupt, so that once the measurement cycle is complete, the interrupt handler can quickly capture the sensor data.

**My Implementation:**  
In our `TSC_init()` function, we follow the steps above:
- **Step 1 (Clocks):**  
  We enable the clocks for the TSC and related GPIO ports using the RCC registers.
- **Step 2 (GPIOs):**  
  We configure the six electrodes (two per group) on PA2/PA3, PA6/PA7, and PB0/PB1 in alternate function mode (AF3 for TSC) without pull-ups/pull-downs.
- **Step 3 (TSC Registers):**  
  We set up `TSC_CR` with:
  - fPGCLK = fHCLK/32 (`0x5U << 12`),
  - Pulse high and low durations (2 × tPGCLK, via `CTPH` and `CTPL` bits),
  - Maximum count value (MCV), and  
  - Enable the TSC (TSCE bit).
- **Step 4 (Interrupts):**  
  We enable the End-of-Acquisition interrupt by setting the relevant bit in `TSC_IER` and then in the NVIC, ensuring that `TSC_IRQHandler` is invoked upon completion.

### Interrupt Handling (`TSC_IRQHandler`)
- **Objective:**  
  Quickly capture the raw sensor data from the three electrode groups as soon as the TSC acquisition completes, keeping the ISR as short as possible.
- **Implementation:**  
  The handler checks for the EOAF flag, clears it, and then reads the sensor data from `TSC_IOG1CR`, `TSC_IOG2CR`, and `TSC_IOG3CR` into a global array. A flag is set to signal the main loop that new data is available for processing.

### USART Functions
- **Purpose:**  
  Provide simple debugging capabilities by sending characters, strings, and numbers over the serial interface.
- **Implementation:**  
  Helper functions (`usart1_send_char`, `usart1_send_string`, `usart1_send_uint32`) manage low-level USART transmission. These functions wait for the transmit data register to be empty before sending, ensuring reliable serial communication.
  
### LED Control (`update_leds`)
- **Approach:**  
  The LED control function directly maps each electrode group to an LED based on threshold comparisons:
  - **Group 1:** If its sensor count is below `group1_threshold`, the red LED (PC6) is lit.
  - **Group 2:** If its sensor count is below `group2_threshold`, the orange LED (PC8) is activated.
  - **Group 3:** If its sensor count is below `group3_threshold`, the green LED (PC9) is illuminated.
  - **Default:** If no group is triggered, the blue LED (PC7) is used to indicate no touch.
- **Design Evolution:**  
  Although more complex methods (such as a min_count method or a slider algorithm to subdivide the electrodes into additional zones) were considered, the simpler threshold-based approach was chosen for its reliability in this demo.


## Challenge Faced
- **Electrode Mapping and Touch Zones:**  
  The user manual suggests the board can be used as a 3-position slider or as 4 discrete touch keys. Initial attempts to subdivide the three electrode groups into 4 zones using a slider algorithm were unstable. The design was reverted to a simpler method where each electrode group is compared directly to a threshold.

## Future Improvements
- **Advanced Touch Processing:**  
  Implement more sophisticated algorithms (e.g., multi-touch detection, filtering, or weighted averaging) to detect varying touch intensities or positions.
  
- **Enhanced LED Feedback:**  
  Consider using PWM for LED brightness control to indicate the strength of a touch or to create more visually engaging effects.
  
- **Power Optimization:**  
  Introduce low-power modes between acquisitions to reduce energy consumption.
  
- **Dynamic Calibration:**  
  Develop a calibration routine that adjusts touch thresholds dynamically based on environmental conditions.
  

