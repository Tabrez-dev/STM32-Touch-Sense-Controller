# STM32F072 TSC LED Demo

This project demonstrates the use of the STM32F072’s Touch Sensing Controller (TSC) in a baremetal environment to drive a linear touch sensor on the STM32F072 Discovery kit. The sensor data is processed in an interrupt-driven manner, and specific LEDs are illuminated based on which electrode group is touched.



https://github.com/user-attachments/assets/80ca6980-1d82-40ec-9837-bbf23c007448

Reference manual to learn about TSC: https://www.st.com/content/ccc/resource/technical/document/application_note/group0/ed/0d/4d/87/04/1d/45/e5/DM00445657/files/DM00445657.pdf/jcr:content/translations/en.DM00445657.pdf

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

## Basics of the Touch Sensing Controller (TSC)

The Touch Sensing Controller (TSC) is a dedicated peripheral in some STM32 microcontrollers (like the STM32F072) that enables capacitive touch sensing. It measures the change in capacitance on specially designed electrodes to detect touch events (for example, when a finger approaches or makes contact).

### Key Terminologies

- **Electrodes:**  
  These are the metal pads on the PCB that form the touch sensor. In my design, each touch “key” (or slider segment) consists of a pair of electrodes:
  - **LS (Low Side) Electrode:** Acts as one part of the sensing pair.
  - **Capacitor Electrode:** Completes the capacitive circuit.
  
- **Capacitance:**  
  The ability of a system to store an electric charge. When a finger (which conducts electricity) comes near the electrode, the capacitance increases. The TSC measures this change in capacitance to detect a touch.
Enable Sampling on Capacitor Electrodes(IOSCR)
This ensures that the pins used as capacitor electrodes (PA3, PA7, PB1) are actively sampled during the acquisition cycle.

Enable Sensing Channels(IOCCR)
These channels (PA2, PA6, PB0) are responsible for detecting changes in capacitance. Enabling them allows the TSC to measure the capacitance on each electrode.

Enable Analog Groups(IOGCSR)
The TSC groups related electrodes into analog groups. Enabling these groups ensures that the electrodes are grouped and measured correctly.

- **fPGCLK (Pulse Clock):**  
  The clock used by the TSC during the measurement process. In my configuration, fPGCLK is derived by dividing the main system clock (fHCLK) by 32.

  This is the clock signal that drives the pulse generator within the TSC. It is derived by dividing the main system clock (fHCLK) by a configurable prescaler (for example, by 32). fPGCLK is measured in Hertz (Hz) and determines how often the pulse is generated.

    Higher fPG Frequency:
    Results in shorter pulse durations, which may speed up the acquisition but could reduce sensitivity.

    Lower fPG Frequency:
    Leads to longer pulse durations, potentially improving sensitivity but slowing down the measurement process.

Thus, configuring fPG correctly is crucial because it directly impacts the accuracy and responsiveness of the touch detection process.

- **tPGCLK (Pulse Period):**  
  The period of the fPGCLK. The TSC uses multiples of this period to define pulse durations for charging and discharging the electrodes.
  This is simply the inverse of fPGCLK (tPGCLK = 1/fPGCLK). It represents the time duration of a single pulse. In TSC configurations, pulse durations (both high and low) are defined as multiples of tPGCLK. For instance, if you set the pulse high time to 2 × tPGCLK, the electrode will be driven high for two pulse periods.

- **Pulse High/Low Durations:**  
  These define the length of time the electrode is charged (high) and discharged (low). In my setup, both durations are set to 2 × tPGCLK, ensuring a consistent measurement window.

- **Maximum Count Value (MCV):**  
  During an acquisition, the TSC counts the number of pulses before reaching a defined threshold. Setting the maximum count (e.g., 16383 pulses) helps define the sensitivity and range of the measurement.

The acquisition cycle in the TSC is the process during which the controller repeatedly charges and discharges the electrodes using its pulse generator. During this cycle, the TSC counts the number of pulses until a threshold is reached. The Maximum Count Value (MCV) defines the upper limit for this pulse count, essentially setting the maximum duration or sensitivity of the acquisition. In a no-touch condition, the electrodes charge quickly, resulting in a count near the MCV, whereas a touch increases capacitance and slows charging, yielding a lower count. This cycle converts the analog touch event into a digital value that the microcontroller can use to determine whether, and where, a touch occurred.

- **End-of-Acquisition Flag (EOAF):**  
  A status flag that indicates the completion of a touch measurement cycle. When this flag is set, it means the TSC has finished charging/discharging the electrodes, and the resulting count value is ready to be processed.

- **Hysteresis:**  
  In touch sensing, hysteresis helps prevent noise or minor fluctuations from triggering false touch events. However, it can also delay the sensor response. In my project, hysteresis is disabled to achieve faster and more immediate measurements.

- **Interrupts:**  
  Instead of polling continuously for measurement completion, the TSC can generate an interrupt when the EOAF flag is set. This allows the processor to quickly capture the sensor data and process it in an interrupt service routine (ISR), keeping the system responsive.

tldr:  Prescaler (PGPSC): Adjust how fast the TSC charges/discharges.
    Charge/Discharge Pulse Length (CTPH, CTPL): Fine-tune measurement accuracy vs. speed.
    Max Count Value (MCV): Prevent saturating the counter if the electrode has a large capacitance.
    Hysteresis, Sampling Capacitor, etc.: Improve stability or handle environmental conditions.


### How It All Comes Together

1. **Electrode Configuration:**  
   The TSC is connected to several electrodes (e.g., PA2/PA3, PA6/PA7, PB0/PB1). These are set to an alternate function mode to allow the TSC peripheral to control them.

2. **Measurement Cycle:**  
   Once the TSC starts a measurement cycle, it charges and discharges the electrodes, counting the number of pulses generated during this process. A lower count typically indicates the presence of a finger (due to increased capacitance).

3. **Interrupt-Driven Operation:**  
   When the measurement cycle is complete (EOAF is set), an interrupt is generated. The ISR then reads the count values for each electrode group and makes them available for processing (e.g., to update LEDs).

When no finger touches the slider, the electrodes maintain their baseline capacitance, allowing them to charge and discharge rapidly during the TSC’s periodic pulses—resulting in a high pulse count. However, when a finger is placed on the slider, its additional capacitance slows the charging/discharging cycles, yielding a lower pulse count. This measurable difference is crucial because it converts a physical touch (an increase in capacitance) into a digital value that the microcontroller can interpret to detect and locate the touch.

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
In my `TSC_init()` function, we follow the steps above:
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
  - Configure TSC Peripheral Registers
The TSC control register (TSC_CR) is set up to:

    Divide the system clock (fHCLK) by 32 to generate the pulse clock (fPGCLK).
    Set the pulse durations (both high and low phases) to 2 × tPGCLK.
    Set the maximum count value to 16383 pulses, which defines the sensitivity and acquisition window.
    Enable the TSC by setting the TSCE bit.
Why?
These parameters determine how the TSC charges/discharges the electrodes and how it measures the capacitance change. A lower measured count generally indicates that a finger is present (increased capacitance).
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

  - **Fourth Zone Overlap with First Electrode:**  
  Although the manual indicates four keys, the physical hardware only implements three independent electrode pairs. From observation it looks like the area corresponding to the fourth visual segment electrically overlaps with the first electrode group. This causes the red LED (associated with the first electrode) to light up when the far end of the slider is touched.

## Future Improvements
- **Advanced Touch Processing:**  
  Implement more sophisticated algorithms (e.g., multi-touch detection, filtering, or weighted averaging) to detect varying touch intensities or positions.
  
- **Enhanced LED Feedback:**  
  Consider using PWM for LED brightness control to indicate the strength of a touch or to create more visually engaging effects.
  
- **Power Optimization:**  
  Introduce low-power modes between acquisitions to reduce energy consumption.
  
- **Dynamic Calibration:**  
  Develop a calibration routine that adjusts touch thresholds dynamically based on environmental conditions.
  
## TL;DR

The TSC (Touch Sensing Controller) detects a touch by converting changes in capacitance into digital values. It does this by using a pulse generator (fPGCLK) to repeatedly charge and discharge touch electrodes during an acquisition cycle. A counter (in the count register) records how many pulses occur during this cycle: a high count means the electrodes charge quickly (no touch), while a lower count indicates a slower charge due to the additional capacitance of a finger. In short, the TSC uses timed pulses and counting to turn a physical touch into a number that tells the system where and when a touch occurs.

