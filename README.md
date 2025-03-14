# Autonomous Gate Control System

## Overview
This project implements a microcontroller-based autonomous gate system for pedestrian access control. The system utilizes an **ARM-based EFM32PG28 microcontroller** to manage the gate, track pedestrian entry/exit, and provide real-time status updates via an **LCD display**. The gate operates using a **servo motor** and supports control through **Bluetooth communication** and physical **push buttons**.

## Features
- **Automated Gate Operation:** The gate opens and closes using a servo motor based on user input.
- **Bluetooth Control:** Users can enter and exit via Bluetooth commands.
- **Push Button Control:** Manual control through onboard buttons for emergency or normal operation.
- **LCD Display:** Real-time display of gate status and the number of people inside.
- **Emergency Mode:** A dedicated button activates emergency mode, keeping the gate open with a buzzer alert.
- **Status Indicators:** LED lights (red, yellow, green) indicate the gate's operational state.
- **Capacity Limit:** The system prevents entry when the predefined capacity is reached.

## System Components
- **Microcontroller:** EFM32PG28 (ARM-based MCU)
- **Servo Motor:** Controls gate movement
- **Bluetooth Module:** Enables wireless communication
- **LCD Display:** Shows gate status and occupancy
- **Push Buttons:** Manual entry/exit control
- **Buzzer:** Alerts during emergency mode
- **LED Indicators:** Provide real-time gate status feedback

## Functional Diagram
```
+----------------+      +-------------------+
| Bluetooth      | ---> | Microcontroller   |
| Buttons       | ---> | (EFM32PG28)       |
+----------------+      +-------------------+
                             |
                +------------+-------------+
                |   Servo Motor (Gate)    |
                |   LCD Display           |
                |   LED Indicators        |
                |   Buzzer (Alarm)        |
                +------------------------+
```

## How It Works
1. **Entry Process:**
   - User sends an entry command via Bluetooth or presses the entry button.
   - The system checks if the max capacity is reached.
   - If space is available, the gate opens, the count increases, and the LCD updates.
   - If full, the gate remains closed, and a message is displayed.

2. **Exit Process:**
   - User sends an exit command via Bluetooth or presses the exit button.
   - If users are inside, the gate opens, the count decreases, and the LCD updates.
   - If no one is inside, the system displays a "No Exit" message.

3. **Emergency Mode:**
   - Pressing the emergency button forces the gate open.
   - The buzzer sounds an alarm.
   - The gate remains open until the button is pressed again.

## Code Structure
- **Initialization Functions:**
  - `Servo_Init()`: Initializes the servo motor
  - `LCD_CONFIGURATIONS()`: Configures the LCD
  - `BLUETOOTH_INIT()`: Sets up Bluetooth communication
  - `BUTTONS_CONFIGURATIONS()`: Configures push buttons

- **Operational Functions:**
  - `openGate()`, `closeGate()`: Controls gate movement
  - `displayTransitionState()`: Updates LCD with status messages
  - `checkButtonState()`: Detects button presses
  - `MaxMinCapacity()`: Checks if entry/exit is allowed

- **Interrupts & Timers:**
  - `EUSART1_RX_IRQHandler()`: Handles Bluetooth commands
  - `GPIOCallBack()`: Manages push button interrupts
  - `PWM & Timer`: Generates pulse signals for servo control

## Installation & Usage
1. **Flash the firmware** onto the **EFM32PG28 microcontroller**.
2. **Pair the Bluetooth module** with a mobile device.
3. **Use predefined commands** to open/close the gate or manually control via buttons.
4. **Monitor gate status** on the LCD display.

## Demonstration Video
https://www.youtube.com/embed/D5bY_PNigpw?feature=oembed

## Contributors
- **Ido Ben Harush**
- **Yuval Hammer**

## License
This project is open-source under the MIT License.
