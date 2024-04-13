# Seat Heater Control System[^1^][1]

## Project Overview
This project implements a seat heater control system for the front two seats of a car, utilizing FreeRTOS on a Tiva C microcontroller[^2^][2]. The system allows users to set the desired temperature for each seat and maintains the temperature within a specified range.

## Features
- **Temperature Control**: Users can select from Off, Low (25°C), Medium (30°C), and High (35°C) heating levels.
- **Temperature Regulation**: The system maintains the set temperature within a +/- 3°C range[^3^][3].
- **Sensor Diagnostics**: Includes diagnostics for temperature sensor failures, with a valid range of 5°C-40°C[^4^][4].
- **User Interface**: A shared screen displays the current state of the system for both seats[^5^][5].
- **Ease of Use**: Three buttons control the heating levels, including an extra button on the steering wheel for the driver's seat.

## Functionality
- **Heating Intensity**: The system adjusts the heating element's intensity based on the current and desired temperatures.
- **Temperature Measurement**: Utilizes an LM35 temperature sensor or a potentiometer for testing purposes[^6^][6].
- **Display**: Current temperature, heating level, and heater state are shown on the car's screen via UART[^7^][7].

## Requirements
- **Controller Unit**: Implemented using Tiva C[^8^][8].
- **FreeRTOS**: The software uses FreeRTOS to handle tasks and objects, ensuring responsiveness and proper data sharing[^9^][9].
- **MCAL Modules**: Includes GPIO, UART, GPTM, and ADC modules[^10^][10].

## Testing Scenario
A detailed testing scenario is provided to demonstrate the system's response to various temperature changes and user inputs.

## Bonus Feature
- **Non-Volatile Memory**: Diagnostics and user settings are saved in non-volatile memory for persistence.

## Deliverables
- Application source code, including tasks and MCAL modules[^11^][11].
- FreeRTOS configuration file[^12^][12].
- Tested and working ELF file[^13^][13].
- Simso simulation project[^14^][14].
- Documentation with system design diagrams, UART output screenshots, simulation results, and runtime measurements.

Thank you to the Edges For Training Team for their support in this project.

## How to Use
Please refer to the documentation for detailed instructions on how to operate the seat heater control system.

## License
This project is licensed under the MIT License - see the LICENSE.md file for details.

## Acknowledgments
- Edges For Training Team[^15^][15]
- All contributors who have participated in this project.
