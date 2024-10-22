# ESP_Buggy_Project_Group3

## Project Overview

This project involves the design and development of an **autonomous line-following buggy** as part of the 2023-24 Embedded Systems Project. The goal was to create a buggy that can autonomously follow a white line on a track, navigate inclines, and perform a 180° turn when instructed via Bluetooth. The buggy integrates mechanical design, custom electronics, advanced control algorithms, and modular software to deliver precise and efficient navigation.

## Features

- **Compact Dual-Tier Chassis**: A mechanically optimized design where the upper tier houses the microcontroller and battery pack, and the lower tier contains the motors and sensor array. This design ensures optimal stability, easy access for maintenance, and improved weight distribution.
  
- **Sensor Array for Line Detection**: Equipped with TCRT5000 infrared sensors, the buggy is able to accurately detect and follow the white line on the track. The sensors are mounted on a custom PCB and carefully positioned for precise tracking.

- **PID Control Algorithm**: The buggy uses a finely tuned Proportional-Integral-Derivative (PID) controller for steering and speed control. This ensures that the buggy can make real-time adjustments to stay on the line and adapt to changes in the track conditions, such as sharp turns and inclines.

- **Bluetooth Integration**: The buggy can be controlled remotely using Bluetooth commands. It supports starting, stopping, and executing 180° turns. Additionally, telemetry data can be sent to a BLE terminal, allowing for remote debugging and real-time PID tuning.

- **Modular Software Architecture**: The software is designed to ensure synchronized sensor readings and motor control at high frequencies, improving responsiveness. It also includes a calibration mode to adjust sensor sensitivity dynamically during operation.

## System Components

### Mechanical
- **Dual-Tier Chassis**: Separates critical components across two levels for improved accessibility and weight distribution.
- **Compact Design**: Enhances the buggy's agility and ability to navigate tight turns without colliding with the track.

### Electronics
- **Custom PCB with TCRT5000 Sensors**: Detects the white line on the track using infrared sensors, which communicate with the microcontroller to adjust the buggy’s trajectory.
- **ULN2003an Darlington Array**: Handles high-current switching for the LEDs in the sensor array, ensuring efficient line detection.

### Control System
- **PID Control**: Chosen for its self-correcting behavior, the PID algorithm dynamically adjusts the buggy’s steering and speed to maintain accurate line following. The control parameters were rigorously tested and optimized for stability and performance.
- **Steering and Speed Control**: By adjusting motor speed based on sensor inputs, the buggy can smoothly navigate turns, inclines, and straight sections of the track.

### Software
- **Real-Time Control**: The software continuously processes sensor inputs and adjusts motor control to ensure smooth operation.
- **Bluetooth Telemetry and Commands**: Allows for remote control and debugging via a Bluetooth connection, sending real-time data to a smartphone or BLE terminal app.

## Performance

During testing, the buggy achieved a top time of **11.28 seconds** in the heats, placing it among the top 5 fastest entries. Its compact design, effective control algorithms, and careful tuning of the PID parameters contributed to its strong performance. The buggy demonstrated excellent line-following accuracy and stability on inclines, with responsive handling of sharp turns.

### Strengths
- **Compact and Agile Design**: The buggy’s small form factor and balanced chassis allowed for sharp turns and smooth navigation.
- **Effective PID Control**: The finely tuned PID controller ensured that the buggy remained on track, even under varying environmental conditions.
- **Bluetooth Debugging**: The ability to adjust PID parameters and receive telemetry remotely reduced the need for manual code uploads, speeding up the tuning process.

### Areas for Improvement
- **Sensor Spacing**: Wider sensor spacing could improve line detection and reduce oscillations.
- **Battery Monitoring**: Integrating a battery monitoring system would allow for better power management and predictability in performance.
- **Durable Shroud**: Replacing the paper-based sensor shroud with a more durable material, such as a flexible 3D-printed component, would enhance longevity and performance under different lighting conditions.

## Future Enhancements

1. **Improved Sensor Design**: Adjusting the sensor array for wider spacing will improve line detection accuracy.
2. **Battery Monitoring System**: Adding real-time battery monitoring will enhance power management.
3. **Durable and Adjustable Shroud**: Replacing the current paper shroud with a 3D-printed version will provide better protection and performance across different lighting conditions.
