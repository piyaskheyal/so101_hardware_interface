# SO101 Hardware Interface

A Python-based ROS 2 package utilizing `ament_cmake` and `ament_cmake_python` to serve as the physical hardware interface for the SO101 robotic arm. It bridges the gap between MoveIt2 (via `topic_based_ros2_control`) and the Feetech STS3215 serial servos on the physical arm.

## Features
- **Custom Serial Driver:** Includes a Python module (`so101_hardware_interface.motors.feetech`) to handle robust communication with Feetech servos over USB.
- **Topic-Based Hardware Abstraction:** Acts as the bridge for `topic_based_ros2_control` by subscribing to `/real_joint_commands` and publishing states to `/real_joint_states`.
- **Safe Startup (No-Drop):** Immediately reads and dynamically locks the physical arm's current joint states on launch before enabling torque. This prevents the arm from jumping aggressively or dropping limp upon initialization.
- **Stable Communication Loop:** Avoids serial lockups (`[TxRxResult] There is no status packet!` errors) by locking the baud rate to 1,000,000 and intentionally capping the update frequency to 25Hz.
- **True Mathematical Alignment:** Bypasses error-prone offset math by using explicitly recorded physical capabilities. It clamps values using physical bounds and calculates standard `-pi` to `pi` radians perfectly aligned to RViz simply by identifying `(range_min + range_max) / 2.0`.

## Calibration

Before launching the arm, you must run the interactive calibration script. This establishes the homing offsets and the absolute minimum and maximum physical limits for each joint to ensure MoveIt's trajectory limits exactly match the physical robot's safely confined ranges.

```bash
# Make sure your workspace is sourced
source install/setup.bash

# Run the calibration tool
ros2 run so101_hardware_interface calibrate.py
```
Follow the interactive prompts in the terminal:
1. Move the arm joints to their visually correct middle/zero positions.
2. Sweep the joints through their absolute physical bounds.
3. The configurations will be recorded and saved to: `src/so101_hardware_interface/resource/so101_calibration.json`.

*(Note: Depending on your build process, you may need to run `colcon build` after calibrating to ensure the updated JSON is deployed to the `install/` space).*

## Usage

This node is typically launched automatically alongside your MoveIt2 planners and `ros2_control` processes. 

To test the physical arm node standalone:
```bash
ros2 run so101_hardware_interface hardware_interface.py
```

To launch the full real robot alongside MoveIt and RViz:
```bash
ros2 launch so101_moveit2_config control_real_so101.launch.py
```

## Structure
- `scripts/`: Contains the executable nodes (`hardware_interface.py`, `calibrate.py`, `joint_pos_control.py`).
- `so101_hardware_interface/motors/`: Contains the library imports (e.g. `feetech.py`) which acts as the core servo motor driver logic.
- `resource/`: Stores the generated calibration data JSON file.
# SO101 Hardware Interface

A Python-based ROS 2 package utilizing `ament_cmake` and `ament_cmake_python` to serve as the physical hardware interface for the SO101 robotic arm. It bridges the gap between MoveIt2 (via `topic_based_ros2_control`) and the Feetech STS3215 serial servos on the physical arm.

## Features
- **Custom Serial Driver:** Includes a Python module (`so101_hardware_interface.motors.feetech`) to handle robust communication with Feetech servos over USB.
- **Topic-Based Hardware Abstraction:** Acts as the bridge for `topic_based_ros2_control` by subscribing to `/real_joint_commands` and publishing states to `/real_joint_states`.
- **Safe Startup (No-Drop):** Immediately reads and dynamically locks the physical arm's current joint states on launch before enabling torque. This prevents the arm from jumping aggressively or dropping limp upon initialization.
- **Stable Communication Loop:** Avoids serial lockups (`[TxRxResult] There is no status packet!`errors)bylockingthebaudrateto`1,000,000`andintentionallycappingtheupdatefrequencyto25Hz.
- **True Mathematical Alignment:** Bypasses error-prone offset math by using explicitly recorded physical capabilities. It clamps values using physical bounds and calculates standard `-pi` to `pi` radians perfectly aligned to RViz simply by identifying `(range_min + range_max) / 2.0`.

## Calibration

Before launching the arm, you must run the interactive calibration script. This establishes the homing offsets and the absolute minimum and maximum physical limits for each joint to ensure MoveIt's trajectory limits exactly match the physical robot's safely confined ranges.

```bash
# Make sure your workspace is sourced
source install/setup.bash

# Run the calibration tool
ros2 run so101_hardware_interface calibrate.py
```
Follow the interactive prompts in the terminal:
1. Move the arm joints to their visually correct middle/zero positions.
2. Sweep the joints through their absolute physical bounds.
3. The configurations will be recorded and saved to: `src/so101_hardware_interface/resource/so101_calibration.json`.

*(Note: Depending on your build process, you may need to run `colcon build` after calibrating to ensure the updated JSON is deployed to the `install/` space).*

## Usage

This node is typically launched automatically alongside your MoveIt2 planners and `ros2_control` processes. 

To test the physical arm node standalone:
```bash
ros2 run so101_hardware_interface hardware_interface.py
```

To launch the full real robot alongside MoveIt and RViz:
```bash
ros2 launch so101_moveit2_config control_real_so101.launch.py
```

## Structure
- `scripts/`: Contains the executable nodes (`hardware_interface.py`, `calibrate.py`, `joint_pos_control.py`).
- `so101_hardware_interface/motors/`: Contains the library imports (e.g. `feetech.py`) which acts as the core servo motor driver logic.
- `resource/`: Stores the generated calibration data JSON file.
