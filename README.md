# Godot SDL Gyro for Joy-Cons

This project is a **GDExtension** for **Godot Engine 4.5**. It is an enhanced fork designed to physically interact with a pair of **Nintendo Switch Joy-Cons**, allowing precise reading of their motion sensors (accelerometer and gyroscope) using external libraries for sensor fusion processing.

The main goal is to expose methods in Godot to easily read the **acceleration**, **angular velocity**, and **absolute orientation** (quaternions) of both Joy-Cons.

## Table of Contents
1. [Introduction](#introduction)
2. [Libraries Used](#libraries-used)
3. [Installation](#installation)
4. [Usage in GDScript](#usage-in-gdscript)
5. [API Reference](#api-reference)
   - [Initialization](#initialization)
   - [Data Polling](#data-polling)
   - [Calibration](#calibration)
   - [Haptic Feedback](#haptic-feedback)
6. [Build Instructions](#build-instructions)
7. [Issues and Improvements](#issues-and-improvements)
8. [Credits](#credits)

---

## Libraries Used

This project relies on the work of great developers and open-source libraries:
*   **[SDL2](https://www.libsdl.org/)**: Simple DirectMedia Layer, used for handling input devices and reading raw data from controllers.
*   **[GamepadMotionHelpers](https://github.com/JibbSmart/GamepadMotionHelpers)**: Library created by **JibbSmart** that processes raw gyroscope and accelerometer data to deliver a stable and calibrated orientation (Sensor Fusion).

---

## Installation

1.  **Download**: Get the latest build from the [Releases](https://github.com/SagaPDev/Godot-SDL-Gyro/releases/latest) section (if available) or build it yourself following the instructions below.
2.  **Add to Project**: Extract the content (usually an `addons` folder or the `.dll` / `.so` libraries along with the `.gdextension` file) into the root of your Godot project.
3.  **Restart**: Restart the Godot editor so it detects the GDExtension.

---

## Usage in GDScript

To use the extension, you must instantiate the `SDLGyro` class in your script. It is recommended to do this in a persistent node or Autoload.

### Basic Example

```gdscript
extends Node

var gyro = SDLGyro.new()

func _ready():
    # 1. Initialize SDL
    gyro.sdl_init()
    
    # 2. Initialize and search for Joy-Cons
    gyro.joycon_init()

func _process(delta):
    # 3. Read data every frame
    var data = gyro.joycon_polling()

    # Access Left Joy-Con data
    var orientation_L = data["quat_l"] # Quaternion
    var acceleration_L = data["accel_l"] # Vector3
    
    # Apply rotation to an object in Godot
    $MyNode.quaternion = orientation_L
```

---

## API Reference

Below is a brief description of the functions available in the `SDLGyro` class.

### Initialization

*   `sdl_init()` -> `void`
    *   Initializes the SDL2 game controller subsystem (`SDL_INIT_GAMECONTROLLER`). Should be called once at startup.

*   `joycon_init()` -> `void`
    *   Specifically searches for a "Nintendo Switch Joy-Con Pair" controller type.
    *   If found, it opens the device and enables the sensors (Gyroscope and Accelerometer) for both left and right sides.
    *   Automatically starts default calibration.

### Data Polling

*   `joycon_polling()` -> `Dictionary`
    *   Should be called in `_process` or `_physics_process`.
    *   Reads sensors, processes sensor fusion, and returns a dictionary with the following keys:
        *   `gyro_l`, `gyro_r` (`Vector3`): Calibrated angular velocity (degrees/second).
        *   `accel_l`, `accel_r` (`Vector3`): Processed acceleration (G-Force, without gravity).
        *   `quat_l`, `quat_r` (`Quaternion`): Absolute orientation in space.
    *   If the controller disconnects, it resets values to neutral.

### Calibration

*   `joycon_calibrate()` -> `void`
    *   Resets the current motion state and starts **continuous calibration**. Useful if you notice sensor drift.

*   `joycon_stop_calibrate()` -> `void`
    *   Stops manual continuous calibration (pauses offset adjustment process).

*   `joycon_auto_calibration()` -> `void`
    *   Activates automatic calibration mode using "Stillness" and "Sensor Fusion". This is the recommended mode and is activated by default in `joycon_init`.

### Haptic Feedback

*   `joycon_rumble(low_freq: int, high_freq: int, duration_ms: int)` -> `bool`
    *   Sends a rumble command to the controller.
    *   `low_freq`: Low frequency intensity (0-65535).
    *   `high_freq`: High frequency intensity (0-65535).
    *   `duration_ms`: Duration in milliseconds.

---

## Build Instructions

If you need to compile the extension from source (e.g., if you modified the C++ code).

### Requirements
*   **SCons**: Build system (Install via Python: `pip install scons`).
*   **C++ Compiler**: MinGW (Windows) or GCC/Clang (Linux).
*   **SDL2 Development Libraries**: You must have SDL2 development libraries.

### Expected Folder Structure
Place the SDL2 libraries (`include` and `lib`) inside the `libs/mingw_dev_lib` folder (for Windows/MinGW) or ensure they are accessible on your system (Linux). You can adjust paths in the `SConstruct` file.

---

## Issues and Improvements

If you find any bugs or have ideas to improve functionality (e.g., support for more controller types, latency improvements), please review or open an "Issue" in the repository:

👉 **[Go to Project Issues](https://github.com/Gon-Code/Godot-SDL-Gyro/issues)**

---

## Contributing

If you want to contribute to the code or build the project yourself to obtain the `.dll` file, you can use the following command (on Windows):

```bash
scons platform=windows target=template_debug
```

*   **Targets**: You can use `template_debug` for debugging or `template_release` for the final optimized build.
*   **Parallel Compilation**: You can add `-j4` (or your number of cores) to speed up compilation.

---

## Credits

*   **Fork by**: [Gon-Code](https://github.com/Gon-Code) (Based on the original work by [SagaPDev](https://github.com/SagaPDev)).
*   **JibbSmart**: For the incredible [GamepadMotionHelpers](https://github.com/JibbSmart/GamepadMotionHelpers) library that makes motion sensor magic possible.
*   **SDL2**: For providing the fundamental hardware abstraction layer.
