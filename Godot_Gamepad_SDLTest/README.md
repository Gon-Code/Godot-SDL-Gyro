# Godot SDL Gyro — Joy-Con demo

This Godot project demonstrates the Joy-Con Pair motion API provided by the `godot-sdl-gyro` GDExtension. The main scene displays separate left and right Joy-Con models, applies each side's orientation quaternion, and shows gyro and processed acceleration values.

## Run

1. Connect a Joy-Con Pair before running the project.
2. Open this folder in Godot and run `JoyConDemo.tscn` (the configured main scene).
3. Automatic stillness/sensor-fusion calibration is configured when the pair is detected. Press **C** to optionally start or stop manual continuous calibration; keep the Joy-Cons still while it is active. Automatic calibration remains configured when manual calibration is stopped.

The native extension must be built for the current platform and present at the path configured by `addons/godot-sdl-gyro/sdlgyro.gdextension`. This repository does not currently include a complete set of release binaries.

## Model credits

The demo redistributes these models under **CC BY 4.0**. Their placement/orientation in the demo scene may be adapted; the original model files are unchanged.

- **Nintendo Switch Joycon** by Hayley Jacka: https://sketchfab.com/3d-models/nintendo-switch-joycon-a530c8c7eef04235bc865b85dcfc0ac8
- **Left Joy-Con** by face3d: https://sketchfab.com/3d-models/left-joy-con-3e0a5d1b47c84f6480d600bbaf8074ab
- License: https://creativecommons.org/licenses/by/4.0/

The model files also retain their embedded glTF attribution metadata. This demo is independent and is not affiliated with or endorsed by Nintendo.
