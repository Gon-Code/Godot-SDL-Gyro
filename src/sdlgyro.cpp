#include "sdlgyro.h"
#include <godot_cpp/core/class_db.hpp>
#include <SDL2/SDL.h>
#include <godot_cpp/variant/utility_functions.hpp>
#include <chrono>
#include <cmath>

using namespace godot;

// Constants for unit conversion
// SDL gyroscope data is in rad/s. GamepadMotion expects deg/s.
static constexpr float toDegPerSec = float(180. / M_PI);
// SDL accelerometer data is in m/s². GamepadMotion expects g-force.
static constexpr float toGs = 1.f / 9.8f;


void SDLGyro::_bind_methods() {
  // Initialization methods
  ClassDB::bind_method(D_METHOD("sdl_init"),&SDLGyro::sdl_init);
  ClassDB::bind_method(D_METHOD("joycon_init"),&SDLGyro::joycon_init);

  // Polling methods
  ClassDB::bind_method(D_METHOD("joycon_polling"),&SDLGyro::joycon_polling);
  // Calibration methods
  ClassDB::bind_method(D_METHOD("joycon_calibrate"),&SDLGyro::joycon_calibrate);
  ClassDB::bind_method(D_METHOD("joycon_stop_calibrate"),&SDLGyro::joycon_stop_calibrate);
  ClassDB::bind_method(D_METHOD("joycon_auto_calibration"),&SDLGyro::joycon_auto_calibration);
  // Rumble
  ClassDB::bind_method(D_METHOD("joycon_rumble"),&SDLGyro::joycon_rumble);
}


void SDLGyro::sdl_init() {
  
  if (SDL_Init(SDL_INIT_GAMECONTROLLER) < 0) {
    UtilityFunctions::print("Error SDL Init: %s\n", SDL_GetError());
  }
  else {
    UtilityFunctions::print("SDL Initialized Joy-Con Pair Combined.");
  }

  oldTime = std::chrono::steady_clock::now();
}


void SDLGyro::joycon_init() {

    if (controller) {
        SDL_GameControllerClose(controller);
        controller = nullptr;
    }

    int total = SDL_NumJoysticks();

    for (int i = 0; i < total; i++) {
        if (!SDL_IsGameController(i)) continue;

        SDL_GameController *gc = SDL_GameControllerOpen(i);
        if (!gc) continue;

        SDL_GameControllerType type = SDL_GameControllerGetType(gc);

        if (type == SDL_CONTROLLER_TYPE_NINTENDO_SWITCH_JOYCON_PAIR) {
            
            controller = gc;
            
            // Enabling Gyro and Accelerometer sensors
            SDL_GameControllerSetSensorEnabled(controller, SDL_SENSOR_GYRO_L, SDL_TRUE);
            SDL_GameControllerSetSensorEnabled(controller, SDL_SENSOR_ACCEL_L, SDL_TRUE);
            SDL_GameControllerSetSensorEnabled(controller, SDL_SENSOR_GYRO_R, SDL_TRUE);
            SDL_GameControllerSetSensorEnabled(controller, SDL_SENSOR_ACCEL_R, SDL_TRUE);

            motion_L.Reset();
            motion_R.Reset();
            joycon_auto_calibration();

            UtilityFunctions::print("¡Joy-Con Pair detectado por ID de Hardware!");
            return; 
        }

        SDL_GameControllerClose(gc);
    }

  UtilityFunctions::print("Joy-Con Pair not found. Verify Bluetooth connection.");
}


Dictionary SDLGyro::joycon_polling() {
    Dictionary result;

    bool connected = (controller != nullptr && SDL_GameControllerGetAttached(controller));

    if (!connected) {
        if (controller) {
            SDL_GameControllerClose(controller);
            controller = nullptr;
        }
        // Return neutral values
        result["gyro_l"] = Vector3(); result["accel_l"] = Vector3(); result["quat_l"] = Quaternion();
        result["gyro_r"] = Vector3(); result["accel_r"] = Vector3(); result["quat_r"] = Quaternion();
        return result;
    }

    auto now = std::chrono::steady_clock::now();
    float dt = ((float)std::chrono::duration_cast<std::chrono::microseconds>(now - oldTime).count()) / 1000000.0f;
    oldTime = now;

    // Safety clamp for dt (prevents physics explosion if game freezes)
    if (dt > 0.1f) dt = 0.1f;

    // Read Sensors from SDL
    float gL[3], aL[3], gR[3], aR[3];
    
    SDL_GameControllerGetSensorData(controller, SDL_SENSOR_GYRO_L, gL, 3);
    SDL_GameControllerGetSensorData(controller, SDL_SENSOR_ACCEL_L, aL, 3);
    SDL_GameControllerGetSensorData(controller, SDL_SENSOR_GYRO_R, gR, 3);
    SDL_GameControllerGetSensorData(controller, SDL_SENSOR_ACCEL_R, aR, 3);

    // Convert SDL units to JibbSmart units
    motion_L.ProcessMotion(
        gL[0] * toDegPerSec, gL[1] * toDegPerSec, gL[2] * toDegPerSec,
        aL[0] * toGs, aL[1] * toGs, aL[2] * toGs, dt
    );

    motion_R.ProcessMotion(
        gR[0] * toDegPerSec, gR[1] * toDegPerSec, gR[2] * toDegPerSec,
        aR[0] * toGs, aR[1] * toGs, aR[2] * toGs, dt
    );

    // Package Data for Godot
    float w, x, y, z; // Temp vars for Quaternion
    float cx, cy, cz; // Temp vars for Vectors

    // --- LEFT ---
    motion_L.GetCalibratedGyro(cx, cy, cz);
    result["gyro_l"] = Vector3(cx, cy, cz); // Angular Velocity

    motion_L.GetProcessedAcceleration(cx, cy, cz);
    result["accel_l"] = Vector3(cx, cy, cz); // Acceleration

    motion_L.GetOrientation(w, x, y, z);

    // GamepadMotion outputs (w,x,y,z). Godot expects (x,y,z,w).
    result["quat_l"] = Quaternion(x, y, z, w); 

    // --- RIGHT ---
    motion_R.GetCalibratedGyro(cx, cy, cz);
    result["gyro_r"] = Vector3(cx, cy, cz);

    motion_R.GetProcessedAcceleration(cx, cy, cz);
    result["accel_r"] = Vector3(cx, cy, cz);

    motion_R.GetOrientation(w, x, y, z);
    result["quat_r"] = Quaternion(x, y, z, w);

    // Keep SDL event loop alive
    SDL_Event event;
    while (SDL_PollEvent(&event)) { 
        // Drain event queue
    }

    return result;
}


// Calibration Methods

void SDLGyro::joycon_calibrate() {
    motion_L.Reset(); 
    motion_R.Reset();

    motion_L.StartContinuousCalibration();
    motion_R.StartContinuousCalibration();
    
    UtilityFunctions::print("JoyCon calibration started...");
}


void SDLGyro::joycon_stop_calibrate() {
    motion_L.PauseContinuousCalibration();
    motion_R.PauseContinuousCalibration();
    
    UtilityFunctions::print("Manual calibration stopped.");
}


void SDLGyro::joycon_auto_calibration() {
    int mode = GamepadMotionHelpers::Stillness | GamepadMotionHelpers::SensorFusion;
    
    motion_L.SetCalibrationMode((GamepadMotionHelpers::CalibrationMode)mode);
    motion_R.SetCalibrationMode((GamepadMotionHelpers::CalibrationMode)mode);
    
    UtilityFunctions::print("Auto-Calibration activated.");
}

// Rumble Method
bool SDLGyro::joycon_rumble(uint16_t low_freq, uint16_t high_freq, uint32_t duration_ms) {
    if (!controller) return false;

    return SDL_GameControllerRumble(controller, low_freq, high_freq, duration_ms) == 0;
}