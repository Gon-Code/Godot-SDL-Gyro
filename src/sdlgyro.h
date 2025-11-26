#ifndef SDLGYRO_H
#define SDLGYRO_H
#define _USE_MATH_DEFINES
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <SDL2/SDL.h>
#include <chrono>
#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/dictionary.hpp>

#include "GamepadMotion.hpp"

using namespace godot;

  class SDLGyro : public Object{
    GDCLASS(SDLGyro,Object);

  public:
    // Initialization
    void sdl_init();
    void joycon_init();

    // Get motion data
    Dictionary joycon_polling();

    // Calibration
    void joycon_calibrate();
    void joycon_stop_calibrate();
    void joycon_auto_calibration();

    // Rumble
    bool joycon_rumble(uint16_t low_freq, uint16_t high_freq, uint32_t duration_ms);

    private:
      // Unique pointer to the SDL GameController
      SDL_GameController *controller = nullptr;

      // JibbSmart Helpers
      GamepadMotion motion_L;
      GamepadMotion motion_R;

      // Timing
      std::chrono::steady_clock::time_point oldTime;

    protected:
      static void _bind_methods();
  };

#endif
