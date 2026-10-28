extends Node2D
var gyro := SDLGyro.new()

# Called when the node enters the scene tree for the first time.
func _ready():
  gyro.sdl_init()
  gyro.joycon_init()

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta):
  var motion_data: Dictionary = gyro.joycon_polling()
  var left_orientation: Quaternion = motion_data.get("quat_l", Quaternion.IDENTITY)
  $Sprite2D.rotation = left_orientation.get_euler().z
