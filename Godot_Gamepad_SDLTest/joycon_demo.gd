extends Node3D

var gyro := SDLGyro.new()
var is_calibrating := false

@onready var joy_con_r: Node3D = $joyConR
@onready var joy_con_l: Node3D = $joyConL
@onready var telemetry_label: Label = $Label

var initial_left_orientation := Quaternion.IDENTITY
var initial_right_orientation := Quaternion.IDENTITY


func _ready() -> void:
	gyro.sdl_init()
	gyro.joycon_init()

	# Keep the models' authored orientations and apply sensor rotation on top.
	initial_left_orientation = joy_con_l.quaternion
	initial_right_orientation = joy_con_r.quaternion


func _process(_delta: float) -> void:
	var motion_data: Dictionary = gyro.joycon_polling()
	var left_orientation: Quaternion = motion_data.get("quat_l", Quaternion.IDENTITY)
	var right_orientation: Quaternion = motion_data.get("quat_r", Quaternion.IDENTITY)
	var left_gyro: Vector3 = motion_data.get("gyro_l", Vector3.ZERO)
	var right_gyro: Vector3 = motion_data.get("gyro_r", Vector3.ZERO)
	var left_accel: Vector3 = motion_data.get("accel_l", Vector3.ZERO)
	var right_accel: Vector3 = motion_data.get("accel_r", Vector3.ZERO)

	joy_con_l.quaternion = left_orientation * initial_left_orientation
	joy_con_r.quaternion = right_orientation * initial_right_orientation

	var calibration_status := "Press C to start continuous calibration"
	if is_calibrating:
		calibration_status = "Calibrating — keep both Joy-Cons still; press C to stop"

	var left_line := "L  gyro (deg/s): %s   accel (g): %s" % [left_gyro, left_accel]
	var right_line := "R  gyro (deg/s): %s   accel (g): %s" % [right_gyro, right_accel]
	telemetry_label.text = "Joy-Con Pair sensor demo\n%s\n%s\n%s" % [
		left_line,
		right_line,
		calibration_status,
	]


func _unhandled_input(event: InputEvent) -> void:
	if not event.is_action_pressed("calibration"):
		return

	is_calibrating = not is_calibrating
	if is_calibrating:
		gyro.joycon_calibrate()
	else:
		gyro.joycon_stop_calibrate()
