extends Camera3D
# Test only: orbits the target with A and D or the left and right arrow keys, at the distance and height it starts from.
# Q and E step Engine.time_scale down and up by 0.1 between 0 and 1. The camera turns in real time, so it still orbits at 0.

@export var target: Node3D
@export var look_height := 1.0
@export var turn_speed := 90.0

var _distance := 0.0
var _height := 0.0
var _yaw := 0.0
var _last_ticks := 0


func _ready() -> void:
	var offset := global_position - target.global_position
	_height = offset.y
	_distance = Vector2(offset.x, offset.z).length()
	_yaw = atan2(offset.x, offset.z)
	_last_ticks = Time.get_ticks_usec()


func _unhandled_input(event: InputEvent) -> void:
	var key := event as InputEventKey
	if key == null or not key.pressed or key.echo:
		return

	if key.physical_keycode == KEY_Q:
		_set_time_scale(Engine.time_scale - 0.1)
	elif key.physical_keycode == KEY_E:
		_set_time_scale(Engine.time_scale + 0.1)


func _set_time_scale(value: float) -> void:
	Engine.time_scale = snappedf(clampf(value, 0.0, 1.0), 0.1)
	print("Time Scale ", Engine.time_scale)


func _process(_delta: float) -> void:
	# Real seconds since the last frame: delta shrinks with Engine.time_scale and is 0 at 0.
	var now := Time.get_ticks_usec()
	var real_delta := (now - _last_ticks) / 1000000.0
	_last_ticks = now

	var input := 0.0
	if Input.is_physical_key_pressed(KEY_A) or Input.is_physical_key_pressed(KEY_LEFT):
		input -= 1.0
	if Input.is_physical_key_pressed(KEY_D) or Input.is_physical_key_pressed(KEY_RIGHT):
		input += 1.0

	_yaw += deg_to_rad(turn_speed) * input * real_delta
	var center := target.global_position
	global_position = center + Vector3(sin(_yaw) * _distance, _height, cos(_yaw) * _distance)
	look_at(center + Vector3(0.0, look_height, 0.0))
