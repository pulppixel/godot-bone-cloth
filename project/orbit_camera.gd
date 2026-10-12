extends Camera3D
# Test only: orbits the target with A and D or the left and right arrow keys, W and S move in and out, Q and E move the
# camera and the point it looks at down and up together. All of these run in real time, so they still work at time scale 0.
# Comma and period step Engine.time_scale down and up by 0.1 between 0 and 1.

@export var target: Node3D
@export var look_height := 1.0
@export var turn_speed := 90.0
@export var zoom_speed := 1.5
@export var lift_speed := 1.0
@export var min_distance := 0.5
@export var max_distance := 10.0

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

	if key.physical_keycode == KEY_COMMA:
		_set_time_scale(Engine.time_scale - 0.1)
	elif key.physical_keycode == KEY_PERIOD:
		_set_time_scale(Engine.time_scale + 0.1)


func _set_time_scale(value: float) -> void:
	Engine.time_scale = snappedf(clampf(value, 0.0, 1.0), 0.1)
	print("Time Scale ", Engine.time_scale)


func _process(_delta: float) -> void:
	# Real seconds since the last frame: delta shrinks with Engine.time_scale and is 0 at 0.
	var now := Time.get_ticks_usec()
	var real_delta := (now - _last_ticks) / 1000000.0
	_last_ticks = now

	_yaw += deg_to_rad(turn_speed) * _axis(KEY_A, KEY_D, KEY_LEFT, KEY_RIGHT) * real_delta
	_distance = clampf(_distance + zoom_speed * _axis(KEY_W, KEY_S) * real_delta, min_distance, max_distance)
	var lift := lift_speed * _axis(KEY_Q, KEY_E) * real_delta
	_height += lift
	look_height += lift

	var center := target.global_position
	global_position = center + Vector3(sin(_yaw) * _distance, _height, cos(_yaw) * _distance)
	look_at(center + Vector3(0.0, look_height, 0.0))


# -1 while a negative key is held, 1 while a positive one is, 0 for both or neither.
func _axis(negative: Key, positive: Key, negative_alt := KEY_NONE, positive_alt := KEY_NONE) -> float:
	var value := 0.0
	if Input.is_physical_key_pressed(negative) or (negative_alt != KEY_NONE and Input.is_physical_key_pressed(negative_alt)):
		value -= 1.0
	if Input.is_physical_key_pressed(positive) or (positive_alt != KEY_NONE and Input.is_physical_key_pressed(positive_alt)):
		value += 1.0
	return value
