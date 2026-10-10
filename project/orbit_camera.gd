extends Camera3D
# Test only: orbits the target with A and D or the left and right arrow keys, at the distance and height it starts from.

@export var target: Node3D
@export var look_height := 1.0
@export var turn_speed := 90.0

var _distance := 0.0
var _height := 0.0
var _yaw := 0.0


func _ready() -> void:
	var offset := global_position - target.global_position
	_height = offset.y
	_distance = Vector2(offset.x, offset.z).length()
	_yaw = atan2(offset.x, offset.z)


func _process(delta: float) -> void:
	var input := 0.0
	if Input.is_physical_key_pressed(KEY_A) or Input.is_physical_key_pressed(KEY_LEFT):
		input -= 1.0
	if Input.is_physical_key_pressed(KEY_D) or Input.is_physical_key_pressed(KEY_RIGHT):
		input += 1.0

	_yaw += deg_to_rad(turn_speed) * input * delta
	var center := target.global_position
	global_position = center + Vector3(sin(_yaw) * _distance, _height, cos(_yaw) * _distance)
	look_at(center + Vector3(0.0, look_height, 0.0))
