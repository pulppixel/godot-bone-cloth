extends Node3D
# Test only: runs her forward, turns her around at each end, and runs back, so the cloth's inertia shows in starts, stops and turns.

@export var speed := 4.0
@export var distance := 6.0
@export var turn_time := 1.0

var _turn_left := 0.0


func _process(delta: float) -> void:
	if _turn_left > 0.0:
		var step := minf(delta, _turn_left)
		rotate_y(PI * step / turn_time)
		_turn_left -= step
		return

	position += basis.z * speed * delta
	if position.z * basis.z.z > 0.0 and absf(position.z) >= distance * 0.5:
		position.z = signf(position.z) * distance * 0.5
		_turn_left = turn_time
