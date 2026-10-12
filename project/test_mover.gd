extends Node3D
# Test only: runs her forward, turns her around at each end, and runs back, so the cloth's inertia shows in starts, stops and turns.
# With circle_radius above 0 she runs round a circle of that radius instead, from the point and heading she is at: no ends,
# so no stops and no U-turns, for a run that only curves.

@export var speed := 4.0
@export var distance := 6.0
@export var turn_time := 1.0
@export var circle_radius := 0.0

var _turn_left := 0.0
var _travelled := 0.0


func _ready() -> void:
	# The first leg starts in the middle of the runway.
	_travelled = distance * 0.5


func _process(delta: float) -> void:
	if _turn_left > 0.0:
		var turn := minf(delta, _turn_left)
		rotate_y(PI * turn / turn_time)
		_turn_left -= turn
		return

	var step := speed * delta
	if circle_radius > 0.0:
		position += basis.z * step
		rotate_y(step / circle_radius)
		return

	step = minf(step, distance - _travelled)
	position += basis.z * step
	_travelled += step
	if _travelled >= distance:
		_travelled = 0.0
		_turn_left = turn_time
