extends Node3D
# Test only: runs her forward, turns her around at each end, and runs back, so the cloth's inertia shows in starts, stops and turns.
# With circle_radius above 0 she runs round a circle of that radius instead, from the point and heading she is at: no ends,
# so no stops and no U-turns, for a run that only curves.
# Each runway leg turns her to face the point she started at, runs through it and ends half a runway past it. On the runway
# that is the same U-turn as before; after the circle, which leaves her anywhere, one leg brings her back.

@export var speed := 4.0
@export var distance := 6.0
@export var turn_time := 1.0
@export var circle_radius := 0.0

var _home := Vector3.ZERO
var _turn_left := 0.0
var _turn_rate := 0.0
var _leg_left := 0.0


func _ready() -> void:
	_home = position
	# The first leg starts in the middle of the runway.
	_leg_left = distance * 0.5


func _process(delta: float) -> void:
	if _turn_left > 0.0:
		var turn := minf(delta, _turn_left)
		rotate_y(_turn_rate * turn)
		_turn_left -= turn
		return

	var step := speed * delta
	if circle_radius > 0.0:
		position += basis.z * step
		rotate_y(step / circle_radius)
		# The leg she was on is lost on the circle, so the runway starts with a new one.
		_leg_left = 0.0
		return

	# Standing (Idle) she keeps her heading, so a stop after the circle stays a plain stop.
	if speed <= 0.0:
		return
	if _leg_left <= 0.0:
		_start_leg()
		return

	step = minf(step, _leg_left)
	position += basis.z * step
	_leg_left -= step
	if _leg_left <= 0.0:
		_start_leg()


func _start_leg() -> void:
	var to_home := _home - position
	to_home.y = 0.0
	var angle := 0.0
	if to_home.length() > 0.01:
		angle = basis.z.signed_angle_to(to_home, Vector3.UP)
	# A half turn takes turn_time, as the U-turn always has.
	_turn_left = turn_time * absf(angle) / PI
	_turn_rate = signf(angle) * PI / turn_time
	_leg_left = to_home.length() + distance * 0.5
