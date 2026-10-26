extends Node3D

@export var mouse_deg_per_px := 0.15
@export var stick_deg_per_sec := 150.0
@export var pitch_min_deg := -60.0
@export var pitch_max_deg := 30.0
@export var zoom_min := 1.0
@export var zoom_max := 6.0
@export var zoom_step := 0.25

var _last_ticks := 0

@onready var _arm: SpringArm3D = $SpringArm3D


func _ready() -> void:
	_arm.add_excluded_object((get_parent() as CollisionObject3D).get_rid())
	_last_ticks = Time.get_ticks_usec()


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseMotion and Input.mouse_mode == Input.MOUSE_MODE_CAPTURED:
		_turn(event.screen_relative * mouse_deg_per_px)
	elif event is InputEventMouseButton and event.pressed:
		if event.button_index == MOUSE_BUTTON_WHEEL_UP:
			_arm.spring_length = maxf(_arm.spring_length - zoom_step, zoom_min)
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			_arm.spring_length = minf(_arm.spring_length + zoom_step, zoom_max)
		else:
			Input.mouse_mode = Input.MOUSE_MODE_CAPTURED
	elif event.is_action_pressed("ui_cancel"):
		Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
	elif event is InputEventKey and event.pressed and not event.echo:
		if event.physical_keycode == KEY_COMMA:
			_set_time_scale(Engine.time_scale - 0.1)
		elif event.physical_keycode == KEY_PERIOD:
			_set_time_scale(Engine.time_scale + 0.1)


func _process(_delta: float) -> void:
	var now := Time.get_ticks_usec()
	var real_delta := (now - _last_ticks) / 1000000.0
	_last_ticks = now

	var look := Input.get_vector("look_left", "look_right", "look_up", "look_down")
	_turn(look * stick_deg_per_sec * real_delta)


func _turn(degrees: Vector2) -> void:
	rotation.y -= deg_to_rad(degrees.x)
	var pitch := _arm.rotation.x - deg_to_rad(degrees.y)
	_arm.rotation.x = clampf(pitch, deg_to_rad(pitch_min_deg), deg_to_rad(pitch_max_deg))


func _set_time_scale(value: float) -> void:
	Engine.time_scale = snappedf(clampf(value, 0.0, 1.0), 0.1)
	print("Time Scale ", Engine.time_scale)
