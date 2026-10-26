extends CharacterBody3D


enum State { GROUND, AIR }


const WALK := 1.064
const SPRINT := 8.997


const STOP_SPEED := 0.2

@export_group("Speed")
@export var walk_speed := WALK
@export var run_speed := SPRINT
@export var acceleration := 20.0
@export var deceleration := 25.0

@export_group("Turn")
@export var turn_speed_deg := 360.0
@export var turn_boost := 4.0

@export_group("Jump")
@export var jump_height := 0.8
@export var max_fall_speed := 20.0

@export var camera_rig: Node3D

var state := State.GROUND

var _tree: AnimationTree
var _scale := 1.0
var _anim := &"idle"

@onready var _skin: Node3D = $Skin
@onready var _gravity: float = ProjectSettings.get_setting("physics/3d/default_gravity")


func use(character: Node3D) -> void:
	_tree = character.get_node("AnimationTree")
	var skeleton: Skeleton3D = character.find_children("*", "Skeleton3D", true, false)[0]
	_scale = skeleton.motion_scale
	_tree.set("parameters/State/transition_request", _anim)


func _physics_process(delta: float) -> void:
	var input := Input.get_vector("move_left", "move_right", "move_forward", "move_back")
	var direction := Basis(Vector3.UP, camera_rig.rotation.y) * Vector3(input.x, 0.0, input.y)
	var top_speed := (run_speed if Input.is_action_pressed("run") else walk_speed) * _scale
	_accelerate(direction * top_speed, delta)

	if state == State.GROUND and Input.is_action_just_pressed("jump"):
		velocity.y = sqrt(2.0 * _gravity * jump_height)
		state = State.AIR
		_request(&"jump_start")
	elif state == State.AIR:
		velocity.y = maxf(velocity.y - _gravity * delta, -max_fall_speed)
	move_and_slide()

	var pressing := not direction.is_zero_approx()
	var speed := Vector2(velocity.x, velocity.z).length()
	if state == State.GROUND and not is_on_floor():
		state = State.AIR
		_request(&"fall")
	elif state == State.AIR and is_on_floor():
		state = State.GROUND
		_request(&"move" if pressing else &"land")
	elif state == State.GROUND and (pressing or (speed > STOP_SPEED and _anim != &"land")):
		_request(&"move")
	elif state == State.GROUND and _anim != &"land":
		_request(&"idle")

	_turn_skin(direction, delta)
	_tree.set("parameters/Move/blend_position", speed / _scale)


func _request(anim: StringName) -> void:
	if anim == _anim:
		return
	_anim = anim
	_tree.set("parameters/State/transition_request", anim)


func _accelerate(target: Vector3, delta: float) -> void:
	var horizontal := Vector3(velocity.x, 0.0, velocity.z)
	var rate := deceleration if target.is_zero_approx() else acceleration
	horizontal = horizontal.move_toward(target, rate * delta)
	velocity.x = horizontal.x
	velocity.z = horizontal.z


func _turn_skin(direction: Vector3, delta: float) -> void:
	if direction.is_zero_approx():
		return
	var target := atan2(direction.x, direction.z)
	var gap := absf(angle_difference(_skin.rotation.y, target)) / PI
	var speed := deg_to_rad(turn_speed_deg) * lerpf(1.0, turn_boost, gap * gap)
	_skin.rotation.y = rotate_toward(_skin.rotation.y, target, speed * delta)
