extends CharacterBody3D


enum State { GROUND, AIR }


const WALK := 1.064
const SPRINT := 7.844


const STOP_SPEED := 0.2
const SHARP_TURN := deg_to_rad(140.0)

@export_group("Speed")
@export var walk_speed := WALK
@export var run_speed := SPRINT
@export var acceleration := 20.0
@export var deceleration := 25.0

@export_group("Turn")
@export var turn_speed_deg := 360.0
@export var turn_boost := 4.0
@export var turn_grip := 27.6
@export var lean_time := 0.15

@export_group("Jump")
@export var jump_height := 0.8
@export var max_fall_speed := 20.0

@export_group("Step")
@export var step_spring := 18.7
@export var step_lag := 0.15

@export var camera_rig: Node3D

var state := State.GROUND

var _tree: AnimationTree
var _skeleton: Skeleton3D
var _left_foot := -1 # LeftFoot's Bone Num
var _right_foot := -1
var _scale := 1.0
var _anim := &"idle"
var _lean := 0.0
var _lean_length := 1.0
var _step_offset := 0.0
var _step_speed := 0.0
var _ground_normal := Vector3.UP
var _skin_y := 0.0
var _rig_y := 0.0

@onready var _skin: Node3D = $Skin
@onready var _gravity: float = ProjectSettings.get_setting("physics/3d/default_gravity")


func _ready() -> void:
	_skin_y = _skin.position.y
	_rig_y = camera_rig.position.y


func use(character: Node3D) -> void:
	_tree = character.get_node("AnimationTree")
	_skeleton = character.find_children("*", "Skeleton3D", true, false)[0]
	_scale = _skeleton.motion_scale
	_left_foot = _skeleton.find_bone(&"LeftFoot")
	_right_foot = _skeleton.find_bone(&"RightFoot")
	_tree.set("parameters/State/transition_request", _anim)
	_tree.set("parameters/LeanOnly/sub_amount", 1.0)
	_tree.set("parameters/Lean/add_amount", 1.0)
	var player: AnimationPlayer = character.get_node("AnimationPlayer")
	_lean_length = player.get_animation(&"demo_motions/Jog_Fwd_LeanR").length

func _process(_delta: float) -> void:
	var length: float = _tree.get("parameters/Move/current_length")
	if length <= 0.0:
		return
	var at: float = _tree.get("parameters/Move/current_position") + _tree.get("parameters/Move/current_delta")
	_tree.set("parameters/LeanPhase/seek_request", fposmod(at, length) / length * _lean_length)


func _physics_process(delta: float) -> void:
	var input := Input.get_vector("move_left", "move_right", "move_forward", "move_back")
	var direction := Basis(Vector3.UP, camera_rig.rotation.y) * Vector3(input.x, 0.0, input.y)
	var top_speed := (run_speed if Input.is_action_pressed("run") else walk_speed) * _scale
	var turned := _steer(direction, top_speed, delta)
	
	if state == State.GROUND and Input.is_action_just_pressed("jump"):
		velocity.y = sqrt(2.0 * _gravity * jump_height)
		state = State.AIR
		_request(&"jump_start")
	elif state == State.AIR:
		velocity.y = maxf(velocity.y - _gravity * delta, -max_fall_speed)
	var grounded := state == State.GROUND and is_on_floor()
	var before := global_position
	move_and_slide()
	if grounded and is_on_floor():
		_absorb_step(global_position - before)
	_ground_normal = _ground()
	_settle_step(delta)
	
	var pressing := not direction.is_zero_approx()
	var speed := Vector2(velocity.x, velocity.z).length()
	if state == State.GROUND and not is_on_floor():
		state = State.AIR
		_request(&"fall")
	elif state == State.AIR and is_on_floor():
		state = State.GROUND
		_tree.set("parameters/IdleFoot/blend_amount", 0.0)
		_request(&"move" if pressing else &"land")
	elif state == State.GROUND and (pressing or (speed > STOP_SPEED and _anim != &"land")):
		_request(&"move")
	elif state == State.GROUND and _anim != &"land":
		if _anim != &"idle":
			_tree.set("parameters/IdleFoot/blend_amount", 1.0 if _right_ahead() else 0.0)
		_request(&"idle")
	
	var facing := Vector3(velocity.x, 0.0, velocity.z) if speed > STOP_SPEED else direction
	_turn_skin(facing, delta)
	_tree.set("parameters/Move/blend_position", speed / _scale)
	
	var lean_target := clampf(-turned / delta * speed / turn_grip, -1.0, 1.0)
	_lean = lerpf(_lean, lean_target, 1.0 - exp(-delta / lean_time))
	_tree.set("parameters/Side/blend_amount", _lean)


func _request(anim: StringName) -> void:
	if anim == _anim:
		return
	_anim = anim
	_tree.set("parameters/State/transition_request", anim)


func _right_ahead() -> bool:
	var left := _skeleton.get_bone_global_pose(_left_foot).origin
	var right := _skeleton.get_bone_global_pose(_right_foot).origin
	return right.z > left.z


func _steer(direction: Vector3, top_speed: float, delta: float) -> float:
	var horizontal := Vector3(velocity.x, 0.0, velocity.z)
	var speed := horizontal.length()
	var yaw := atan2(horizontal.x, horizontal.z)
	var target := atan2(direction.x, direction.z)
	var wanted := direction.length() * top_speed
	var turned := 0.0
	if direction.is_zero_approx():
		speed = move_toward(speed, 0.0, deceleration * delta)
	elif speed < STOP_SPEED:
		yaw = target
		speed = move_toward(speed, _aligned(yaw) * wanted, acceleration * delta)
	elif absf(angle_difference(yaw, target)) > SHARP_TURN:
		speed = move_toward(speed, 0.0, deceleration * delta)
	else:
		var rate := minf(turn_grip / speed, deg_to_rad(turn_speed_deg))
		var to := rotate_toward(yaw, target, rate * delta)
		turned = angle_difference(yaw, to)
		yaw = to
		speed = move_toward(speed, _aligned(yaw) * wanted, acceleration * delta)
	velocity.x = sin(yaw) * speed
	velocity.z = cos(yaw) * speed
	return turned


func _aligned(yaw: float) -> float:
	return maxf(cos(angle_difference(_skin.rotation.y, yaw)), 0.0)


func _turn_skin(direction: Vector3, delta: float) -> void:
	if direction.is_zero_approx():
		return
	var target := atan2(direction.x, direction.z)
	var gap := absf(angle_difference(_skin.rotation.y, target)) / PI
	var speed := deg_to_rad(turn_speed_deg) * lerpf(1.0, turn_boost, gap * gap)
	_skin.rotation.y = rotate_toward(_skin.rotation.y, target, speed * delta)


func _absorb_step(moved: Vector3) -> void:
	var n := _ground_normal
	var slope_rise := -(n.x * moved.x + n.z * moved.z) / n.y
	_step_offset = clampf(_step_offset - (moved.y - slope_rise), -step_lag, step_lag)


func _ground() -> Vector3:
	var from := global_position + Vector3.UP * 0.3
	var query := PhysicsRayQueryParameters3D.create(from, from + Vector3.DOWN * 0.6, collision_mask, [get_rid()])
	var hit := get_world_3d().direct_space_state.intersect_ray(query)
	if hit.is_empty() or hit.normal.y < cos(floor_max_angle):
		return Vector3.UP
	return hit.normal


func _settle_step(delta: float) -> void:
	var w := step_spring
	var e := exp(-w * delta)
	var k := (_step_speed + w * _step_offset) * delta
	_step_offset = (_step_offset + k) * e
	_step_speed = (_step_speed - w * k) * e
	_skin.position.y = _skin_y + _step_offset
	camera_rig.position.y = _rig_y + _step_offset
