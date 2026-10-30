extends CharacterBody3D


enum State { GROUND, AIR }


const WALK := 1.064
const RUN := 5.844


const STOP_SPEED := 0.2
const SHARP_TURN := deg_to_rad(140.0)
const EXIT_FROM := 0.95
const EXIT_AT := 0.45
const EXIT_WINDOW := 0.25
const EXIT_STOP := 0.45
const EXIT_GRACE := 0.2
const FLAT_AHEAD := 3.0
const EXIT_AFTER := 2.0
const FLAT_RISE := 0.03

@export_group("Speed")
@export var walk_speed := WALK
@export var run_speed := RUN
@export var acceleration := 6.0
@export var deceleration := 25.0
@export var run_brake := 6.0

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
var _left_foot := -1
var _right_foot := -1
var _scale := 1.0
var _anim := &"idle"
var _lean := 0.0
var _lean_length := 1.0
var _sprint_at := 0.0
var _half := 0.0
var _step_offset := 0.0
var _step_speed := 0.0
var _ground_normal := Vector3.UP
var _skin_y := 0.0
var _rig_y := 0.0
var _exit_length := 0.0
var _clip_time := 0.0
var _clip_length := 0.0
var _stop_decel := 0.0
var _brake := 0.0
var _since_top := INF
var _exit_pending := false
var _run_time := 0.0
var _run_off := INF

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
	_tree.set("parameters/Rest/transition_request", &"still")
	_tree.set("parameters/LeanOnly/sub_amount", 1.0)
	_tree.set("parameters/Lean/add_amount", 1.0)
	var player: AnimationPlayer = character.get_node("AnimationPlayer")
	_lean_length = player.get_animation(&"demo_motions/Jog_Fwd_LeanR").length
	var move: AnimationNodeBlendSpace1D = (_tree.tree_root as AnimationNodeBlendTree).get_node(&"Move")
	_sprint_at = move.get_blend_point_position(1)
	_exit_length = player.get_animation(&"demo_motions/Sprint_Exit").length

func _process(_delta: float) -> void:
	var length: float = _tree.get("parameters/Move/current_length")
	if length <= 0.0:
		return
	var at: float = _tree.get("parameters/Move/current_position") + _tree.get("parameters/Move/current_delta")
	_tree.set("parameters/LeanPhase/seek_request", fposmod(at / length + _half, 1.0) * _lean_length)


func _physics_process(delta: float) -> void:
	var input := Input.get_vector("move_left", "move_right", "move_forward", "move_back").normalized()
	var direction := Basis(Vector3.UP, camera_rig.rotation.y) * Vector3(input.x, 0.0, input.y)
	_run_off = 0.0 if Input.is_action_pressed("run") else _run_off + delta
	var top_speed := (run_speed if _run_off <= EXIT_GRACE else walk_speed) * _scale
	var turned := 0.0
	var clip_now := Vector2(velocity.x, velocity.z).length() / _scale
	_since_top = 0.0 if clip_now >= EXIT_FROM * run_speed else _since_top + delta
	if _anim != &"move" or clip_now <= walk_speed + 0.01:
		_run_time = 0.0
	elif not direction.is_zero_approx():
		_run_time += delta
	if state != State.GROUND or _anim != &"move" or not direction.is_zero_approx():
		_exit_pending = false
	elif not _exit_pending and _since_top <= EXIT_GRACE and _run_time >= EXIT_AFTER and _flat_ahead(Vector3(velocity.x, 0.0, velocity.z).normalized()):
		_exit_pending = true
	var exit_wait := _exit_pending
	if _anim == &"stop":
		_clip_time += delta
		var flat := Vector3(velocity.x, 0.0, velocity.z)
		var slower := move_toward(flat.length(), 0.0, _stop_decel * delta)
		flat = flat.normalized() * slower if slower > 0.0 else Vector3.ZERO
		velocity.x = flat.x
		velocity.z = flat.z
	elif not exit_wait:
		var braking := state == State.GROUND and _anim == &"move" and direction.is_zero_approx() and clip_now > 0.0
		_brake = run_brake if braking and clip_now > walk_speed + 0.01 else deceleration
		turned = _steer(direction, top_speed, delta)
	
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
	elif state == State.GROUND and _anim == &"stop":
		if pressing:
			_request(&"move")
		elif _clip_time >= _clip_length:
			_anim = &"idle"
	elif exit_wait:
		var foot := _exit_foot()
		if foot >= 0:
			_stop(foot, speed)
	elif state == State.GROUND and (pressing or (speed > STOP_SPEED and _anim != &"land")):
		_request(&"move")
	elif state == State.GROUND and _anim != &"land":
		if _anim != &"idle":
			_tree.set("parameters/IdleFoot/blend_amount", 1.0 if _right_ahead() else 0.0)
		_request(&"idle")
	
	var facing := Vector3(velocity.x, 0.0, velocity.z) if speed > STOP_SPEED else direction
	_turn_skin(facing, delta)
	var clip := speed / _scale
	var rate := 1.0
	if clip > walk_speed:
		rate = clip / lerpf(walk_speed, _sprint_at, clampf(inverse_lerp(walk_speed, run_speed, clip), 0.0, 1.0))
	_tree.set("parameters/MoveRate/scale", rate)
	_tree.set("parameters/Move/blend_position", clip / rate)
	
	var lean_target := clampf(-turned / delta * speed / turn_grip, -1.0, 1.0)
	_lean = lerpf(_lean, lean_target, 1.0 - exp(-delta / lean_time))
	_tree.set("parameters/Side/blend_amount", _lean)


# anim: idle, move, stop(Exit), jump_start, fall, land.
func _request(anim: StringName) -> void:
	if anim == _anim:
		return
	if anim == &"move":
		_start_foot()
	var tree_state := anim
	if anim == &"stop":
		tree_state = &"idle"
		_tree.set("parameters/Rest/transition_request", &"exit")
	elif anim == &"idle":
		_tree.set("parameters/Rest/transition_request", &"still")
	_anim = anim
	_tree.set("parameters/State/transition_request", tree_state)


func _exit_foot() -> int:
	var length: float = _tree.get("parameters/Move/current_length")
	if length <= 0.0:
		return -1
	var phase := fposmod(float(_tree.get("parameters/Move/current_position")) / length + _half, 1.0)
	for foot in 2:
		if absf(angle_difference(TAU * phase, TAU * (EXIT_AT + 0.5 * foot))) <= TAU * EXIT_WINDOW:
			return foot
	return -1


func _flat_ahead(dir: Vector3) -> bool:
	var space := get_world_3d().direct_space_state
	for k in range(0, int(FLAT_AHEAD / 0.5) + 1):
		var from := global_position + dir * (0.5 * k) + Vector3.UP * 0.6
		var query := PhysicsRayQueryParameters3D.create(from, from + Vector3.DOWN * 1.2, collision_mask, [get_rid()])
		var hit := space.intersect_ray(query)
		if hit.is_empty() or absf(hit.position.y - global_position.y) > FLAT_RISE:
			return false
	return true


func _stop(foot: int, speed: float) -> void:
	_tree.set("parameters/ExitFoot/blend_amount", float(foot))
	_tree.set("parameters/IdleFoot/blend_amount", float(foot))
	_clip_time = 0.0
	_clip_length = _exit_length
	_stop_decel = speed / EXIT_STOP
	_request(&"stop")


func _start_foot() -> void:
	_half = 0.5 if _tree.get("parameters/IdleFoot/blend_amount") > 0.5 else 0.0
	var move: AnimationNodeBlendSpace1D = (_tree.tree_root as AnimationNodeBlendTree).get_node(&"Move")
	for i in move.get_blend_point_count():
		var clip := move.get_blend_point_node(i) as AnimationNodeAnimation
		var length := _tree.get_animation(clip.animation).length
		clip.use_custom_timeline = true
		clip.timeline_length = length
		clip.loop_mode = Animation.LOOP_LINEAR
		clip.start_offset = length * _half


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
		speed = move_toward(speed, 0.0, _brake * delta)
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
