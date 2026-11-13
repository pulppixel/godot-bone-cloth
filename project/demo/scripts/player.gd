extends CharacterBody3D

const CharacterSkin := preload("res://demo/scripts/skin.gd")
const CameraRig := preload("res://demo/scripts/camera_rig.gd")


enum Anim { IDLE, MOVE, STOP, JUMP_START, FALL, LAND }
const TREE_STATE: Array[StringName] = [&"idle", &"move", &"idle", &"jump_start", &"fall", &"land"]

const WALK := 1.064
const RUN := 7.65

const STOP_SPEED := 0.2
const SHARP_TURN := deg_to_rad(140.0)
const SPEED_SLACK := 0.01

const EXIT_FROM := 0.95
const EXIT_AT := 0.45
const EXIT_WINDOW := 0.25
const EXIT_STOP := 0.45
const EXIT_GRACE := 0.2
const EXIT_AFTER := 2.0

const FLAT_AHEAD := 3.0
const FLAT_STEP := 0.5
const FLAT_RAY_UP := 0.6
const FLAT_RAY_DOWN := 1.2
const FLAT_RISE := 0.03
const GROUND_RAY_UP := 0.3
const GROUND_RAY_DOWN := 0.6
const BLOCK_CHECK := 0.1
const BLOCK_HOLD := 0.2

@export_group("Speed")
@export var walk_speed := WALK
@export var run_speed := RUN
@export var acceleration := 12.0
@export var deceleration := 25.0
@export var run_brake := 12.0
@export var walk_down := 14.0

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

@export_group("Board")
@export var board_height := 0.4
@export var board_speed := 12.0
@export var board_acceleration := 8.0
@export var board_jump := 6.0
@export var board_spring := 6.0
@export var board_damping := 0.4
@export var board_reach := 3.0

@export var camera_rig: Node3D
@export var board: Node3D

var _character: CharacterSkin
var _skeleton: Skeleton3D
var _left_foot := -1
var _right_foot := -1
var _scale := 1.0
var _anim := Anim.IDLE
var _stop: Stop
var _lean := 0.0
var _step_offset := 0.0
var _step_speed := 0.0
var _ground_normal := Vector3.UP
var _skin_y := 0.0
var _rig_y := 0.0
var _brake := 0.0
var _since_top := INF
var _exit_pending := false
var _run_time := 0.0
var _run_off := INF
var _clip_speed := 0.0
var _block_held := false
var _state_age := 0.0
var _boarding := false
var _floor_body: Object
var _ride: Node3D

@onready var _skin: Node3D = $Skin
@onready var _gravity: float = ProjectSettings.get_setting("physics/3d/default_gravity")


class Stop:
	var time := 0.0
	var decel := 0.0


func _ready() -> void:
	_skin_y = _skin.position.y
	_rig_y = camera_rig.position.y


func use(character: Node3D) -> void:
	if _anim == Anim.STOP:
		_anim = Anim.IDLE
		_stop = null
		velocity = Vector3(0.0, velocity.y, 0.0)
	_character = character as CharacterSkin
	_skeleton = _character.skeleton
	_scale = _skeleton.motion_scale
	_left_foot = _skeleton.find_bone(&"LeftFoot")
	_right_foot = _skeleton.find_bone(&"RightFoot")
	_character.request(TREE_STATE[_anim])
	_character.rest(&"still")
	(camera_rig as CameraRig).frame(_character)
	_anchor_cloth()


func _process(_delta: float) -> void:
	if _character.cycle_length() <= 0.0:
		return
	_character.seek_lean(_character.phase(true))


func _physics_process(delta: float) -> void:
	if delta <= 0.0:
		return
	
	var input := Input.get_vector("move_left", "move_right", "move_forward", "move_back").normalized()
	var direction := Basis(Vector3.UP, camera_rig.rotation.y) * Vector3(input.x, 0.0, input.y)
	if Input.is_action_just_pressed("board"):
		_set_boarding(not _boarding)
	if _boarding:
		_fly(direction, delta)
		_follow_ride()
		return
	
	_run_off = 0.0 if Input.is_action_pressed("run") else _run_off + delta
	var top_speed := (run_speed if _run_off <= EXIT_GRACE else walk_speed) * _scale
	var exit_wait := _watch_exit(direction, delta)
	_state_age += delta
	var blocked := _blocked(direction)
	if blocked != _block_held and _state_age >= BLOCK_HOLD:
		_block_held = blocked
	if not _airborne() and _block_held:
		direction = Vector3.ZERO
	var turned := 0.0
	if _anim == Anim.STOP:
		_decelerate(delta)
	elif not exit_wait:
		var clip_now := Vector2(velocity.x, velocity.z).length() / _scale
		var braking := not _airborne() and _anim == Anim.MOVE and direction.is_zero_approx() and clip_now > 0.0
		_brake = run_brake if braking and clip_now > walk_speed + SPEED_SLACK else deceleration
		turned = _steer(direction, top_speed, delta)
	
	if not _airborne() and Input.is_action_just_pressed("jump"):
		velocity.y = sqrt(2.0 * _gravity * jump_height)
		_enter(Anim.JUMP_START)
	elif _airborne():
		velocity.y = maxf(velocity.y - _gravity * delta, -max_fall_speed)
	_slide(delta)
	
	var pressing := not direction.is_zero_approx() and not _block_held
	var speed := Vector2(velocity.x, velocity.z).length()
	_transition(pressing, speed, exit_wait)

	var facing := Vector3(velocity.x, 0.0, velocity.z) if speed > STOP_SPEED else direction
	_turn_skin(facing, delta)
	_animate(speed, turned, delta)


func _airborne() -> bool:
	return _anim == Anim.JUMP_START or _anim == Anim.FALL


func _watch_exit(direction: Vector3, delta: float) -> bool:
	var clip_now := Vector2(velocity.x, velocity.z).length() / _scale
	_since_top = 0.0 if clip_now >= EXIT_FROM * run_speed else _since_top + delta
	if _anim != Anim.MOVE or clip_now <= walk_speed + SPEED_SLACK:
		_run_time = 0.0
	elif not direction.is_zero_approx():
		_run_time += delta
	if _airborne() or _anim != Anim.MOVE or not direction.is_zero_approx():
		_exit_pending = false
	elif not _exit_pending and _since_top <= EXIT_GRACE and _run_time >= EXIT_AFTER and _flat_ahead(Vector3(velocity.x, 0.0, velocity.z).normalized()):
		_exit_pending = true
	return _exit_pending


func _decelerate(delta: float) -> void:
	_stop.time += delta
	var flat := Vector3(velocity.x, 0.0, velocity.z)
	var slower := move_toward(flat.length(), 0.0, _stop.decel * delta)
	flat = flat.normalized() * slower if slower > 0.0 else Vector3.ZERO
	velocity.x = flat.x
	velocity.z = flat.z


func _slide(delta: float) -> void:
	var grounded := not _airborne() and is_on_floor()
	var before := global_position
	move_and_slide()
	if grounded and is_on_floor():
		_absorb_step(global_position - before)
	_ground_normal = _ground()
	_settle_step(delta)
	_follow_ride()


func _transition(pressing: bool, speed: float, exit_wait: bool) -> void:
	if not _airborne() and not is_on_floor():
		_enter(Anim.FALL)
	elif _airborne() and is_on_floor():
		_character.set_idle_foot(false)
		_enter(Anim.MOVE if pressing else Anim.LAND)
	elif _anim == Anim.STOP:
		if pressing:
			_enter(Anim.MOVE)
		elif _stop.time >= _character.exit_length:
			_anim = Anim.IDLE
			_stop = null
	elif exit_wait and not _airborne():
		var foot := _exit_foot()
		if foot >= 0:
			_enter_stop(foot, speed)
	elif not _airborne() and (pressing or (speed > STOP_SPEED and _anim != Anim.LAND)):
		_enter(Anim.MOVE)
	elif not _airborne() and _anim != Anim.LAND:
		if _anim != Anim.IDLE:
			_character.set_idle_foot(_right_ahead())
		_enter(Anim.IDLE)


func _animate(speed: float, turned: float, delta: float) -> void:
	_clip_speed = maxf(speed / _scale, _clip_speed - deceleration / _scale * delta)
	var clip := _clip_speed
	var rate := 1.0
	if clip > walk_speed:
		rate = clip / lerpf(walk_speed, _character.sprint_at, clampf(inverse_lerp(walk_speed, run_speed, clip), 0.0, 1.0))
	_character.set_move(clip / rate, rate)
	
	var lean_target := clampf(-turned / delta * speed / turn_grip, -1.0, 1.0)
	_lean = lerpf(_lean, lean_target, 1.0 - exp(-delta / lean_time))
	_character.set_lean(_lean)


func _enter(anim: Anim) -> void:
	if anim == _anim:
		return
	if anim == Anim.MOVE:
		_character.start_cycle(_character.idle_foot_right())
	if anim == Anim.STOP:
		_character.rest(&"exit")
	else:
		_character.rest(&"still")
	_anim = anim
	_state_age = 0.0
	_character.request(TREE_STATE[anim])


func _enter_stop(foot: int, speed: float) -> void:
	_character.set_exit_foot(foot)
	_stop = Stop.new()
	_stop.decel = speed / EXIT_STOP
	_enter(Anim.STOP)


func _exit_foot() -> int:
	if _character.cycle_length() <= 0.0:
		return -1
	var phase := _character.phase()
	for foot in 2:
		if absf(angle_difference(TAU * phase, TAU * (EXIT_AT + 0.5 * foot))) <= TAU * EXIT_WINDOW:
			return foot
	return -1


func _blocked(direction: Vector3) -> bool:
	if direction.is_zero_approx():
		return false
	var dir := direction.normalized()
	var from := global_transform
	var motion := dir * BLOCK_CHECK
	var hit := KinematicCollision3D.new()
	var walls := 0
	for i in max_slides:
		if not test_move(from, motion, hit):
			return false
		var normal := hit.get_normal()
		from.origin += hit.get_travel()
		if normal.angle_to(up_direction) <= floor_max_angle:
			motion = hit.get_remainder().slide(normal)
			continue
		walls += 1
		var wall := normal.slide(up_direction).normalized()
		if walls > 1 or wall.angle_to(-dir) < wall_min_slide_angle:
			return true
		motion = hit.get_remainder().slide(wall)
	return false


func _flat_ahead(dir: Vector3) -> bool:
	var space := get_world_3d().direct_space_state
	for k in range(0, int(FLAT_AHEAD / FLAT_STEP) + 1):
		var from := global_position + dir * (FLAT_STEP * k) + Vector3.UP * FLAT_RAY_UP
		var query := PhysicsRayQueryParameters3D.create(from, from + Vector3.DOWN * FLAT_RAY_DOWN, collision_mask, [get_rid()])
		var hit := space.intersect_ray(query)
		if hit.is_empty() or absf(hit.position.y - global_position.y) > FLAT_RISE:
			return false
	return true


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
		var change := walk_down if speed > wanted else acceleration
		speed = move_toward(speed, _aligned(yaw) * wanted, change * delta)
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
	var from := global_position + Vector3.UP * GROUND_RAY_UP
	var query := PhysicsRayQueryParameters3D.create(from, from + Vector3.DOWN * GROUND_RAY_DOWN, collision_mask, [get_rid()])
	var hit := get_world_3d().direct_space_state.intersect_ray(query)
	_floor_body = hit.get("collider")
	if hit.is_empty() or hit.normal.y < cos(floor_max_angle):
		return Vector3.UP
	return hit.normal


func _set_boarding(on: bool) -> void:
	_boarding = on
	board.visible = on
	_stop = null
	_enter(Anim.IDLE if on else Anim.FALL)
	var hit := _board_ground()
	if on and not hit.is_empty():
		global_position.y = maxf(global_position.y, hit.position.y + board_height)
		velocity.y = 0.0


func _board_ground() -> Dictionary:
	var from := global_position + Vector3.UP * GROUND_RAY_UP
	var query := PhysicsRayQueryParameters3D.create(from, from + Vector3.DOWN * board_reach, collision_mask, [get_rid()])
	return get_world_3d().direct_space_state.intersect_ray(query)


func _fly(direction: Vector3, delta: float) -> void:
	var flat := Vector3(velocity.x, 0.0, velocity.z).move_toward(direction * board_speed * _scale, board_acceleration * _scale * delta)
	var hit := _board_ground()
	if hit.is_empty():
		velocity.y = maxf(velocity.y - _gravity * delta, -max_fall_speed)
	else:
		var offset: float = global_position.y - hit.position.y - board_height
		velocity.y += (-board_spring * board_spring * offset - 2.0 * board_damping * board_spring * velocity.y) * delta
		if Input.is_action_just_pressed("jump") and absf(offset) < board_height:
			velocity.y = board_jump
	velocity.x = flat.x
	velocity.z = flat.z
	move_and_slide()
	_turn_skin(flat if flat.length() > STOP_SPEED else direction, delta)
	_character.set_move(0.0, 1.0)


func _follow_ride() -> void:
	var ride: Node3D = board if _boarding else (_floor_body as AnimatableBody3D if is_on_floor() else null)
	if ride == _ride:
		return
	_ride = ride
	_anchor_cloth()


func _anchor_cloth() -> void:
	for simulator: BoneClothSimulator3D in _character.find_children("*", "BoneClothSimulator3D", true, false):
		simulator.anchor = simulator.get_path_to(_ride) if _ride else NodePath()


func _settle_step(delta: float) -> void:
	var w := step_spring
	var e := exp(-w * delta)
	var k := (_step_speed + w * _step_offset) * delta
	_step_offset = (_step_offset + k) * e
	_step_speed = (_step_speed - w * k) * e
	_skin.position.y = _skin_y + _step_offset
	camera_rig.position.y = _rig_y + _step_offset
