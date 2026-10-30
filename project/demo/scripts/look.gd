extends Node


const BODY := [
	[&"UpperChest", Vector2(25.0, 10.0), 0.7],
	[&"Neck", Vector2(35.0, 30.0), 0.7],
	[&"Head", Vector2(45.0, 30.0), 0.85],
]
const EYE_LIMIT := Vector2(10.0, 10.0)
const HEAD_FOLLOW := 3.5
const REACH := 90.0
const REACH_OUT := 105.0
const MOVE_REACH := 45.0
const AHEAD := 3.0
const MOVING := 0.3
const WALK_PITCH := -15.0
const RUN_PITCH := -21.0

const PURSUIT := 6.0
const SACCADE_TRIGGER := 3.0
const SACCADE_BASE := 0.025
const SACCADE_PER_DEGREE := 0.0024
const SACCADE_LATENCY := 0.125
const CROSSING := Vector2(0.04, 0.18)
const BLINK_SHIFT := 15.0
const MUTUAL_TIME := Vector2(2.0, 5.0)
const GLANCE_CHANCE := 0.35
const GLANCE_TIME := Vector2(0.2, 0.6)
const RUN_FROM := 3.454
const WALK_AHEAD_TIME := Vector2(3.0, 6.0)
const WALK_GLANCE_TIME := Vector2(0.4, 1.0)
const HAND_OVER := 3.0

var _skeleton: Skeleton3D
var _body: CharacterBody3D
var _expression: Node
var _tree: AnimationTree
var _looks: Array[LookAtModifier3D] = []
var _chest_weight := 0.0
var _weight := 1.0
var _walk_glance := false
var _next_walk_glance := 0.0
var _head_target: Node3D
var _eye_target: Node3D
var _time := 0.0
var _at_camera := false
var _last_facing := 0.0
var _turn := 1.0
var _head := Vector2.ZERO
var _eye := Vector2.ZERO
var _pursuit := Vector2.ZERO
var _from := Vector2.ZERO
var _move_start := 0.0
var _move_time := 0.0
var _glance := Vector2.ZERO
var _next_glance := 0.0
var _last_wanted := Vector2.ZERO
var _saccade_at := -1.0


func _ready() -> void:
	var parent := get_parent()
	_skeleton = parent.find_children("*", "Skeleton3D", true, false)[0]
	_expression = parent.get_node_or_null("Expression")
	_tree = parent.get_node("AnimationTree")
	var node := parent
	while node != null and node is not CharacterBody3D:
		node = node.get_parent()
	_body = node
	_head_target = Node3D.new()
	_eye_target = Node3D.new()
	for marker in [_head_target, _eye_target]:
		marker.visible = false
		marker.physics_interpolation_mode = Node.PHYSICS_INTERPOLATION_MODE_OFF
	add_child(_head_target)
	add_child(_eye_target)
	for part in BODY:
		var look := _add_look(part[0], _head_target, SkeletonModifier3D.BONE_AXIS_PLUS_Z, Vector3.AXIS_Y, part[1], true)
		look.influence = part[2]
		_looks.append(look)
	_looks.append(_add_look(&"LeftEye", _eye_target, SkeletonModifier3D.BONE_AXIS_PLUS_Y, Vector3.AXIS_Z, EYE_LIMIT, false))
	_looks.append(_add_look(&"RightEye", _eye_target, SkeletonModifier3D.BONE_AXIS_PLUS_Y, Vector3.AXIS_Z, EYE_LIMIT, false))
	_chest_weight = BODY[0][2]
	_last_facing = _facing()
	_eye = _head
	_pursuit = _head
	_from = _head
	_next_glance = randf_range(MUTUAL_TIME.x, MUTUAL_TIME.y)
	_next_walk_glance = randf_range(WALK_AHEAD_TIME.x, WALK_AHEAD_TIME.y)


func _add_look(bone: StringName, target: Node3D, forward: SkeletonModifier3D.BoneAxis, turn: Vector3.Axis, limit: Vector2, relative: bool) -> LookAtModifier3D:
	var look := LookAtModifier3D.new()
	look.name = "Look" + bone
	look.bone_name = bone
	look.forward_axis = forward
	look.primary_rotation_axis = turn
	look.relative = relative
	look.use_angle_limitation = true
	look.symmetry_limitation = true
	look.primary_limit_angle = deg_to_rad(limit.x * 2.0)
	look.secondary_limit_angle = deg_to_rad(limit.y * 2.0)
	look.primary_damp_threshold = 0.5
	look.secondary_damp_threshold = 0.5
	_skeleton.add_child(look)
	for child in _skeleton.get_children():
		if child is BoneClothSimulator3D:
			_skeleton.move_child(look, child.get_index())
			break
	look.target_node = look.get_path_to(target)
	return look


func _process(delta: float) -> void:
	_time += delta
	var eyes := _eye_height_point()
	var facing := _facing()
	var turned := angle_difference(_last_facing, facing)
	if absf(turned) > 0.001:
		_turn = signf(turned)
	_last_facing = facing
	
	# 1
	var camera := get_viewport().get_camera_3d()
	var camera_at := camera.get_global_transform_interpolated().origin if camera != null else Vector3.ZERO
	var stepping := _speed() > MOVING
	var clip_speed := float(_tree.get("parameters/Move/blend_position")) * float(_tree.get("parameters/MoveRate/scale"))
	var walking := stepping and clip_speed < RUN_FROM
	var sees := camera != null and _within(camera_at - eyes, REACH_OUT if _at_camera else REACH)
	if not walking:
		_walk_glance = false
		_next_walk_glance = _time + randf_range(WALK_AHEAD_TIME.x, WALK_AHEAD_TIME.y)
	elif _time >= _next_walk_glance:
		_walk_glance = not _walk_glance and sees
		var span := WALK_GLANCE_TIME if _walk_glance else WALK_AHEAD_TIME
		_next_walk_glance = _time + randf_range(span.x, span.y)
	_at_camera = sees and (not stepping or _walk_glance)
	var goal := Vector2.ZERO
	var distance := AHEAD
	if _at_camera:
		goal = _direction(camera_at - eyes)
		distance = eyes.distance_to(camera_at)
	elif stepping:
		goal = Vector2(_direction(_body.velocity).x, deg_to_rad(WALK_PITCH if walking else RUN_PITCH))
	else:
		goal = Vector2(facing, 0.0)
	goal.x = _clamp_to_reach(angle_difference(facing, goal.x), REACH_OUT if _at_camera else (MOVE_REACH if stepping else REACH))
	
	# 2
	_head = _rotate_toward(_head, goal, 1.0 - exp(-HEAD_FOLLOW * delta))
	_head_target.global_position = eyes + _vector(_head, facing) * distance
	var state: StringName = _tree.get("parameters/State/current_state")
	var to_clip := state == &"land" or ((state == &"jump_start" or state == &"fall") and _body.velocity.y < 0.0)
	_weight = move_toward(_weight, 0.0 if to_clip else 1.0, delta * HAND_OVER)
	_chest_weight = move_toward(_chest_weight, BODY[0][2] if not stepping else 0.0, delta * 2.0)
	for i in _looks.size():
		var base: float = 1.0 if i >= BODY.size() else (_chest_weight if i == 0 else BODY[i][2])
		_looks[i].influence = base * _weight
	
	# 3
	if not _at_camera or stepping:
		_glance = Vector2.ZERO
	elif _time >= _next_glance:
		if _glance != Vector2.ZERO or randf() >= GLANCE_CHANCE:
			_glance = Vector2.ZERO
			_next_glance = _time + randf_range(MUTUAL_TIME.x, MUTUAL_TIME.y)
		else:
			_glance = _glance_offset()
			_next_glance = _time + randf_range(GLANCE_TIME.x, GLANCE_TIME.y)
	
	# 4
	var wanted := goal + _glance
	var moving := _time - _move_start < _move_time
	if not moving:
		var last := _pursuit
		_pursuit = _rotate_toward(_pursuit, wanted, 1.0 - exp(-PURSUIT * delta))
		if _saccade_at < 0.0 and delta > 0.0 and _wants_saccade(wanted, last, delta):
			_saccade_at = _time + SACCADE_LATENCY
		if _saccade_at >= 0.0 and _time >= _saccade_at:
			_saccade_at = -1.0
			var shift := rad_to_deg(_vector(_eye, 0.0).angle_to(_vector(wanted, 0.0)))
			_from = _eye
			_pursuit = wanted
			_move_start = _time
			_move_time = SACCADE_BASE + SACCADE_PER_DEGREE * shift
			if shift > BLINK_SHIFT and _expression != null:
				_expression.call(&"blink")
	var t := clampf((_time - _move_start) / _move_time, 0.0, 1.0) if _move_time > 0.0 else 1.0
	_eye = _rotate_toward(_from, _pursuit, t)
	_eye_target.global_position = eyes + _vector(_eye, facing) * distance
	_last_wanted = wanted


func _wants_saccade(wanted: Vector2, last: Vector2, delta: float) -> bool:
	if rad_to_deg(_vector(_pursuit, 0.0).angle_to(_vector(wanted, 0.0))) <= SACCADE_TRIGGER:
		return false
	var error := _difference(_pursuit, wanted)
	# slip(보려는 곳이 도는 속도 - 눈이 도는 속도, 라디안/초)
	var slip := (_difference(_last_wanted, wanted) - _difference(last, _pursuit)) / delta
	var closing := -slip.dot(error.normalized())
	if closing <= 0.0:
		return true
	var crossing := error.length() / closing
	return crossing < CROSSING.x or crossing > CROSSING.y


# 방향 from에서 to까지의 차이(라디안).
func _difference(from: Vector2, to: Vector2) -> Vector2:
	return Vector2(angle_difference(from.x, to.x), to.y - from.y)


func _rotate_toward(from: Vector2, to: Vector2, weight: float) -> Vector2:
	return Vector2(lerp_angle(from.x, to.x, weight), lerpf(from.y, to.y, weight))


func _clamp_to_reach(off: float, limit: float) -> float:
	var reach := deg_to_rad(limit)
	if absf(off) > deg_to_rad(150.0):
		off = _turn * absf(off)
	return clampf(off, -reach, reach)


func _within(offset: Vector3, limit: float) -> bool:
	return absf(angle_difference(_facing(), _direction(offset).x)) < deg_to_rad(limit)


func _direction(v: Vector3) -> Vector2:
	return Vector2(atan2(v.x, v.z), atan2(v.y, Vector2(v.x, v.z).length()))


func _vector(d: Vector2, facing: float) -> Vector3:
	var yaw := facing + d.x
	return Vector3(sin(yaw) * cos(d.y), sin(d.y), cos(yaw) * cos(d.y))


func _speed() -> float:
	return Vector2(_body.velocity.x, _body.velocity.z).length() if _body != null else 0.0


func _glance_offset() -> Vector2:
	var size := deg_to_rad(minf(-6.9 * log(randf_range(0.01, 15.0) / 15.7), EYE_LIMIT.x))
	var directions := [Vector2(1.0, 0.0), Vector2(-1.0, 0.0), Vector2(0.0, 0.5), Vector2(0.0, -0.5)]
	return directions[randi() % directions.size()] * size


func _facing() -> float:
	var z := (get_parent() as Node3D).global_basis.z
	return atan2(z.x, z.z)


func _eye_height_point() -> Vector3:
	var head := _skeleton.get_global_transform_interpolated() * _skeleton.get_bone_global_pose(_skeleton.find_bone(&"Head"))
	return head.origin + Vector3(0.0, 0.06, 0.0)
