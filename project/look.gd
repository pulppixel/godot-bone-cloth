extends Node3D


const HEAD_LIMIT := Vector2(50.0, 20.0)
const HEAD_SHARE := 0.85
const HEAD_FOLLOW := 3.5
const EYE_LIMIT := Vector2(10.0, 10.0)
const CAMERA_CONE := 55.0
const AHEAD := 3.0
const MOVING := 0.3

const PURSUIT := 6.0
const SACCADE_TRIGGER := 3.0
const SACCADE_BASE := 0.025
const SACCADE_PER_DEGREE := 0.0024
const BLINK_SHIFT := 15.0
const MUTUAL_TIME := Vector2(2.0, 5.0)
const GLANCE_CHANCE := 0.35
const GLANCE_TIME := Vector2(0.2, 0.6)

var _skeleton: Skeleton3D
var _body: CharacterBody3D
var _expression: Node
var _head_target: Node3D
var _eye_target: Node3D
var _time := 0.0
var _from := Vector3.ZERO
var _to := Vector3.ZERO
var _move_start := 0.0
var _move_time := 0.0
var _glance := Vector3.ZERO
var _next_glance := 0.0


func _ready() -> void:
	var parent := get_parent()
	_skeleton = parent.find_children("*", "Skeleton3D", true, false)[0]
	_expression = parent.get_node_or_null("Expression")
	var node := parent
	while node != null and node is not CharacterBody3D:
		node = node.get_parent()
	_body = node
	_head_target = Node3D.new()
	_eye_target = Node3D.new()
	add_child(_head_target)
	add_child(_eye_target)
	_head_target.top_level = true
	_eye_target.top_level = true
	_add_look(&"Head", _head_target, SkeletonModifier3D.BONE_AXIS_PLUS_Z, Vector3.AXIS_Y, HEAD_LIMIT, true).influence = HEAD_SHARE
	_add_look(&"LeftEye", _eye_target, SkeletonModifier3D.BONE_AXIS_PLUS_Y, Vector3.AXIS_Z, EYE_LIMIT, false)
	_add_look(&"RightEye", _eye_target, SkeletonModifier3D.BONE_AXIS_PLUS_Y, Vector3.AXIS_Z, EYE_LIMIT, false)
	var start := _eye_height_point(AHEAD)
	_head_target.global_position = start
	_eye_target.global_position = start
	_from = start
	_to = start
	_next_glance = randf_range(MUTUAL_TIME.x, MUTUAL_TIME.y)


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
	look.target_node = look.get_path_to(target)
	return look


func _process(delta: float) -> void:
	_time += delta
	var camera := get_viewport().get_camera_3d()
	var at_camera := _sees(camera)
	var goal := camera.global_position if at_camera else _ahead()
	
	_head_target.global_position = _head_target.global_position.lerp(goal, 1.0 - exp(-HEAD_FOLLOW * delta))
	
	# 카메라를 볼 땐 눈만 잠깐 돌리기.
	if not at_camera:
		_glance = Vector3.ZERO
	elif _time >= _next_glance:
		if _glance != Vector3.ZERO or randf() >= GLANCE_CHANCE:
			_glance = Vector3.ZERO
			_next_glance = _time + randf_range(MUTUAL_TIME.x, MUTUAL_TIME.y)
		else:
			_glance = _glance_offset(goal)
			_next_glance = _time + randf_range(GLANCE_TIME.x, GLANCE_TIME.y)
	
	var wanted := goal + _glance
	var eyes := _eye_height_point(0.0)
	var moving := _time - _move_start < _move_time
	if not moving:
		_to = _to.lerp(wanted, 1.0 - exp(-PURSUIT * delta))
	if not moving and rad_to_deg((_to - eyes).angle_to(wanted - eyes)) > SACCADE_TRIGGER:
		var shift := rad_to_deg((_eye_target.global_position - eyes).angle_to(wanted - eyes))
		_from = _eye_target.global_position
		_to = wanted
		_move_start = _time
		_move_time = SACCADE_BASE + SACCADE_PER_DEGREE * shift
		if shift > BLINK_SHIFT and _expression != null:
			_expression.call(&"blink")
	var t := clampf((_time - _move_start) / _move_time, 0.0, 1.0) if _move_time > 0.0 else 1.0
	_eye_target.global_position = _from.lerp(_to, t)


func _sees(camera: Camera3D) -> bool:
	if camera == null or _speed() > MOVING:
		return false
	var to_camera := camera.global_position - _eye_height_point(0.0)
	return _forward().angle_to(Vector3(to_camera.x, 0.0, to_camera.z)) < deg_to_rad(CAMERA_CONE)


func _ahead() -> Vector3:
	var velocity := _body.velocity if _body != null else Vector3.ZERO
	velocity.y = 0.0
	var direction := velocity.normalized() if velocity.length() > MOVING else _forward()
	return _eye_height_point(0.0) + direction * AHEAD


func _speed() -> float:
	return Vector2(_body.velocity.x, _body.velocity.z).length() if _body != null else 0.0


func _glance_offset(goal: Vector3) -> Vector3:
	var size := minf(-6.9 * log(randf_range(0.01, 15.0) / 15.7), EYE_LIMIT.x)
	var side := _forward().cross(Vector3.UP)
	var directions := [side, -side, Vector3.UP * 0.5, Vector3.DOWN * 0.5]
	var direction: Vector3 = directions[randi() % directions.size()]
	var distance := goal.distance_to(_eye_height_point(0.0))
	return direction * tan(deg_to_rad(size)) * distance


func _forward() -> Vector3:
	var z := (get_parent() as Node3D).global_basis.z
	return Vector3(z.x, 0.0, z.z).normalized()


func _eye_height_point(distance: float) -> Vector3:
	var head := _skeleton.global_transform * _skeleton.get_bone_global_pose(_skeleton.find_bone(&"Head"))
	return head.origin + Vector3(0.0, 0.06, 0.0) + _forward() * distance
