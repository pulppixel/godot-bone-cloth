extends Node

const LEGS := [
	[&"LeftUpperLeg", &"LeftLowerLeg", &"LeftFoot", &"LeftToes"],
	[&"RightUpperLeg", &"RightLowerLeg", &"RightFoot", &"RightToes"],
]
const REACH := 0.35
const FOOT_SPRING := 31.6
const HIP_SPRING := 18.7
const HEEL := 0.06
const TIP := 0.05
const FOOT_GROUND := 2
const TREAD_REACH := 0.6
const PLANT_SPEED := 0.6
const PLANT_HEIGHT := 0.1
const MIN_EXTENSION := 0.2
const MAX_OFFSET := 0.5
const FADE := 0.4

var _skeleton: Skeleton3D
var _body: CharacterBody3D
var _hips := -1
var _bones := []
var _targets: Array[Node3D] = []
var _poles: Array[Node3D] = []
var _foot: Array[Vector2] = [Vector2.ZERO, Vector2.ZERO]
var _hip := Vector2.ZERO
var _floor_y := NAN
var _foot_basis: Array[Basis] = [Basis(), Basis()]
var _toes_floor := 0.0
var _last_toes: Array[Vector3] = [Vector3.INF, Vector3.INF]
var _planted: Array[bool] = [true, true]
var _weight: Array[float] = [1.0, 1.0]
var _hip_target := 0.0


class FootGround extends SkeletonModifier3D:
	var feet: Node

	func _process_modification_with_delta(delta: float) -> void:
		feet.call(&"_ground", delta)


class FootRestore extends SkeletonModifier3D:
	var feet: Node

	func _process_modification_with_delta(_delta: float) -> void:
		feet.call(&"_restore")


func _ready() -> void:
	var parent := get_parent()
	_skeleton = parent.find_children("*", "Skeleton3D", true, false)[0]
	var node := parent
	while node != null and node is not CharacterBody3D:
		node = node.get_parent()
	_body = node
	_hips = _skeleton.find_bone(&"Hips")
	for leg in LEGS:
		_bones.append(leg.map(func(bone: StringName) -> int: return _skeleton.find_bone(bone)))
		var target := Node3D.new()
		var pole := Node3D.new()
		add_child(target)
		add_child(pole)
		_targets.append(target)
		_poles.append(pole)
	
	_toes_floor = _skeleton.get_bone_global_rest(_bones[0][3]).origin.y
	var ground := FootGround.new()
	ground.name = "FootGround"
	ground.feet = self
	var ik := TwoBoneIK3D.new()
	ik.name = "FootIK"
	var restore := FootRestore.new()
	restore.name = "FootRestore"
	restore.feet = self
	for modifier in [ground, ik, restore]:
		_skeleton.add_child(modifier)
	ik.setting_count = LEGS.size()
	for i in LEGS.size():
		ik.set_root_bone_name(i, LEGS[i][0])
		ik.set_middle_bone_name(i, LEGS[i][1])
		ik.set_end_bone_name(i, LEGS[i][2])
		ik.set_target_node(i, ik.get_path_to(_targets[i]))
		ik.set_pole_node(i, ik.get_path_to(_poles[i]))
	_skeleton.move_child(ground, 0)
	_skeleton.move_child(ik, 1)
	_skeleton.move_child(restore, 2)


func _ground(delta: float) -> void:
	var space := _skeleton.global_transform
	var floor_y := space.origin.y
	var grounded := _body.is_on_floor()
	var lift := [0.0, 0.0]
	var moved := 0.0
	if grounded and not is_nan(_floor_y):
		moved = floor_y - _floor_y
	_floor_y = floor_y
	
	for i in LEGS.size():
		var ankle := space * _skeleton.get_bone_global_pose(_bones[i][2]).origin
		var toes := space * _skeleton.get_bone_global_pose(_bones[i][3]).origin
		var forward := ((toes - ankle) * Vector3(1, 0, 1)).normalized()
		if grounded:
			lift[i] = maxf(_floor_under(ankle - forward * HEEL, floor_y), _floor_under(toes + forward * TIP, floor_y)) - floor_y
		_foot[i] = _spring(_carried(_foot[i], lift[i], moved), lift[i], FOOT_SPRING, delta)
	
	for i in LEGS.size():
		_planted[i] = _plants(i, space, delta)
		_weight[i] = move_toward(_weight[i], 1.0 if _planted[i] else 0.0, delta / FADE)
	_hip_target = _pelvis(space)
	_hip = _spring(_carried(_hip, _hip_target, moved), _hip_target, HIP_SPRING, delta)
	
	for i in LEGS.size():
		var hip := space * _skeleton.get_bone_global_pose(_bones[i][0]).origin
		var knee := space * _skeleton.get_bone_global_pose(_bones[i][1]).origin
		var ankle := space * _skeleton.get_bone_global_pose(_bones[i][2]).origin
		var target := ankle + Vector3.UP * _foot[i].x
		var reach := _reach(hip, knee, ankle)
		var from_hip := target - (hip + Vector3.UP * _hip.x)
		if from_hip.length() > reach:
			target = hip + Vector3.UP * _hip.x + from_hip.normalized() * reach
		_targets[i].global_position = target
		var root := hip + Vector3.UP * _hip.x
		var line := (target - root).normalized()
		var thigh_x := space.basis * _skeleton.get_bone_global_pose(_bones[i][0]).basis.x
		var axis := thigh_x * signf(_skeleton.get_bone_global_rest(_bones[i][0]).basis.x.x)
		var out := line.cross(axis).normalized()
		_poles[i].global_position = (root + target) * 0.5 + out * 0.5
		_foot_basis[i] = _skeleton.get_bone_global_pose(_bones[i][2]).basis

	var pose := _skeleton.get_bone_pose_position(_hips)
	var parent := _skeleton.get_bone_parent(_hips)
	var up := space.basis.inverse() * Vector3.UP * _hip.x
	if parent >= 0:
		up = _skeleton.get_bone_global_pose(parent).basis.inverse() * up
	_skeleton.set_bone_pose_position(_hips, pose + up)


func _carried(state: Vector2, target: float, moved: float) -> Vector2:
	var carried := state.x - moved
	if absf(carried - target) < absf(state.x - target):
		state.x = carried
	return state


func _plants(i: int, space: Transform3D, delta: float) -> bool:
	var toes := _skeleton.get_bone_global_pose(_bones[i][3]).origin
	var world := space * toes
	var speed := 0.0
	if _last_toes[i] != Vector3.INF:
		speed = Vector2(world.x - _last_toes[i].x, world.z - _last_toes[i].z).length() / delta
	_last_toes[i] = world
	return toes.y < _toes_floor + PLANT_HEIGHT and speed < PLANT_SPEED


func _pelvis(space: Transform3D) -> float:
	var desired := []
	var mosts := []
	var least := -INF
	var total := 0.0
	var mean := 0.0
	for i in LEGS.size():
		var hip := space * _skeleton.get_bone_global_pose(_bones[i][0]).origin
		var knee := space * _skeleton.get_bone_global_pose(_bones[i][1]).origin
		var ankle := space * _skeleton.get_bone_global_pose(_bones[i][2]).origin
		var target := ankle + Vector3.UP * _foot[i].x
		var want := hip.distance_to(ankle)
		var full := hip.distance_to(knee) + knee.distance_to(ankle)
		var h := Vector2(target.x - hip.x, target.z - hip.z).length()
		desired.append(_rise_to(hip, target, h, want))
		mosts.append(_rise_to(hip, target, h, _reach(hip, knee, ankle)))
		var short := minf(want, MIN_EXTENSION * full)
		if h < short and _weight[i] > 0.0:
			least = maxf(least, _rise_to(hip, target, h, short))
		total += _weight[i]
		mean += _weight[i] * desired[i]
	if total <= 0.0:
		return _hip_target
	mean /= total
	var high: float = desired.max()
	var low := INF
	var most := INF
	for i in LEGS.size():
		low = minf(low, lerpf(high, desired[i], _weight[i]))
		most = minf(most, mosts[i] + (1.0 - _weight[i]) * MAX_OFFSET * 2.0)
	var offset := low
	if most > low and mean > low:
		offset = low + (mean - low) * (most - low) / ((mean - low) + (most - low))
	return clampf(clampf(offset, least, most), -MAX_OFFSET, MAX_OFFSET)


func _rise_to(hip: Vector3, target: Vector3, h: float, r: float) -> float:
	if h >= r:
		return target.y - hip.y
	return target.y + sqrt(r * r - h * h) - hip.y


func _reach(hip: Vector3, knee: Vector3, ankle: Vector3) -> float:
	var desired := hip.distance_to(ankle)
	var full := hip.distance_to(knee) + knee.distance_to(ankle)
	return desired + 0.5 * (full - desired)


func _restore() -> void:
	for i in LEGS.size():
		var foot: int = _bones[i][2]
		var parent := _skeleton.get_bone_parent(foot)
		var local: Basis = _skeleton.get_bone_global_pose(parent).basis.inverse() * _foot_basis[i]
		_skeleton.set_bone_pose_rotation(foot, local.get_rotation_quaternion())


func _floor_under(point: Vector3, floor_y: float) -> float:
	var hit := _ray(point, floor_y, TREAD_REACH, FOOT_GROUND)
	if hit.is_empty():
		hit = _ray(point, floor_y, REACH, _body.collision_mask)
	if hit.is_empty() or hit.normal.y < 0.7:
		return floor_y
	return hit.position.y


func _ray(point: Vector3, floor_y: float, reach: float, mask: int) -> Dictionary:
	var from := Vector3(point.x, floor_y + reach, point.z)
	var to := Vector3(point.x, floor_y - reach, point.z)
	var query := PhysicsRayQueryParameters3D.create(from, to, mask, [_body.get_rid()])
	return _skeleton.get_world_3d().direct_space_state.intersect_ray(query)


func _spring(state: Vector2, target: float, w: float, delta: float) -> Vector2:
	var x := state.x - target
	var e := exp(-w * delta)
	var k := (state.y + w * x) * delta
	return Vector2((x + k) * e + target, (state.y - w * k) * e)
