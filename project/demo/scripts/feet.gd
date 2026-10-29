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
const MIN_EXTENSION := 0.2
const MAX_OFFSET := 0.5
const SAMPLES := 60
const LIFTED := 0.04
const SUPPORT := 0.05
const SHIFT := 0.3
const STEP := 0.02
const EDGE := 0.02
const SWEEP := 6
const MARGIN := 0.04
const SLIDE := 2.0
const HEEL_LIFT := 0.5
const HORIZONTAL := 0.1

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
var _tree: AnimationTree
var _move: AnimationNodeBlendSpace1D
var _cycles := []
var _phase := 0.0
var _planted: Array[bool] = [false, false]
var _offset: Array[Vector3] = [Vector3.ZERO, Vector3.ZERO]
var _offset_from: Array[Vector3] = [Vector3.ZERO, Vector3.ZERO]
var _offset_to: Array[Vector3] = [Vector3.ZERO, Vector3.ZERO]
var _shift: Array[float] = [NAN, NAN]
var _progress: Array[float] = [0.0, 0.0]


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
	_tree = parent.get_node("AnimationTree")
	_move = (_tree.tree_root as AnimationNodeBlendTree).get_node(&"Move")
	_hips = _skeleton.find_bone(&"Hips")
	for leg in LEGS:
		_bones.append(leg.map(func(bone: StringName) -> int: return _skeleton.find_bone(bone)))
		var target := Node3D.new()
		var pole := Node3D.new()
		for marker in [target, pole]:
			marker.visible = false
			marker.physics_interpolation_mode = Node.PHYSICS_INTERPOLATION_MODE_OFF
		add_child(target)
		add_child(pole)
		_targets.append(target)
		_poles.append(pole)
	
	var player: AnimationPlayer = parent.get_node("AnimationPlayer")
	for i in _move.get_blend_point_count():
		var clip := _move.get_blend_point_node(i) as AnimationNodeAnimation
		_cycles.append(_analyze(player.get_animation(clip.animation)))
	
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


func _analyze(anim: Animation) -> Array:
	var legs := []
	for i in LEGS.size():
		var ankle_rest := _skeleton.get_bone_global_rest(_bones[i][2]).origin.y
		var toes_rest := _skeleton.get_bone_global_rest(_bones[i][3]).origin.y
		var ankles: Array[Vector3] = []
		var toes: Array[Vector3] = []
		var low: Array[float] = []
		var high := 0.0
		var middle := 0.0
		for s in SAMPLES:
			var t := anim.length * s / SAMPLES
			ankles.append(_clip_bone(anim, _bones[i][2], t))
			toes.append(_clip_bone(anim, _bones[i][3], t))
			var heel := ankles[s].y - ankle_rest
			var toe := toes[s].y - toes_rest
			low.append(minf(heel, toe))
			high = maxf(high, maxf(heel, toe))
			middle += (ankles[s].z + toes[s].z) * 0.5 / SAMPLES
		var span := 0.0
		for s in SAMPLES:
			span = maxf(span, absf((ankles[s].z + toes[s].z) * 0.5 - middle))
		var stance := 0
		var best := INF
		for s in SAMPLES:
			var heel := ankles[s].y - ankle_rest
			var toe := toes[s].y - toes_rest
			var cost := maxf(heel, toe) / high + absf((ankles[s].z + toes[s].z) * 0.5 - middle) / span
			if cost < best:
				best = cost
				stance = s
		var off := stance
		while low[off % SAMPLES] < LIFTED:
			off += 1
		var strike := stance + SAMPLES
		while low[(strike - 1) % SAMPLES] < LIFTED:
			strike -= 1
		legs.append({
			"stance": float(stance) / SAMPLES,
			"off": float(off - stance) / SAMPLES,
			"strike": float(strike - stance) / SAMPLES,
			"off_z": ankles[off % SAMPLES].z,
			"strike_z": ankles[strike % SAMPLES].z,
			"off_ankle": ankles[off % SAMPLES],
			"off_toes": toes[off % SAMPLES],
			"strike_ankle": ankles[strike % SAMPLES],
			"strike_toes": toes[strike % SAMPLES],
		})
	return legs


func _clip_bone(anim: Animation, bone: int, time: float) -> Vector3:
	var chain: Array[int] = []
	while bone >= 0:
		chain.push_front(bone)
		bone = _skeleton.get_bone_parent(bone)
	var pose := Transform3D()
	for b in chain:
		var rest := _skeleton.get_bone_rest(b)
		var path := NodePath("%GeneralSkeleton:" + _skeleton.get_bone_name(b))
		var position := rest.origin
		var rotation := rest.basis.get_rotation_quaternion()
		var track := anim.find_track(path, Animation.TYPE_POSITION_3D)
		if track >= 0:
			position = anim.position_track_interpolate(track, time) * _skeleton.motion_scale
		track = anim.find_track(path, Animation.TYPE_ROTATION_3D)
		if track >= 0:
			rotation = anim.rotation_track_interpolate(track, time)
		pose = pose * Transform3D(Basis(rotation), position)
	return pose.origin


func _cycle(delta: float) -> Array:
	if _tree.get("parameters/State/current_state") != &"move":
		return []
	var length: float = _tree.get("parameters/Move/current_length")
	if length <= 0.0:
		return []
	var clip := _move.get_blend_point_node(0) as AnimationNodeAnimation
	var half := clip.start_offset / clip.timeline_length if clip.use_custom_timeline else 0.0
	var phase := fposmod(_tree.get("parameters/Move/current_position") / length + half, 1.0)
	var advance := fposmod(phase - _phase, 1.0)
	_phase = phase
	if advance > 0.0 and advance < 0.5 and delta > 0.0:
		length = delta / advance
	var at: float = _tree.get("parameters/Move/blend_position")
	var weights := []
	for p in _move.get_blend_point_count():
		weights.append(0.0)
	var count := _move.get_blend_point_count()
	if at <= _move.get_blend_point_position(0):
		weights[0] = 1.0
	elif at >= _move.get_blend_point_position(count - 1):
		weights[count - 1] = 1.0
	else:
		for p in count - 1:
			var a := _move.get_blend_point_position(p)
			var b := _move.get_blend_point_position(p + 1)
			if at >= a and at <= b:
				weights[p + 1] = (at - a) / (b - a)
				weights[p] = 1.0 - weights[p + 1]
	var legs := []
	for i in LEGS.size():
		var leg := {"stance": 0.0, "off": 0.0, "strike": 0.0, "off_z": 0.0, "strike_z": 0.0, "off_ankle": Vector3.ZERO, "off_toes": Vector3.ZERO, "strike_ankle": Vector3.ZERO, "strike_toes": Vector3.ZERO}
		var first: float = _cycles[0][i].stance
		for p in count:
			var w: float = weights[p]
			var c: Dictionary = _cycles[p][i]
			leg.stance += w * (first + wrapf(c.stance - first, -0.5, 0.5))
			for key in ["off", "strike", "off_z", "strike_z", "off_ankle", "off_toes", "strike_ankle", "strike_toes"]:
				leg[key] += w * c[key]
		leg.time = fposmod(phase - leg.stance, 1.0)
		leg.length = length
		legs.append(leg)
	return legs


func _ground(delta: float) -> void:
	var space := _skeleton.get_global_transform_interpolated()
	var floor_y := space.origin.y
	var grounded := _body.is_on_floor()
	var lift := [0.0, 0.0]
	var moved := 0.0
	if grounded and not is_nan(_floor_y):
		moved = floor_y - _floor_y
	_floor_y = floor_y
	
	var cycle := _cycle(delta) if grounded else []
	var heights := [floor_y, floor_y]
	var flights := [0.0, 0.0]
	
	for i in LEGS.size():
		var ankle := space * _skeleton.get_bone_global_pose(_bones[i][2]).origin
		var toes := space * _skeleton.get_bone_global_pose(_bones[i][3]).origin
		if not grounded:
			_planted[i] = false
			continue
		var swinging: bool = not cycle.is_empty() and cycle[i].time > cycle[i].off and cycle[i].time < cycle[i].strike
		if not swinging or not _planted[i]:
			_offset_from[i] = _offset[i]
			_offset_to[i] = _offset[i]
			_shift[i] = NAN
			_progress[i] = 0.0
			_planted[i] = true
		else:
			var leg: Dictionary = cycle[i]
			var velocity: Vector3 = _body.velocity * Vector3(1, 0, 1)
			if velocity.length() > 0.1:
				_shift[i] = _footprint(space, leg, velocity, floor_y, 0.0 if is_nan(_shift[i]) else _shift[i])
				_offset_to[i] = velocity.normalized() * _shift[i]
			var z := _skeleton.get_bone_global_pose(_bones[i][2]).origin.z
			var s := maxf(_progress[i], clampf(inverse_lerp(leg.off_z, leg.strike_z, z), 0.0, 1.0))
			_progress[i] = s
			_offset[i] = _offset[i].move_toward(_offset_from[i].lerp(_offset_to[i], s), SLIDE * delta)
			flights[i] = sin(PI * (leg.time - leg.off) / (leg.strike - leg.off))
		heights[i] = _ground_at(ankle + _offset[i], toes + _offset[i], floor_y, TREAD_REACH)
	
	if not cycle.is_empty():
		var total := 0.0
		var support := 0.0
		for i in LEGS.size():
			var w := (cos(TAU * cycle[i].time) + 1.0) * 0.5 + SUPPORT
			total += w
			support += w * heights[i]
		support /= total
		for i in LEGS.size():
			heights[i] = maxf(heights[i], lerpf(heights[i], support, flights[i]))
	
	for i in LEGS.size():
		if grounded:
			lift[i] = heights[i] - floor_y
		_foot[i] = _spring(_carried(_foot[i], lift[i], moved), lift[i], FOOT_SPRING, delta)
	
	var hip_target := _pelvis(space)
	_hip = _spring(_carried(_hip, hip_target, moved), hip_target, HIP_SPRING, delta)
	
	for i in LEGS.size():
		var hip := space * _skeleton.get_bone_global_pose(_bones[i][0]).origin
		var knee := space * _skeleton.get_bone_global_pose(_bones[i][1]).origin
		var ankle := space * _skeleton.get_bone_global_pose(_bones[i][2]).origin
		var target := ankle + _offset[i] + Vector3.UP * _foot[i].x
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


func _footprint(space: Transform3D, leg: Dictionary, velocity: Vector3, floor_y: float, prefer: float) -> float:
	var dir := velocity.normalized()
	var at_strike: Vector3 = space.origin + velocity * leg.length * (leg.strike - leg.time)
	var at_off: Vector3 = space.origin + velocity * leg.length * (1.0 + leg.off - leg.time)
	var heel_strike: Vector3 = at_strike + space.basis * leg.strike_ankle - dir * HEEL
	var origin := heel_strike * Vector3(1, 0, 1)
	var heel := [0.0, (at_off + space.basis * leg.off_ankle - dir * HEEL - heel_strike).dot(dir)]
	var toe := [(at_strike + space.basis * leg.strike_toes + dir * TIP - heel_strike).dot(dir), (at_off + space.basis * leg.off_toes + dir * TIP - heel_strike).dot(dir)]
	var first := minf(heel[0], heel[1]) - SHIFT - MARGIN
	var last := maxf(toe[0], toe[1]) + SHIFT + MARGIN
	var count := int(ceil((last - first) / STEP)) + 1
	var heights: Array[float] = []
	for k in count:
		heights.append(_floor_under(origin + dir * (first + k * STEP), floor_y, TREAD_REACH + last))
	var limit := int(round(SHIFT / STEP))
	var order: Array[int] = []
	for n in range(-limit, limit + 1):
		order.append(n)
	order.sort_custom(func(x: int, y: int) -> bool: return absf(x * STEP - prefer) < absf(y * STEP - prefer))
	var fallback := NAN
	for n in order:
		var d := n * STEP
		var low := INF
		var high := -INF
		for t in SWEEP:
			var f := float(t) / (SWEEP - 1)
			for m in [-MARGIN, 0.0, MARGIN]:
				var h := maxf(heights[int(round((lerpf(heel[0], heel[1], f) + d + m - first) / STEP))], heights[int(round((lerpf(toe[0], toe[1], f) + d + m - first) / STEP))])
				low = minf(low, h)
				high = maxf(high, h)
		if is_nan(fallback):
			fallback = d
		if high - low <= EDGE:
			return d
	return fallback


func _ground_at(ankle: Vector3, toes: Vector3, floor_y: float, reach: float) -> float:
	var forward := ((toes - ankle) * Vector3(1, 0, 1)).normalized()
	return maxf(_floor_under(ankle - forward * HEEL, floor_y, reach), _floor_under(toes + forward * TIP, floor_y, reach))


func _carried(state: Vector2, target: float, moved: float) -> Vector2:
	if signf(target - state.x) == -signf(moved):
		state.x = move_toward(state.x, target, absf(moved))
	return state


func _pelvis(space: Transform3D) -> float:
	var low := INF
	var mean := 0.0
	var most := INF
	var least := -INF
	for i in LEGS.size():
		var hip := space * _skeleton.get_bone_global_pose(_bones[i][0]).origin
		var knee := space * _skeleton.get_bone_global_pose(_bones[i][1]).origin
		var ankle_local := _skeleton.get_bone_global_pose(_bones[i][2]).origin
		var toes_local := _skeleton.get_bone_global_pose(_bones[i][3]).origin
		var ankle := space * ankle_local
		var target := ankle + _offset[i] + Vector3.UP * _foot[i].x
		var want := hip.distance_to(ankle)
		var full := hip.distance_to(knee) + knee.distance_to(ankle)
		var reach := _reach(hip, knee, ankle)
		var at_want := target
		var at_reach := target
		var above := hip.y - target.y
		if above > 0.0:
			var fk := Vector3(ankle.x, target.y, ankle.z)
			var under_hip := Vector3(hip.x, target.y, hip.z)
			var foot_length := _skeleton.get_bone_rest(_bones[i][3]).origin.length()
			target += (fk - target).limit_length(foot_length * HEEL_LIFT)
			var away := target.distance_to(under_hip)
			var clip_away := fk.distance_to(under_hip)
			at_want = _pulled(target, under_hip, away, maxf(clip_away, sqrt(maxf(0.0, want * want - (above - HORIZONTAL) * (above - HORIZONTAL)))))
			at_reach = _pulled(target, under_hip, away, maxf(clip_away, sqrt(maxf(0.0, reach * reach - (above - HORIZONTAL) * (above - HORIZONTAL)))))
		var desired := _rise_to(hip, at_want, Vector2(at_want.x - hip.x, at_want.z - hip.z).length(), want)
		low = minf(low, desired)
		mean += desired / LEGS.size()
		most = minf(most, _rise_to(hip, at_reach, Vector2(at_reach.x - hip.x, at_reach.z - hip.z).length(), reach))
		target = at_want
		least = maxf(least, target.y + minf(want, MIN_EXTENSION * full) - hip.y - minf(ankle_local.y, toes_local.y))
	var to_mean := mean - low
	var to_most := most - low
	var offset := low
	if not is_zero_approx(to_mean + to_most):
		offset = low + to_mean * to_most / (to_mean + to_most)
	return clampf(clampf(offset, least, most), -MAX_OFFSET, MAX_OFFSET)


func _pulled(target: Vector3, under_hip: Vector3, away: float, limit: float) -> Vector3:
	if away > limit:
		return target + (under_hip - target).normalized() * (away - limit)
	return target


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


func _floor_under(point: Vector3, floor_y: float, reach := TREAD_REACH) -> float:
	var hit := _ray(point, floor_y, reach, FOOT_GROUND)
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
