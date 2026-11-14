extends Node

const CharacterSkin := preload("res://demo/scripts/skin.gd")

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
const PADS: Array[NodePath] = [^"LeftPad", ^"RightPad"]
const MOVING := 0.1
const SHIFT := 0.3
const STEP := 0.02
const EDGE := 0.02
const SWEEP := 6
const MARGIN := 0.04
const SLIDE := 2.0
const HEEL_LIFT := 0.5
const HORIZONTAL := 0.1
const POLE_OUT := 0.5
const FLOOR_NORMAL := 0.7
const STRETCH := 0.5

var _skeleton: Skeleton3D
var _hips := -1
var _legs: Array[Leg] = []
var _hip := Vector2.ZERO
var _floor_y := NAN
var _riding := false
var board: Node3D
var _skin: CharacterSkin
var _cycles := []
var _phase := 0.0


class Leg:
	var upper := -1
	var lower := -1
	var ankle := -1
	var toes := -1
	var target: Node3D
	var pole: Node3D
	var lift := Vector2.ZERO
	var ankle_basis := Basis()
	var planted := false
	var offset := Vector3.ZERO
	var offset_from := Vector3.ZERO
	var offset_to := Vector3.ZERO
	var shift := NAN
	var progress := 0.0


class Cycle:
	var stance := 0.0
	var off := 0.0
	var strike := 0.0
	var off_z := 0.0
	var strike_z := 0.0
	var off_ankle := Vector3.ZERO
	var off_toes := Vector3.ZERO
	var strike_ankle := Vector3.ZERO
	var strike_toes := Vector3.ZERO
	var time := 0.0
	var length := 0.0


class FootGround extends SkeletonModifier3D:
	var ground: Callable
	
	func _process_modification_with_delta(delta: float) -> void:
		ground.call(delta)


class FootRestore extends SkeletonModifier3D:
	var restore: Callable
	
	func _process_modification_with_delta(_delta: float) -> void:
		restore.call()


func _ready() -> void:
	_skin = get_parent() as CharacterSkin
	_skeleton = _skin.skeleton
	_hips = _skeleton.find_bone(&"Hips")
	for names in LEGS:
		var leg := Leg.new()
		leg.upper = _skeleton.find_bone(names[0])
		leg.lower = _skeleton.find_bone(names[1])
		leg.ankle = _skeleton.find_bone(names[2])
		leg.toes = _skeleton.find_bone(names[3])
		leg.target = Node3D.new()
		leg.pole = Node3D.new()
		for marker in [leg.target, leg.pole]:
			marker.visible = false
			marker.physics_interpolation_mode = Node.PHYSICS_INTERPOLATION_MODE_OFF
		add_child(leg.target)
		add_child(leg.pole)
		_legs.append(leg)
	
	for i in _skin.clip_count():
		_cycles.append(_analyze(_skin.clip(i)))
	
	var ground := FootGround.new()
	ground.name = "FootGround"
	ground.ground = _ground
	var ik := TwoBoneIK3D.new()
	ik.name = "FootIK"
	var restore := FootRestore.new()
	restore.name = "FootRestore"
	restore.restore = _restore
	for modifier in [ground, ik, restore]:
		_skeleton.add_child(modifier)
	ik.setting_count = LEGS.size()
	for i in LEGS.size():
		ik.set_root_bone_name(i, LEGS[i][0])
		ik.set_middle_bone_name(i, LEGS[i][1])
		ik.set_end_bone_name(i, LEGS[i][2])
		ik.set_target_node(i, ik.get_path_to(_legs[i].target))
		ik.set_pole_node(i, ik.get_path_to(_legs[i].pole))
	_skeleton.move_child(ground, 0)
	_skeleton.move_child(ik, 1)
	_skeleton.move_child(restore, 2)


func _analyze(anim: Animation) -> Array[Cycle]:
	var legs: Array[Cycle] = []
	for leg in _legs:
		var ankle_rest := _skeleton.get_bone_global_rest(leg.ankle).origin.y
		var toes_rest := _skeleton.get_bone_global_rest(leg.toes).origin.y
		var ankles: Array[Vector3] = []
		var toes: Array[Vector3] = []
		var low: Array[float] = []
		var high := 0.0
		var middle := 0.0
		for s in SAMPLES:
			var t := anim.length * s / SAMPLES
			ankles.append(_clip_bone(anim, leg.ankle, t))
			toes.append(_clip_bone(anim, leg.toes, t))
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
		var cycle := Cycle.new()
		cycle.stance = float(stance) / SAMPLES
		cycle.off = float(off - stance) / SAMPLES
		cycle.strike = float(strike - stance) / SAMPLES
		cycle.off_z = ankles[off % SAMPLES].z
		cycle.strike_z = ankles[strike % SAMPLES].z
		cycle.off_ankle = ankles[off % SAMPLES]
		cycle.off_toes = toes[off % SAMPLES]
		cycle.strike_ankle = ankles[strike % SAMPLES]
		cycle.strike_toes = toes[strike % SAMPLES]
		legs.append(cycle)
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


func _cycle(delta: float) -> Array[Cycle]:
	var none: Array[Cycle] = []
	if _skin.state != &"move":
		return none
	var length := _skin.cycle_length()
	if length <= 0.0:
		return none
	var phase := _skin.phase()
	var advance := fposmod(phase - _phase, 1.0)
	_phase = phase
	if advance > 0.0 and advance < 0.5 and delta > 0.0:
		length = delta / advance
	var weights := _skin.blend_weights()
	var count := weights.size()
	var legs: Array[Cycle] = []
	for i in LEGS.size():
		var leg := Cycle.new()
		var first: float = _cycles[0][i].stance
		for p in count:
			var w: float = weights[p]
			var c: Cycle = _cycles[p][i]
			leg.stance += w * (first + wrapf(c.stance - first, -0.5, 0.5))
			leg.off += w * c.off
			leg.strike += w * c.strike
			leg.off_z += w * c.off_z
			leg.strike_z += w * c.strike_z
			leg.off_ankle += w * c.off_ankle
			leg.off_toes += w * c.off_toes
			leg.strike_ankle += w * c.strike_ankle
			leg.strike_toes += w * c.strike_toes
		leg.time = fposmod(phase - leg.stance, 1.0)
		leg.length = length
		legs.append(leg)
	return legs


func _ground(delta: float) -> void:
	var space := _skeleton.get_global_transform_interpolated()
	if board != null:
		_ride(space)
		return
	if _riding:
		_riding = false
		_hip = Vector2.ZERO
		_floor_y = NAN
		for leg in _legs:
			leg.offset = Vector3.ZERO
			leg.lift = Vector2.ZERO
	var floor_y := space.origin.y
	var grounded := _skin.body.is_on_floor()
	var moved := 0.0
	if grounded and not is_nan(_floor_y):
		moved = floor_y - _floor_y
	_floor_y = floor_y

	var none: Array[Cycle] = []
	var cycle := _cycle(delta) if grounded else none
	var heights := [floor_y, floor_y]
	var flights := [0.0, 0.0]

	for i in LEGS.size():
		var leg := _legs[i]
		if not grounded:
			leg.planted = false
			continue
		flights[i] = _swing(leg, cycle[i] if not cycle.is_empty() else null, space, floor_y, delta)
		var ankle := space * _skeleton.get_bone_global_pose(leg.ankle).origin
		var toes := space * _skeleton.get_bone_global_pose(leg.toes).origin
		heights[i] = _ground_at(ankle + leg.offset, toes + leg.offset, floor_y, TREAD_REACH)

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
		var lift: float = heights[i] - floor_y if grounded else 0.0
		_legs[i].lift = _spring(_carried(_legs[i].lift, lift, moved), lift, FOOT_SPRING, delta)

	var hip_target := _pelvis(space)
	_hip = _spring(_carried(_hip, hip_target, moved), hip_target, HIP_SPRING, delta)

	for i in LEGS.size():
		_aim(_legs[i], space, heights[i], grounded and cycle.is_empty())
	_raise_hips(space)


func _ride(space: Transform3D) -> void:
	_riding = true
	_hip = Vector2.ZERO
	var ankle_height := _skeleton.get_bone_global_rest(_legs[0].ankle).origin.y * space.basis.get_scale().y
	for i in LEGS.size():
		var leg := _legs[i]
		var pad := board.get_node(PADS[i]) as Node3D
		var at := pad.get_global_transform_interpolated()
		var ankle := space * _skeleton.get_bone_global_pose(leg.ankle).origin
		leg.offset = at.origin + at.basis.y.normalized() * ankle_height - ankle
		leg.lift = Vector2.ZERO
		leg.planted = false
		_aim(leg, space, 0.0, false)


func _swing(leg: Leg, cycle: Cycle, space: Transform3D, floor_y: float, delta: float) -> float:
	var swinging: bool = cycle != null and cycle.time > cycle.off and cycle.time < cycle.strike
	if not swinging or not leg.planted:
		leg.offset_from = leg.offset
		leg.offset_to = leg.offset
		leg.shift = NAN
		leg.progress = 0.0
		leg.planted = true
		return 0.0
	var velocity: Vector3 = _skin.body.velocity * Vector3(1, 0, 1)
	if velocity.length() > MOVING:
		leg.shift = _footprint(space, cycle, velocity, floor_y, 0.0 if is_nan(leg.shift) else leg.shift)
		leg.offset_to = velocity.normalized() * leg.shift
	var z := _skeleton.get_bone_global_pose(leg.ankle).origin.z
	var s := maxf(leg.progress, clampf(inverse_lerp(cycle.off_z, cycle.strike_z, z), 0.0, 1.0))
	leg.progress = s
	leg.offset = leg.offset.move_toward(leg.offset_from.lerp(leg.offset_to, s), SLIDE * delta)
	return sin(PI * (cycle.time - cycle.off) / (cycle.strike - cycle.off))


func _aim(leg: Leg, space: Transform3D, height: float, p_standing: bool) -> void:
	var hip := space * _skeleton.get_bone_global_pose(leg.upper).origin
	var knee := space * _skeleton.get_bone_global_pose(leg.lower).origin
	var ankle := space * _skeleton.get_bone_global_pose(leg.ankle).origin
	var target := ankle + leg.offset + Vector3.UP * leg.lift.x
	if p_standing:
		target.y = maxf(target.y, height + _skeleton.get_bone_global_rest(leg.ankle).origin.y * space.basis.get_scale().y)
	var reach := _reach(hip, knee, ankle)
	var from_hip := target - (hip + Vector3.UP * _hip.x)
	if from_hip.length() > reach:
		target = hip + Vector3.UP * _hip.x + from_hip.normalized() * reach
	leg.target.global_position = target
	var root := hip + Vector3.UP * _hip.x
	var line := (target - root).normalized()
	var thigh_x := space.basis * _skeleton.get_bone_global_pose(leg.upper).basis.x
	var axis := thigh_x * signf(_skeleton.get_bone_global_rest(leg.upper).basis.x.x)
	var out := line.cross(axis).normalized()
	leg.pole.global_position = (root + target) * 0.5 + out * POLE_OUT
	leg.ankle_basis = _skeleton.get_bone_global_pose(leg.ankle).basis


func _raise_hips(space: Transform3D) -> void:
	var pose := _skeleton.get_bone_pose_position(_hips)
	var parent := _skeleton.get_bone_parent(_hips)
	var up := space.basis.inverse() * Vector3.UP * _hip.x
	if parent >= 0:
		up = _skeleton.get_bone_global_pose(parent).basis.inverse() * up
	_skeleton.set_bone_pose_position(_hips, pose + up)


func _footprint(space: Transform3D, leg: Cycle, velocity: Vector3, floor_y: float, prefer: float) -> float:
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
	for leg in _legs:
		var hip := space * _skeleton.get_bone_global_pose(leg.upper).origin
		var knee := space * _skeleton.get_bone_global_pose(leg.lower).origin
		var ankle_local := _skeleton.get_bone_global_pose(leg.ankle).origin
		var toes_local := _skeleton.get_bone_global_pose(leg.toes).origin
		var ankle := space * ankle_local
		var target := ankle + leg.offset + Vector3.UP * leg.lift.x
		var want := hip.distance_to(ankle)
		var full := hip.distance_to(knee) + knee.distance_to(ankle)
		var reach := _reach(hip, knee, ankle)
		var at_want := target
		var at_reach := target
		var above := hip.y - target.y
		if above > 0.0:
			var fk := Vector3(ankle.x, target.y, ankle.z)
			var under_hip := Vector3(hip.x, target.y, hip.z)
			var foot_length := _skeleton.get_bone_rest(leg.toes).origin.length()
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
	return desired + STRETCH * (full - desired)


func _restore() -> void:
	for leg in _legs:
		var parent := _skeleton.get_bone_parent(leg.ankle)
		var local: Basis = _skeleton.get_bone_global_pose(parent).basis.inverse() * leg.ankle_basis
		_skeleton.set_bone_pose_rotation(leg.ankle, local.get_rotation_quaternion())


func _floor_under(point: Vector3, floor_y: float, reach := TREAD_REACH) -> float:
	var hit := _ray(point, floor_y, reach, FOOT_GROUND)
	if hit.is_empty():
		hit = _ray(point, floor_y, REACH, _skin.body.collision_mask)
	if hit.is_empty() or hit.normal.y < FLOOR_NORMAL:
		return floor_y
	return hit.position.y


func _ray(point: Vector3, floor_y: float, reach: float, mask: int) -> Dictionary:
	var from := Vector3(point.x, floor_y + reach, point.z)
	var to := Vector3(point.x, floor_y - reach, point.z)
	var query := PhysicsRayQueryParameters3D.create(from, to, mask, [_skin.body.get_rid()])
	return _skeleton.get_world_3d().direct_space_state.intersect_ray(query)


func _spring(state: Vector2, target: float, w: float, delta: float) -> Vector2:
	var x := state.x - target
	var e := exp(-w * delta)
	var k := (state.y + w * x) * delta
	return Vector2((x + k) * e + target, (state.y - w * k) * e)
