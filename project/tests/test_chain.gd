extends RefCounted

const BONE_LENGTH := 0.1 # m
const TIP := 4
const TIP_POSE := Vector3(0, -BONE_LENGTH * TIP, 0) # m
const JOINT_RADIUS := 0.02 # m

const FRAME_RATE_TOLERANCE := 0.002 # m
const TELEPORT_TOLERANCE := 0.00001 # m
const SWING_MIN := 0.005 # m
const LENGTH_TOLERANCE := 0.000001 # m
const ANGLE_TOLERANCE := 0.0001 # rad
const SCALE_TOLERANCE := 0.00002 # m
const BONE_COLLISION_TOLERANCE := 0.001 # m
const DEPTH_INERTIA_RATIO := 0.5
const LOCAL_SPEED_LIMIT_RATIO := 0.5
const SYNC_TOLERANCE := 0.0001 # m
const SQUEEZE_TOLERANCE := 0.0001 # m
const REST_OVERLAP_TOLERANCE := 0.0001 # m
const LINK_SPHERE_RADIUS := 0.03 # m
const LINK_SPACING_TOLERANCE := 0.001 # m
const LINK_SPACING_MIN := 0.03 # m
const ROOT_SPRING_LIMIT := 0.03 # m
const ROOT_SPRING_TOLERANCE := 0.001 # m
const ROOT_SPRING_FORWARD_LIMIT := 0.005 # m
const SHAPE_TOLERANCE := 0.005 # m
const WIND_MIN := 0.02 # m
const WIND_TOLERANCE := 0.0001 # m
const MAX_DISTANCE := 0.05 # m
const MAX_DISTANCE_CURVE_RATIO := 0.5
const BACKSTOP_OFFSET := Vector3(0.1, 0.0, 0.0) # m
const BACKSTOP := {"backstop/bone_name": "b0", "backstop/position_offset": BACKSTOP_OFFSET}
const BACKSTOP_RADIUS := 10.0 # m
const BACKSTOP_ROUND_RADIUS := 0.05 # m
const BACKSTOP_DISTANCE := 0.02 # m
const MOVEMENT_LIMIT_TOLERANCE := 0.00001 # m
const ANCHOR_TOLERANCE := 0.00001 # m
const TETHER_LIMIT := 0.8
const TETHER_CLOSEST := 0.5
const FRICTION_SPHERE_OFFSET := Vector3(0.5, -0.25, 0.0) # m
const FRICTION_SPHERE_RADIUS := 0.45 # m
const FRICTION_SLIDE_RATIO := 0.5
const GRAVITY_SAG_MIN := 0.1 # m
const TICKS_TOLERANCE := 0.005 # m
const TICKS_RATIO := 0.5
const MAX_STEPS_RATIO := 0.5
const STABILIZATION_RATIO := 0.5
const CULLING_TOLERANCE := 0.00001 # m
const CULLING_RESET_RATIO := 0.1
const BENDING_RATIO := 1.5
const SHOCK_RATIO := 0.2
const SHOCK_SNAP := 0.6 # rad
const TIME_MONITOR := &"bone_cloth/time"
const JOINTS_MONITOR := &"bone_cloth/joints"


#   Stage
#    +-- character (scale: 캐릭터 크기)
#         +-- armature (scale: 0.01이면 cm 리그)
#              +-- skeleton (b0 ~ b4, 0.4 m)
#                   +-- simulator
class Rig:
	var stage := Node3D.new()
	var character := Node3D.new()
	var skeleton := Skeleton3D.new()
	var simulator: Node
	var poses: Array[Transform3D] = []

	func _init(p_tree: SceneTree, p_character_scale := 1.0, p_armature_scale := 1.0, p_owner := "model", p_collision := false) -> void:
		var armature := Node3D.new()
		stage.name = "Stage"
		character.name = "Character"
		armature.name = "Armature"
		skeleton.name = "Skeleton"
		character.scale = Vector3.ONE * p_character_scale
		armature.scale = Vector3.ONE * p_armature_scale
		p_tree.root.add_child(stage)
		stage.add_child(character)
		character.add_child(armature)
		armature.add_child(skeleton)

		for i in TIP + 1:
			skeleton.add_bone("b%d" % i)
			if i > 0:
				skeleton.set_bone_parent(i, i - 1)
				skeleton.set_bone_rest(i, Transform3D(Basis(), Vector3(0, -BONE_LENGTH / p_armature_scale, 0)))
			skeleton.set_bone_pose(i, skeleton.get_bone_rest(i))
		skeleton.modifier_callback_mode_process = Skeleton3D.MODIFIER_CALLBACK_MODE_PROCESS_MANUAL

		simulator = ClassDB.instantiate("BoneClothSimulator3D")
		simulator.name = "Simulator"
		skeleton.add_child(simulator)
		simulator.set("chain_count", 1)
		simulator.set("chains/0/root_bone_name", "b0")
		simulator.set("chains/0/end_bone_name", "b%d" % TIP)
		simulator.set("chains/0/extend_end_bone", true)
		simulator.set("chains/0/end_bone/length", 0.05)
		simulator.set("stiffness/value", 0.02)
		simulator.set("radius/value", JOINT_RADIUS)
		simulator.set("gravity/value", 9.8)
		if p_collision:
			var sphere: Node = ClassDB.instantiate("BoneClothCollisionSphere3D")
			simulator.add_child(sphere)
			sphere.set("bone_name", "b1")
			sphere.set("position_offset", Vector3(0.06, -0.15, 0.0))
			sphere.set("radius", 0.06)

		if p_owner != "":
			character.owner = stage
			armature.owner = character
			skeleton.owner = character
			var simulator_owner: Node = character if p_owner == "model" else stage
			simulator.owner = simulator_owner
			for child in simulator.get_children():
				child.owner = simulator_owner

		skeleton.skeleton_updated.connect(_read_poses)

	# 모디파이어는 이 프레임 끝의 deferred update에서 돌아서, poses는 한 번 앞의 advance() 결과다.
	func advance(p_delta: float) -> void:
		skeleton.advance(p_delta)
		await skeleton.get_tree().process_frame

	func tip() -> Vector3:
		var world := skeleton.global_transform * poses[TIP].origin
		return character.global_transform.affine_inverse() * world

	func remove() -> void:
		stage.free()

	func _read_poses() -> void:
		poses.clear()
		for i in skeleton.get_bone_count():
			poses.append(skeleton.get_bone_global_pose(i))


#   r +-- a0 - a1 - ...   (p_roots: 이름 앞글자 -> r에서 떨어진 위치)
#     +-- c0 - c1 - ...
class Chains:
	var skeleton := Skeleton3D.new()
	var simulator: Node = ClassDB.instantiate("BoneClothSimulator3D")
	var origins: Array[Vector3] = []

	func _init(p_tree: SceneTree, p_roots: Dictionary, p_bone_count: int, p_link_mode: int) -> void:
		p_tree.root.add_child(skeleton)
		skeleton.add_bone("r")
		for prefix in p_roots:
			for i in p_bone_count:
				var bone := skeleton.add_bone("%s%d" % [prefix, i])
				skeleton.set_bone_parent(bone, 0 if i == 0 else bone - 1)
				skeleton.set_bone_rest(bone, Transform3D(Basis(), p_roots[prefix] if i == 0 else Vector3(0, -BONE_LENGTH, 0)))
				skeleton.set_bone_pose(bone, skeleton.get_bone_rest(bone))
		skeleton.modifier_callback_mode_process = Skeleton3D.MODIFIER_CALLBACK_MODE_PROCESS_MANUAL
		skeleton.add_child(simulator)
		simulator.set("chain_count", p_roots.size())
		for c in p_roots.size():
			simulator.set("chains/%d/root_bone_name" % c, "%s0" % p_roots.keys()[c])
			simulator.set("chains/%d/end_bone_name" % c, "%s%d" % [p_roots.keys()[c], p_bone_count - 1])
		simulator.set("link_mode", p_link_mode)
		skeleton.skeleton_updated.connect(_read_origins)
		_read_origins()

	func advance() -> void:
		skeleton.advance(1.0 / 60.0)
		await skeleton.get_tree().process_frame

	func origin(p_bone: String) -> Vector3:
		return origins[skeleton.find_bone(p_bone)]

	func remove() -> void:
		skeleton.free()

	func _read_origins() -> void:
		origins.clear()
		for i in skeleton.get_bone_count():
			origins.append(skeleton.get_bone_global_pose(i).origin)


static func run(t) -> void:
	await _frame_rate(t)
	await _teleport(t)
	await _teleport_reset(t)
	await _limit_angle(t)
	await _long_step(t)
	await _scale(t)
	await _bone_collision(t)
	await _depth_inertia(t)
	await _depth_speed_limit(t)
	await _local_speed_limit(t)
	await _local_rotation_limit(t)
	await _centrifugal(t)
	await _shock_propagation(t)
	await _shock_threshold(t)
	await _sync_bone(t)
	await _sync_bone_other_side(t)
	await _sync_bone_above(t)
	await _sync_bone_direction(t)
	await _sync_bone_falloff(t)
	await _squeeze(t)
	await _rest_overlap(t)
	await _link_split(t)
	await _link_spacing(t)
	await _link_stretch_limit(t)
	await _same_place_chains(t)
	await _loop_order(t)
	await _root_spring(t)
	await _root_spring_noise(t)
	await _root_spring_ellipse(t)
	await _wind(t)
	await _wind_direction_noise(t)
	await _wind_scale(t)
	await _max_distance(t)
	await _backstop(t)
	await _anchor(t)
	await _tether(t)
	await _collision_friction(t)
	await _gravity_falloff(t)
	await _stiffness_velocity_damping(t)
	await _ticks_per_second(t)
	await _max_steps_per_frame(t)
	await _stabilization_time(t)
	await _camera_culling(t)
	await _distance_culling(t)
	await _per_axis_limit(t)
	await _box_collision(t)
	await _link_bending(t)
	await _monitors(t)
	_bone_lookup(t)
	await _plane_and_inside(t)
	_preset_round_trip(t)
	await _teleport_on_anchor(t)
	await _teleport_off(t)
	await _sync_fade(t)
	await _zero_length_bone(t)
	await _rate_above_one(t)
	await _rest_change(t)
	await _same_place_collision_list(t)
	await _camera_culling_root_motion(t)
	await _collision_scale(t)
	await _short_capsule(t)
	_negative_radius(t)


static func _teleport_on_anchor(t) -> void:
	var rig := Rig.new(t)
	_set_all(rig.simulator, {"gravity/value": 0.0, "teleport_mode": 1, "anchor": rig.simulator.get_path_to(rig.stage)})
	for frame in 20:
		rig.stage.position.x += 0.2
		await rig.advance(1.0 / 60.0)
	rig.character.position += Vector3(0, 0, 2)
	var worst := 0.0
	for frame in 10:
		rig.stage.position.x += 0.2
		await rig.advance(1.0 / 60.0)
		worst = maxf(worst, rig.tip().distance_to(TIP_POSE))
	rig.remove()
	t.check(worst < ANCHOR_TOLERANCE, "Teleport Reset on a moving anchor: after the teleport the tip is up to %.6f m off its pose." % worst)


static func _teleport_off(t) -> void:
	var rig := Rig.new(t)
	_set_all(rig.simulator, {"gravity/value": 0.0, "teleport_distance": 0.0, "teleport_angle": 0.0})
	var swing := 0.0
	for frame in 20:
		_move(rig, float(frame + 1) / 60.0)
		await rig.advance(1.0 / 60.0)
		swing = maxf(swing, _off_pose(rig, TIP))
	rig.remove()
	t.check(swing > SWING_MIN, "Teleport distance and angle 0.0: the tip swung only %.6f m, so every move counted as a teleport." % swing)


static func _sync_fade(t) -> void:
	var tips := {}
	for c in ["animation", "synced", "fading"]:
		var rig := _sync_rig(t, Quaternion(Vector3.RIGHT, 0.5))
		rig.simulator.active = c != "animation"
		rig.simulator.set("stabilization_time", 10.0 if c == "fading" else 0.0)
		for frame in 2:
			await rig.advance(1.0 / 60.0)
		tips[c] = rig.tip()
		rig.remove()
	var synced: float = tips["synced"].distance_to(tips["animation"])
	var fading: float = tips["fading"].distance_to(tips["animation"])
	t.check(synced > SWING_MIN and fading < synced * CULLING_RESET_RATIO, "Sync and stabilization time: at the start the tip is %.4f m from the animation, %.4f m when fully synced." % [fading, synced])


static func _zero_length_bone(t) -> void:
	var rig := Rig.new(t)
	rig.skeleton.set_bone_rest(2, Transform3D())
	rig.skeleton.set_bone_pose(2, Transform3D())
	rig.simulator.set("gravity/direction", Vector3(1, 0, 0))
	for frame in 60:
		await rig.advance(1.0 / 60.0)
	var miss := 0.0
	for i in range(2, TIP):
		miss = maxf(miss, (rig.poses[i] * Vector3(0, -BONE_LENGTH, 0)).distance_to(rig.poses[i + 1].origin))
	rig.remove()
	t.check(miss < SYNC_TOLERANCE, "Zero-length bone: a bone below it points %.5f m past its child." % miss)


static func _rate_above_one(t) -> void:
	var rig := Rig.new(t)
	_set_all(rig.simulator, {"ticks_per_second": 90, "max_distance/value": MAX_DISTANCE, "movement_limit_stiffness": 1.5})
	for frame in 30:
		_move(rig, float(frame + 1) / 60.0)
		await rig.advance(1.0 / 60.0)
	var tip := rig.tip()
	rig.remove()
	t.check(tip.is_finite(), "Movement limit stiffness 1.5 at 90 ticks: the tip is at %s." % tip)


static func _rest_change(t) -> void:
	var rig := Rig.new(t)
	for frame in 5:
		await rig.advance(1.0 / 60.0)
	rig.skeleton.set_bone_rest(TIP, Transform3D(Basis(), Vector3(BONE_LENGTH, 0, 0)))
	rig.skeleton.set_bone_pose(TIP, rig.skeleton.get_bone_rest(TIP))
	for frame in 120:
		await rig.advance(1.0 / 60.0)
	var droop: float = -(rig.poses[TIP].basis * Vector3.RIGHT).normalized().y
	rig.remove()
	t.check(droop > GRAVITY_SAG_MIN, "Rest changed at run time: the end bone now points sideways, and its extended tip droops only %.4f under gravity." % droop)


static func _same_place_collision_list(t) -> void:
	var chains := Chains.new(t, {"a": Vector3(-0.05, 0, 0), "b": Vector3(-0.05, 0, 0)}, 3, 0)
	_set_all(chains.simulator, {"gravity/value": 0.0, "radius/value": 0.01})
	_add_collision(chains.simulator, {"position": Vector3(-0.05, -0.15, 0.01), "radius": LINK_SPHERE_RADIUS})
	for frame in 5:
		await chains.advance()
	chains.simulator.set("chains/1/enable_all_child_collisions", false)
	var apart := 0.0
	for frame in 60:
		await chains.advance()
		apart = maxf(apart, chains.origin("a2").distance_to(chains.origin("b2")))
	chains.remove()
	t.check(apart > SWING_MIN, "Same place chains: after one chain's collision list changed, the two move only %.4f m apart." % apart)


static func _camera_culling_root_motion(t) -> void:
	var rig := Rig.new(t)
	_add_camera(rig, Vector3(3.0, -0.2, 2.0))
	rig.simulator.set("camera_culling", 2)
	for frame in 10:
		await rig.advance(1.0 / 60.0)
	rig.skeleton.set_bone_pose_position(0, Vector3(3.0, 0, 0))
	for frame in 5:
		await rig.advance(1.0 / 60.0)
	Performance.get_custom_monitor(JOINTS_MONITOR)
	for frame in 5:
		await rig.advance(1.0 / 60.0)
	var joints: int = Performance.get_custom_monitor(JOINTS_MONITOR)
	rig.remove()
	t.check(joints > 0, "Camera culling: after the animation moved the chain into view, the monitor still shows %d joints." % joints)


static func _collision_scale(t) -> void:
	var center := Vector3(0.05, -0.15, 0.0)
	var rig := Rig.new(t)
	rig.simulator.set("radius/rest_overlap", false)
	_add_collision(rig.simulator, {"position": center, "radius": 0.03, "scale": Vector3.ONE * 2.0})
	_add_collision(rig.simulator, {"position": Vector3(-0.07, -0.35, 0.0), "size": Vector3.ONE * 0.06, "scale": Vector3(2.0, 1.0, 1.0)}, "BoneClothCollisionBox3D")
	for frame in 120:
		await rig.advance(1.0 / 60.0)
	var depth := _sphere_depth(rig, center, 0.06)
	var box_depth := -INF
	for i in range(1, TIP + 1):
		for k in 21:
			var over := (rig.poses[i - 1].origin.lerp(rig.poses[i].origin, k / 20.0) - Vector3(-0.07, -0.35, 0.0)).abs() - Vector3(0.06, 0.03, 0.03) - Vector3.ONE * JOINT_RADIUS
			box_depth = maxf(box_depth, -maxf(over.x, maxf(over.y, over.z)))
	rig.remove()
	t.check(depth < SHAPE_TOLERANCE, "Collision scale 2: the chain is %.4f m inside the scaled sphere." % depth)
	t.check(box_depth < SHAPE_TOLERANCE, "Collision scale (2, 1, 1): a joint is %.4f m inside the scaled box." % box_depth)


static func _short_capsule(t) -> void:
	var rig := Rig.new(t)
	_set_all(rig.simulator, {"gravity/value": 0.0, "stiffness/value": 0.5, "radius/rest_overlap": false})
	var head := Vector3(-0.01, -0.35, 0.005)
	var tail := Vector3(0.01, -0.35, 0.005)
	_add_collision(rig.simulator, {"transform": Transform3D(Basis(Vector3.BACK, PI * 0.5), Vector3(0, -0.35, 0.005)), "radius": 0.02, "height": 0.06}, "BoneClothCollisionCapsule3D")
	for frame in 30:
		await rig.advance(1.0 / 60.0)
	var nearest := INF
	for k in 21:
		var point := rig.poses[TIP - 1].origin.lerp(rig.poses[TIP].origin, k / 20.0)
		nearest = minf(nearest, point.distance_to(Geometry3D.get_closest_point_to_segment(point, head, tail)))
	rig.remove()
	t.check(nearest > 0.02, "Short capsule across a bone: the bone passes %.4f m from its axis, inside the 0.02 m radius." % nearest)


static func _negative_radius(t) -> void:
	t.expected_errors += 3
	for shape in ["BoneClothCollisionSphere3D", "BoneClothCollisionCapsule3D", "BoneClothCollisionTaperedCapsule3D"]:
		var collision: Node = ClassDB.instantiate(shape)
		var key := "top_radius" if shape == "BoneClothCollisionTaperedCapsule3D" else "radius"
		var before: float = collision.get(key)
		collision.set(key, -0.1)
		t.check(collision.get(key) == before, "%s: a negative %s was taken as %s." % [shape, key, collision.get(key)])
		collision.free()


static func _move(p_rig: Rig, p_time: float) -> void:
	var size := p_rig.character.scale.x
	p_rig.character.position = Vector3(sin(p_time * 6.0) * 0.6 * size, 0, 0)
	p_rig.character.rotation = Vector3(0, sin(p_time * 4.0) * 0.8, 0)


static func _set_all(p_node: Node, p_values: Dictionary) -> void:
	for key in p_values:
		p_node.set(key, p_values[key])


static func _add_collision(p_simulator: Node, p_values: Dictionary, p_class := "BoneClothCollisionSphere3D") -> void:
	var collision: Node = ClassDB.instantiate(p_class)
	p_simulator.add_child(collision)
	_set_all(collision, p_values)


static func _add_wind(p_rig: Rig, p_values := {}) -> void:
	var wind: Node3D = ClassDB.instantiate("BoneClothWind3D")
	wind.rotation_degrees = Vector3(0, 90, 0)
	_set_all(wind, p_values)
	p_rig.stage.add_child(wind)


static func _flat_curve(p_value: float) -> Curve:
	var curve := Curve.new()
	curve.add_point(Vector2(0.0, p_value))
	curve.add_point(Vector2(1.0, p_value))
	return curve


# 시뮬레이터가 루트 뼈도 돌리므로, p_turned가 아니면 루트의 위치만 따른다.
static func _off_pose(p_rig: Rig, p_joint: int, p_turned := false) -> float:
	var pose := Vector3(0, -BONE_LENGTH * p_joint, 0)
	return p_rig.poses[p_joint].origin.distance_to(p_rig.poses[0] * pose if p_turned else p_rig.poses[0].origin + pose)


static func _sphere_depth(p_rig: Rig, p_center: Vector3, p_radius: float) -> float:
	var deepest := -INF
	for i in range(1, TIP + 1):
		var nearest := Geometry3D.get_closest_point_to_segment(p_center, p_rig.poses[i - 1].origin, p_rig.poses[i].origin)
		deepest = maxf(deepest, p_radius + JOINT_RADIUS - p_center.distance_to(nearest))
	return deepest


static func _frame_rate(t) -> void:
	for wind in [false, true]:
		var tips := {}
		for fps in [30, 60, 120]:
			var rig := Rig.new(t)
			if wind:
				rig.simulator.set("gravity/value", 0.0)
				_add_wind(rig)
			else:
				rig.simulator.set("gravity/direction", Vector3(-1, 0, 0))
			var list: Array[Vector3] = []
			for frame in 2 * fps:
				await rig.advance(1.0 / fps)
				if (frame + 1) % (fps / 30) == 0:
					list.append(rig.tip())
			tips[fps] = list
			rig.remove()

		for fps in [30, 120]:
			var largest := 0.0
			for i in tips[60].size():
				largest = maxf(largest, tips[fps][i].distance_to(tips[60][i]))
			t.check(largest < FRAME_RATE_TOLERANCE, "%s %d fps: the tip is up to %.4f m from 60 fps." % ["Wind," if wind else "Frame rate", fps, largest])


static func _teleport(t) -> void:
	var cases := [
		["move 1 m", Vector3(1, 0, 0), 0.0, deg_to_rad(90.0), true],
		["turn 100 deg", Vector3(), deg_to_rad(100.0), deg_to_rad(90.0), true],
		["move 0.3 m", Vector3(0.3, 0, 0), 0.0, deg_to_rad(90.0), false],
		["turn 60 deg", Vector3(), deg_to_rad(60.0), deg_to_rad(90.0), false],
		["turn 175 deg, limit 170 deg", Vector3(), deg_to_rad(175.0), deg_to_rad(170.0), true],
		["turn 150 deg, limit 170 deg", Vector3(), deg_to_rad(150.0), deg_to_rad(170.0), false],
	]
	for c in cases:
		var rig := Rig.new(t)
		rig.simulator.set("gravity/value", 0.0)
		rig.simulator.teleport_angle = c[3]
		for frame in 10:
			await rig.advance(1.0 / 60.0)
		var before := rig.tip()
		rig.character.position += c[1]
		rig.character.rotate_z(c[2])
		var largest := 0.0
		for frame in 30:
			await rig.advance(1.0 / 60.0)
			largest = maxf(largest, rig.tip().distance_to(before))
		rig.remove()

		if c[4]:
			t.check(largest < TELEPORT_TOLERANCE, "Teleport, %s: the tip moved %.6f m." % [c[0], largest])
		else:
			t.check(largest > SWING_MIN, "No teleport, %s: the tip swung only %.6f m." % [c[0], largest])


static func _limit_angle(t) -> void:
	var limit := deg_to_rad(30.0)
	var widest := {}
	var longest := {}
	for stiffness in [1.0, 0.0, 0.3]:
		var rig := Rig.new(t)
		_set_all(rig.simulator, {"limit_angle/value": limit, "limit_angle/stiffness": stiffness})
		widest[stiffness] = 0.0
		longest[stiffness] = 0.0
		for frame in 180:
			_move(rig, float(frame + 1) / 60.0)
			await rig.advance(1.0 / 60.0)
			for i in range(1, TIP + 1):
				var bone := rig.poses[i].origin - rig.poses[i - 1].origin
				longest[stiffness] = maxf(longest[stiffness], absf(bone.length() - BONE_LENGTH))
				widest[stiffness] = maxf(widest[stiffness], bone.angle_to(Vector3.DOWN))
		rig.remove()

	t.check(longest[1.0] < LENGTH_TOLERANCE, "Lengths: a bone differs from its pose by %.7f m." % longest[1.0])
	t.check(widest[1.0] < limit + ANGLE_TOLERANCE, "Limit angle 30 deg: a bone leaned %.4f deg." % rad_to_deg(widest[1.0]))
	t.check(widest[1.0] > limit - deg_to_rad(5.0), "Limit angle 30 deg: the bones leaned only %.4f deg, so the limit was not tested." % rad_to_deg(widest[1.0]))
	var soft_longest := maxf(longest[0.0], longest[0.3])
	t.check(soft_longest < LENGTH_TOLERANCE, "Soft limit: a bone differs from its pose by %.7f m." % soft_longest)
	t.check(widest[0.3] > limit + deg_to_rad(5.0), "Soft limit 0.3: the bones leaned only %.4f deg, as with a hard limit of 30 deg." % rad_to_deg(widest[0.3]))
	t.check(widest[0.3] < widest[0.0] - deg_to_rad(5.0), "Soft limit 0.3: the bones leaned %.4f deg, as without a limit (%.4f deg)." % [rad_to_deg(widest[0.3]), rad_to_deg(widest[0.0])])


static func _long_step(t) -> void:
	var rig := Rig.new(t)
	for frame in 30:
		_move(rig, float(frame + 1) / 60.0)
		await rig.advance(1.0 / 60.0)
	await rig.advance(5.0)
	var finite := true
	for pose in rig.poses:
		finite = finite and pose.is_finite()
	var tip_distance := rig.tip().length()
	rig.remove()

	t.check(finite, "A 5 s step: a bone pose is not finite.")
	t.check(tip_distance < 1.0, "A 5 s step: the tip is %.4f m from the character." % tip_distance)


static func _scale(t) -> void:
	var cases := [
		["scale 1", 1.0, 1.0, "model", false],
		["cm rig", 1.0, 0.01, "model", false],
		["giant", 2.0, 1.0, "model", false],
		["cm giant", 2.0, 0.01, "model", false],
		["cm rig, added in the stage", 1.0, 0.01, "stage", false],
		["giant, added in the stage", 2.0, 1.0, "stage", false],
		["cm giant, added in the stage", 2.0, 0.01, "stage", false],
		["cm rig, no owners", 1.0, 0.01, "", false],
		["scale 1 with a collision", 1.0, 1.0, "model", true],
		["cm rig with a collision", 1.0, 0.01, "model", true],
		["giant with a collision, added in the stage", 2.0, 1.0, "stage", true],
		["cm giant with a collision", 2.0, 0.01, "model", true],
	]
	var baselines := {}
	for c in cases:
		var rig := Rig.new(t, c[1], c[2], c[3], c[4])
		var tips: Array[Vector3] = []
		for frame in 180:
			_move(rig, float(frame + 1) / 60.0)
			await rig.advance(1.0 / 60.0)
			tips.append(rig.tip())
		rig.remove()

		if not baselines.has(c[4]):
			baselines[c[4]] = tips
			continue
		var largest := 0.0
		for frame in tips.size():
			largest = maxf(largest, tips[frame].distance_to(baselines[c[4]][frame]))
		t.check(largest < SCALE_TOLERANCE, "Scale, %s: the tip is up to %.6f m from scale 1." % [c[0], largest])


static func _bone_collision(t) -> void:
	var offset := Vector3(0.05, -0.15, 0.0)
	var rig := Rig.new(t)
	rig.simulator.set("gravity/value", 0.0)
	_add_collision(rig.simulator, {"position": offset, "radius": 0.04})
	for frame in 60:
		await rig.advance(1.0 / 60.0)
	var depth := _sphere_depth(rig, rig.poses[0].origin + offset, 0.04)
	rig.remove()

	t.check(depth < BONE_COLLISION_TOLERANCE, "Bone collision: a bone is %.4f m into the sphere." % depth)


static func _shaken_offset(t, p_values: Dictionary, p_turn: bool, p_joint: int) -> float:
	var rig := Rig.new(t)
	_set_all(rig.simulator, p_values)
	var sum := 0.0
	for frame in 120:
		var wave := sin(float(frame + 1) / 60.0 * 12.0)
		if p_turn:
			rig.skeleton.set_bone_pose_rotation(0, Quaternion(Vector3.BACK, wave * 0.3))
		else:
			rig.skeleton.set_bone_pose_position(0, Vector3(wave * 0.05, 0, 0))
		await rig.advance(1.0 / 60.0)
		sum += _off_pose(rig, p_joint, p_turn)
	rig.remove()
	return sum / 120.0


static func _depth_inertia(t) -> void:
	var offsets := {}
	for value in [0.0, 1.0]:
		offsets[value] = await _shaken_offset(t, {"depth_inertia": value}, false, 1)

	t.check(offsets[1.0] < offsets[0.0] * DEPTH_INERTIA_RATIO, "Depth inertia 1: the joint next to the root is %.4f m off its place, against %.4f m at 0." % [offsets[1.0], offsets[0.0]])


static func _depth_speed_limit(t) -> void:
	var trails := {}
	for value in [0.0, 1.0]:
		var rig := Rig.new(t)
		_set_all(rig.simulator, {"depth_inertia": value, "joint_speed_limit": 1.0})
		var sum := 0.0
		for frame in 120:
			rig.character.position = Vector3(float(frame + 1) / 60.0 * 3.0, 0, 0)
			await rig.advance(1.0 / 60.0)
			if frame >= 60:
				sum += (rig.poses[0].origin + Vector3(0, -BONE_LENGTH, 0) - rig.poses[1].origin).x
		trails[value] = sum / 60.0
		rig.remove()

	t.check(trails[1.0] < trails[0.0] * DEPTH_INERTIA_RATIO, "Depth inertia 1 under joint_speed_limit: the joint next to the root trails %.4f m, against %.4f m at 0." % [trails[1.0], trails[0.0]])


static func _local_speed_limit(t) -> void:
	var offsets := {}
	for limit in [0.0, 0.1]:
		offsets[limit] = await _shaken_offset(t, {"gravity/value": 0.0, "local_movement_speed_limit": limit}, false, TIP)

	t.check(offsets[0.1] < offsets[0.0] * LOCAL_SPEED_LIMIT_RATIO, "Local movement speed limit 0.1 m/s: the tip is %.4f m off its place, against %.4f m without the limit." % [offsets[0.1], offsets[0.0]])


static func _local_rotation_limit(t) -> void:
	var offsets := {}
	for limit in [0.0, 0.5]:
		offsets[limit] = await _shaken_offset(t, {"gravity/value": 0.0, "local_rotation_speed_limit": limit}, true, TIP)

	t.check(offsets[0.5] < offsets[0.0] * LOCAL_SPEED_LIMIT_RATIO, "Local rotation speed limit 0.5 rad/s: the tip is %.4f m off its place, against %.4f m without the limit." % [offsets[0.5], offsets[0.0]])


static func _shock_propagation(t) -> void:
	var offsets := {}
	for value in [0.0, 1.0]:
		offsets[value] = await _shaken_offset(t, {"gravity/value": 0.0, "shock_propagation": value}, true, TIP)

	t.check(offsets[1.0] < offsets[0.0] * SHOCK_RATIO, "Shock propagation 1: the tip is %.5f m off its place, against %.5f m without it." % [offsets[1.0], offsets[0.0]])


static func _root_spring_ellipse(t) -> void:
	var rig := Rig.new(t)
	_set_all(rig.simulator, {"root_spring_stiffness": 0.15, "root_spring_limit": ROOT_SPRING_LIMIT, "root_spring_forward_limit": ROOT_SPRING_FORWARD_LIMIT})
	var over := -INF
	var up := 0.0
	var tested := false
	for frame in 240:
		var time := float(frame + 1) / 60.0
		rig.character.position = Vector3(sin(time * 9.0) * 0.3, sin(time * 12.0) * 0.1, 0)
		await rig.advance(1.0 / 60.0)
		if rig.poses.is_empty():
			continue
		var offset := rig.poses[0].origin
		var side := minf(Vector2(offset.x, offset.z).length() / ROOT_SPRING_LIMIT, 1.0)
		var along := -offset.y
		over = maxf(over, along - ROOT_SPRING_FORWARD_LIMIT * sqrt(1.0 - side * side))
		up = maxf(up, offset.y)
		tested = tested or (side > 0.5 and along > 0.0)
	rig.remove()

	t.check(over < LENGTH_TOLERANCE, "Root spring forward limit: the root moves %.5f m past the ellipse along its bone." % over)
	t.check(tested, "Root spring forward limit: the root never moved forward while far to the side, so the ellipse was not tested.")
	t.check(up > ROOT_SPRING_FORWARD_LIMIT * 2.0, "Root spring forward limit: the root moves only %.4f m against its bone." % up)


static func _root_spring_noise(t) -> void:
	var paths := {}
	var largest := 0.0
	for noise in [0.0, 1.0]:
		var rig := Rig.new(t)
		_set_all(rig.simulator, {"root_spring_stiffness": 0.15, "root_spring_limit": ROOT_SPRING_LIMIT, "root_spring_noise": noise})
		paths[noise] = []
		for frame in 120:
			_move(rig, float(frame + 1) / 60.0)
			await rig.advance(1.0 / 60.0)
			paths[noise].append(rig.poses[0].origin)
			largest = maxf(largest, rig.poses[0].origin.length())
		rig.remove()

	var apart := 0.0
	for i in paths[0.0].size():
		apart = maxf(apart, paths[0.0][i].distance_to(paths[1.0][i]))
	t.check(apart > ROOT_SPRING_TOLERANCE, "Root spring noise 1: the root's path is at most %.5f m from the path without noise." % apart)
	t.check(largest < ROOT_SPRING_LIMIT + LENGTH_TOLERANCE, "Root spring noise: the root moves up to %.4f m (limit %.2f m)." % [largest, ROOT_SPRING_LIMIT])


static func _centrifugal(t) -> void:
	var reach := {}
	for acceleration in [0.0, 0.5]:
		var rig := Rig.new(t)
		rig.simulator.set("centrifugal_acceleration", acceleration)
		rig.skeleton.set_bone_pose_rotation(0, Quaternion(Vector3.BACK, 0.5))
		var sum := 0.0
		for frame in 120:
			rig.character.rotation = Vector3(0, float(frame + 1) / 60.0 * 9.0, 0)
			await rig.advance(1.0 / 60.0)
			if frame >= 60:
				var tip := rig.poses[TIP].origin
				sum += Vector2(tip.x, tip.z).length()
		reach[acceleration] = sum / 60.0
		rig.remove()

	t.check(reach[0.5] > reach[0.0] + SWING_MIN, "Centrifugal acceleration 0.5: the tip turns %.4f m from the axis, as without it (%.4f m)." % [reach[0.5], reach[0.0]])


static func _teleport_reset(t) -> void:
	var offsets := {}
	for mode in [0, 1]:
		var rig := Rig.new(t)
		_set_all(rig.simulator, {"gravity/value": 0.0, "teleport_mode": mode})
		for frame in 20:
			_move(rig, float(frame + 1) / 60.0)
			await rig.advance(1.0 / 60.0)
		rig.character.position += Vector3(2, 0, 0)
		for frame in 2:
			await rig.advance(1.0 / 60.0)
		offsets[mode] = rig.tip().distance_to(TIP_POSE)
		rig.remove()

	t.check(offsets[1] < TELEPORT_TOLERANCE, "Teleport mode Reset: on the teleport frame the tip is %.6f m off its pose." % offsets[1])
	t.check(offsets[0] > SWING_MIN, "Teleport mode Keep: on the teleport frame the tip is only %.6f m off its pose, so the swing was not kept." % offsets[0])


static func _shock_threshold(t) -> void:
	var offsets := {}
	for threshold in [0.0, 1000.0, 5.0]:
		var rig := Rig.new(t)
		_set_all(rig.simulator, {"gravity/value": 0.0, "shock_threshold": threshold})
		for frame in 10:
			await rig.advance(1.0 / 60.0)
		rig.skeleton.set_bone_pose_rotation(0, Quaternion(Vector3.BACK, SHOCK_SNAP))
		for frame in 2:
			await rig.advance(1.0 / 60.0)
		offsets[threshold] = _off_pose(rig, TIP, true)
		rig.remove()

	t.check(offsets[5.0] < offsets[0.0] * SHOCK_RATIO, "Shock threshold 5 m/s^2: after a %.1f rad snap the tip is %.5f m off its place, against %.5f m without it." % [SHOCK_SNAP, offsets[5.0], offsets[0.0]])
	t.check(is_equal_approx(offsets[1000.0], offsets[0.0]), "Shock threshold 1000 m/s^2: the snap passed it, the tip is %.5f m off against %.5f m." % [offsets[1000.0], offsets[0.0]])


static func _sync_rig(t, p_turn: Quaternion, p_rest := Transform3D(Basis(Vector3.BACK, PI), Vector3(0, 0, 0.05))) -> Rig:
	var rig := Rig.new(t)
	rig.skeleton.set_bone_rest(1, Transform3D(Basis(), Vector3(0, -BONE_LENGTH, 0.05)))
	rig.skeleton.set_bone_pose(1, rig.skeleton.get_bone_rest(1))
	var thigh := rig.skeleton.add_bone("thigh")
	rig.skeleton.set_bone_parent(thigh, 0)
	rig.skeleton.set_bone_rest(thigh, p_rest)
	rig.skeleton.set_bone_pose(thigh, p_rest)
	rig.skeleton.set_bone_pose_rotation(thigh, p_turn * p_rest.basis.get_rotation_quaternion())
	_set_all(rig.simulator, {"gravity/value": 0.0, "stiffness/value": 1.0, "sync/bone_count": 1, "sync/bones/0/bone_name": "thigh"})
	return rig


static func _hung(p_rest: Vector3, p_origin: Vector3, p_swing: Quaternion, p_share: float, p_value: float) -> Vector3:
	return p_origin + Quaternion().slerp(p_swing, p_share) * -p_origin + Quaternion().slerp(p_swing, p_share * p_value) * p_rest


static func _sync_bone(t) -> void:
	var front := Vector3(0, 0, 0.05)
	var cases := {
		"rest": [front, Quaternion(), 1.0, 1.0],
		"toward": [front, Quaternion(Vector3.RIGHT, 0.5), 1.0, 1.0],
		"away": [front, Quaternion(Vector3.RIGHT, -0.5), 1.0, 0.0],
		"half": [front, Quaternion(Vector3.RIGHT, 0.5), 0.5, 1.0],
		"zero": [front, Quaternion(Vector3.RIGHT, 0.5), 0.0, 1.0],
		"diagonal": [Vector3(0.05, 0, 0.05), Quaternion(Vector3.RIGHT, 0.5), 1.0, 0.5 + 0.5 * sqrt(0.5)],
		"twist": [front, Quaternion(Vector3.UP, 0.5), 1.0, 0.0],
	}
	var tips := {}
	var tilt := 0.0
	for case in cases:
		var rig := _sync_rig(t, cases[case][1], Transform3D(Basis(Vector3.BACK, PI), cases[case][0]))
		rig.simulator.set("sync/value", cases[case][2])
		for frame in 30:
			await rig.advance(1.0 / 60.0)
		tips[case] = rig.tip()
		if case == "toward":
			tilt = (rig.poses[0].basis * Vector3(0, -BONE_LENGTH, 0.05)).angle_to(rig.poses[1].origin - rig.poses[0].origin)
		rig.remove()

	for case in cases:
		var expected := _hung(tips["rest"], cases[case][0], cases[case][1], cases[case][3], cases[case][2])
		t.check(tips[case].distance_to(expected) < SYNC_TOLERANCE, "Sync bone, %s: the chain end is %.5f m off the chain hung from the bone by %.2f of its swing." % [case, tips[case].distance_to(expected), cases[case][3]])
	t.check(tips["toward"].distance_to(tips["rest"]) > SWING_MIN, "Sync bone, toward: the chain end moves %.4f m." % tips["toward"].distance_to(tips["rest"]))
	t.check(tilt < ANGLE_TOLERANCE, "Sync bone, toward: the root bone's rotation points %.4f rad off the moved joint." % tilt)


static func _sync_bone_other_side(t) -> void:
	var rest := Vector3(0, -BONE_LENGTH, 0.05) + Vector3(0, -BONE_LENGTH * (TIP - 1), 0)
	var origin := Vector3(0.1, 0, 0.05)
	var swing := Quaternion(Vector3.RIGHT, 0.5)
	for other in [false, true]:
		var rig := _sync_rig(t, swing, Transform3D(Basis(Vector3.BACK, PI), origin))
		if other:
			var thigh := rig.skeleton.add_bone("other_thigh")
			rig.skeleton.set_bone_parent(thigh, 0)
			rig.skeleton.set_bone_rest(thigh, Transform3D(Basis(Vector3.BACK, PI), Vector3(0, 0, 0.05)))
			rig.skeleton.set_bone_pose(thigh, rig.skeleton.get_bone_rest(thigh))
			_set_all(rig.simulator, {"sync/bone_count": 2, "sync/bones/1/bone_name": "other_thigh"})
		for frame in 30:
			await rig.advance(1.0 / 60.0)
		var share := 0.0 if other else 0.5 + 0.5 * (-origin).normalized().dot(Vector3.FORWARD)
		var expected := _hung(rest, origin, swing, share, 1.0)
		t.check(rig.tip().distance_to(expected) < SYNC_TOLERANCE, "Sync bone, %s: the chain end is %.5f m off the chain hung from the bone by %.2f of its swing." % ["beside the other bone" if other else "alone", rig.tip().distance_to(expected), share])
		rig.remove()


static func _sync_bone_above(t) -> void:
	var swing := Quaternion(Vector3.RIGHT, 0.5)
	for value in [1.0, 0.5, 0.0]:
		var rig := Rig.new(t)
		rig.simulator.set("chains/0/root_bone_name", "b1")
		rig.skeleton.set_bone_pose_rotation(0, swing)
		_set_all(rig.simulator, {"gravity/value": 0.0, "stiffness/value": 1.0, "sync/bone_count": 1, "sync/bones/0/bone_name": "b0", "sync/value": value})
		for frame in 30:
			await rig.advance(1.0 / 60.0)
		var root := swing * Vector3(0, -BONE_LENGTH, 0)
		var expected := root + Quaternion().slerp(swing, value) * Vector3(0, -BONE_LENGTH * (TIP - 1), 0)
		t.check(rig.tip().distance_to(expected) < SYNC_TOLERANCE, "Sync bone above the chain, value %.1f: the chain end is %.5f m off the chain that follows that share of the bone's swing." % [value, rig.tip().distance_to(expected)])
		rig.remove()


static func _sync_bone_direction(t) -> void:
	var rest := Vector3(0, -BONE_LENGTH, 0.05) + Vector3(0, -BONE_LENGTH * (TIP - 1), 0)
	var cases := {
		"positive z, tip to -z": [1, 0.5, 0.05, 0.0],
		"positive z, tip to +z": [1, -0.5, -0.05, 1.0],
		"negative z, tip to -z": [2, 0.5, 0.05, 1.0],
		"none z": [3, -0.5, -0.05, 0.0],
	}
	for c in cases:
		var origin := Vector3(0, 0, cases[c][2])
		var rig := _sync_rig(t, Quaternion(Vector3.RIGHT, cases[c][1]), Transform3D(Basis(Vector3.BACK, PI), origin))
		rig.simulator.set("sync/bones/0/direction_z", cases[c][0])
		for frame in 30:
			await rig.advance(1.0 / 60.0)
		var expected := _hung(rest, origin, Quaternion(Vector3.RIGHT, cases[c][1]), cases[c][3], 1.0)
		t.check(rig.tip().distance_to(expected) < SYNC_TOLERANCE, "Sync bone direction, %s: the chain end is %.5f m off the chain hung from the bone." % [c, rig.tip().distance_to(expected)])
		rig.remove()

	var simulator: Node = ClassDB.instantiate("BoneClothSimulator3D")
	_set_all(simulator, {"sync/bone_count": 1, "sync/bones/0/direction_x": 2, "sync/bones/0/direction_y": 3})
	var directions := ["x", "y", "z"].map(func(axis: String) -> int: return simulator.get("sync/bones/0/direction_" + axis))
	t.check(directions == [2, 3, 0], "Sync bone direction: x Negative, y None and z Both read back as %s." % [directions])
	simulator.free()


static func _sync_bone_falloff(t) -> void:
	var swing := Quaternion(Vector3.RIGHT, 0.5)
	for amount in [1.0, 0.5]:
		var rig := _sync_rig(t, swing)
		_set_all(rig.simulator, {"sync/bones/0/falloff_inner": 0.15, "sync/bones/0/falloff_outer": 0.25, "sync/bones/0/falloff_amount": amount})
		for frame in 30:
			await rig.advance(1.0 / 60.0)
		var bone := Vector3(0, -BONE_LENGTH, 0)
		var origin := Vector3(0, 0, 0.05)
		var expected := origin - swing * origin + swing * (Vector3(0, -BONE_LENGTH, 0.05) + bone) + Quaternion().slerp(swing, 1.0 - 0.5 * amount) * bone + Quaternion().slerp(swing, 1.0 - amount) * bone
		t.check(rig.tip().distance_to(expected) < SYNC_TOLERANCE, "Sync bone falloff, amount %.1f: the chain end is %.5f m off the chain bent by distance." % [amount, rig.tip().distance_to(expected)])
		var miss := 0.0
		for i in range(1, TIP):
			miss = maxf(miss, (rig.poses[i] * Vector3(0, -BONE_LENGTH, 0)).distance_to(rig.poses[i + 1].origin))
		t.check(miss < SYNC_TOLERANCE, "Sync bone falloff, amount %.1f: a bone points %.5f m past its child." % [amount, miss])
		var length := 0.0
		for i in range(1, TIP + 1):
			length += rig.poses[i].origin.distance_to(rig.poses[i - 1].origin)
		var rest_length := Vector3(0, -BONE_LENGTH, 0.05).length() + BONE_LENGTH * (TIP - 1)
		t.check(absf(length - rest_length) < LENGTH_TOLERANCE * 10.0, "Sync bone falloff, amount %.1f: the chain is %.6f m long, at rest %.6f m." % [amount, length, rest_length])
		rig.remove()


static func _squeeze(t) -> void:
	var rig := Rig.new(t)
	rig.simulator.set("gravity/value", 0.0)
	for side in [1.0, -1.0]:
		_add_collision(rig.simulator, {"position": Vector3(0.05 * side, -0.15, 0.0), "radius": 0.04})
	var largest := 0.0
	for frame in 60:
		await rig.advance(1.0 / 60.0)
		for i in range(1, TIP + 1):
			largest = maxf(largest, absf(rig.poses[i].origin.x))
	rig.remove()

	t.check(largest < SQUEEZE_TOLERANCE, "Squeeze: a chain between two spheres pushing opposite ways moves %.5f m sideways." % largest)


static func _rest_overlap(t) -> void:
	var center := Vector3(0.05, -0.15, 0.0)
	var depths := {}
	var largest := 0.0
	for case in ["rest", "bent", "off"]:
		var rig := Rig.new(t)
		_set_all(rig.simulator, {"gravity/value": 0.0, "stiffness/value": 0.2})
		if case == "off":
			rig.simulator.set("radius/rest_overlap", false)
		_add_collision(rig.simulator, {"bone_name": "b0", "position_offset": center, "radius": 0.04})
		if case == "bent":
			rig.skeleton.set_bone_pose_rotation(1, Quaternion(Vector3(0, 0, 1), 0.3))
		for frame in 60:
			await rig.advance(1.0 / 60.0)
			if case == "rest":
				for i in range(1, TIP + 1):
					largest = maxf(largest, absf(rig.poses[i].origin.x))
		depths[case] = _sphere_depth(rig, center, 0.04)
		rig.remove()

	t.check(largest < REST_OVERLAP_TOLERANCE, "Rest overlap: a chain that overlaps a sphere at rest moves %.5f m." % largest)
	t.check(absf(depths["bent"] - depths["rest"]) < BONE_COLLISION_TOLERANCE, "Rest overlap: bent toward the sphere, a bone is %.4f m in, %.4f m at rest." % [depths["bent"], depths["rest"]])
	t.check(depths["off"] < BONE_COLLISION_TOLERANCE, "Rest overlap off: a chain that overlaps a sphere at rest stays %.4f m in." % depths["off"])


static func _link_split(t) -> void:
	var center := Vector3(0.0, -BONE_LENGTH, 0.01)
	var nearest := {}
	for split in [false, true]:
		var chains := Chains.new(t, {"a": Vector3(-0.05, 0, 0), "c": Vector3(0.05, 0, 0)}, 3, 1)
		_set_all(chains.simulator, {"link_split": split, "stiffness/value": 0.2, "radius/value": 0.01})
		_add_collision(chains.simulator, {"position": center, "radius": LINK_SPHERE_RADIUS})
		for frame in 60:
			await chains.advance()
		nearest[split] = center.distance_to(Geometry3D.get_closest_point_to_segment(center, chains.origin("a1"), chains.origin("c1")))
		chains.remove()

	t.check(nearest[false] > LINK_SPHERE_RADIUS, "Link split off: a link across a sphere stays %.4f m from its center." % nearest[false])
	t.check(nearest[true] < 0.02, "Link split: a link across a sphere is pushed out to %.4f m from its center." % nearest[true])


static func _link_spacing(t) -> void:
	var side := 0.01
	var gap := {}
	for spacing in [0.0, 1.0]:
		var chains := Chains.new(t, {"a": Vector3(-side, 0, 0), "c": Vector3(side, 0, 0)}, 3, 1)
		_set_all(chains.simulator, {"link_spacing/value": spacing, "stiffness/value": 0.05, "radius/value": 0.02})
		for frame in 60:
			await chains.advance()
		gap[spacing] = chains.origin("a2").distance_to(chains.origin("c2"))
		chains.remove()

	t.check(absf(gap[0.0] - 2.0 * side) < LINK_SPACING_TOLERANCE, "Link spacing off: two chains 2 cm apart stay %.4f m apart." % gap[0.0])
	t.check(gap[1.0] > LINK_SPACING_MIN, "Link spacing: two chains 2 cm apart with 2 cm joints are pushed %.4f m apart." % gap[1.0])


static func _link_stretch_limit(t) -> void:
	var gap := {}
	for limit in [0.0, 1.2]:
		var chains := Chains.new(t, {"a": Vector3(-0.05, 0, 0), "c": Vector3(0.05, 0, 0)}, 3, 1)
		_set_all(chains.simulator, {"link_stretch_limit": limit, "stiffness/value": 0.02, "radius/value": 0.01})
		_add_collision(chains.simulator, {"position": Vector3(0, -0.2, 0), "radius": 0.12})
		for frame in 60:
			await chains.advance()
		gap[limit] = chains.origin("a2").distance_to(chains.origin("c2"))
		chains.remove()

	t.check(gap[0.0] > 0.1 * 1.2 + LINK_SPACING_TOLERANCE, "Link stretch limit off: a sphere pushes two chains 0.1 m apart to %.4f m." % gap[0.0])
	t.check(gap[1.2] < 0.1 * 1.2 + LINK_SPACING_TOLERANCE, "Link stretch limit 1.2: the same chains part to %.4f m, at most 0.12 m." % gap[1.2])


static func _same_place_chains(t) -> void:
	var center := Vector3(0.0, -BONE_LENGTH, 0.01)
	var chains := Chains.new(t, {"a": Vector3(-0.05, 0, 0), "b": Vector3(-0.05, 0, 0), "c": Vector3(0.05, 0, 0)}, 3, 1)
	_set_all(chains.simulator, {"stiffness/value": 0.2, "radius/value": 0.01})
	_add_collision(chains.simulator, {"position": center, "radius": LINK_SPHERE_RADIUS})
	var apart := 0.0
	for frame in 60:
		await chains.advance()
		apart = maxf(apart, chains.origin("a1").distance_to(chains.origin("b1")))
	var nearest := center.distance_to(Geometry3D.get_closest_point_to_segment(center, chains.origin("b1"), chains.origin("c1")))
	chains.remove()

	t.check(apart < LENGTH_TOLERANCE, "Same place chains: two chains at the same place move %.4f m apart." % apart)
	t.check(nearest > LINK_SPHERE_RADIUS, "Same place chains: the link next to them stays %.4f m from the sphere's center." % nearest)


#        n (z -0.05)
#   w          e    (x -0.05, x 0.05)
#        s (z 0.05)
static func _loop_order(t) -> void:
	var radius := 0.02
	var center := Vector3(0.03 / sqrt(2.0), -BONE_LENGTH, -0.03 / sqrt(2.0))
	var chains := Chains.new(t, {"e": Vector3(0.05, 0, 0), "w": Vector3(-0.05, 0, 0), "n": Vector3(0, 0, -0.05), "s": Vector3(0, 0, 0.05)}, 3, 2)
	_set_all(chains.simulator, {"stiffness/value": 0.2, "radius/value": 0.01})
	_add_collision(chains.simulator, {"position": center, "radius": radius})
	for frame in 60:
		await chains.advance()
	var nearest := center.distance_to(Geometry3D.get_closest_point_to_segment(center, chains.origin("e1"), chains.origin("n1")))
	chains.remove()

	t.check(nearest > radius, "Loop order: the link between two neighbors listed apart stays %.4f m from the sphere's center." % nearest)


static func _root_spring(t) -> void:
	var moved := {}
	var settled := {}
	for stiffness in [0.0, 0.15]:
		var rig := Rig.new(t)
		_set_all(rig.simulator, {"root_spring_stiffness": stiffness, "root_spring_limit": ROOT_SPRING_LIMIT})
		var largest := 0.0
		for frame in 240:
			if frame < 120:
				_move(rig, float(frame + 1) / 60.0)
			await rig.advance(1.0 / 60.0)
			largest = maxf(largest, rig.poses[0].origin.length())
		moved[stiffness] = largest
		settled[stiffness] = rig.poses[0].origin.length()
		rig.remove()

	t.check(moved[0.0] < LENGTH_TOLERANCE, "Root spring 0: the root moves %.6f m." % moved[0.0])
	t.check(moved[0.15] > SWING_MIN and moved[0.15] < ROOT_SPRING_LIMIT + LENGTH_TOLERANCE, "Root spring: the root moves up to %.4f m (limit %.2f m)." % [moved[0.15], ROOT_SPRING_LIMIT])
	t.check(settled[0.15] < ROOT_SPRING_TOLERANCE, "Root spring: 2 s after the character stops, the root is %.5f m off its pose." % settled[0.15])


static func _monitors(t) -> void:
	if not (Performance.has_custom_monitor(TIME_MONITOR) and Performance.has_custom_monitor(JOINTS_MONITOR)):
		t.check(false, "Monitors: the plugin has not added %s and %s." % [TIME_MONITOR, JOINTS_MONITOR])
		return

	var rigs: Array[Rig] = [Rig.new(t), Rig.new(t)]
	Performance.get_custom_monitor(TIME_MONITOR)
	Performance.get_custom_monitor(JOINTS_MONITOR)
	for frame in 10:
		rigs[0].skeleton.advance(1.0 / 60.0)
		await rigs[1].advance(1.0 / 60.0)
	var time: float = Performance.get_custom_monitor(TIME_MONITOR)
	var joints: int = Performance.get_custom_monitor(JOINTS_MONITOR)
	for rig in rigs:
		rig.remove()

	t.check(joints == 2 * (TIP + 2), "Monitors: Joints shows %d for two chains of %d joints." % [joints, TIP + 2])
	t.check(time > 0.0, "Monitors: Time shows %f s." % time)


static func _bone_lookup(t) -> void:
	var rig := Rig.new(t, 1.0, 1.0, "model", true)
	rig.simulator.set("sync/bone_count", 1)
	var sphere: Node = rig.simulator.get_child(0)
	var pairs := [
		[rig.simulator, "chains/0/root_bone_name", "chains/0/root_bone"],
		[rig.simulator, "chains/0/end_bone_name", "chains/0/end_bone"],
		[rig.simulator, "sync/bones/0/bone_name", "sync/bones/0/bone"],
		[sphere, "bone_name", "bone"],
		[sphere, "mirror_bone_name", "mirror_bone"],
	]
	for pair in pairs:
		var node: Node = pair[0]
		var name_property: String = pair[1]
		var index_property: String = pair[2]
		node.set(index_property, 2)
		t.check(node.get(name_property) == "b2", "Bone lookup, %s 2: the name reads %s." % [index_property, node.get(name_property)])
		node.set(index_property, 99)
		t.check(node.get(index_property) == -1 and node.get(name_property) == "b2", "Bone lookup, %s 99 (out of range): %s, %s." % [index_property, node.get(index_property), node.get(name_property)])
		node.set(name_property, "nope")
		t.check(node.get(index_property) == -1 and node.get(name_property) == "nope", "Bone lookup, %s nope: %s, %s." % [name_property, node.get(index_property), node.get(name_property)])
		node.set(name_property, "b3")
		t.check(node.get(index_property) == 3, "Bone lookup, %s b3: the index reads %s." % [name_property, node.get(index_property)])
	rig.remove()


static func _plane_and_inside(t) -> void:
	var tilt := Basis(Vector3.BACK, deg_to_rad(30.0))
	var center := Vector3(0.03, -0.15, 0.0)
	var axis_center := Vector3(0.03, -0.2, 0.0)
	var shapes := [
		["BoneClothCollisionPlane3D", {"transform": Transform3D(tilt, Vector3(0.0, -0.25, 0.0))},
			func(p: Vector3) -> float: return -((p - Vector3(0.0, -0.25, 0.0)).dot(tilt.y) - JOINT_RADIUS)],
		["BoneClothCollisionSphere3D", {"bone_name": "b0", "position_offset": center, "radius": 0.25, "inside": true},
			func(p: Vector3) -> float: return p.distance_to(center) - (0.25 - JOINT_RADIUS)],
		["BoneClothCollisionCapsule3D", {"bone_name": "b0", "position_offset": axis_center, "radius": 0.1, "height": 0.4, "inside": true},
			func(p: Vector3) -> float: return p.distance_to(Vector3(0.03, clampf(p.y, -0.3, -0.1), 0.0)) - (0.1 - JOINT_RADIUS)],
		["BoneClothCollisionTaperedCapsule3D", {"bone_name": "b0", "position_offset": axis_center, "top_radius": 0.1, "bottom_radius": 0.1, "height": 0.4, "inside": true},
			func(p: Vector3) -> float: return p.distance_to(Vector3(0.03, clampf(p.y, -0.3, -0.1), 0.0)) - (0.1 - JOINT_RADIUS)],
	]
	for shape in shapes:
		var rig := Rig.new(t)
		rig.simulator.set("radius/rest_overlap", false)
		_add_collision(rig.simulator, shape[1], shape[0])
		for frame in 180:
			await rig.advance(1.0 / 60.0)
		var worst := 0.0
		for i in range(1, TIP + 1):
			worst = maxf(worst, shape[2].call(rig.poses[i].origin))
		rig.remove()
		t.check(worst <= SHAPE_TOLERANCE, "Plane and inside, %s: a bone is %.4f m on the wrong side." % [shape[0], worst])


static func _wind(t) -> void:
	var still := await _wind_tip(t, 0.0, 1.0, null, false, 0.0)
	var blown := await _wind_tip(t, 5.0, 1.0, null, false, 0.0)
	t.check(blown.x < still.x - WIND_MIN, "Wind: the tip moved %.4f m along the wind." % (still.x - blown.x))
	t.check(absf(blown.z - still.z) < WIND_TOLERANCE, "Wind: the tip moved %.4f m across the wind." % absf(blown.z - still.z))

	var none := await _wind_tip(t, 5.0, 0.0, null, false, 0.0)
	t.check(none.distance_to(still) < WIND_TOLERANCE, "Wind, influence 0: the tip moved %.4f m." % none.distance_to(still))

	var half := await _wind_tip(t, 5.0, 0.5, null, false, 0.0)
	t.check(still.x - half.x < still.x - blown.x, "Wind, influence 0.5: the tip moved as far as at 1.")
	t.check(half.x < still.x - WIND_MIN, "Wind, influence 0.5: the tip moved only %.4f m along the wind." % (still.x - half.x))

	var curved := await _wind_tip(t, 5.0, 1.0, _flat_curve(0.0), false, 0.0)
	t.check(curved.distance_to(still) < WIND_TOLERANCE, "Wind, curve at 0: the tip moved %.4f m." % curved.distance_to(still))

	var hidden := await _wind_tip(t, 5.0, 1.0, null, true, 0.0)
	t.check(hidden.distance_to(still) < WIND_TOLERANCE, "Wind, hidden node: the tip moved %.4f m." % hidden.distance_to(still))

	var turned := await _wind_tip(t, 5.0, 1.0, null, false, PI)
	t.check(turned.x > still.x + WIND_MIN, "Wind, turned character: the tip moved %.4f m along local X." % (turned.x - still.x))

	var rig := Rig.new(t)
	_set_all(rig.simulator, {"gravity/value": 0.0, "root_spring_stiffness": 0.5})
	_add_wind(rig)
	for i in 60:
		await rig.advance(1.0 / 60.0)
	var root_moved: float = rig.poses[0].origin.length()
	rig.remove()
	t.check(root_moved < WIND_TOLERANCE, "Wind, root spring: the root moved %.4f m." % root_moved)


static func _wind_direction_noise(t) -> void:
	var drift := {}
	for angle in [0.0, deg_to_rad(45.0)]:
		var rig := Rig.new(t)
		rig.simulator.set("gravity/value", 0.0)
		_add_wind(rig, {"turbulence": 0.0, "direction_noise_angle": angle, "direction_noise_period": 2.0})
		var low := INF
		var high := -INF
		for i in 300:
			await rig.advance(1.0 / 60.0)
			low = minf(low, rig.tip().z)
			high = maxf(high, rig.tip().z)
		drift[angle] = high - low
		rig.remove()
	t.check(drift[0.0] < WIND_TOLERANCE, "Wind direction noise, angle 0: the tip moved %.4f m across the wind." % drift[0.0])
	t.check(drift[deg_to_rad(45.0)] > WIND_MIN, "Wind direction noise, 45 deg: the tip moved only %.4f m across the wind." % drift[deg_to_rad(45.0)])


static func _wind_scale(t) -> void:
	var tips: Array[Vector3] = []
	for armature_scale in [1.0, 0.01]:
		var rig := Rig.new(t, 1.0, armature_scale)
		rig.simulator.set("gravity/value", 0.0)
		_add_wind(rig, {"turbulence": 0.0})
		for i in 120:
			await rig.advance(1.0 / 60.0)
		tips.append(rig.tip())
		rig.remove()
	t.check(tips[0].distance_to(tips[1]) < SCALE_TOLERANCE, "Wind, cm rig: the tip is %.5f m from the m rig." % tips[0].distance_to(tips[1]))


static func _wind_tip(t, p_strength: float, p_influence: float, p_curve: Curve, p_hidden: bool, p_turn: float) -> Vector3:
	var rig := Rig.new(t)
	_set_all(rig.simulator, {"gravity/value": 0.0, "wind_influence/value": p_influence, "wind_influence/damping_curve": p_curve})
	rig.character.rotation.y = p_turn
	if p_strength > 0.0:
		_add_wind(rig, {"strength": p_strength, "turbulence": 0.0, "visible": not p_hidden})
	for i in 120:
		await rig.advance(1.0 / 60.0)
	var tip := rig.tip()
	rig.remove()
	return tip


static func _largest(t, p_values: Dictionary, p_read: Callable) -> Array[float]:
	var rig := Rig.new(t)
	_set_all(rig.simulator, p_values)
	var largest: Array[float] = []
	largest.resize(TIP + 1)
	for frame in 180:
		_move(rig, float(frame + 1) / 60.0)
		await rig.advance(1.0 / 60.0)
		for i in range(1, TIP + 1):
			largest[i] = maxf(largest[i], p_read.call(rig, i))
	rig.remove()
	return largest


static func _max_distance(t) -> void:
	var curve := Curve.new()
	curve.add_point(Vector2(0.0, 1.0))
	curve.add_point(Vector2(1.0, 0.0))
	var off: Array[float] = await _largest(t, {"max_distance/value": 0.0}, _off_pose)
	var on: Array[float] = await _largest(t, {"max_distance/value": MAX_DISTANCE}, _off_pose)
	var soft: Array[float] = await _largest(t, {"max_distance/value": MAX_DISTANCE, "movement_limit_stiffness": 0.3}, _off_pose)
	var curved: Array[float] = await _largest(t, {"max_distance/value": MAX_DISTANCE, "max_distance/damping_curve": curve}, _off_pose)

	t.check(on.max() < MAX_DISTANCE + MOVEMENT_LIMIT_TOLERANCE, "Max distance %.2f m: a bone went %.4f m from its pose." % [MAX_DISTANCE, on.max()])
	t.check(off.max() > MAX_DISTANCE * 2.0, "Max distance off: the bones went only %.4f m from their pose, so the limit was not tested." % off.max())
	t.check(soft.max() > on.max() + SWING_MIN, "Movement limit stiffness 0.3: the bones went %.4f m from their pose, as with 1.0 (%.4f m)." % [soft.max(), on.max()])
	t.check(curved[TIP] < on[TIP] * MAX_DISTANCE_CURVE_RATIO, "Max distance with a curve down to 0 at the tips: the chain end went %.4f m from its pose, without the curve %.4f m." % [curved[TIP], on[TIP]])


static func _behind_backstop(p_radius: float) -> Callable:
	return func(p_rig: Rig, p_joint: int) -> float:
		var pose := Vector3(0, -BONE_LENGTH * p_joint, 0)
		var center := pose - (pose - BACKSTOP_OFFSET).normalized() * p_radius
		return p_radius - p_rig.poses[p_joint].origin.distance_to(center)


static func _backstop(t) -> void:
	var flat := _behind_backstop(BACKSTOP_RADIUS)
	var off: Array[float] = await _largest(t, {}, flat)
	var on: Array[float] = await _largest(t, BACKSTOP, flat)
	var soft: Array[float] = await _largest(t, BACKSTOP.merged({"movement_limit_stiffness": 0.3}), flat)
	var distant: Array[float] = await _largest(t, BACKSTOP.merged({"backstop/distance": BACKSTOP_DISTANCE}), flat)
	var curved: Array[float] = await _largest(t, BACKSTOP.merged({"backstop/distance": BACKSTOP_DISTANCE, "backstop/damping_curve": _flat_curve(0.0)}), flat)
	var round_values := BACKSTOP.merged({"backstop/radius": BACKSTOP_ROUND_RADIUS})
	var round_wall: Array[float] = await _largest(t, round_values, _behind_backstop(BACKSTOP_ROUND_RADIUS))
	var round_flat: Array[float] = await _largest(t, round_values, flat)
	var none: Array[float] = await _largest(t, BACKSTOP.merged({"backstop/radius": 0.0}), flat)

	t.check(on[1] < MOVEMENT_LIMIT_TOLERANCE, "Backstop: b1 went %.6f m behind the wall." % on[1])
	t.check(off[1] > SWING_MIN, "Backstop off: b1 went only %.4f m behind the wall, so the backstop was not tested." % off[1])
	t.check(soft[1] > on[1] + SWING_MIN, "Movement limit stiffness 0.3 with the backstop: b1 went %.4f m behind the wall, as with 1.0 (%.4f m)." % [soft[1], on[1]])
	t.check(distant[1] < BACKSTOP_DISTANCE + MOVEMENT_LIMIT_TOLERANCE and distant[1] > BACKSTOP_DISTANCE * 0.5, "Backstop distance %.2f m: b1 went %.4f m behind its pose." % [BACKSTOP_DISTANCE, distant[1]])
	t.check(curved[1] < MOVEMENT_LIMIT_TOLERANCE, "Backstop distance with a curve at 0: b1 went %.6f m behind its pose." % curved[1])
	t.check(round_wall[1] < MOVEMENT_LIMIT_TOLERANCE, "Backstop radius %.2f m: b1 went %.6f m into the round wall." % [BACKSTOP_ROUND_RADIUS, round_wall[1]])
	t.check(round_flat[1] > SWING_MIN, "Backstop radius %.2f m: b1 went only %.4f m behind the flat wall, as with radius %.0f m." % [BACKSTOP_ROUND_RADIUS, round_flat[1], BACKSTOP_RADIUS])
	t.check(none[1] > SWING_MIN, "Backstop radius 0: b1 went only %.4f m behind the wall, so the backstop is still on." % none[1])


static func _anchor(t) -> void:
	var inertia := {"still": -1.0, "moving": -1.0, "anchored": 0.0, "inertia 1": 1.0, "inertia 0.5": 0.5}
	var tips := {}
	for c in inertia:
		var rig := Rig.new(t)
		if inertia[c] >= 0.0:
			_set_all(rig.simulator, {"anchor": rig.simulator.get_path_to(rig.stage), "anchor_inertia": inertia[c]})
		var list: Array[Vector3] = []
		for frame in 120:
			var time := float(frame + 1) / 60.0
			rig.character.position = Vector3(sin(time * 6.0) * 0.3, 0, 0)
			if c != "still":
				rig.stage.position = Vector3(time * 3.0, 0, sin(time * 5.0) * 0.5)
				rig.stage.rotation = Vector3(0, time * 2.0, 0)
			await rig.advance(1.0 / 60.0)
			list.append(rig.tip())
		tips[c] = list
		rig.remove()

	var off := {}
	for c in inertia:
		off[c] = 0.0
		for i in tips["still"].size():
			off[c] = maxf(off[c], tips[c][i].distance_to(tips["still" if c != "inertia 1" else "moving"][i]))
	t.check(off["moving"] > SWING_MIN, "Anchor: without the anchor, the moving stage moved the tip only %.6f m." % off["moving"])
	t.check(off["anchored"] < ANCHOR_TOLERANCE, "Anchor: on the moving anchor the tip is up to %.6f m from the still stage." % off["anchored"])
	t.check(off["inertia 1"] < ANCHOR_TOLERANCE, "Anchor inertia 1: the tip is up to %.6f m from the tip without the anchor." % off["inertia 1"])
	t.check(off["inertia 0.5"] > SWING_MIN and off["inertia 0.5"] < off["moving"] - SWING_MIN, "Anchor inertia 0.5: the moving stage moved the tip %.6f m, without the anchor %.6f m." % [off["inertia 0.5"], off["moving"]])


static func _tether(t) -> void:
	var closest := {}
	for limit in [0.0, TETHER_LIMIT]:
		var rig := Rig.new(t)
		_set_all(rig.simulator, {"stiffness/value": 0.0, "tether_limit": limit})
		closest[limit] = 1.0
		for frame in 300:
			var time := float(frame + 1) / 60.0
			rig.character.position = Vector3(sin(time * 6.0) * 0.6, sin(time * 5.0) * 0.4, 0)
			rig.character.rotation = Vector3(sin(time * 3.0) * 1.2, sin(time * 4.0) * 0.8, 0)
			await rig.advance(1.0 / 60.0)
			closest[limit] = minf(closest[limit], rig.poses[TIP].origin.length() / (BONE_LENGTH * TIP))
		rig.remove()

	t.check(closest[TETHER_LIMIT] > TETHER_CLOSEST, "Tether limit %.1f: the tip came to %.4f of its pose distance from the root." % [TETHER_LIMIT, closest[TETHER_LIMIT]])
	t.check(closest[0.0] < TETHER_CLOSEST, "Tether off: the tip came only to %.4f of its pose distance from the root, so the tether was not tested." % closest[0.0])


static func _collision_friction(t) -> void:
	var slide := {}
	for friction in [0.0, 0.5]:
		var rig := Rig.new(t)
		_add_collision(rig.simulator, {"bone_name": "b0", "position_offset": FRICTION_SPHERE_OFFSET, "radius": FRICTION_SPHERE_RADIUS})
		_set_all(rig.simulator, {"gravity/direction": Vector3(1, 0, 0), "collision_friction": friction})
		slide[friction] = 0.0
		var last := 0.0
		for frame in 240:
			rig.character.position = Vector3(0, 0, sin(float(frame + 1) / 60.0 * 4.0) * 0.3)
			await rig.advance(1.0 / 60.0)
			var z := rig.poses[TIP].origin.z
			if frame >= 60:
				slide[friction] += absf(z - last)
			last = z
		rig.remove()

	t.check(slide[0.5] < slide[0.0] * FRICTION_SLIDE_RATIO, "Collision friction 0.5: the tip slid %.4f m across the sphere, without friction %.4f m." % [slide[0.5], slide[0.0]])


static func _gravity_falloff(t) -> void:
	var cases := {
		"off": [0.0, Quaternion()],
		"upright": [1.0, Quaternion()],
		"turned": [1.0, Quaternion(Vector3.UP, PI)],
	}
	var sag := {}
	for c in cases:
		var rig := Rig.new(t)
		_set_all(rig.simulator, {"gravity/direction": Vector3(-1, 0, 0), "gravity/falloff": cases[c][0]})
		rig.skeleton.set_bone_pose_rotation(0, cases[c][1])
		for frame in 180:
			await rig.advance(1.0 / 60.0)
		sag[c] = _off_pose(rig, TIP)
		rig.remove()

	t.check(sag["off"] > GRAVITY_SAG_MIN, "Gravity falloff off: the tip sagged only %.4f m, so the falloff was not tested." % sag["off"])
	t.check(sag["upright"] < SWING_MIN, "Gravity falloff 1, upright: the tip sagged %.4f m." % sag["upright"])
	t.check(absf(sag["turned"] - sag["off"]) < SHAPE_TOLERANCE, "Gravity falloff 1, turned over: the tip sagged %.4f m, without falloff %.4f m." % [sag["turned"], sag["off"]])


static func _stiffness_velocity_damping(t) -> void:
	var crossings := {}
	for damping in [0.0, 0.8]:
		var rig := Rig.new(t)
		_set_all(rig.simulator, {"stiffness/value": 0.2, "gravity/value": 0.0, "stiffness/velocity_damping": damping})
		crossings[damping] = 0
		var last_x := 0.0
		for frame in 240:
			if frame < 60:
				_move(rig, float(frame + 1) / 60.0)
			await rig.advance(1.0 / 60.0)
			var x := rig.poses[TIP].origin.x
			if frame > 60 and signf(x) != signf(last_x):
				crossings[damping] += 1
			last_x = x
		rig.remove()

	t.check(crossings[0.8] < crossings[0.0], "Stiffness velocity damping 0.8: the tip swung across its pose %d times while settling, without it %d." % [crossings[0.8], crossings[0.0]])


static func _ticks_per_second(t) -> void:
	var cases := {
		"default": {},
		"stiffness 0.2, velocity damping 0.8": {"stiffness/value": 0.2, "stiffness/velocity_damping": 0.8},
	}
	for c in cases:
		var tips := {}
		for ticks in [60, 120, 240]:
			var rig := Rig.new(t)
			_set_all(rig.simulator, {"gravity/direction": Vector3(-1, 0, 0), "ticks_per_second": ticks})
			_set_all(rig.simulator, cases[c])
			for frame in 300:
				await rig.advance(1.0 / 60.0)
			tips[ticks] = rig.tip()
			rig.remove()

		var off_60: float = tips[60].distance_to(tips[240])
		var off_120: float = tips[120].distance_to(tips[240])
		t.check(off_120 < off_60 * TICKS_RATIO, "Ticks per second, %s: at 120 ticks the tip settles %.4f m from 240 ticks, at 60 ticks %.4f m." % [c, off_120, off_60])
		if c == "default":
			var off: float = tips[120].distance_to(tips[60])
			t.check(off < TICKS_TOLERANCE, "Ticks per second 120: the tip settles %.4f m from 60 ticks." % off)


static func _max_steps_per_frame(t) -> void:
	var moved := {}
	for steps in [1, 8]:
		var rig := Rig.new(t)
		_set_all(rig.simulator, {"gravity/direction": Vector3(-1, 0, 0), "max_steps_per_frame": steps})
		for frame in 2:
			await rig.advance(1.0 / 60.0)
		var before := rig.tip()
		await rig.advance(0.1)
		# 0.1 s 프레임의 결과가 poses에 들어오도록 한 번 더 돌린다.
		await rig.advance(0.0)
		moved[steps] = rig.tip().distance_to(before)
		rig.remove()

	t.check(moved[1] < moved[8] * MAX_STEPS_RATIO, "Max steps per frame 1: a 0.1 s frame moved the tip %.4f m, with 8 steps %.4f m." % [moved[1], moved[8]])


static func _stabilization_time(t) -> void:
	var early := {}
	var late := {}
	for seconds in [0.0, 0.5]:
		var rig := Rig.new(t)
		rig.simulator.set("stabilization_time", seconds)
		early[seconds] = 0.0
		late[seconds] = 0.0
		for frame in 90:
			_move(rig, float(frame + 1) / 60.0)
			await rig.advance(1.0 / 60.0)
			var off := _off_pose(rig, TIP)
			if frame < 18:
				early[seconds] = maxf(early[seconds], off)
			elif frame >= 60:
				late[seconds] = maxf(late[seconds], off)
		rig.remove()

	t.check(early[0.5] < early[0.0] * STABILIZATION_RATIO, "Stabilization time 0.5 s: in the first 0.3 s the tip swung %.4f m from its pose, without it %.4f m." % [early[0.5], early[0.0]])
	t.check(late[0.5] > SWING_MIN, "Stabilization time 0.5 s: after 1 s the tip swung only %.4f m, so the motion never came back." % late[0.5])


static func _add_camera(p_rig: Rig, p_camera_position: Vector3) -> Camera3D:
	var camera := Camera3D.new()
	p_rig.stage.add_child(camera)
	camera.position = p_camera_position
	camera.current = true
	return camera


static func _camera_culling(t) -> void:
	var cases := {"reset": 1, "keep": 2}
	for c in cases:
		var rig := Rig.new(t)
		var camera := _add_camera(rig, Vector3(0, -0.2, 2.0))
		_set_all(rig.simulator, {"gravity/direction": Vector3(-1, 0, 0), "camera_culling": cases[c]})
		for frame in 240:
			await rig.advance(1.0 / 60.0)
		var before := rig.tip()
		camera.rotation = Vector3(0, PI, 0)
		await rig.advance(1.0 / 60.0)
		Performance.get_custom_monitor(JOINTS_MONITOR)
		for frame in 10:
			await rig.advance(1.0 / 60.0)
		var joints: int = Performance.get_custom_monitor(JOINTS_MONITOR)
		camera.rotation = Vector3()
		for frame in 3:
			await rig.advance(1.0 / 60.0)
		var after := rig.tip()
		rig.remove()

		t.check(joints == 0, "Camera culling, %s: off screen the monitor shows %d joints." % [c, joints])
		if c == "reset":
			t.check(after.distance_to(TIP_POSE) < after.distance_to(before) * CULLING_RESET_RATIO, "Camera culling, reset: back on screen the tip is %.4f m from its pose and %.4f m from where it left." % [after.distance_to(TIP_POSE), after.distance_to(before)])
		else:
			t.check(after.distance_to(before) < SWING_MIN, "Camera culling, keep: back on screen the tip is %.4f m from where it left." % after.distance_to(before))


static func _distance_culling(t) -> void:
	var rigs: Array[Rig] = [Rig.new(t), Rig.new(t)]
	var camera := _add_camera(rigs[0], Vector3(0, 0, 3.75))
	for rig in rigs:
		rig.simulator.set("gravity/direction", Vector3(-1, 0, 0))
	_set_all(rigs[0].simulator, {"distance_culling": 5.0, "distance_culling_fade": 0.5})
	for frame in 120:
		rigs[1].skeleton.advance(1.0 / 60.0)
		await rigs[0].advance(1.0 / 60.0)
	var expected := Vector3(0, -BONE_LENGTH, 0).lerp(rigs[1].poses[1].origin, 0.5)
	var off := rigs[0].poses[1].origin.distance_to(expected)

	camera.position.z = 6.0
	for frame in 2:
		rigs[1].skeleton.advance(1.0 / 60.0)
		await rigs[0].advance(1.0 / 60.0)
	Performance.get_custom_monitor(JOINTS_MONITOR)
	for frame in 10:
		rigs[1].skeleton.advance(1.0 / 60.0)
		await rigs[0].advance(1.0 / 60.0)
	var joints: int = Performance.get_custom_monitor(JOINTS_MONITOR)
	for rig in rigs:
		rig.remove()

	t.check(off < CULLING_TOLERANCE, "Distance culling fade: halfway into the fade a bone is %.6f m from halfway between pose and motion." % off)
	t.check(joints == TIP + 2, "Distance culling: past the distance the monitor shows %d joints, for the other rig's %d." % [joints, TIP + 2])


static func _per_axis_limit(t) -> void:
	var hinge_lower := Vector3(0.0, -PI, deg_to_rad(-30.0))
	var hinge_upper := Vector3(0.0, PI, deg_to_rad(30.0))
	var cases := {
		"off": [0, Vector3(), Vector3(), 1.0],
		"hinge": [1, hinge_lower, hinge_upper, 1.0],
		"one way": [1, Vector3(0.0, -PI, 0.0), Vector3(0.0, PI, deg_to_rad(40.0)), 1.0],
		"ellipse": [1, Vector3(deg_to_rad(-10.0), -PI, deg_to_rad(-40.0)), Vector3(deg_to_rad(10.0), PI, deg_to_rad(40.0)), 1.0],
		"hinge, curve at 0.5": [1, hinge_lower, hinge_upper, 0.5],
	}
	var lowest := {}
	var highest := {}
	for c in cases:
		var rig := Rig.new(t)
		_set_all(rig.simulator, {"limit_angle/mode": cases[c][0], "limit_angle/lower": cases[c][1], "limit_angle/upper": cases[c][2]})
		if cases[c][3] != 1.0:
			rig.simulator.set("limit_angle/damping_curve", _flat_curve(cases[c][3]))
		lowest[c] = Vector3.ONE * INF
		highest[c] = -Vector3.ONE * INF
		for frame in 180:
			var time := float(frame + 1) / 60.0
			rig.character.position = Vector3(sin(time * 6.0) * 0.6, 0, sin(time * 5.0) * 0.5)
			rig.character.rotation = Vector3(0, sin(time * 4.0) * 0.8, 0)
			await rig.advance(1.0 / 60.0)
			for i in range(1, TIP + 1):
				var bone := (rig.poses[i].origin - rig.poses[i - 1].origin).normalized()
				var swing := Vector3.DOWN.cross(bone).normalized() * Vector3.DOWN.angle_to(bone)
				lowest[c] = lowest[c].min(swing)
				highest[c] = highest[c].max(swing)
		rig.remove()

	t.check(highest["off"].x > deg_to_rad(15.0) and lowest["off"].z < deg_to_rad(-35.0) and highest["off"].z > deg_to_rad(45.0), "Per axis limit off: the bones swung only %s to %s deg, so the limit was not tested." % [lowest["off"] * 180.0 / PI, highest["off"] * 180.0 / PI])
	for c in ["hinge", "one way", "ellipse", "hinge, curve at 0.5"]:
		var lower: Vector3 = cases[c][1] * cases[c][3]
		var upper: Vector3 = cases[c][2] * cases[c][3]
		var inside: bool = lowest[c].x > lower.x - ANGLE_TOLERANCE and highest[c].x < upper.x + ANGLE_TOLERANCE and lowest[c].z > lower.z - ANGLE_TOLERANCE and highest[c].z < upper.z + ANGLE_TOLERANCE
		t.check(inside, "Per axis limit, %s: the bones swung %s to %s deg around X and Z." % [c, lowest[c] * 180.0 / PI, highest[c] * 180.0 / PI])


static func _box_distance(p_point: Vector3, p_half_size: Vector3) -> float:
	var over := p_point.abs() - p_half_size
	return over.max(Vector3.ZERO).length() + minf(maxf(over.x, maxf(over.y, over.z)), 0.0)


static func _box_collision(t) -> void:
	var center := Vector3(0.03, -0.25, 0.0)
	var half_size := Vector3(0.05, 0.05, 0.1)
	var deepest := {}
	for on in [false, true]:
		var rig := Rig.new(t)
		rig.simulator.set("radius/rest_overlap", false)
		if on:
			_add_collision(rig.simulator, {"bone_name": "b0", "position_offset": center, "size": half_size * 2.0}, "BoneClothCollisionBox3D")
		for frame in 180:
			await rig.advance(1.0 / 60.0)
		deepest[on] = -INF
		for i in range(1, TIP + 1):
			deepest[on] = maxf(deepest[on], JOINT_RADIUS - _box_distance(rig.poses[i].origin - center, half_size))
		rig.remove()

	t.check(deepest[true] <= SHAPE_TOLERANCE, "Box collision: a bone is %.4f m into the box." % deepest[true])
	t.check(deepest[false] > SWING_MIN, "Box collision off: the bones came only %.4f m into the box, so it was not tested." % deepest[false])


static func _link_bending(t) -> void:
	var followed := {}
	for bending in [0.0, 1.0]:
		var chains := Chains.new(t, {"a": Vector3(-0.1, 0, 0), "b": Vector3(), "c": Vector3(0.1, 0, 0)}, 4, 1)
		for i in 3:
			chains.simulator.set("chains/%d/extend_end_bone" % i, false)
		_set_all(chains.simulator, {"link_bending": bending, "stiffness/value": 0.05, "radius/value": 0.02})
		_add_collision(chains.simulator, {"position": Vector3(0.0, -0.25, -0.03), "radius": 0.05})
		for i in [0, 2]:
			chains.simulator.call("set_enable_all_child_collisions", i, false)
		for frame in 60:
			await chains.advance()
		followed[bending] = minf(chains.origin("a3").z, chains.origin("c3").z)
		chains.remove()

	t.check(followed[0.0] < followed[1.0] / BENDING_RATIO, "Link bending 1: the chains beside a pushed chain followed it %.4f m, without bending %.4f m." % [followed[1.0], followed[0.0]])


static func _preset_round_trip(t) -> void:
	var resource_names: Array = Resource.new().get_property_list().map(func(property): return property.name)
	var names: Array[String] = []
	for property in ClassDB.instantiate("BoneClothPreset").get_property_list():
		if property.usage & PROPERTY_USAGE_EDITOR and not property.name in resource_names:
			names.append(property.name)
	t.check(names.size() == 56, "Preset round trip: the preset holds %d values." % names.size())

	var source: Node = ClassDB.instantiate("BoneClothSimulator3D")
	var target: Node = ClassDB.instantiate("BoneClothSimulator3D")
	var stored: Array = source.call("make_preset").get_property_list().filter(func(property): return property.usage & PROPERTY_USAGE_STORAGE and not property.name in resource_names)
	t.check(stored.is_empty(), "Preset round trip: a preset of default values stores %s." % [stored.map(func(property): return property.name)])
	for name in names:
		var value: Variant = source.get(name)
		match typeof(value):
			TYPE_BOOL:
				source.set(name, not value)
			TYPE_INT:
				source.set(name, value + 1)
			TYPE_FLOAT:
				source.set(name, 0.5 * value + 0.01)
			TYPE_VECTOR3:
				source.set(name, Vector3(0.0, 0.0, -1.0))
			TYPE_OBJECT, TYPE_NIL:
				var curve := Curve.new()
				curve.add_point(Vector2(0.0, 0.25))
				source.set(name, curve)

	var kept := {"anchor": NodePath("Stage"), "anchor_inertia": 0.5, "sync/value": 0.5, "camera_culling": 1, "link_mode": 1, "distance_culling": 5.0, "distance_culling_fade": 0.5, "backstop/bone_name": "b0", "backstop/position_offset": Vector3(0.0, 0.0, 1.0), "ticks_per_second": 90, "max_steps_per_frame": 6, "chain_count": 1, "sync/bone_count": 1}
	_set_all(target, kept)
	var preset: Resource = source.call("make_preset")
	target.call("apply_preset", preset)
	for name in names:
		var expected: Variant = source.get(name)
		var actual: Variant = target.get(name)
		if expected is Curve:
			t.check(actual is Curve and actual != expected and actual.sample(0.0) == expected.sample(0.0), "Preset round trip, %s: the curve did not arrive as its own copy." % name)
		else:
			t.check(actual == expected, "Preset round trip, %s: %s arrived as %s." % [name, expected, actual])
	for name in kept:
		t.check(target.get(name) == kept[name], "Preset round trip, %s outside the preset: applying it changed %s to %s." % [name, kept[name], target.get(name)])
	source.free()
	target.free()
