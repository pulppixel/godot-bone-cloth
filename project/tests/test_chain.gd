extends RefCounted

const BONE_LENGTH := 0.1 # m
const TIP := 4

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
const ROOT_SPRING_LIMIT := 0.03 # m
const ROOT_SPRING_TOLERANCE := 0.001 # m
const ROOT_SPRING_FORWARD_LIMIT := 0.005 # m
const SHAPE_TOLERANCE := 0.005 # m
const WIND_MIN := 0.02 # m
const WIND_TOLERANCE := 0.0001 # m
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
	var poses: Array[Transform3D] = [] # 지난 frame에 시뮬레이터가 쓴 bone pose (skeleton 공간)

	# p_owner: "model"이면 simulator가 모델(character)에 저장된 경우, "stage"면 Stage에서 더한 경우, ""이면 owner가 없는 경우.
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
		simulator.set("radius/value", 0.02)
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

	# 시뮬레이터를 p_delta초 진행하고, 결과가 skeleton에 쓰일 때까지 기다린다.
	func advance(p_delta: float) -> void:
		skeleton.advance(p_delta)
		await skeleton.get_tree().process_frame

	# 끝 bone의 위치. character 기준이고, 캐릭터 크기로 나눈 m 단위다(scale 1과 바로 비교할 수 있다).
	func tip() -> Vector3:
		var world := skeleton.global_transform * poses[TIP].origin
		return character.global_transform.affine_inverse() * world

	func remove() -> void:
		stage.free()

	func _read_poses() -> void:
		poses.clear()
		for i in skeleton.get_bone_count():
			poses.append(skeleton.get_bone_global_pose(i))


# t: run_tests.gd.
static func run(t) -> void:
	await _frame_rate(t)
	await _teleport(t)
	await _limits_and_lengths(t)
	await _soft_limit(t)
	await _long_step(t)
	await _scale(t)
	await _bone_collision(t)
	await _depth_inertia(t)
	await _depth_speed_limit(t)
	await _local_speed_limit(t)
	await _local_rotation_limit(t)
	await _sync_bone(t)
	await _squeeze(t)
	await _rest_overlap(t)
	await _rest_overlap_off(t)
	await _link_split(t)
	await _link_spacing(t)
	await _root_spring(t)
	await _root_spring_forward(t)
	await _wind(t)
	await _wind_frame_rate(t)
	await _wind_direction_noise(t)
	await _wind_scale(t)
	await _monitors(t)
	_bone_lookup(t)
	await _plane_and_inside(t)
	_preset_round_trip(t)


# 캐릭터가 좌우로 오가며 몸을 돌리는 움직임. p_time초 때의 자세를 정한다.
static func _move(p_rig: Rig, p_time: float) -> void:
	var size := p_rig.character.scale.x
	p_rig.character.position = Vector3(sin(p_time * 6.0) * 0.6 * size, 0, 0)
	p_rig.character.rotation = Vector3(0, sin(p_time * 4.0) * 0.8, 0)


# frame rate가 달라도 같은 시각에는 같은 자리에 있다.
# 옆으로 거는 중력으로만 흔든다. 캐릭터를 움직이면 30 fps에서는 움직임 자체가 거칠게 들어가서, 천과 상관없이 달라진다.
static func _frame_rate(t) -> void:
	var tips := {} # fps마다, 1/30초마다 읽은 끝 bone 위치
	for fps in [30, 60, 120]:
		var rig := Rig.new(t)
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
		t.check(largest < FRAME_RATE_TOLERANCE, "Frame rate %d fps: the tip is up to %.4f m from 60 fps." % [fps, largest])


# 한 frame에 teleport_distance(기본 0.5 m)보다 멀리 가거나 teleport_angle(기본 90도)보다 많이 돌면, 천이 흔들리지 않고 그대로 따라간다.
# 중력을 0으로 둔다. 중력이 있으면 돈 뒤에 chain이 중력 쪽으로 다시 흔들려서, teleport와 상관없이 움직인다.
static func _teleport(t) -> void:
	# 이름, 이동, Z축 회전, teleport_angle, teleport인지
	var cases := [
		["move 1 m", Vector3(1, 0, 0), 0.0, deg_to_rad(90.0), true],
		["turn 100 deg", Vector3(), deg_to_rad(100.0), deg_to_rad(90.0), true],
		["move 0.3 m", Vector3(0.3, 0, 0), 0.0, deg_to_rad(90.0), false],
		["turn 60 deg", Vector3(), deg_to_rad(60.0), deg_to_rad(90.0), false],
		# 120도보다 많이 돈 회전도 teleport_angle을 기준으로 나뉘어야 한다.
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


# 크게 흔들려도 bone 길이는 그대로이고, limit_angle보다 기울지 않는다.
static func _limits_and_lengths(t) -> void:
	var limit := deg_to_rad(30.0)
	var rig := Rig.new(t)
	rig.simulator.set("limit_angle/value", limit)
	var longest := 0.0 # pose 길이와 가장 많이 다른 양 (m)
	var widest := 0.0 # pose 방향과 가장 많이 벌어진 각 (rad)
	for frame in 180:
		_move(rig, float(frame + 1) / 60.0)
		await rig.advance(1.0 / 60.0)
		for i in range(1, TIP + 1):
			var bone := rig.poses[i].origin - rig.poses[i - 1].origin
			longest = maxf(longest, absf(bone.length() - BONE_LENGTH))
			widest = maxf(widest, bone.angle_to(Vector3.DOWN))
	rig.remove()

	t.check(longest < LENGTH_TOLERANCE, "Lengths: a bone differs from its pose by %.7f m." % longest)
	t.check(widest < limit + ANGLE_TOLERANCE, "Limit angle 30 deg: a bone leaned %.4f deg." % rad_to_deg(widest))
	t.check(widest > limit - deg_to_rad(5.0), "Limit angle 30 deg: the bones leaned only %.4f deg, so the limit was not tested." % rad_to_deg(widest))


# limit_angle/stiffness: 1보다 작으면 bone이 limit_angle을 넘었다가 다음 step들에 걸쳐 돌아온다. 길이는 그대로다.
static func _soft_limit(t) -> void:
	var limit := deg_to_rad(30.0)
	var widest := {} # stiffness별, pose 방향과 가장 많이 벌어진 각 (rad)
	var longest := 0.0 # pose 길이와 가장 많이 다른 양 (m)
	for stiffness in [0.0, 0.3]:
		var rig := Rig.new(t)
		rig.simulator.set("limit_angle/value", limit)
		rig.simulator.set("limit_angle/stiffness", stiffness)
		widest[stiffness] = 0.0
		for frame in 180:
			_move(rig, float(frame + 1) / 60.0)
			await rig.advance(1.0 / 60.0)
			for i in range(1, TIP + 1):
				var bone := rig.poses[i].origin - rig.poses[i - 1].origin
				longest = maxf(longest, absf(bone.length() - BONE_LENGTH))
				widest[stiffness] = maxf(widest[stiffness], bone.angle_to(Vector3.DOWN))
		rig.remove()

	t.check(longest < LENGTH_TOLERANCE, "Soft limit: a bone differs from its pose by %.7f m." % longest)
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


# 균일한 scale은 cm 리그든 크게 키운 캐릭터든 scale 1과 똑같이 움직인다(크기만 다르다).
static func _scale(t) -> void:
	# 이름, 캐릭터 scale, armature scale, owner, collision. 첫 줄과 collision이 있는 첫 줄이 기준이다.
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


# joint 사이의 bone도 collision 밖으로 밀린다.
static func _bone_collision(t) -> void:
	var rig := Rig.new(t)
	rig.simulator.set("gravity/value", 0.0)
	var sphere: Node = ClassDB.instantiate("BoneClothCollisionSphere3D")
	rig.simulator.add_child(sphere)
	sphere.set("position", Vector3(0.05, -0.15, 0.0))
	sphere.set("radius", 0.04)
	for frame in 60:
		await rig.advance(1.0 / 60.0)
	var center := rig.poses[0].origin + Vector3(0.05, -0.15, 0.0)
	var nearest := INF
	for i in range(1, TIP + 1):
		var a := rig.poses[i - 1].origin
		var b := rig.poses[i].origin
		nearest = minf(nearest, center.distance_to(Geometry3D.get_closest_point_to_segment(center, a, b)))
	rig.remove()

	var depth := 0.04 + 0.02 - nearest
	t.check(depth < BONE_COLLISION_TOLERANCE, "Bone collision: a bone is %.4f m into the sphere." % depth)


static func _depth_inertia(t) -> void:
	var offsets := {} # depth_inertia마다, b1이 제자리에서 벗어난 평균 거리 (m)
	for value in [0.0, 1.0]:
		var rig := Rig.new(t)
		rig.simulator.set("depth_inertia", value)
		var sum := 0.0
		for frame in 120:
			rig.skeleton.set_bone_pose_position(0, Vector3(sin(float(frame + 1) / 60.0 * 12.0) * 0.05, 0, 0))
			await rig.advance(1.0 / 60.0)
			sum += rig.poses[1].origin.distance_to(rig.poses[0].origin + Vector3(0, -BONE_LENGTH, 0))
		offsets[value] = sum / 120.0
		rig.remove()

	t.check(offsets[1.0] < offsets[0.0] * DEPTH_INERTIA_RATIO, "Depth inertia 1: the joint next to the root is %.4f m off its place, against %.4f m at 0." % [offsets[1.0], offsets[0.0]])


static func _depth_speed_limit(t) -> void:
	var trails := {} # depth_inertia마다, b1이 제자리에서 뒤로 처진 평균 거리 (m)
	for value in [0.0, 1.0]:
		var rig := Rig.new(t)
		rig.simulator.set("depth_inertia", value)
		rig.simulator.set("joint_speed_limit", 1.0)
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
	var offsets := {} # 제한마다, b4가 제자리에서 벗어난 평균 거리 (m)
	for limit in [0.0, 0.1]:
		var rig := Rig.new(t)
		rig.simulator.set("gravity/value", 0.0)
		rig.simulator.set("local_movement_speed_limit", limit)
		var sum := 0.0
		for frame in 120:
			rig.skeleton.set_bone_pose_position(0, Vector3(sin(float(frame + 1) / 60.0 * 12.0) * 0.05, 0, 0))
			await rig.advance(1.0 / 60.0)
			sum += rig.poses[TIP].origin.distance_to(rig.poses[0].origin + Vector3(0, -BONE_LENGTH * TIP, 0))
		offsets[limit] = sum / 120.0
		rig.remove()

	t.check(offsets[0.1] < offsets[0.0] * LOCAL_SPEED_LIMIT_RATIO, "Local movement speed limit 0.1 m/s: the tip is %.4f m off its place, against %.4f m without the limit." % [offsets[0.1], offsets[0.0]])


static func _local_rotation_limit(t) -> void:
	var offsets := {} # 제한마다, b4가 제자리에서 벗어난 평균 거리 (m)
	for limit in [0.0, 0.5]:
		var rig := Rig.new(t)
		rig.simulator.set("gravity/value", 0.0)
		rig.simulator.set("local_rotation_speed_limit", limit)
		var sum := 0.0
		for frame in 120:
			rig.skeleton.set_bone_pose_rotation(0, Quaternion(Vector3.BACK, sin(float(frame + 1) / 60.0 * 12.0) * 0.3))
			await rig.advance(1.0 / 60.0)
			sum += rig.poses[TIP].origin.distance_to(rig.poses[0] * Vector3(0, -BONE_LENGTH * TIP, 0))
		offsets[limit] = sum / 120.0
		rig.remove()

	t.check(offsets[0.5] < offsets[0.0] * LOCAL_SPEED_LIMIT_RATIO, "Local rotation speed limit 0.5 rad/s: the tip is %.4f m off its place, against %.4f m without the limit." % [offsets[0.5], offsets[0.0]])


static func _sync_bone(t) -> void:
	var cases := { # 경우: [thigh 위치, thigh pose 회전, sync/value, 기대하는 사슬 회전]
		"rest": [Vector3(0, 0, 0.05), Quaternion(), 1.0, Quaternion()],
		"swing": [Vector3(0, 0, 0.05), Quaternion(Vector3.RIGHT, 0.5), 1.0, Quaternion(Vector3.RIGHT, 0.5)],
		"back": [Vector3(0, 0, 0.05), Quaternion(Vector3.RIGHT, -0.5), 1.0, Quaternion(Vector3.RIGHT, -0.5)],
		"half": [Vector3(0, 0, 0.05), Quaternion(Vector3.RIGHT, 0.5), 0.5, Quaternion(Vector3.RIGHT, 0.25)],
		"side": [Vector3(0.05, 0, 0), Quaternion(Vector3.RIGHT, 0.5), 1.0, Quaternion(Vector3.RIGHT, 0.125)],
		"twist": [Vector3(0, 0, 0.05), Quaternion(Vector3.UP, 0.5), 1.0, Quaternion()],
		"off": [Vector3(0, 0, 0.05), Quaternion(Vector3.RIGHT, 0.5), 0.0, Quaternion()],
	}
	var tips := {}
	var tilt := 0.0 # swing에서 root bone 회전이 가리키는 b1 방향과 실제 b1 방향 사이의 각도 (rad)
	for case in cases:
		var rig := Rig.new(t)
		rig.skeleton.set_bone_rest(1, Transform3D(Basis(), Vector3(0, -BONE_LENGTH, 0.05)))
		rig.skeleton.set_bone_pose(1, rig.skeleton.get_bone_rest(1))
		var thigh := rig.skeleton.add_bone("thigh")
		rig.skeleton.set_bone_parent(thigh, 0)
		rig.skeleton.set_bone_rest(thigh, Transform3D(Basis(), cases[case][0]))
		rig.skeleton.set_bone_pose(thigh, rig.skeleton.get_bone_rest(thigh))
		rig.skeleton.set_bone_pose_rotation(thigh, cases[case][1])
		rig.simulator.set("gravity/value", 0.0)
		rig.simulator.set("stiffness/value", 1.0)
		rig.simulator.set("sync/bone_count", 1)
		rig.simulator.set("sync/bones/0/bone_name", "thigh")
		rig.simulator.set("sync/value", cases[case][2])
		for frame in 30:
			await rig.advance(1.0 / 60.0)
		tips[case] = rig.tip()
		if case == "swing":
			tilt = (rig.poses[0].basis * Vector3(0, -BONE_LENGTH, 0.05)).angle_to(rig.poses[1].origin - rig.poses[0].origin)
		rig.remove()

	for case in cases:
		var expected: Vector3 = cases[case][3] * tips["rest"]
		t.check(tips[case].distance_to(expected) < SYNC_TOLERANCE, "Sync bone, %s: the chain end is %.5f m off the chain turned from its root." % [case, tips[case].distance_to(expected)])
	t.check(tips["swing"].distance_to(tips["rest"]) > SWING_MIN, "Sync bone, swing: the chain end moves %.4f m." % tips["swing"].distance_to(tips["rest"]))
	t.check(tilt < ANGLE_TOLERANCE, "Sync bone, swing: the root bone's rotation points %.4f rad off the moved joint." % tilt)


static func _squeeze(t) -> void:
	var rig := Rig.new(t)
	rig.simulator.set("gravity/value", 0.0)
	for side in [1.0, -1.0]:
		var sphere: Node = ClassDB.instantiate("BoneClothCollisionSphere3D")
		rig.simulator.add_child(sphere)
		sphere.set("position", Vector3(0.05 * side, -0.15, 0.0))
		sphere.set("radius", 0.04)
	var largest := 0.0 # b1 ~ 끝 bone이 옆(X)으로 간 가장 먼 거리 (m)
	for frame in 60:
		await rig.advance(1.0 / 60.0)
		for i in range(1, TIP + 1):
			largest = maxf(largest, absf(rig.poses[i].origin.x))
	rig.remove()

	t.check(largest < SQUEEZE_TOLERANCE, "Squeeze: a chain between two spheres pushing opposite ways moves %.5f m sideways." % largest)


static func _rest_overlap(t) -> void:
	var center := Vector3(0.05, -0.15, 0.0)
	var depths := {} # 경우마다, bone이 구에 들어간 가장 깊은 거리 (m)
	var largest := 0.0 # 그대로 둔 경우, b1 ~ 끝 bone이 옆(X)으로 간 가장 먼 거리 (m)
	for case in ["rest", "bent"]:
		var rig := Rig.new(t)
		rig.simulator.set("gravity/value", 0.0)
		rig.simulator.set("stiffness/value", 0.2)
		var sphere: Node = ClassDB.instantiate("BoneClothCollisionSphere3D")
		rig.simulator.add_child(sphere)
		sphere.set("bone_name", "b0")
		sphere.set("position_offset", center)
		sphere.set("radius", 0.04)
		if case == "bent":
			rig.skeleton.set_bone_pose_rotation(1, Quaternion(Vector3(0, 0, 1), 0.3))
		for frame in 60:
			await rig.advance(1.0 / 60.0)
			if case == "rest":
				for i in range(1, TIP + 1):
					largest = maxf(largest, absf(rig.poses[i].origin.x))
		var deepest := 0.0
		for i in range(1, TIP + 1):
			var nearest := Geometry3D.get_closest_point_to_segment(center, rig.poses[i - 1].origin, rig.poses[i].origin)
			deepest = maxf(deepest, 0.04 + 0.02 - center.distance_to(nearest))
		depths[case] = deepest
		rig.remove()

	t.check(largest < REST_OVERLAP_TOLERANCE, "Rest overlap: a chain that overlaps a sphere at rest moves %.5f m." % largest)
	t.check(absf(depths["bent"] - depths["rest"]) < BONE_COLLISION_TOLERANCE, "Rest overlap: bent toward the sphere, a bone is %.4f m in, %.4f m at rest." % [depths["bent"], depths["rest"]])


# radius/rest_overlap을 끄면, rest 자세에서 구와 겹친 chain도 구 밖으로 밀린다.
static func _rest_overlap_off(t) -> void:
	var center := Vector3(0.05, -0.15, 0.0)
	var rig := Rig.new(t)
	rig.simulator.set("gravity/value", 0.0)
	rig.simulator.set("stiffness/value", 0.2)
	rig.simulator.set("radius/rest_overlap", false)
	var sphere: Node = ClassDB.instantiate("BoneClothCollisionSphere3D")
	rig.simulator.add_child(sphere)
	sphere.set("bone_name", "b0")
	sphere.set("position_offset", center)
	sphere.set("radius", 0.04)
	for frame in 60:
		await rig.advance(1.0 / 60.0)
	var deepest := 0.0 # bone이 구에 들어간 가장 깊은 거리 (m)
	for i in range(1, TIP + 1):
		var nearest := Geometry3D.get_closest_point_to_segment(center, rig.poses[i - 1].origin, rig.poses[i].origin)
		deepest = maxf(deepest, 0.04 + 0.02 - center.distance_to(nearest))
	rig.remove()

	t.check(deepest < BONE_COLLISION_TOLERANCE, "Rest overlap off: a chain that overlaps a sphere at rest stays %.4f m in." % deepest)


# link_split: 두 chain을 잇는 link만 구를 가로지른다. 켜면 구가 link를 밀지 않고, 끄면 link가 구 밖으로 밀린다.
#   r +-- a0 - a1 - a2   (x -0.05)
#     +-- c0 - c1 - c2   (x 0.05)
#   구는 a1과 c1 사이, 조금 앞(z 0.01)에 있다.
static func _link_split(t) -> void:
	var center := Vector3(0.0, -BONE_LENGTH, 0.01)
	var nearest := {} # 경우마다, link a1 - c1과 구 중심 사이의 거리 (m)
	for split in [false, true]:
		var skeleton := Skeleton3D.new()
		t.root.add_child(skeleton)
		skeleton.add_bone("r")
		for side in [["a", -0.05], ["c", 0.05]]:
			for i in 3:
				var bone := skeleton.add_bone("%s%d" % [side[0], i])
				skeleton.set_bone_parent(bone, 0 if i == 0 else bone - 1)
				skeleton.set_bone_rest(bone, Transform3D(Basis(), Vector3(side[1], 0, 0) if i == 0 else Vector3(0, -BONE_LENGTH, 0)))
				skeleton.set_bone_pose(bone, skeleton.get_bone_rest(bone))
		skeleton.modifier_callback_mode_process = Skeleton3D.MODIFIER_CALLBACK_MODE_PROCESS_MANUAL
		var simulator: Node = ClassDB.instantiate("BoneClothSimulator3D")
		skeleton.add_child(simulator)
		simulator.set("chain_count", 2)
		simulator.set("chains/0/root_bone_name", "a0")
		simulator.set("chains/0/end_bone_name", "a2")
		simulator.set("chains/1/root_bone_name", "c0")
		simulator.set("chains/1/end_bone_name", "c2")
		simulator.set("link_mode", 1)
		simulator.set("link_split", split)
		simulator.set("stiffness/value", 0.2)
		simulator.set("radius/value", 0.01)
		var sphere: Node = ClassDB.instantiate("BoneClothCollisionSphere3D")
		simulator.add_child(sphere)
		sphere.set("position", center)
		sphere.set("radius", 0.03)
		var joints: Array[Vector3] = [Vector3(), Vector3()] # a1, c1 (skeleton 공간)
		skeleton.skeleton_updated.connect(func() -> void:
			joints[0] = skeleton.get_bone_global_pose(2).origin
			joints[1] = skeleton.get_bone_global_pose(5).origin)
		for frame in 60:
			skeleton.advance(1.0 / 60.0)
			await t.process_frame
		nearest[split] = center.distance_to(Geometry3D.get_closest_point_to_segment(center, joints[0], joints[1]))
		skeleton.free()

	t.check(nearest[false] > 0.03, "Link split off: a link across a sphere stays %.4f m from its center." % nearest[false])
	t.check(nearest[true] < 0.02, "Link split: a link across a sphere is pushed out to %.4f m from its center." % nearest[true])


# link_spacing: 두 chain이 2 cm 떨어져 매달려 있다. joint 반지름이 2 cm라서 반지름 합(4 cm)보다 가깝다.
#   r +-- a0 - a1 - a2   (x -0.01)
#     +-- c0 - c1 - c2   (x 0.01)
# 끄면 두 끝은 2 cm 그대로, 켜면 반지름 합 쪽으로 벌어진다.
static func _link_spacing(t) -> void:
	var gap := {} # 경우마다, 60 frame 뒤 두 끝(a2, c2) 사이의 거리 (m)
	for spacing in [0.0, 1.0]:
		var skeleton := Skeleton3D.new()
		t.root.add_child(skeleton)
		skeleton.add_bone("r")
		for side in [["a", -0.01], ["c", 0.01]]:
			for i in 3:
				var bone := skeleton.add_bone("%s%d" % [side[0], i])
				skeleton.set_bone_parent(bone, 0 if i == 0 else bone - 1)
				skeleton.set_bone_rest(bone, Transform3D(Basis(), Vector3(side[1], 0, 0) if i == 0 else Vector3(0, -BONE_LENGTH, 0)))
				skeleton.set_bone_pose(bone, skeleton.get_bone_rest(bone))
		skeleton.modifier_callback_mode_process = Skeleton3D.MODIFIER_CALLBACK_MODE_PROCESS_MANUAL
		var simulator: Node = ClassDB.instantiate("BoneClothSimulator3D")
		skeleton.add_child(simulator)
		simulator.set("chain_count", 2)
		simulator.set("chains/0/root_bone_name", "a0")
		simulator.set("chains/0/end_bone_name", "a2")
		simulator.set("chains/1/root_bone_name", "c0")
		simulator.set("chains/1/end_bone_name", "c2")
		simulator.set("link_mode", 1)
		simulator.set("link_spacing/value", spacing)
		simulator.set("stiffness/value", 0.05)
		simulator.set("radius/value", 0.02)
		var tips: Array[Vector3] = [Vector3(), Vector3()] # a2, c2 (skeleton 공간)
		skeleton.skeleton_updated.connect(func() -> void:
			tips[0] = skeleton.get_bone_global_pose(3).origin
			tips[1] = skeleton.get_bone_global_pose(6).origin)
		for frame in 60:
			skeleton.advance(1.0 / 60.0)
			await t.process_frame
		gap[spacing] = tips[0].distance_to(tips[1])
		skeleton.free()
	
	t.check(absf(gap[0.0] - 0.02) < 0.001, "Link spacing off: two chains 2 cm apart stay %.4f m apart." % gap[0.0])
	t.check(gap[1.0] > 0.03, "Link spacing: two chains 2 cm apart with 2 cm joints are pushed %.4f m apart." % gap[1.0])


# root spring이 있으면 root bone(b0)도 흔들린다.
static func _root_spring(t) -> void:
	var moved := {} # stiffness마다, 움직이는 동안 root가 pose에서 가장 멀리 간 거리 (m)
	var settled := {} # stiffness마다, 멈추고 2초 뒤 root가 pose에서 떨어진 거리 (m)
	for stiffness in [0.0, 0.15]:
		var rig := Rig.new(t)
		rig.simulator.set("root_spring_stiffness", stiffness)
		rig.simulator.set("root_spring_limit", ROOT_SPRING_LIMIT)
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


static func _root_spring_forward(t) -> void:
	var rig := Rig.new(t)
	rig.simulator.set("root_spring_stiffness", 0.15)
	rig.simulator.set("root_spring_limit", ROOT_SPRING_LIMIT)
	rig.simulator.set("root_spring_forward_limit", ROOT_SPRING_FORWARD_LIMIT)
	var down := 0.0 # root가 pose보다 아래(bone 방향)로 간 가장 큰 거리 (m)
	var up := 0.0 # root가 pose보다 위로 간 가장 큰 거리 (m)
	for frame in 120:
		rig.character.position = Vector3(0, sin(float(frame + 1) / 60.0 * 12.0) * 0.1, 0)
		await rig.advance(1.0 / 60.0)
		down = maxf(down, -rig.poses[0].origin.y)
		up = maxf(up, rig.poses[0].origin.y)
	rig.remove()
	
	t.check(down < ROOT_SPRING_FORWARD_LIMIT + LENGTH_TOLERANCE, "Root spring forward limit: the root moves %.4f m along its bone (limit %.3f m)." % [down, ROOT_SPRING_FORWARD_LIMIT])
	t.check(up > ROOT_SPRING_FORWARD_LIMIT * 2.0, "Root spring forward limit: the root moves only %.4f m against its bone." % up)


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
			func(p: Vector3) -> float: return -((p - Vector3(0.0, -0.25, 0.0)).dot(tilt.y) - 0.02)],
		["BoneClothCollisionSphere3D", {"bone_name": "b0", "position_offset": center, "radius": 0.25, "inside": true},
			func(p: Vector3) -> float: return p.distance_to(center) - (0.25 - 0.02)],
		["BoneClothCollisionCapsule3D", {"bone_name": "b0", "position_offset": axis_center, "radius": 0.1, "height": 0.4, "inside": true},
			func(p: Vector3) -> float: return p.distance_to(Vector3(0.03, clampf(p.y, -0.3, -0.1), 0.0)) - (0.1 - 0.02)],
		["BoneClothCollisionTaperedCapsule3D", {"bone_name": "b0", "position_offset": axis_center, "top_radius": 0.1, "bottom_radius": 0.1, "height": 0.4, "inside": true},
			func(p: Vector3) -> float: return p.distance_to(Vector3(0.03, clampf(p.y, -0.3, -0.1), 0.0)) - (0.1 - 0.02)],
	]
	for shape in shapes:
		var rig := Rig.new(t)
		rig.simulator.set("radius/rest_overlap", false)
		var collision: Node = ClassDB.instantiate(shape[0])
		rig.simulator.add_child(collision)
		for property in shape[1]:
			collision.set(property, shape[1][property])
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
	
	var curve := Curve.new()
	curve.add_point(Vector2(0.0, 0.0))
	curve.add_point(Vector2(1.0, 0.0))
	var curved := await _wind_tip(t, 5.0, 1.0, curve, false, 0.0)
	t.check(curved.distance_to(still) < WIND_TOLERANCE, "Wind, curve at 0: the tip moved %.4f m." % curved.distance_to(still))
	
	var hidden := await _wind_tip(t, 5.0, 1.0, null, true, 0.0)
	t.check(hidden.distance_to(still) < WIND_TOLERANCE, "Wind, hidden node: the tip moved %.4f m." % hidden.distance_to(still))
	
	var turned := await _wind_tip(t, 5.0, 1.0, null, false, PI)
	t.check(turned.x > still.x + WIND_MIN, "Wind, turned character: the tip moved %.4f m along local X." % (turned.x - still.x))
	
	var rig := Rig.new(t)
	rig.simulator.set("gravity/value", 0.0)
	rig.simulator.set("root_spring_stiffness", 0.5)
	var wind: Node3D = ClassDB.instantiate("BoneClothWind3D")
	wind.rotation_degrees = Vector3(0, 90, 0)
	rig.stage.add_child(wind)
	for i in 60:
		await rig.advance(1.0 / 60.0)
	var root_moved: float = rig.poses[0].origin.length()
	rig.remove()
	t.check(root_moved < WIND_TOLERANCE, "Wind, root spring: the root moved %.4f m." % root_moved)


static func _wind_frame_rate(t) -> void:
	var tips := {}
	for fps in [30, 60, 120]:
		var rig := Rig.new(t)
		rig.simulator.set("gravity/value", 0.0)
		var wind: Node3D = ClassDB.instantiate("BoneClothWind3D")
		wind.rotation_degrees = Vector3(0, 90, 0)
		rig.stage.add_child(wind)
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
		t.check(largest < FRAME_RATE_TOLERANCE, "Wind, %d fps: the tip is up to %.4f m from 60 fps." % [fps, largest])


static func _wind_direction_noise(t) -> void:
	var drift := {}
	for angle in [0.0, deg_to_rad(45.0)]:
		var rig := Rig.new(t)
		rig.simulator.set("gravity/value", 0.0)
		var wind: Node3D = ClassDB.instantiate("BoneClothWind3D")
		wind.rotation_degrees = Vector3(0, 90, 0)
		wind.set("turbulence", 0.0)
		wind.set("direction_noise_angle", angle)
		wind.set("direction_noise_period", 2.0)
		rig.stage.add_child(wind)
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
		var wind: Node3D = ClassDB.instantiate("BoneClothWind3D")
		wind.rotation_degrees = Vector3(0, 90, 0)
		wind.set("turbulence", 0.0)
		rig.stage.add_child(wind)
		for i in 120:
			await rig.advance(1.0 / 60.0)
		tips.append(rig.tip())
		rig.remove()
	t.check(tips[0].distance_to(tips[1]) < SCALE_TOLERANCE, "Wind, cm rig: the tip is %.5f m from the m rig." % tips[0].distance_to(tips[1]))


static func _wind_tip(t, p_strength: float, p_influence: float, p_curve: Curve, p_hidden: bool, p_turn: float) -> Vector3:
	var rig := Rig.new(t)
	rig.simulator.set("gravity/value", 0.0)
	rig.simulator.set("wind_influence/value", p_influence)
	rig.simulator.set("wind_influence/damping_curve", p_curve)
	rig.character.rotation.y = p_turn
	if p_strength > 0.0:
		var wind: Node3D = ClassDB.instantiate("BoneClothWind3D")
		wind.rotation_degrees = Vector3(0, 90, 0)
		wind.set("strength", p_strength)
		wind.set("turbulence", 0.0)
		wind.visible = not p_hidden
		rig.stage.add_child(wind)
	for i in 120:
		await rig.advance(1.0 / 60.0)
	var tip := rig.tip()
	rig.remove()
	return tip


static func _preset_round_trip(t) -> void:
	var names: Array[String] = []
	for property in ClassDB.class_get_property_list("BoneClothPreset", true):
		if property.usage & PROPERTY_USAGE_STORAGE:
			names.append(property.name)
	t.check(names.size() == 35, "Preset round trip: the preset stores %d values." % names.size())
	
	var source: Node = ClassDB.instantiate("BoneClothSimulator3D")
	var target: Node = ClassDB.instantiate("BoneClothSimulator3D")
	var defaults: Resource = ClassDB.instantiate("BoneClothPreset")
	var simulator_names: Array[String] = []
	for property in source.get_property_list():
		simulator_names.append(property.name)
	for name in names:
		t.check(name in simulator_names, "Preset round trip: the simulator has no %s." % name)
		t.check(source.get(name) == defaults.get(name), "Preset round trip, %s: the simulator starts at %s, the preset at %s." % [name, source.get(name), defaults.get(name)])
		var value: Variant = source.get(name)
		match typeof(value):
			TYPE_BOOL:
				source.set(name, not value)
			TYPE_FLOAT:
				source.set(name, 0.5 * value + 0.01)
			TYPE_VECTOR3:
				source.set(name, Vector3(0.0, 0.0, -1.0))
			TYPE_OBJECT, TYPE_NIL:
				var curve := Curve.new()
				curve.add_point(Vector2(0.0, 0.25))
				source.set(name, curve)

	var preset: Resource = source.call("make_preset")
	target.call("apply_preset", preset)
	for name in names:
		var expected: Variant = source.get(name)
		var actual: Variant = target.get(name)
		if expected is Curve:
			t.check(actual is Curve and actual != expected and actual.sample(0.0) == expected.sample(0.0), "Preset round trip, %s: the curve did not arrive as its own copy." % name)
		else:
			t.check(actual == expected, "Preset round trip, %s: %s arrived as %s." % [name, expected, actual])
	source.free()
	target.free()
