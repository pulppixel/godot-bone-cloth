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


# t: run_tests.gd. check와 skip으로 결과를 센다.
static func run(t) -> void:
	await _frame_rate(t)
	await _teleport(t)
	await _limits_and_lengths(t)
	await _long_step(t)
	await _scale(t)
	await _bone_collision(t)
	await _depth_inertia(t)


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


# frame 하나가 5초 걸려도(로딩 멈춤 등) 값이 망가지지 않는다.
# 캐릭터는 그 frame에 옮기지 않는다. 옮기면 teleport로 처리되어 다른 길을 검사하게 된다.
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
# 구를 b1과 b2 사이 옆에 둔다. 두 joint는 구 중심에서 7.1 cm라 밖에 있고(구 4 cm + joint 2 cm = 6 cm), 그 사이 bone 가운데는 5 cm라 구 안을 지난다.
# 구는 root bone(b0)에 붙여서, chain이 흔들려도 같은 자리에 있다.
static func _bone_collision(t) -> void:
	var rig := Rig.new(t)
	rig.simulator.set("gravity/value", 0.0)
	var sphere: Node = ClassDB.instantiate("BoneClothCollisionSphere3D")
	rig.simulator.add_child(sphere)
	sphere.set("bone_name", "b0")
	sphere.set("position_offset", Vector3(0.05, -0.15, 0.0))
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


# depth_inertia만큼 root 근처 joint는 root의 움직임을 그대로 따라간다.
# root bone(b0)을 skeleton 안에서 좌우로 흔든다(달릴 때 골반이 흔들리는 것처럼). 캐릭터는 움직이지 않는다.
# root 다음 joint(b1)가 제자리(root에서 rest만큼 아래)에서 벗어난 거리의 평균을 depth_inertia 0과 1에서 비교한다.
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
