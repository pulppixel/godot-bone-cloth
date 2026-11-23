extends Node3D


const Feet := preload("res://demo/scripts/feet.gd")
const Look := preload("res://demo/scripts/look.gd")
const Face := preload("res://demo/scripts/face.gd")
const GUN := preload("res://demo/shooter/blaster.glb")
const MuzzleFlash := preload("res://demo/shooter/muzzle_flash.tscn")
const Bolt := preload("res://demo/shooter/bolt.tscn")
const GUN_GRIP := Transform3D(Vector3(0.016, 0.0505, -0.5977), Vector3(0.5955, 0.0704, 0.0219), Vector3(0.0719, -0.5937, -0.0483), Vector3(0.0228, 0.1758, 0.0229))

const STATE := &"parameters/State/transition_request"
const STATE_NOW := &"parameters/State/current_state"
const REST := &"parameters/Rest/transition_request"
const MOVE_AT := &"parameters/Move/blend_position"
const MOVE_POSITION := &"parameters/Move/current_position"
const MOVE_LENGTH := &"parameters/Move/current_length"
const MOVE_DELTA := &"parameters/Move/current_delta"
const MOVE_RATE := &"parameters/MoveRate/scale"
const SIDE := &"parameters/Side/blend_amount"
const LEAN_ONLY := &"parameters/LeanOnly/sub_amount"
const LEAN_ADD := &"parameters/Lean/add_amount"
const LEAN_PHASE := &"parameters/LeanPhase/seek_request"
const IDLE_FOOT := &"parameters/IdleFoot/blend_amount"
const EXIT_FOOT := &"parameters/ExitFoot/blend_amount"
const BOARD_CROUCH := &"parameters/Board/blend_amount"
const AIM := &"parameters/Aim/blend_amount"
const AIM_PITCH := &"parameters/AimPitch/blend_amount"
const STRAFE := &"parameters/Strafe/blend_position"
const PITCH_RANGE := deg_to_rad(70.0)
const HOLD := Transform3D(Vector3(-0.0309, 0.9923, -0.1198), Vector3(-0.0848, 0.1169, 0.9895), Vector3(0.9959, 0.0408, 0.0805), Vector3(-0.1654, 0.0404, 0.2557))
const LEFT_GRIP := Transform3D(Vector3(-0.9986, 0.0045, -0.0531), Vector3(-0.0045, 0.9858, 0.1678), Vector3(0.0531, 0.1678, -0.9844), Vector3(-0.012, 0.1744, 0.027))
const MUZZLE := Vector3(0.0, 0.039, -0.29)
const KICK_MAX := 1.5
const POLES: Array[Vector3] = [Vector3(-0.5, 0.8, 0.2), Vector3(0.5, 0.8, 0.2)]
const ARMS := [["RightUpperArm", "RightLowerArm", "RightHand"], ["LeftUpperArm", "LeftLowerArm", "LeftHand"]]

var tree: AnimationTree
var skeleton: Skeleton3D
var move: AnimationNodeBlendSpace1D
var face: MeshInstance3D
var body: CharacterBody3D
@onready var feet: Feet = $Feet
@onready var look: Look = $Look
@onready var expression: Face = $Expression
@onready var lean_length: float = tree.get_animation(&"demo_motions/Jog_Fwd_LeanR").length
@onready var exit_length: float = tree.get_animation(&"demo_motions/Sprint_Exit").length
@onready var sprint_at: float = move.get_blend_point_position(1)
@onready var land_end: float = tree.get_animation(&"demo_motions/Jump_Land").length - (tree.tree_root as AnimationNodeBlendTree).get_node(&"State").xfade_time

var half := 0.0
var aim := 0.0
var _grip: HandGrip
var _arms: TwoBoneIK3D
var _hands: CopyTransformModifier3D
var _muzzle: Node3D
var state: StringName:
	get:
		return tree.get(STATE_NOW)


class HandGrip extends SkeletonModifier3D:
	const KICK_TIME := 0.1
	const KICK_RISE := deg_to_rad(14.0)
	const KICK_BACK := 0.07
	const KICK_BEND := deg_to_rad(7.0)
	
	var chest := -1
	var spine := -1
	var pitch := 0.0
	var kick := 0.0
	var upright := Quaternion.IDENTITY
	var hold := Transform3D()
	var left_grip := Transform3D()
	var targets: Array[Node3D] = []
	
	func _process_modification_with_delta(delta: float) -> void:
		var skeleton := get_skeleton()
		kick *= exp(-delta / KICK_TIME)
		var parent := skeleton.get_bone_global_pose(skeleton.get_bone_parent(spine)).basis.get_rotation_quaternion()
		var turn := upright * parent.inverse()
		var untwist := Quaternion(0.0, turn.y, 0.0, turn.w).normalized()
		var bent := Quaternion(Vector3.RIGHT, -kick * KICK_BEND) * untwist * skeleton.get_bone_global_pose(spine).basis.get_rotation_quaternion()
		skeleton.set_bone_pose_rotation(spine, parent.inverse() * bent)
		var at := skeleton.global_transform * skeleton.get_bone_global_pose(chest).origin
		at += skeleton.global_basis * Vector3(0.0, 0.0, -kick * KICK_BACK)
		var right := Transform3D(skeleton.global_basis * Basis(Vector3.RIGHT, -pitch - kick * KICK_RISE), at) * hold
		targets[0].global_transform = right
		targets[1].global_transform = right * left_grip


func _enter_tree() -> void:
	tree = $AnimationTree
	skeleton = find_children("*", "Skeleton3D", true, false)[0]
	move = (tree.tree_root as AnimationNodeBlendTree).get_node(&"Move")
	face = find_child("Face", true, false)


func _ready() -> void:
	tree.set(LEAN_ONLY, 1.0)
	tree.set(LEAN_ADD, 1.0)
	look.glanced.connect(expression.blink)
	var hand := BoneAttachment3D.new()
	hand.bone_name = &"RightHand"
	skeleton.add_child(hand)
	var gun := GUN.instantiate() as Node3D
	gun.transform = GUN_GRIP
	hand.add_child(gun)
	_muzzle = MuzzleFlash.instantiate()
	_muzzle.position = MUZZLE
	gun.add_child(_muzzle)
	_hold_with_both_hands()


func _hold_with_both_hands() -> void:
	_grip = HandGrip.new()
	_grip.chest = skeleton.find_bone(&"UpperChest")
	_grip.spine = skeleton.find_bone(&"Chest")
	_grip.upright = _pose_in(&"demo_motions/Idle", skeleton.get_bone_parent(_grip.spine))
	_grip.hold = HOLD
	_grip.left_grip = LEFT_GRIP
	_arms = TwoBoneIK3D.new()
	_hands = CopyTransformModifier3D.new()
	var cloth := skeleton.get_child_count()
	for child in skeleton.get_children():
		if child is BoneClothSimulator3D:
			cloth = child.get_index()
			break
	for modifier in [_grip, _arms, _hands]:
		skeleton.add_child(modifier)
		skeleton.move_child(modifier, cloth)
		cloth += 1
	_arms.setting_count = ARMS.size()
	_hands.setting_count = ARMS.size()
	for i in ARMS.size():
		var target := Node3D.new()
		var pole := Node3D.new()
		for marker in [target, pole]:
			marker.visible = false
			marker.physics_interpolation_mode = Node.PHYSICS_INTERPOLATION_MODE_OFF
			add_child(marker)
		pole.position = POLES[i]
		_grip.targets.append(target)
		_arms.set_root_bone_name(i, ARMS[i][0])
		_arms.set_middle_bone_name(i, ARMS[i][1])
		_arms.set_end_bone_name(i, ARMS[i][2])
		_arms.set_target_node(i, _arms.get_path_to(target))
		_arms.set_pole_node(i, _arms.get_path_to(pole))
		_hands.set_apply_bone_name(i, ARMS[i][2])
		_hands.set_reference_type(i, BoneConstraint3D.REFERENCE_TYPE_NODE)
		_hands.set_reference_node(i, _hands.get_path_to(target))
		_hands.set_copy_position(i, false)
		_hands.set_copy_scale(i, false)
	set_aim(0.0, 0.0)


func _pose_in(p_clip: StringName, p_bone: int) -> Quaternion:
	var clip := tree.get_animation(p_clip)
	var chain: Array[int] = []
	while p_bone >= 0:
		chain.push_front(p_bone)
		p_bone = skeleton.get_bone_parent(p_bone)
	var pose := Quaternion.IDENTITY
	for bone in chain:
		var track := clip.find_track(NodePath("%GeneralSkeleton:" + skeleton.get_bone_name(bone)), Animation.TYPE_ROTATION_3D)
		pose *= clip.rotation_track_interpolate(track, 0.0) if track >= 0 else skeleton.get_bone_rest(bone).basis.get_rotation_quaternion()
	return pose


func request(p_state: StringName) -> void:
	tree.set(STATE, p_state)


func rest(p_kind: StringName) -> void:
	tree.set(REST, p_kind)


func set_idle_foot(p_right: bool) -> void:
	tree.set(IDLE_FOOT, 1.0 if p_right else 0.0)


func idle_foot_right() -> bool:
	return tree.get(IDLE_FOOT) > 0.5


func set_exit_foot(p_foot: int) -> void:
	tree.set(EXIT_FOOT, float(p_foot))
	tree.set(IDLE_FOOT, float(p_foot))


func set_move(p_clip_speed: float, p_rate: float) -> void:
	tree.set(MOVE_RATE, p_rate)
	tree.set(MOVE_AT, p_clip_speed)


func set_lean(p_amount: float) -> void:
	tree.set(SIDE, p_amount)


func set_board_crouch(p_amount: float) -> void:
	tree.set(BOARD_CROUCH, p_amount)


func set_aim(p_amount: float, p_pitch: float) -> void:
	aim = p_amount
	_grip.pitch = p_pitch
	tree.set(AIM, p_amount)
	for modifier: SkeletonModifier3D in [_grip, _arms, _hands]:
		modifier.active = p_amount > 0.0
		modifier.influence = p_amount
	tree.set(AIM_PITCH, clampf(p_pitch / PITCH_RANGE, -1.0, 1.0))


func muzzle() -> Vector3:
	return _muzzle.global_position


func shoot(p_at: Vector3, p_normal: Vector3) -> void:
	_muzzle.flash()
	_grip.kick = minf(_grip.kick + 1.0, KICK_MAX)
	var bolt := Bolt.instantiate()
	body.add_sibling(bolt)
	bolt.launch(_muzzle.global_position, p_at, p_normal)


func set_strafe(p_at: Vector2) -> void:
	tree.set(STRAFE, p_at)


func seek_lean(p_phase: float) -> void:
	tree.set(LEAN_PHASE, p_phase * lean_length)


func start_cycle(p_right_foot: bool) -> void:
	half = 0.5 if p_right_foot else 0.0
	for i in move.get_blend_point_count():
		var node := move.get_blend_point_node(i) as AnimationNodeAnimation
		var length := tree.get_animation(node.animation).length
		node.use_custom_timeline = true
		node.timeline_length = length
		node.loop_mode = Animation.LOOP_LINEAR
		node.start_offset = length * half


func cycle_length() -> float:
	return tree.get(MOVE_LENGTH)


func phase(p_ahead := false) -> float:
	var at: float = tree.get(MOVE_POSITION)
	if p_ahead:
		at += tree.get(MOVE_DELTA)
	return fposmod(at / cycle_length() + half, 1.0)


func clip_speed() -> float:
	return float(tree.get(MOVE_AT)) * float(tree.get(MOVE_RATE))


func clip_count() -> int:
	return move.get_blend_point_count()


func clip(p_index: int) -> Animation:
	return tree.get_animation((move.get_blend_point_node(p_index) as AnimationNodeAnimation).animation)


func blend_weights() -> PackedFloat64Array:
	var at: float = tree.get(MOVE_AT)
	var count := move.get_blend_point_count()
	var weights := PackedFloat64Array()
	weights.resize(count)
	if at <= move.get_blend_point_position(0):
		weights[0] = 1.0
	elif at >= move.get_blend_point_position(count - 1):
		weights[count - 1] = 1.0
	else:
		for p in count - 1:
			var a := move.get_blend_point_position(p)
			var b := move.get_blend_point_position(p + 1)
			if at >= a and at <= b:
				weights[p + 1] = (at - a) / (b - a)
				weights[p] = 1.0 - weights[p + 1]
	return weights
