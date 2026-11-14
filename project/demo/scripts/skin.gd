extends Node3D


const Feet := preload("res://demo/scripts/feet.gd")
const Look := preload("res://demo/scripts/look.gd")
const Face := preload("res://demo/scripts/face.gd")

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

var half := 0.0

var state: StringName:
	get:
		return tree.get(STATE_NOW)


func _enter_tree() -> void:
	tree = $AnimationTree
	skeleton = find_children("*", "Skeleton3D", true, false)[0]
	move = (tree.tree_root as AnimationNodeBlendTree).get_node(&"Move")
	face = find_child("Face", true, false)


func _ready() -> void:
	tree.set(LEAN_ONLY, 1.0)
	tree.set(LEAN_ADD, 1.0)
	look.glanced.connect(expression.blink)


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
