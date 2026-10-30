extends Node

@export_enum("calm", "bright") var style := "calm"

const STYLES := {
	"calm": {
		"idle": {"Fcl_ALL_Fun": 0.2},
		"thought": {"Fcl_ALL_Fun": 0.1, "Fcl_BRW_Surprised": 0.3, "Fcl_MTH_Fun": 0.2},
		"walk": {"Fcl_EYE_Natural": 0.3},
		"run": {"Fcl_BRW_Angry": 0.4, "Fcl_EYE_Angry": 0.2},
		"jump_start": {"Fcl_BRW_Surprised": 0.35, "Fcl_EYE_Surprised": 0.25, "Fcl_MTH_O": 0.25},
		"fall": {"Fcl_ALL_Surprised": 0.45, "Fcl_MTH_O": 0.15},
		"impact": {"Fcl_EYE_Close": 0.55, "Fcl_BRW_Angry": 0.4, "Fcl_MTH_Angry": 0.4},
		"relief": {"Fcl_ALL_Fun": 0.3},
	},
	"bright": {
		"idle": {"Fcl_ALL_Fun": 0.45},
		"thought": {"Fcl_ALL_Fun": 0.3, "Fcl_BRW_Surprised": 0.4, "Fcl_MTH_Fun": 0.4},
		"walk": {"Fcl_MTH_Fun": 0.35, "Fcl_EYE_Natural": 0.3},
		"run": {"Fcl_BRW_Angry": 0.25, "Fcl_EYE_Angry": 0.1, "Fcl_MTH_Fun": 0.15},
		"jump_start": {"Fcl_BRW_Surprised": 0.45, "Fcl_EYE_Surprised": 0.3, "Fcl_MTH_O": 0.35},
		"fall": {"Fcl_ALL_Surprised": 0.7, "Fcl_MTH_O": 0.2},
		"impact": {"Fcl_EYE_Close": 0.55, "Fcl_BRW_Angry": 0.4, "Fcl_MTH_Angry": 0.4},
		"relief": {"Fcl_ALL_Joy": 0.4, "Fcl_MTH_Fun": 0.3},
	},
}

const FADE := {"idle": 0.35, "thought": 0.4, "walk": 0.25, "run": 0.25, "jump_start": 0.08, "fall": 0.15, "impact": 0.06, "relief": 0.3}

const WALK_SPEED := 1.064
const RUN_SPEED := 7.65

const IMPACT_TIME := 0.12
const RELIEF_TIME := 0.8
const THOUGHT_EVERY := Vector2(4.0, 8.0)

const BREATH_PERIOD := 0.667
const BREATH := Vector2(0.12, 0.4)

const BLINK_EVERY := Vector2(1.5, 6.0)
const BLINK_CLOSE := 0.08
const BLINK_HOLD := 0.04
const BLINK_OPEN := 0.18
const BLINK_COOLDOWN := 1.0
const EYE_CLOSERS := ["Fcl_ALL_Joy", "Fcl_EYE_Joy", "Fcl_EYE_Close"]

var _tree: AnimationTree
var _face: MeshInstance3D
var _shapes: Dictionary[String, int] = {}
var _weights: Dictionary[String, float] = {}
var _key := "idle"
var _time := 0.0
var _land_time := -100.0
var _last_state: StringName
var _thought := false
var _next_thought := 0.0
var _blink_start := -100.0
var _next_blink := 0.0


func _ready() -> void:
	_tree = get_parent().get_node("AnimationTree")
	_face = get_parent().find_child("Face", true, false)
	for key in STYLES[style]:
		for shape in STYLES[style][key]:
			_shapes[shape] = -1
	_shapes["Fcl_MTH_A"] = -1
	_shapes["Fcl_EYE_Close"] = -1
	for shape in _shapes:
		_shapes[shape] = _face.find_blend_shape_by_name(shape)
		if _shapes[shape] < 0:
			push_error("%s: 얼굴에 blend shape %s가 없다." % [get_parent().name, shape])
			set_process(false)
			return
		_weights[shape] = 0.0
	_next_thought = randf_range(THOUGHT_EVERY.x, THOUGHT_EVERY.y)
	_next_blink = randf_range(BLINK_EVERY.x, BLINK_EVERY.y)


func _process(delta: float) -> void:
	_time += delta
	var state: StringName = _tree.get("parameters/State/current_state")
	var speed: float = _tree.get("parameters/Move/blend_position") * _tree.get("parameters/MoveRate/scale")
	if state == &"land" and _last_state != &"land":
		_land_time = _time
	_last_state = state

	# 1. 상태 얼굴을 고른다.
	var key := String(state) if STYLES[style].has(String(state)) else "idle"
	var since_land := _time - _land_time
	var run := 0.0 # 달리기 얼굴의 비율 (0 걷기, 1 달리기)
	if state == &"move":
		run = clampf((speed - WALK_SPEED) / (RUN_SPEED - WALK_SPEED), 0.0, 1.0)
		key = "run" if run >= 0.5 else "walk"
	elif (state == &"land" or state == &"idle") and since_land < IMPACT_TIME:
		key = "impact"
	elif (state == &"land" or state == &"idle") and since_land < IMPACT_TIME + RELIEF_TIME:
		key = "relief"
	elif state == &"idle":
		if _time >= _next_thought:
			_thought = not _thought
			_next_thought = _time + randf_range(THOUGHT_EVERY.x, THOUGHT_EVERY.y)
		key = "thought" if _thought else "idle"
	if key != _key:
		_key = key
		_request_blink()

	# 2. 지금 얼굴을 그 얼굴 쪽으로 옮긴다.
	var faces: Dictionary = STYLES[style]
	var target: Dictionary = {}
	if key == "walk" or key == "run":
		for shape in faces["walk"]:
			target[shape] = target.get(shape, 0.0) + faces["walk"][shape] * (1.0 - run)
		for shape in faces["run"]:
			target[shape] = target.get(shape, 0.0) + faces["run"][shape] * run
	else:
		target = faces[key]
	var step: float = delta / FADE[key]
	for shape in _weights:
		_weights[shape] = move_toward(_weights[shape], target.get(shape, 0.0), step)

	# 3. 호흡을 더한다.
	var values := _weights.duplicate()
	var breath := 0.5 - 0.5 * cos(TAU * _time / BREATH_PERIOD)
	values["Fcl_MTH_A"] += run * lerpf(BREATH.x, BREATH.y, breath)

	# 4. 눈 깜빡임을 더한다.
	if _time >= _next_blink:
		_blink_start = _time
		_next_blink = _time + randf_range(BLINK_EVERY.x, BLINK_EVERY.y)
	var closed := 0.0
	for shape in EYE_CLOSERS:
		closed += values.get(shape, 0.0)
	values["Fcl_EYE_Close"] += _blink_amount(_time - _blink_start) * (1.0 - clampf(closed, 0.0, 1.0))

	for shape in values:
		_face.set_blend_shape_value(_shapes[shape], clampf(values[shape], 0.0, 1.0))


func _request_blink() -> void:
	if _time - _blink_start >= BLINK_COOLDOWN:
		_blink_start = _time
		_next_blink = _time + randf_range(BLINK_EVERY.x, BLINK_EVERY.y)


func _blink_amount(t: float) -> float:
	if t < 0.0 or t > BLINK_CLOSE + BLINK_HOLD + BLINK_OPEN:
		return 0.0
	if t < BLINK_CLOSE:
		return t / BLINK_CLOSE
	if t < BLINK_CLOSE + BLINK_HOLD:
		return 1.0
	return 1.0 - (t - BLINK_CLOSE - BLINK_HOLD) / BLINK_OPEN


func blink() -> void:
	_request_blink()
