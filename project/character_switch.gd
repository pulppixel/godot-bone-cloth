extends Node3D
# Test only: keys 1 to 9 show that character and hide the others. A hidden character's process mode is Disabled, so its
# animation, test_mover and cloth wait where they were until it is shown again. The orbit camera follows the shown one.
# Tab steps every character to the next motion. Each motion carries the test_mover speed its legs walk at,
# measured from the planted foot sliding back with the mover off, so the feet do not slide, and the circle it runs round
# (0 is the runway). The run goes round a circle: a U-turn at that speed reads as a catwalk.

const MOTIONS := [
	[&"rpg_motions/Stand_Idle_A_1", 0.0, 0.0],
	[&"rpg_motions/Walk_A_F", 1.9, 0.0],
	[&"rpg_motions/Run_A_F", 5.1, 4.0],
]
# Seconds the AnimationPlayer blends into the next motion, so the cloth is not handed a pose that jumps in one frame.
const BLEND_TIME := 0.2

@export var characters: Array[Node3D] = []
@export var camera: Camera3D

var _motion := 0


func _ready() -> void:
	_show(0)
	_play_motion()


func _unhandled_input(event: InputEvent) -> void:
	var key := event as InputEventKey
	if key == null or not key.pressed or key.echo:
		return

	if key.physical_keycode == KEY_TAB:
		_motion = (_motion + 1) % MOTIONS.size()
		_play_motion()
		return

	var index := key.physical_keycode - KEY_1
	if index >= 0 and index < characters.size():
		_show(index)


func _show(index: int) -> void:
	for i in characters.size():
		var shown := i == index
		characters[i].visible = shown
		characters[i].process_mode = Node.PROCESS_MODE_INHERIT if shown else Node.PROCESS_MODE_DISABLED
	camera.set("target", characters[index])


func _play_motion() -> void:
	var clip: StringName = MOTIONS[_motion][0]
	var speed: float = MOTIONS[_motion][1]
	var circle_radius: float = MOTIONS[_motion][2]
	for character in characters:
		var player: AnimationPlayer = character.get_node("AnimationPlayer")
		player.play(clip, BLEND_TIME)
		character.set("speed", speed)
		character.set("circle_radius", circle_radius)
	print("Motion ", clip, ", speed ", speed, ", circle ", circle_radius)
