extends Node3D


const MOTIONS := [
	[&"demo_motions/Idle", 0.0, 0.0],
	[&"demo_motions/Walk", 1.064, 0.0],
	[&"demo_motions/Jog_Fwd", 5.844, 4.0],
]

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
		var skeleton: Skeleton3D = character.find_children("*", "Skeleton3D", true, false)[0]
		player.play(clip, BLEND_TIME)
		character.set("speed", speed * skeleton.motion_scale)
		character.set("circle_radius", circle_radius)
	print("Motion ", clip, ", speed ", speed, ", circle ", circle_radius)
