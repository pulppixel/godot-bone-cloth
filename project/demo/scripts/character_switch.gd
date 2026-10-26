extends Node3D


@export var characters: Array[Node3D] = []
@export var player: CharacterBody3D


func _ready() -> void:
	_show(0)


func _unhandled_input(event: InputEvent) -> void:
	var key := event as InputEventKey
	if key == null or not key.pressed or key.echo:
		return
	
	var index := key.physical_keycode - KEY_1
	if index >= 0 and index < characters.size():
		_show(index)


func _show(index: int) -> void:
	for i in characters.size():
		var shown := i == index
		characters[i].visible = shown
		characters[i].process_mode = Node.PROCESS_MODE_INHERIT if shown else Node.PROCESS_MODE_DISABLED
	player.call(&"use", characters[index])
