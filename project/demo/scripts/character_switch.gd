extends Node3D


@export var characters: Array[Node3D] = []
@export var player: CharacterBody3D
@export var character_buttons: Array[Button] = []
@export var cloth_button: CheckButton
@export var time_slider: HSlider
@export var time_value: Label

var _character := -1
var _cloth := true


func _ready() -> void:
	for i in character_buttons.size():
		character_buttons[i].pressed.connect(func() -> void: _choose(i, _cloth))
	cloth_button.toggled.connect(func(on: bool) -> void: _choose(_character, on))
	time_slider.value_changed.connect(func(value: float) -> void: Engine.time_scale = value)
	_show(0)


func _process(_delta: float) -> void:
	time_slider.set_value_no_signal(Engine.time_scale)
	time_value.text = "%.1fx" % Engine.time_scale


func _unhandled_input(event: InputEvent) -> void:
	var key := event as InputEventKey
	if key == null or not key.pressed or key.echo:
		return
	
	var index := key.physical_keycode - KEY_1
	if index >= 0 and index < characters.size():
		_show(index)


func _show(index: int) -> void:
	_choose(index, _cloth)


func _choose(character: int, cloth: bool) -> void:
	var changed := character != _character
	_character = character
	_cloth = cloth
	for i in characters.size():
		var shown := i == character
		characters[i].visible = shown
		characters[i].process_mode = Node.PROCESS_MODE_INHERIT if shown else Node.PROCESS_MODE_DISABLED
		character_buttons[i].set_pressed_no_signal(shown)
	cloth_button.set_pressed_no_signal(cloth)
	for simulator: BoneClothSimulator3D in characters[character].find_children("*", "BoneClothSimulator3D", true, false):
		if cloth and not simulator.active:
			simulator.reset()
		simulator.active = cloth
	if changed:
		player.call(&"use", characters[character])
