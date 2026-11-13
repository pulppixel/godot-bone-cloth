extends Node3D
const Player := preload("res://demo/scripts/player.gd")
const CharacterSkin := preload("res://demo/scripts/skin.gd")


@export var characters: Array[Node3D] = []
@export var player: CharacterBody3D
@export var character_buttons: Array[Button] = []
@export var cloth_button: CheckButton
@export var time_slider: HSlider
@export var time_value: Label
@export var wind: BoneClothWind3D
@export var wind_angle_slider: HSlider
@export var wind_angle_value: Label
@export var anchor_inertia_slider: HSlider
@export var anchor_inertia_value: Label
@export var wind_direction_slider: HSlider
@export var wind_direction_value: Label

var _character := -1
var _cloth := true


func _ready() -> void:
	for character in characters:
		(character as CharacterSkin).body = player
	for i in character_buttons.size():
		character_buttons[i].pressed.connect(func() -> void: _choose(i, _cloth))
	cloth_button.toggled.connect(func(on: bool) -> void: _choose(_character, on))
	time_slider.value_changed.connect(func(value: float) -> void: Engine.time_scale = value)
	wind_angle_slider.set_value_no_signal(rad_to_deg(wind.direction_noise_angle))
	wind_angle_slider.value_changed.connect(func(value: float) -> void: wind.direction_noise_angle = deg_to_rad(value))
	wind_direction_slider.set_value_no_signal(wrapf(rad_to_deg(wind.rotation.y), 0.0, 360.0))
	wind_direction_slider.value_changed.connect(func(value: float) -> void: wind.rotation.y = deg_to_rad(value))
	anchor_inertia_slider.value_changed.connect(_set_anchor_inertia)
	_set_anchor_inertia(anchor_inertia_slider.value)
	_show(0)


func _process(_delta: float) -> void:
	time_slider.set_value_no_signal(Engine.time_scale)
	time_value.text = "%.1fx" % Engine.time_scale
	wind_angle_value.text = "%d deg" % roundi(wind_angle_slider.value)
	anchor_inertia_value.text = "%.2f" % anchor_inertia_slider.value
	wind_direction_value.text = "%d deg" % roundi(wind_direction_slider.value)


func _unhandled_input(event: InputEvent) -> void:
	var key := event as InputEventKey
	if key == null or not key.pressed or key.echo:
		return
	
	var index := key.physical_keycode - KEY_1
	if index >= 0 and index < characters.size():
		_show(index)


func _set_anchor_inertia(value: float) -> void:
	for character in characters:
		for simulator: BoneClothSimulator3D in character.find_children("*", "BoneClothSimulator3D", true, false):
			simulator.anchor_inertia = value


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
		(player as Player).use(characters[character])
