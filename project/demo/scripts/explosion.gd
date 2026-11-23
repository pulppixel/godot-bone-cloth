extends Node3D

@export var light_time := 0.3
@export var gust := 40.0
@export var gust_range := 8.0
@export var gust_rise := 0.08
@export var gust_fall := 0.5
@export var mark_time := 8.0

@onready var _light: OmniLight3D = $Light
@onready var _wind: BoneClothWind3D = $Gust
@onready var _mark: Decal = $Mark


func _ready() -> void:
	for particles: GPUParticles3D in find_children("*", "GPUParticles3D"):
		particles.emitting = true
	_mark.rotate_y(randf() * TAU)
	var away := get_viewport().get_camera_3d().global_position - global_position
	away.y = 0.0
	var strength := gust * clampf(1.0 - away.length() / gust_range, 0.0, 1.0)
	_wind.look_at(global_position + away)
	_wind.strength = 0.0
	var tween := create_tween()
	tween.tween_property(_light, "light_energy", 0.0, light_time)
	tween.parallel().tween_property(_wind, "strength", strength, gust_rise)
	tween.tween_property(_wind, "strength", 0.0, gust_fall)
	tween.tween_property(_mark, "modulate:a", 0.0, 1.0).set_delay(mark_time)
	tween.tween_callback(queue_free)
