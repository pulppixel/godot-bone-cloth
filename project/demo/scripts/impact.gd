extends Node3D

@export var flash_time := 0.08
@export var ring_time := 0.18
@export var mark_time := 3.0

@onready var _flash: MeshInstance3D = $Flash
@onready var _ring: MeshInstance3D = $Ring
@onready var _light: OmniLight3D = $Light
@onready var _mark: Decal = $Mark


func _ready() -> void:
	$Sparks.emitting = true
	$Smoke.emitting = true
	_mark.rotate_y(randf() * TAU)
	var tween := create_tween().set_parallel()
	tween.tween_property(_flash, "scale", Vector3.ONE * 0.3, flash_time).from(Vector3.ONE)
	tween.tween_property(_flash, "transparency", 1.0, flash_time)
	tween.tween_property(_light, "light_energy", 0.0, flash_time)
	tween.tween_property(_ring, "scale", Vector3.ONE * 1.2, ring_time).from(Vector3.ONE * 0.2).set_ease(Tween.EASE_OUT).set_trans(Tween.TRANS_CUBIC)
	tween.tween_property(_ring, "transparency", 1.0, ring_time)
	tween.tween_property(_mark, "modulate:a", 0.0, 1.0).set_delay(mark_time)
	tween.chain().tween_callback(queue_free)
