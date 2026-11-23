extends Node3D

@export var shown := 0.04

var _left := 0.0
@onready var _shot: AudioStreamPlayer3D = $Shot


func _ready() -> void:
	visible = false


func flash() -> void:
	rotation.z = randf() * TAU
	scale = Vector3.ONE * randf_range(0.8, 1.2)
	visible = true
	_left = shown
	_shot.play()


func _process(delta: float) -> void:
	if _left <= 0.0:
		return
	_left -= delta
	if _left <= 0.0:
		visible = false
