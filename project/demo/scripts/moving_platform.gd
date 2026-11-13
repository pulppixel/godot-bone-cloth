extends Node3D

@export var end: Marker3D
@export var pause := 1.5
@export var move_time := 2.0

@onready var _body: AnimatableBody3D = $Body


func _ready() -> void:
	var tween := create_tween().set_loops().set_process_mode(Tween.TWEEN_PROCESS_PHYSICS).set_trans(Tween.TRANS_SINE).set_ease(Tween.EASE_IN_OUT)
	tween.tween_property(_body, "position", end.position, move_time).set_delay(pause)
	tween.tween_property(_body, "position", Vector3.ZERO, move_time).set_delay(pause)
