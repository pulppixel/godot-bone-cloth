extends Node3D

@export var gun: PackedScene

var _shown: Node3D


func _ready() -> void:
	add_to_group(&"gun_slots")
	put_back()


func is_full() -> bool:
	return _shown != null


func take() -> void:
	_shown.queue_free()
	_shown = null


func put_back() -> void:
	_shown = gun.instantiate()
	add_child(_shown)
