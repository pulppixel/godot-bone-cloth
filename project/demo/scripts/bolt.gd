extends Node3D

const Impact := preload("res://demo/shooter/impact.tscn")

@export var speed := 90.0

var _to := Vector3.ZERO
var _normal := Vector3.ZERO


func launch(p_from: Vector3, p_to: Vector3, p_normal: Vector3) -> void:
	_to = p_to
	_normal = p_normal
	look_at_from_position(p_from, p_to)


func _process(delta: float) -> void:
	var step := speed * delta
	if step >= global_position.distance_to(_to):
		if not _normal.is_zero_approx():
			var impact := Impact.instantiate() as Node3D
			add_sibling(impact)
			impact.global_transform = Transform3D(Basis(Quaternion(Vector3.UP, _normal)), _to)
		queue_free()
		return
	global_position = global_position.move_toward(_to, step)
