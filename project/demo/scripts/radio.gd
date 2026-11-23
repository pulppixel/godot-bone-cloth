extends RigidBody3D

const Explosion := preload("res://demo/shooter/explosion.tscn")

@export var charred: Material
@export var blast_speed := 5.0
@export var blast_spin := 10.0
@export var push := 3.0
@export var hop := 0.5
@export var smoke_time := 6.0

var _broken := false

@onready var _music: AudioStreamPlayer3D = $Music
@onready var _smoke: GPUParticles3D = $Smoke


func hit(at: Vector3, direction: Vector3) -> void:
	if _broken:
		var shove := Vector3(direction.x, 0.0, direction.z).normalized() + Vector3.UP * hop
		apply_impulse(shove * push, at - global_position)
		return
	_broken = true
	_music.stop()
	var explosion := Explosion.instantiate() as Node3D
	add_sibling(explosion)
	explosion.global_position = global_position
	for mesh: MeshInstance3D in find_children("*", "MeshInstance3D"):
		mesh.material_overlay = charred
	freeze = false
	linear_velocity = Vector3(randf_range(-1.0, 1.0), 2.0, randf_range(-1.0, 1.0)).normalized() * blast_speed
	angular_velocity = Vector3(randf_range(-1.0, 1.0), randf_range(-1.0, 1.0), randf_range(-1.0, 1.0)) * blast_spin
	_smoke.emitting = true
	get_tree().create_timer(smoke_time).timeout.connect(func(): _smoke.emitting = false)
