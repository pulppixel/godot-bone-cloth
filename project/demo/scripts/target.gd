extends StaticBody3D

const FragmentLarge := preload("res://demo/shooter/target_fragment_large.tscn")
const FragmentSmall := preload("res://demo/shooter/target_fragment_small.tscn")

@export var break_hits := 5
@export var fall_time := 0.45
@export var down_time := 2.0
@export var rise_time := 0.5
@export var respawn_time := 4.0
@export var fragments := 8
@export var fragment_speed := 3.0
@export var fragment_spin := 15.0
@export var fragment_life := 3.0

var _hits := 0
var _down := false
var _tween: Tween

@onready var _model: Node3D = $Model
@onready var _shape: CollisionShape3D = $CollisionShape3D
@onready var _hit_sound: AudioStreamPlayer3D = $Hit
@onready var _break_sound: AudioStreamPlayer3D = $Break


func hit(at: Vector3, direction: Vector3) -> void:
	_hits += 1
	if _hits >= break_hits:
		_shatter(at, direction)
		return
	_hit_sound.play()
	if not _down:
		_fall()


func _fall() -> void:
	_down = true
	_tween = create_tween()
	_tween.tween_property(self, "rotation:x", -PI / 2.0, fall_time).set_trans(Tween.TRANS_BOUNCE).set_ease(Tween.EASE_OUT)
	_tween.tween_interval(down_time)
	_rise()


func _rise() -> void:
	_tween.tween_property(self, "rotation:x", 0.0, rise_time).set_trans(Tween.TRANS_BACK).set_ease(Tween.EASE_OUT)
	_tween.tween_callback(func(): _down = false)


func _shatter(at: Vector3, direction: Vector3) -> void:
	if _tween:
		_tween.kill()
	_break_sound.play()
	_model.visible = false
	_shape.disabled = true
	var push := Vector3(direction.x, 0.0, direction.z).normalized()
	for i in fragments:
		var piece: RigidBody3D = (FragmentLarge if i % 2 == 0 else FragmentSmall).instantiate()
		add_sibling(piece)
		piece.global_position = at + Vector3(randf_range(-0.3, 0.3), randf_range(-0.3, 0.3), randf_range(-0.1, 0.1))
		piece.linear_velocity = (push + Vector3(randf_range(-1.0, 1.0), randf_range(0.5, 1.5), randf_range(-0.3, 0.3))) * fragment_speed
		piece.angular_velocity = Vector3(randf_range(-1.0, 1.0), randf_range(-1.0, 1.0), randf_range(-1.0, 1.0)) * fragment_spin
		get_tree().create_timer(fragment_life + randf_range(0.0, 0.5)).timeout.connect(_fade.bind(piece))
	_hits = 0
	get_tree().create_timer(respawn_time).timeout.connect(_respawn)


func _fade(piece: RigidBody3D) -> void:
	var tween := piece.create_tween()
	tween.tween_property(piece.get_node("Model"), "scale", Vector3.ZERO, 0.3)
	tween.tween_callback(piece.queue_free)


func _respawn() -> void:
	rotation.x = -PI / 2.0
	_model.visible = true
	_shape.disabled = false
	_down = true
	_tween = create_tween()
	_rise()
