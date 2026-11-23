extends RigidBody3D

@export var speed := 21.0
@export var flight_gravity := 0.3
@export var impact_keep := 0.2
@export var keep := 40

var _flying := true


func _ready() -> void:
	gravity_scale = flight_gravity
	body_entered.connect(_on_body_entered)


func launch(p_from: Vector3, p_to: Vector3, _normal: Vector3, _target: Object) -> void:
	add_to_group(&"darts")
	var darts := get_tree().get_nodes_in_group(&"darts")
	if darts.size() > keep:
		darts[0].queue_free()
	look_at_from_position(p_from, p_to)
	reset_physics_interpolation()
	linear_velocity = (p_to - p_from).normalized() * speed


func _integrate_forces(state: PhysicsDirectBodyState3D) -> void:
	if _flying and not state.linear_velocity.is_zero_approx():
		state.transform.basis = Basis.looking_at(state.linear_velocity)
		state.angular_velocity = Vector3.ZERO


func _on_body_entered(body: Node) -> void:
	if not _flying:
		return
	_flying = false
	gravity_scale = 1.0
	linear_velocity *= impact_keep
	if body.has_method("hit"):
		body.hit(global_position, -global_basis.z)
