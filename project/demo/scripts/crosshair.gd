extends Control

@export var texture: Texture2D
@export var center_px := 16.0

var spread_deg := 0.0:
	set(value):
		if value != spread_deg:
			spread_deg = value
			queue_redraw()


func _draw() -> void:
	var full := texture.get_size()
	var to_screen := size / full
	var lo := (full - Vector2(center_px, center_px)) / 2.0
	var hi := lo + Vector2(center_px, center_px)
	var push := _spread_px()
	_piece(Rect2(lo, hi - lo), Vector2.ZERO, to_screen)
	_piece(Rect2(lo.x, 0.0, center_px, lo.y), Vector2(0.0, -push), to_screen)
	_piece(Rect2(lo.x, hi.y, center_px, full.y - hi.y), Vector2(0.0, push), to_screen)
	_piece(Rect2(0.0, lo.y, lo.x, center_px), Vector2(-push, 0.0), to_screen)
	_piece(Rect2(hi.x, lo.y, full.x - hi.x, center_px), Vector2(push, 0.0), to_screen)


func _piece(region: Rect2, offset: Vector2, to_screen: Vector2) -> void:
	draw_texture_rect_region(texture, Rect2(region.position * to_screen + offset, region.size * to_screen), region)


func _spread_px() -> float:
	var camera := get_viewport().get_camera_3d()
	var half_fov := deg_to_rad(camera.fov / 2.0)
	return tan(deg_to_rad(spread_deg)) / tan(half_fov) * get_viewport_rect().size.y / 2.0
