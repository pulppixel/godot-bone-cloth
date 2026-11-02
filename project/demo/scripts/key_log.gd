extends Node


var _file: FileAccess
var _run := false
var _captured := false


func _ready() -> void:
	_file = FileAccess.open("user://key_log.txt", FileAccess.WRITE)
	_line("start, file in %s" % OS.get_user_data_dir())


func _input(event: InputEvent) -> void:
	var key := event as InputEventKey
	if key == null or (key.keycode != KEY_SHIFT and key.physical_keycode != KEY_SHIFT):
		return
	_line("Shift %s%s location=%d keycode=%d physical=%d" % ["down" if key.pressed else "up", " echo" if key.echo else "", key.location, key.keycode, key.physical_keycode])


func _physics_process(_delta: float) -> void:
	var run := Input.is_action_pressed(&"run")
	if run != _run:
		_run = run
		_line("run %s, Shift key %s" % ["on" if run else "off", "down" if Input.is_physical_key_pressed(KEY_SHIFT) else "up"])
	var captured := Input.mouse_mode == Input.MOUSE_MODE_CAPTURED
	if captured != _captured:
		_captured = captured
		_line("mouse %s" % ("captured" if captured else "visible"))


func _notification(what: int) -> void:
	match what:
		NOTIFICATION_APPLICATION_FOCUS_OUT:
			_line("focus out")
		NOTIFICATION_APPLICATION_FOCUS_IN:
			_line("focus in")


func _line(text: String) -> void:
	var line := "%8.3f f%d %s" % [Time.get_ticks_msec() / 1000.0, Engine.get_physics_frames(), text]
	print(line)
	if _file != null:
		_file.store_line(line)
		_file.flush()
