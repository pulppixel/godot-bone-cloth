extends SceneTree
# 실행 (저장소 루트에서):
#   godot --headless --fixed-fps 60 --path project --script res://tests/run_tests.gd

const TestChain := preload("res://tests/test_chain.gd")
const TestStage := preload("res://tests/test_stage.gd")

const TIMEOUT_MSEC := 300000

var passes := 0
var failures := 0
var skips := 0
var _start_msec := 0
var _delta := 0.0 # 이번 frame의 시간(초)


func _initialize() -> void:
	_start_msec = Time.get_ticks_msec()
	_run()


func _process(delta: float) -> bool:
	_delta = delta
	if Time.get_ticks_msec() - _start_msec > TIMEOUT_MSEC:
		check(false, "Tests did not finish in %d s." % (TIMEOUT_MSEC / 1000))
		_finish()
	return false


func check(condition: bool, message: String) -> void:
	if condition:
		passes += 1
	else:
		failures += 1
		print("FAILED: ", message)


func skip(message: String) -> void:
	skips += 1
	print("SKIPPED: ", message)


func _run() -> void:
	# 처음 3 frame을 본다. --fixed-fps가 없으면 처음 frame들이 1/120초로 시작한다.
	for frame in 3:
		await process_frame
		if absf(_delta - 1.0 / 60.0) > 1e-9:
			check(false, "Run with --fixed-fps 60 (a frame took %f s)." % _delta)
			_finish()
			return

	await TestChain.run(self)
	await TestStage.run(self)
	_finish()


func _finish() -> void:
	print("==== TESTS FINISHED ==== passes %d, failures %d, skipped %d" % [passes, failures, skips])
	quit(1 if failures > 0 else 0)
