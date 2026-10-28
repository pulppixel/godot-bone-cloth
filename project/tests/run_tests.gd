extends SceneTree
# 실행 (저장소 루트에서):
#   godot --headless --fixed-fps 60 --path project --script res://tests/run_tests.gd
# 막 clone해서 project/.godot이 없으면 먼저 한 번 import한다. 그래야 플러그인을 불러온다:
#   godot --headless --import --path project

const TestAddChains := preload("res://tests/test_add_chains.gd")
const TestChain := preload("res://tests/test_chain.gd")
const TestStage := preload("res://tests/test_stage.gd")
const TestWarnings := preload("res://tests/test_warnings.gd")

const TIMEOUT_MSEC := 300000
const EDITOR_PROJECT := "godot-bone-cloth-editor-tests" # OS 임시 폴더 안에 만드는 에디터 테스트용 프로젝트

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
	# 플러그인이 로드되었는지 체크한다.
	if not ClassDB.class_exists(&"BoneClothSimulator3D"):
		check(false, "The plugin is not loaded (no BoneClothSimulator3D class). In a new clone, run --headless --import first.")
		_finish()
		return

	if Engine.is_editor_hint():
		await TestWarnings.run(self)
		await TestAddChains.run(self)
		_finish()
		return
	
	for frame in 3:
		await process_frame
		if absf(_delta - 1.0 / 60.0) > 1e-9:
			check(false, "Run with --fixed-fps 60 (a frame took %f s)." % _delta)
			_finish()
			return

	await TestChain.run(self)
	await TestStage.run(self)
	_run_editor_tests()
	_finish()


# 임시 폴더에 addon과 tests만 든 프로젝트를 만들고, 거기서 headless 에디터로 이 파일을 다시 실행한다.
func _run_editor_tests() -> void:
	var project := OS.get_temp_dir().path_join(EDITOR_PROJECT)
	_remove(project)
	_copy("res://addons/bone_cloth", project.path_join("addons/bone_cloth"))
	_copy("res://tests", project.path_join("tests"))
	FileAccess.open(project.path_join("project.godot"), FileAccess.WRITE).store_string("config_version=5\n")

	# 먼저 import로 .godot을 만든다. 없으면 에디터가 처음 검사하면서 스크립트를 다시 불러와, 기다리던 테스트가 취소된다.
	var output := []
	OS.execute(OS.get_executable_path(), ["--headless", "--import", "--path", project], output, true)
	output.clear()
	var arguments := ["--headless", "--editor", "--path", project, "--script", "res://tests/run_tests.gd"]
	var exit_code := OS.execute(OS.get_executable_path(), arguments, output, true)
	var text: String = output[0]
	var counts := RegEx.create_from_string(r"passes (\d+), failures (\d+), skipped (\d+)").search(text)
	if counts == null:
		print(text)
		check(false, "Editor tests did not finish (exit code %d); the editor's output is above." % exit_code)
		return

	for line in text.split("\n"):
		if line.begins_with("FAILED: ") or line.begins_with("SKIPPED: "):
			print(line.strip_edges())
	passes += counts.get_string(1).to_int()
	failures += counts.get_string(2).to_int()
	skips += counts.get_string(3).to_int()


# 폴더를 통째로 복사한다. 빌드가 남기는 디버그 파일과 에디터가 쓰는 DLL 사본(~로 시작)은 뺀다.
func _copy(p_from: String, p_to: String) -> void:
	DirAccess.make_dir_recursive_absolute(p_to)
	for file in DirAccess.get_files_at(p_from):
		if not (file.begins_with("~") or file.get_extension() in ["pdb", "ilk"]):
			DirAccess.copy_absolute(p_from.path_join(file), p_to.path_join(file))
	for folder in DirAccess.get_directories_at(p_from):
		_copy(p_from.path_join(folder), p_to.path_join(folder))


# 폴더를 통째로 지운다. 지난 실행의 파일과 에디터 캐시(.godot)가 남지 않게 한다.
func _remove(p_path: String) -> void:
	if not DirAccess.dir_exists_absolute(p_path):
		return
	for file in DirAccess.get_files_at(p_path):
		DirAccess.remove_absolute(p_path.path_join(file))
	for folder in DirAccess.get_directories_at(p_path):
		_remove(p_path.path_join(folder))
	DirAccess.remove_absolute(p_path)


func _finish() -> void:
	print("==== TESTS FINISHED ==== passes %d, failures %d, skipped %d" % [passes, failures, skips])
	quit(1 if failures > 0 else 0)
