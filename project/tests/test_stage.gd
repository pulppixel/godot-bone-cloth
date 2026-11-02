extends RefCounted


const GOLDEN_DIR := "res://tests/stage_golden/"
const FRAMES := 1440
const SAMPLE_EVERY := 6
const TOLERANCE := 0.001
const INPUT := [
	[0, [&"move_forward"]],
	[180, [&"move_forward", &"run"]],
	[600, []],
	[780, [&"jump"]],
	[781, []],
	[900, [&"move_forward", &"run"]],
	[960, [&"move_forward", &"move_right", &"run"]],
	[1020, [&"move_forward", &"run"]],
	[1080, [&"move_forward", &"run", &"jump"]],
	[1081, [&"move_forward", &"run"]],
	[1200, []],
]
const BODY_BONES: Array[StringName] = [&"Hips", &"LeftFoot", &"RightFoot", &"UpperChest", &"Neck", &"Head", &"LeftEye", &"RightEye"]


static func run(t) -> void:
	var out_dir := OS.get_temp_dir().path_join("godot-bone-cloth-stage")
	DirAccess.make_dir_recursive_absolute(out_dir)
	var first: Node = load("res://demo/stage.tscn").instantiate()
	var count: int = first.get(&"characters").size()
	first.free()
	for index in count:
		var stage: Node = load("res://demo/stage.tscn").instantiate()
		var name := String(stage.get(&"characters")[index].name)
		var actual := await _run_character(t, stage, index)
		var actual_path := out_dir.path_join(name + ".txt")
		FileAccess.open(actual_path, FileAccess.WRITE).store_string(actual)
		var golden_path := GOLDEN_DIR + name + ".txt"
		if not FileAccess.file_exists(golden_path):
			t.skip("Stage, %s: no golden yet. Copy %s to %s." % [name, actual_path, ProjectSettings.globalize_path(golden_path)])
			continue
		var verdict := _compare(FileAccess.get_file_as_string(golden_path), actual)
		t.check(verdict == "", "Stage, %s: %s This run is at %s." % [name, verdict, actual_path])


static func _run_character(t, p_stage: Node, p_index: int) -> String:
	var character: Node3D = p_stage.get(&"characters")[p_index]
	var skeleton: Skeleton3D = character.find_children("*", "Skeleton3D", true, false)[0]
	var face: MeshInstance3D = character.find_child("Face", true, false)
	var bones := PackedInt32Array()
	var names := PackedStringArray(["pose:Body"])
	for bone in BODY_BONES:
		bones.append(skeleton.find_bone(bone))
		names.append("pose:" + bone)
	var recorder := Recorder.new()
	recorder.skeleton = skeleton
	recorder.face = face
	recorder.bones = bones
	recorder.chains = _chain_bones(skeleton)
	recorder.shape_count = face.get_blend_shape_count() if face != null else 0

	seed(1)
	skeleton.reset_bone_poses()
	t.root.add_child(p_stage)
	skeleton.skeleton_updated.connect(recorder.record)
	p_stage.call(&"_show", p_index)
	var held: Array = []
	for frame in FRAMES:
		recorder.frame = frame
		for row in INPUT:
			if row[0] == frame:
				for action in held:
					if action not in row[1]:
						Input.action_release(action)
				for action in row[1]:
					if action not in held:
						Input.action_press(action)
				held = row[1]
		await t.process_frame
	for action in held:
		Input.action_release(action)
	skeleton.skeleton_updated.disconnect(recorder.record)
	
	var kept_shapes := PackedInt32Array() # 한 번도 0이 아니었던 blend shape
	for i in recorder.shape_count:
		for shapes in recorder.shape_rows:
			if shapes[i] != 0.0:
				kept_shapes.append(i)
				break
	for i in kept_shapes:
		names.append("shape:" + face.mesh.get_blend_shape_name(i))
	for bone in recorder.chains:
		names.append("chain:" + skeleton.get_bone_name(bone))
	p_stage.free()
	var lines := PackedStringArray(["frames %d every %d" % [FRAMES, SAMPLE_EVERY], " ".join(names)])
	for r in recorder.rows.size():
		var words := PackedStringArray([str(recorder.frames[r])])
		for value in recorder.rows[r]:
			words.append("%.4f" % value)
		for i in kept_shapes:
			words.append("%.4f" % recorder.shape_rows[r][i])
		lines.append(" ".join(words))
	return "\n".join(lines) + "\n"


# frame마다 Skeleton3D의 skeleton_updated 신호 안에서 pose를 읽는다.
class Recorder:
	var skeleton: Skeleton3D
	var face: MeshInstance3D
	var bones := PackedInt32Array()
	var chains := PackedInt32Array()
	var shape_count := 0
	var frame := 0
	var frames := PackedInt32Array()
	var rows: Array[PackedFloat32Array] = []
	var shape_rows: Array[PackedFloat32Array] = []
	
	func record() -> void:
		if frame % SAMPLE_EVERY != 0 or (not frames.is_empty() and frames[-1] == frame):
			return
		frames.append(frame)
		var values := PackedFloat32Array()
		_append_pose(values, skeleton.global_transform, Vector3.AXIS_Z)
		for bone in bones:
			_append_pose(values, skeleton.get_bone_global_pose(bone), Vector3.AXIS_Y)
		for bone in chains:
			var origin := skeleton.get_bone_global_pose(bone).origin
			values.append_array([origin.x, origin.y, origin.z])
		rows.append(values)
		var shapes := PackedFloat32Array()
		for i in shape_count:
			shapes.append(face.get_blend_shape_value(i))
		shape_rows.append(shapes)
	
	static func _append_pose(p_values: PackedFloat32Array, p_pose: Transform3D, p_axis: Vector3.Axis) -> void:
		var axis := p_pose.basis[p_axis]
		p_values.append_array([p_pose.origin.x, p_pose.origin.y, p_pose.origin.z, axis.x, axis.y, axis.z])


static func _compare(p_expected: String, p_actual: String) -> String:
	var expected := p_expected.strip_edges().split("\n")
	var actual := p_actual.strip_edges().split("\n")
	if expected.size() < 2 or expected[0] != actual[0] or expected[1] != actual[1]:
		return "the layout changed (frames, bones, chains or blend shapes differ from the golden). Re-record it if that is intended."
	if expected.size() != actual.size():
		return "the golden has %d samples, this run %d." % [expected.size() - 2, actual.size() - 2]
	var columns := expected[1].split(" ")
	var worst := 0.0
	var worst_at := ""
	var first_at := ""
	var over := 0
	for line in range(2, expected.size()):
		var a := expected[line].split(" ")
		var b := actual[line].split(" ")
		if a.size() != b.size():
			return "line %d has %d values in the golden and %d in this run." % [line + 1, a.size(), b.size()]
		var frame := a[0]
		var at := 1
		var line_over := false
		for column in columns:
			var width := 6 if column.begins_with("pose:") else (3 if column.begins_with("chain:") else 1)
			for i in width:
				var diff := absf(a[at + i].to_float() - b[at + i].to_float())
				if diff > TOLERANCE:
					line_over = true
					if first_at == "":
						first_at = "frame %s (%s)" % [frame, column.get_slice(":", 1)]
				if diff > worst:
					worst = diff
					worst_at = "frame %s (%s)" % [frame, column.get_slice(":", 1)]
			at += width
		if line_over:
			over += 1
	if over == 0:
		return ""
	return "differs by up to %.4f at %s, tolerance %.4f; first at %s; %d of %d samples differ." % [worst, worst_at, TOLERANCE, first_at, over, expected.size() - 2]


# 모든 BoneClothSimulator3D의 chain마다 끝 본을 모은다.
static func _chain_bones(p_skeleton: Skeleton3D) -> PackedInt32Array:
	var bones := PackedInt32Array()
	for simulator in p_skeleton.get_children():
		if not simulator.is_class("BoneClothSimulator3D"):
			continue
		for i in simulator.get("chain_count"):
			bones.append(p_skeleton.find_bone(simulator.get("chains/%d/end_bone_name" % i)))
	return bones
