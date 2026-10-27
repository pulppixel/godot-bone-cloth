extends RefCounted

const GOLDEN_PATH := "res://tests/stage_golden.cfg"
const FRAMES := 720 # 12초. run_tests.gd가 frame마다 1/60초인지 확인한다.


static func run(t) -> void:
	var golden := ConfigFile.new()
	golden.load(GOLDEN_PATH)
	var first: Node = load("res://demo/stage.tscn").instantiate()
	var count: int = first.get(&"characters").size()
	first.free()
	for index in count:
		var stage: Node = load("res://demo/stage.tscn").instantiate()
		var character: Node3D = stage.get(&"characters")[index]
		var name := String(character.name)
		var skeleton: Skeleton3D = character.find_children("*", "Skeleton3D", true, false)[0]
		if _chain_bones(skeleton).is_empty():
			t.skip("Stage, %s: no BoneClothSimulator3D yet." % name)
			stage.free()
			continue
		var md5 := await _run_character(t, stage, index)
		var expected: String = golden.get_value("run", name, "")
		if expected == "":
			t.skip("Stage, %s: no saved value. Add this line under [run] in %s:" % [name, GOLDEN_PATH])
			print("GOLDEN %s=\"%s\"" % [name, md5])
		else:
			t.check(md5 == expected, "Stage, %s: the 12 s Run changed (md5 %s, saved %s)." % [name, md5, expected])


# 캐릭터 하나를 보이게 하고, 앞으로 가는 키(move_forward)를 누른 채 FRAMES만큼 돌린다.
static func _run_character(t, p_stage: Node, p_index: int) -> String:
	var character: Node3D = p_stage.get(&"characters")[p_index]
	var skeleton: Skeleton3D = character.find_children("*", "Skeleton3D", true, false)[0]
	var bones := _chain_bones(skeleton)
	var rows := PackedStringArray()
	var record := func() -> void:
		var columns := PackedStringArray()
		for bone in bones:
			var pose := skeleton.get_bone_global_pose(bone)
			columns.append("%.9f %.9f %.9f %.9f %.9f %.9f" % [pose.origin.x, pose.origin.y, pose.origin.z, pose.basis.y.x, pose.basis.y.y, pose.basis.y.z])
		rows.append(",".join(columns))
	
	skeleton.reset_bone_poses()
	t.root.add_child(p_stage)
	skeleton.skeleton_updated.connect(record)
	p_stage.call(&"_show", p_index)
	Input.action_press(&"move_forward")
	for frame in FRAMES:
		await t.process_frame
	Input.action_release(&"move_forward")
	for frame in FRAMES:
		await t.process_frame
	skeleton.skeleton_updated.disconnect(record)
	p_stage.free()
	return "\n".join(rows).md5_text()


static func _chain_bones(p_skeleton: Skeleton3D) -> PackedInt32Array:
	var bones := PackedInt32Array()
	for simulator in p_skeleton.get_children():
		if not simulator.is_class("BoneClothSimulator3D"):
			continue
		for i in simulator.get("chain_count"):
			var root_name: String = simulator.get("chains/%d/root_bone_name" % i)
			var chain := PackedInt32Array()
			var bone := p_skeleton.find_bone(simulator.get("chains/%d/end_bone_name" % i))
			while bone >= 0:
				chain.insert(0, bone)
				if p_skeleton.get_bone_name(bone) == root_name:
					break
				bone = p_skeleton.get_bone_parent(bone)
			bones.append_array(chain)
	return bones
