extends RefCounted

# configuration warning은 에디터의 Scene dock에만 보인다(스크립트에서 부를 수 있는 함수가 없다).
# 그래서 run_tests.gd가 headless 에디터 안에서 이 파일을 부르고, Scene dock의 경고 버튼 툴팁을 읽는다.

const WARNING_BUTTON := 5
const BULLET := "\u2022  "
const REFRESH_WAIT := 1.0 # 초

const EXPECTED := {
	"NoChains": [
		["No chains. Add a chain under Chains and pick its root bone and end bone."],
		["Chain 0: pick a root bone and an end bone."],
	],
	"EmptyNames": [
		["Chain 0: pick a root bone and an end bone."],
		["Chain 0: pick an end bone."],
	],
	"NamesNotInSkeleton": [
		["Chain 0: root bone 'nope_root' is not in the skeleton; end bone 'nope_end' is not in the skeleton."],
		["Chain 0: root bone 'nope_root' is not in the skeleton; end bone 'nope_end' is not in the skeleton."],
	],
	"EndNotUnderRoot": [
		["Chain 0: end bone 'c2' is neither root bone 'b1' nor under it."],
		["Chain 0: end bone 'c2' is neither root bone 'b1' nor under it."],
	],
	"Valid": [
		[],
		["The Skeleton3D has a non-uniform scale (2.0, 1.0, 1.0), so the cloth moves differently. Make the scale of the Skeleton3D and its parents uniform (the same on all axes)."],
	],
	"BranchBeforeTrunk": [
		["Chain 0: root bone 'b2' is on chain 1, so move chain 1 above this chain."],
		[],
	],
	"TrunkBeforeBranch": [[], []],
	"SharedRoot": [[], []],
	"PathToNothing": [
		["Chain 0: exclude collision 'Missing' is not a BoneClothCollision3D child of this node."],
		[],
	],
	"ValidCollisionList": [
		[],
		["Chain 0: collision 'Sphere' is not a BoneClothCollision3D child of this node."],
	],
	"Scaled001": [[], []],
	"ScaledX": [
		["The Skeleton3D has a non-uniform scale (2.0, 1.0, 1.0), so the cloth moves differently. Make the scale of the Skeleton3D and its parents uniform (the same on all axes)."],
		[],
	],
}


static func run(t) -> void:
	while EditorInterface.get_resource_filesystem().is_scanning():
		await t.process_frame

	var cases := _build()
	EditorInterface.add_root_node(cases)
	await t.create_timer(REFRESH_WAIT).timeout
	_compare(t, cases, "open", 0)

	_change(cases)
	await t.create_timer(REFRESH_WAIT).timeout
	_compare(t, cases, "changed", 1)

	EditorInterface.close_scene()


static func _build() -> Node3D:
	var cases := Node3D.new()
	cases.name = "Cases"
	for case_name in EXPECTED:
		var holder := Node3D.new()
		holder.name = case_name
		cases.add_child(holder)
		var skeleton := _skeleton()
		holder.add_child(skeleton)
		var simulator: Node = ClassDB.instantiate("BoneClothSimulator3D")
		simulator.name = "Sim"
		skeleton.add_child(simulator)
		_set_up(case_name, simulator)

	cases.get_node("ValidCollisionList/Skeleton3D/Sim").add_child(_collision("BoneClothCollisionSphere3D", "Sphere"))
	cases.get_node("Scaled001").scale = Vector3.ONE * 0.01
	cases.get_node("ScaledX/Skeleton3D").scale = Vector3(2, 1, 1)
	for node in cases.find_children("*", "", true, false):
		node.owner = cases
	return cases


# b0 > b1 > b2 > b3 > b4, b0 > c1 > c2, 따로 떨어진 d0
static func _skeleton() -> Skeleton3D:
	var skeleton := Skeleton3D.new()
	skeleton.name = "Skeleton3D"
	var parents := {"b0": "", "b1": "b0", "b2": "b1", "b3": "b2", "b4": "b3", "c1": "b0", "c2": "c1", "d0": ""}
	for bone_name in parents:
		var bone := skeleton.add_bone(bone_name)
		if parents[bone_name] != "":
			skeleton.set_bone_parent(bone, skeleton.find_bone(parents[bone_name]))
			skeleton.set_bone_rest(bone, Transform3D(Basis(), Vector3(0, -0.1, 0)))
		skeleton.set_bone_pose(bone, skeleton.get_bone_rest(bone))
	return skeleton


# 경우에 맞게 chain과 collision 목록을 넣는다. 따로 적지 않은 경우는 b0에서 b4까지 chain 하나다.
static func _set_up(p_case: String, p_simulator: Node) -> void:
	# chain마다 root bone, end bone. EmptyNames는 막 더한 chain처럼 둘 다 비워 둔다.
	var chains := {
		"NoChains": [],
		"EmptyNames": [["", ""]],
		"NamesNotInSkeleton": [["nope_root", "nope_end"]],
		"EndNotUnderRoot": [["b1", "c2"]],
		"BranchBeforeTrunk": [["b2", "b4"], ["b0", "b4"]],
		"TrunkBeforeBranch": [["b0", "b4"], ["b2", "b4"]],
		"SharedRoot": [["b0", "b4"], ["b0", "c2"]],
	}
	var list: Array = chains.get(p_case, [["b0", "b4"]])
	p_simulator.set("chain_count", list.size())
	for i in list.size():
		p_simulator.set("chains/%d/root_bone_name" % i, list[i][0])
		p_simulator.set("chains/%d/end_bone_name" % i, list[i][1])

	if p_case == "PathToNothing":
		p_simulator.set("chains/0/exclude_collision_count", 1)
		p_simulator.set("chains/0/exclude_collisions/0", NodePath("Missing"))
	if p_case == "ValidCollisionList":
		p_simulator.set("chains/0/enable_all_child_collisions", false)
		p_simulator.set("chains/0/collision_count", 1)
		p_simulator.set("chains/0/collisions/0", NodePath("Sphere"))


# 이름을 붙인 collision node. p_class는 BoneClothCollisionSphere3D 같은 클래스 이름이다.
static func _collision(p_class: String, p_name: String) -> Node:
	var collision: Node = ClassDB.instantiate(p_class)
	collision.name = p_name
	return collision


static func _change(p_cases: Node3D) -> void:
	p_cases.get_node("ValidCollisionList/Skeleton3D/Sim/Sphere").free()
	var missing := _collision("BoneClothCollisionCapsule3D", "Missing")
	p_cases.get_node("PathToNothing/Skeleton3D/Sim").add_child(missing)
	missing.owner = p_cases

	p_cases.get_node("Valid").scale = Vector3(2, 1, 1)
	p_cases.get_node("Scaled001").scale = Vector3.ONE
	p_cases.get_node("ScaledX/Skeleton3D").scale = Vector3.ONE

	p_cases.get_node("EmptyNames/Skeleton3D/Sim").set("chains/0/root_bone_name", "b0")
	var branch := p_cases.get_node("BranchBeforeTrunk/Skeleton3D/Sim")
	branch.set("chains/0/root_bone_name", "b0")
	branch.set("chains/1/root_bone_name", "b2")
	p_cases.get_node("NoChains/Skeleton3D/Sim").set("chain_count", 1)


static func _compare(t, p_cases: Node3D, p_label: String, p_column: int) -> void:
	var shown := _read_scene_dock(p_cases)
	for case_name in EXPECTED:
		var path := p_cases.get_node("%s/Skeleton3D/Sim" % case_name).get_path()
		if not shown.has(path):
			t.check(false, "Warnings, %s, %s: the Scene dock has no item for Sim." % [case_name, p_label])
			continue
		var expected := PackedStringArray(EXPECTED[case_name][p_column])
		t.check(shown[path] == expected, "Warnings, %s, %s: shows %s, expected %s." % [case_name, p_label, shown[path], expected])


# Scene dock의 항목마다 경고를 읽는다. 결과는 node path -> 경고 목록.
static func _read_scene_dock(p_cases: Node3D) -> Dictionary:
	var dock: Node = EditorInterface.get_base_control().find_children("*", "SceneTreeDock", true, false)[0]
	var shown := {}
	for tree: Tree in dock.find_children("*", "Tree", true, false):
		var root := tree.get_root()
		if root == null or root.get_metadata(0) != p_cases.get_path():
			continue
		var items: Array[TreeItem] = [root]
		while not items.is_empty():
			var item: TreeItem = items.pop_back()
			items.append_array(item.get_children())
			var warnings := PackedStringArray()
			var button := item.get_button_by_id(0, WARNING_BUTTON)
			if button >= 0:
				var parts := item.get_button_tooltip_text(0, button).split(BULLET)
				for i in range(1, parts.size()):
					warnings.append(parts[i].strip_edges())
			shown[item.get_metadata(0)] = warnings
	return shown
