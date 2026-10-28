extends RefCounted

const PARENTS := {"a0": "", "a1": "a0", "a2": "a1", "a3": "a2", "b1": "a1", "b2": "b1", "c1": "a0", "d0": "", "d1": "d0"}
const EXPECTED := [["a0", "a3"], ["a1", "b2"], ["a0", "c1"], ["d1", "d1"]]


static func run(t) -> void:
	var skeleton := Skeleton3D.new()
	skeleton.name = "AddChains"
	for bone_name in PARENTS:
		var bone := skeleton.add_bone(bone_name)
		if PARENTS[bone_name] != "":
			skeleton.set_bone_parent(bone, skeleton.find_bone(PARENTS[bone_name]))
			skeleton.set_bone_rest(bone, Transform3D(Basis(), Vector3(0, -0.1, 0)))
	var simulator := BoneClothSimulator3D.new()
	simulator.name = "Sim"
	skeleton.add_child(simulator)
	EditorInterface.add_root_node(skeleton)
	EditorInterface.edit_node(simulator)
	await t.process_frame

	var tree := _press_edit_chains(t)
	if tree == null:
		EditorInterface.close_scene()
		return
	_find_item(tree, "a0").set_checked(0, true)
	_find_item(tree, "d1").set_checked(0, true)
	tree.get_parent().confirmed.emit()

	t.check(_chains(simulator) == EXPECTED, "Edit Chains: made %s, expected %s." % [_chains(simulator), EXPECTED])

	var history := EditorInterface.get_editor_undo_redo().get_history_undo_redo(EditorInterface.get_editor_undo_redo().get_object_history_id(simulator))
	history.undo()
	t.check(simulator.chain_count == 0, "Edit Chains: %d chains after undo, expected 0." % simulator.chain_count)
	history.redo()

	# 다시 열면 chain에 든 bone은 체크로 보인다.
	simulator.set("chains/3/extend_end_bone", false)
	tree = _press_edit_chains(t)
	if tree == null:
		EditorInterface.close_scene()
		return
	for bone_name in PARENTS:
		var item := _find_item(tree, bone_name)
		var state := [item.is_checked(0), item.is_editable(0)]
		var expected := [bone_name != "d0", bone_name in ["a0", "d1"]]
		t.check(state == expected, "Edit Chains, reopened: %s checked and editable %s, expected %s." % [bone_name, state, expected])

	# a0의 체크를 풀면 a0 아래 chain 3개가 빠진다.
	_find_item(tree, "a0").set_checked(0, false)
	tree.get_parent().confirmed.emit()
	t.check(_chains(simulator) == [["d1", "d1"]], "Edit Chains, a0 unchecked: %s, expected [[d1, d1]]." % [_chains(simulator)])
	t.check(simulator.get("chains/0/extend_end_bone") == false, "Edit Chains, a0 unchecked: the d1 chain lost its Extend End Bone value.")

	history.undo()
	var extend: bool = simulator.get("chains/3/extend_end_bone")
	t.check(_chains(simulator) == EXPECTED and not extend, "Edit Chains, undo: %s with extend %s, expected %s with false." % [_chains(simulator), extend, EXPECTED])
	EditorInterface.close_scene()


static func _chains(p_simulator: BoneClothSimulator3D) -> Array:
	var chains := []
	for i in p_simulator.chain_count:
		chains.append([p_simulator.get_root_bone_name(i), p_simulator.get_end_bone_name(i)])
	return chains


static func _press_edit_chains(t) -> Tree:
	for menu: MenuButton in EditorInterface.get_base_control().find_children("*", "MenuButton", true, false):
		if menu.text != "BoneClothSimulator3D":
			continue
		var popup := menu.get_popup()
		for i in popup.item_count:
			if popup.get_item_text(i) == "Edit Chains...":
				popup.id_pressed.emit(popup.get_item_id(i))
		for dialog: ConfirmationDialog in menu.find_children("*", "ConfirmationDialog", true, false):
			if dialog.title == "Edit Chains" and dialog.visible:
				return dialog.find_children("*", "Tree", true, false)[0]
	t.check(false, "Edit Chains: no BoneClothSimulator3D menu with an open Edit Chains window.")
	return null


static func _find_item(p_tree: Tree, p_bone_name: String) -> TreeItem:
	var item := p_tree.get_root()
	while item != null and item.get_text(0) != p_bone_name:
		item = item.get_next_in_tree()
	return item
