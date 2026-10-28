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
	
	var tree := _press_add_chains(t)
	if tree == null:
		EditorInterface.close_scene()
		return
	_find_item(tree, "a0").set_checked(0, true)
	_find_item(tree, "d1").set_checked(0, true)
	tree.get_parent().confirmed.emit()
	
	var chains := []
	for i in simulator.chain_count:
		chains.append([simulator.get_root_bone_name(i), simulator.get_end_bone_name(i)])
	t.check(chains == EXPECTED, "Add Chains: made %s, expected %s." % [chains, EXPECTED])
	
	var history := EditorInterface.get_editor_undo_redo().get_history_undo_redo(EditorInterface.get_editor_undo_redo().get_object_history_id(simulator))
	history.undo()
	t.check(simulator.chain_count == 0, "Add Chains: %d chains after undo, expected 0." % simulator.chain_count)
	history.redo()
	
	tree = _press_add_chains(t)
	if tree != null:
		for bone_name in PARENTS:
			var item := _find_item(tree, bone_name)
			var state := [item.is_checked(0), item.is_editable(0)]
			var expected := [bone_name in ["a0", "a1", "d1"], false]
			t.check(state == expected, "Add Chains, reopened: %s checked and editable %s, expected %s." % [bone_name, state, expected])
		tree.get_parent().hide()
	EditorInterface.close_scene()


static func _press_add_chains(t) -> Tree:
	for menu: MenuButton in EditorInterface.get_base_control().find_children("*", "MenuButton", true, false):
		if menu.text != "BoneClothSimulator3D":
			continue
		var popup := menu.get_popup()
		for i in popup.item_count:
			if popup.get_item_text(i) == "Add Chains...":
				popup.id_pressed.emit(popup.get_item_id(i))
		for dialog: ConfirmationDialog in menu.find_children("*", "ConfirmationDialog", true, false):
			if dialog.title == "Add Chains" and dialog.visible:
				return dialog.find_children("*", "Tree", true, false)[0]
	t.check(false, "Add Chains: no BoneClothSimulator3D menu with an open Add Chains window.")
	return null


static func _find_item(p_tree: Tree, p_bone_name: String) -> TreeItem:
	var item := p_tree.get_root()
	while item != null and item.get_text(0) != p_bone_name:
		item = item.get_next_in_tree()
	return item
