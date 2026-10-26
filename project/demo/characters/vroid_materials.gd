@tool
extends EditorScenePostImport


const MATERIALS := "res://demo/characters/materials/%s_%s.tres"


func _post_import(scene: Node) -> Object:
	var character := get_source_file().get_file().get_basename()
	for mesh_instance: MeshInstance3D in scene.find_children("*", "MeshInstance3D", true, false):
		var mesh := mesh_instance.mesh
		for i in mesh.get_surface_count():
			var path := MATERIALS % [character, mesh.surface_get_material(i).resource_name]
			if not ResourceLoader.exists(path):
				push_error("%s: %s가 없다." % [get_source_file(), path])
				continue
			mesh.surface_set_material(i, load(path))
	return scene
