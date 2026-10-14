#!/usr/bin/env python
# Builds the BoneClothSimulator3D GDExtension into project/addons/bone_cloth/bin/.
# Targets Godot 4.7's extension API through godot-cpp 10 (the submodule at 10.0.0-stable).

env = SConscript("godot-cpp/SConstruct", {"api_version": "4.7"})

# The object files go to build/, leaving src/ to the sources; duplicate=False compiles from src/ without copying it.
env.VariantDir("build", "src", duplicate=False)

env.Append(CPPPATH=["src/"])
sources = Glob("build/*.cpp") + Glob("build/collision/*.cpp") + Glob("build/editor/*.cpp")

# The class reference in doc_classes/, made by godot --doctool, compiled into the editor and debug builds for F1 and the
# inspector tooltips, as the godot-cpp template does; generated into build/ beside the object files.
if env["target"] in ["editor", "template_debug"]:
    doc_data = env.GodotCPPDocData("build/gen/doc_data.gen.cpp", source=Glob("doc_classes/*.xml"))
    sources.append(doc_data)

library = env.SharedLibrary(
    "project/addons/bone_cloth/bin/libbonecloth{}{}".format(env["suffix"], env["SHLIBSUFFIX"]),
    source=sources,
)

env.NoCache(library)
Default(library)
