#!/usr/bin/env python
# Builds the BoneClothSimulator3D GDExtension into project/addons/bone_cloth/bin/.
# Targets Godot 4.7's extension API through godot-cpp 10 (the submodule at 10.0.0-stable).

env = SConscript("godot-cpp/SConstruct", {"api_version": "4.7"})

env.Append(CPPPATH=["src/"])
sources = Glob("src/*.cpp")

library = env.SharedLibrary(
    "project/addons/bone_cloth/bin/libbonecloth{}{}".format(env["suffix"], env["SHLIBSUFFIX"]),
    source=sources,
)

env.NoCache(library)
Default(library)
