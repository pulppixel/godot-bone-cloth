#!/usr/bin/env python
# Builds the BoneClothSimulator3D GDExtension into project/addons/bone_cloth/bin/.
# Targets Godot 4.7's extension API through godot-cpp 10 (the submodule at 10.0.0-stable).

env = SConscript("godot-cpp/SConstruct", {"api_version": "4.7"})

# The object files go to build/, leaving src/ to the sources; duplicate=False compiles from src/ without copying it.
env.VariantDir("build", "src", duplicate=False)

env.Append(CPPPATH=["src/"])
sources = Glob("build/*.cpp") + Glob("build/collision/*.cpp") + Glob("build/editor/*.cpp")

library = env.SharedLibrary(
    "project/addons/bone_cloth/bin/libbonecloth{}{}".format(env["suffix"], env["SHLIBSUFFIX"]),
    source=sources,
)

env.NoCache(library)
Default(library)
