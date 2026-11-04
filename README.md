# Bone Cloth

Bone-based cloth for Godot 4.7: hair, skirts, capes, tails and breasts that sway with the character.

## Classes

- `BoneClothSimulator3D`: a `SkeletonModifier3D` that sways bone chains.
- `BoneClothCollisionSphere3D`, `BoneClothCollisionCapsule3D`, `BoneClothCollisionTaperedCapsule3D`,
  `BoneClothCollisionPlane3D`: keep the chains out of the body.
- `BoneClothPreset`: saved values. Eight presets come with the plugin.

Every class is documented in the editor (F1).

## Install

Copy `project/addons/bone_cloth` into your project's `addons/`, with the binaries in its `bin/`.

The binaries for Windows, Linux and macOS are built by the Builds workflow (Actions, `bone-cloth-bin`). To build them yourself:

```
git clone --recursive https://github.com/pulppixel/godot-bone-cloth.git
cd godot-bone-cloth
scons target=template_debug
scons target=template_release
```

## Use

1. Add a `BoneClothSimulator3D` under a `Skeleton3D`.
2. Add a chain and set its root and end bones.
3. Pick a preset from the node's toolbar menu.
4. Add collisions under the simulator and set their bones.

## Demo

Open `project/` in Godot 4.7 and run it.

## License

MIT. The parts from KawaiiPhysics and Godot, and the demo characters' conditions of use, are in
`THIRD_PARTY_NOTICES.md`.
