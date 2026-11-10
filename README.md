# Bone Cloth

Bone-based cloth for Godot 4.7: hair, skirts, capes, tails and breasts that sway with the character.

<p>
<img width="99%" alt="Long hair parting over the shoulders on landing" src="media/hero.gif" /><br>
<img width="33%" alt="A skirt swinging as she stops" src="media/skirt.gif" /><img width="33%" alt="A tail whipping through a vault" src="media/tail_vault.gif" /><img width="33%" alt="Hair flowing over the shoulders" src="media/collision.gif" />
</p>

- Sphere, capsule and plane collisions keep cloth out of the body.
- Chains can be linked for skirts and capes, and long hair can part over the shoulders.
- Eight presets to start from, editor gizmos, and docs in the editor (F1).

## Videos

<a href="https://youtu.be/B8XE65gJaQk"><img width="49%" alt="Tutorial video on YouTube" src="https://img.youtube.com/vi/B8XE65gJaQk/maxresdefault.jpg" /></a> <a href="https://youtu.be/QDqhtDSR8PA"><img width="49%" alt="On/Off comparison video on YouTube" src="https://img.youtube.com/vi/QDqhtDSR8PA/maxresdefault.jpg" /></a>

## Install

Get it from the [Godot Asset Store](https://store.godotengine.org/asset/pulppixel/bone-cloth/), or download the zip from
[Releases](https://github.com/pulppixel/godot-bone-cloth/releases) and extract it into your project.
It runs on Windows, Linux and macOS.

<details>
<summary>Build it yourself</summary>

```
git clone --recursive https://github.com/pulppixel/godot-bone-cloth.git
cd godot-bone-cloth
scons target=template_debug
scons target=template_release
```

Then copy `project/addons/bone_cloth` into your project's `addons/`.

</details>

## Use

1. Add a `BoneClothSimulator3D` under a `Skeleton3D`.
2. Add a chain and set its root and end bones.
3. Pick a preset from the node's toolbar menu.
4. Add collisions under the simulator and set their bones.

To try the demo, open `project/` in Godot 4.7 and run it.

## Credits

The solver and the sphere and capsule collisions are ported from [KawaiiPhysics](https://github.com/pafuhana1213/KawaiiPhysics).
The demo characters are [VRoid Studio](https://vroid.com/en/studio) sample models.

## License

MIT. Third-party licenses are in `THIRD_PARTY_NOTICES.md`.
