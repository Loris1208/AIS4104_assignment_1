# Meshes carried here

Every file below whose row names an original is a byte-for-byte copy of it. None was exported,
re-saved or converted. A row naming a model source instead was produced here from that source.

| File | Copied from | SHA256 | Origin and terms |
| --- | --- | --- | --- |
| `adapter-humerus.stl` | `models/adapter-humerus.stl` in the `biomechanics` repository | `85c80b8e8257bd186257fe2a1c84f9d980cb2743a18f99f6c84b197ecd0585ec` | The humerus is third-party; see "Humerus attribution" below. The adapter it is joined to is this course's own CAD. |
| `humerus-rig.stl` | `resources/laboratory.stl` in the `cortex-ros-gazebo` repository | `8ffaa500ec5f76e037937e1c23ed3d6a7c8b8af0212ea0d269805bc34f62b392` | This course's own CAD. |
| `cylindrical-adapter-rod-ball.stl` | `resources/cylindrical-adapter-rod-ball.stl` in the `cortex-ros-gazebo` repository | `a206a46449d34f87987df7e6cc830f6d55e2307f3f9ebe7cb03a22c815f6acf6` | This course's own CAD. `cylindrical-adapter-rod-ball-bent.stl` is its bent variant. |
| `vertical-adapter-rod-ball.stl` | `resources/vertical-adapter-rod-ball.stl` in the `cortex-ros-gazebo` repository | `773f1c1a1704aff7fb64fd7e1e6d8850b96133eaf3c631e436b2eb96665afc0d` | This course's own CAD. |
| `cylindrical-adapter-rod-ball-bent.stl` | Not copied: produced here from `cylindrical-adapter-rod-ball-bent.scad`, which is tracked beside it | `d202359f52891bf4d169f8f7503e93737dc84419cb8f6de37d30bca42fedbbc0` | This course's own CAD, carrying no third-party terms. |

## Humerus attribution

The humerus in `adapter-humerus.stl` originated from

```
Humerus 3D Model
Bluelink Anatomy - University of Michigan
CC Attribution-NonCommercial-NoDerivs
sketchfab.com
```

## Other names the same bytes arrive under

`adapter-humerus.stl` is byte-identical to two further files in the `cortex-ros-gazebo`
repository: `resources/adapter.stl` and `resources/vertical-adapter-humerus.stl`. All three carry
digest `85c80b8e8257bd186257fe2a1c84f9d980cb2743a18f99f6c84b197ecd0585ec`.

## Regenerating the bent variant

```
openscad -o cylindrical-adapter-rod-ball-bent.stl --export-format asciistl \
         cylindrical-adapter-rod-ball-bent.scad
```

OpenSCAD 2021.01 emits the facets in a different order on each run, so a regenerated file
carries the same vertices and the same triangles as the tracked one but not the same digest.
