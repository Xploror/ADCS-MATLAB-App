# assets/ — optional files for the 3D View

The 3D View works without any file in this folder. When a file below is present, it replaces the built-in fallback automatically.

| File | Used by | Fallback when missing |
|---|---|---|
| `models/iss.stl` | `spacecraftMeshes` (ISS mesh) | Built-in ISS made of boxes and cylinders (109 m truss) |

The chaser is always the built-in shape.

## Adding the NASA ISS model

NASA offers the ISS model "International Space Station (ISS) (A)" as a GLB file on NASA 3D Resources (https://science.nasa.gov/3d-resources/international-space-station-iss-a/). Base MATLAB cannot read GLB, so convert it to STL once:

1. Download the `.glb` file.
2. Convert it to STL, for example with Blender (File > Import > glTF, then File > Export > STL) or with Python: `import trimesh; trimesh.load('iss.glb', force='mesh').export('iss.stl')`.
3. Save it as `assets/models/iss.stl`.

The loader centres the model and scales its largest extent to 109 m. If the model appears rotated, pass a correcting rotation through `spacecraftMeshes(struct('iss_R', R))`. Credit: NASA/Ames Research Center. NASA media is generally not under US copyright, but credit is required and NASA logos may not be used.

It is advised to **kept large files in the gitignore** to avoid large file upload issues.
