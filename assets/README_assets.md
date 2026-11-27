# assets/ — optional files for the 3D View

The 3D View works without any file in this folder. When a file below is present, it replaces the built-in fallback automatically.

| File | Used by | Fallback when missing |
|---|---|---|
| `models/iss.stl` | `spacecraftMeshes` (ISS mesh) | Built-in ISS made of boxes and cylinders (109 m truss) |
| `textures/earth.jpg` | `drawScene3D` (Earth texture) | `topo.mat` if it is on the MATLAB path, else a plain blue sphere |

The chaser is always the built-in shape.

## Adding the NASA ISS model

NASA offers the ISS model "International Space Station (ISS) (A)" as a GLB file on NASA 3D Resources (https://science.nasa.gov/3d-resources/international-space-station-iss-a/). Base MATLAB cannot read GLB, so convert it to STL once:

1. Download the `.glb` file.
2. Convert it to STL, for example with Blender (File > Import > glTF, then File > Export > STL) or with Python: `import trimesh; trimesh.load('iss.glb', force='mesh').export('iss.stl')`.
3. Save it as `assets/models/iss.stl`.

The loader centres the model and scales its largest extent to 109 m. If the model appears rotated, pass a correcting rotation through `spacecraftMeshes(struct('iss_R', R))`. Credit: NASA/Ames Research Center. NASA media is generally not under US copyright, but credit is required and NASA logos may not be used.

## Adding an Earth texture

Save any equi-rectangular (2:1) Earth image as `textures/earth.jpg`, with north at the top and longitude −180° on the left. NASA Blue Marble images (https://visibleearth.nasa.gov/) work well at 2048×1024. Credit: NASA Earth Observatory. The model ignores Earth rotation and fixes the ascending node at zero, so the Earth's longitude in the view is arbitrary.

It is advised to **keep large files in the gitignore** to avoid large file upload issues.