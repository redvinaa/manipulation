import os
import sys
import open3d as o3d

def downsample_obj(file_path, ratio):
    print(f"Processing {file_path}")

    # Load the mesh
    mesh = o3d.io.read_triangle_mesh(file_path)
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()

    # Target number of triangles
    target_triangles = int(len(mesh.triangles) * ratio)
    if target_triangles < 4:
        print(f"⚠️  Too few triangles after downsampling ({target_triangles}), skipping {file_path}")
        return

    # Simplify (quadric decimation preserves normals well)
    mesh_simplified = mesh.simplify_quadric_decimation(target_triangles)
    mesh_simplified.compute_vertex_normals()

    # Overwrite the original file
    o3d.io.write_triangle_mesh(file_path, mesh_simplified)
    print(f"✅ Downsampled {file_path} to {len(mesh_simplified.triangles)} triangles")

def process_dir(base_dir, ratio):
    for root, _, files in os.walk(base_dir):
        for f in files:
            if f.lower().endswith(".obj"):
                downsample_obj(os.path.join(root, f), ratio)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python downsample_all.py <ratio> <path>")
        sys.exit(1)

    ratio = float(sys.argv[1])
    if not (0 < ratio <= 1):
        print("Error: Ratio must be between 0 and 1.")
        sys.exit(1)

    path = sys.argv[2]
    if not os.path.isdir(path):
        print(f"Error: {path} is not a valid directory.")
        sys.exit(1)

    process_dir(path, ratio)
