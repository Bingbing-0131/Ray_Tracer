'''
def process_obj(input_path, output_path):
    with open(input_path, 'r') as infile, open(output_path, 'w') as outfile:
        for line in infile:
            if line.startswith('v '):
                parts = line.strip().split()
                x, y, z = float(parts[1])+60, float(parts[2]), float(parts[3])+80
                outfile.write(f"v {x} {y} {z}\n")
            else:
                outfile.write(line)

# Example usage:
process_obj('rose_2_modify.obj', 'rose_2_modify_modify.obj')


def process_obj_faces(input_path, output_path):
    with open(input_path, 'r') as infile, open(output_path, 'w') as outfile:
        for line in infile:
            if line.startswith('f '):
                parts = line.strip().split()
                new_parts = ['f']
                for triplet in parts[1:]:
                    v_t_n = triplet.split('/')
                    # Subtract 698 from each index (handle missing entries gracefully)
                    v = str(int(v_t_n[0]) - 698) if len(v_t_n) > 0 and v_t_n[0] else ''
                    t = str(int(v_t_n[1]) - 698) if len(v_t_n) > 1 and v_t_n[1] else ''
                    n = str(int(v_t_n[2]) - 698) if len(v_t_n) > 2 and v_t_n[2] else ''
                    new_triplet = '/'.join([v, t, n])
                    new_parts.append(new_triplet)
                outfile.write(' '.join(new_parts) + '\n')
            else:
                outfile.write(line)

# Example usage:
process_obj_faces('rose_2_modify_modify.obj', 'rose_2_modify.obj')
'''
'''
def extract_triangle_positions_only(input_path, output_path):
    with open(input_path, 'r') as infile, open(output_path, 'w') as outfile:
        for line in infile:
            if line.startswith('f '):
                parts = line.strip().split()
                if len(parts) >= 4:
                    # Only get the first 3 vertices
                    v1 = parts[1].split('/')[0]
                    v2 = parts[2].split('/')[0]
                    v3 = parts[3].split('/')[0]
                    outfile.write(f"f {v1} {v2} {v3}\n")
                # ignore invalid or non-triangle faces
            else:
                pass  # skip non-face lines
'''
# Example usage:
#extract_triangle_positions_only("input.obj", "tri_pos_only.obj")


# Example usage:
#extract_first_triangle_faces("input.obj", "triangles_only.obj")


# Example usage:
#extract_triangle_positions_only("rose_1.obj", "rose_1_modify.obj")
'''
def process_obj_file(input_path, output_path):
    with open(input_path, 'r') as infile, open(output_path, 'w') as outfile:
        for line in infile:
            if line.startswith('f '):
                parts = line.strip().split()
                if len(parts) > 2:
                    # Remove last vertex index in the face
                    new_line = ' '.join(parts[:-1]) + '\n'
                    outfile.write(new_line)
                else:
                    # If somehow it's a malformed face, just skip or keep it as is
                    outfile.write(line)
            else:
                # Write non-face lines unchanged
                outfile.write(line)

# Example usage
process_obj_file('rose_1.obj', 'rose_1_modify.obj')
'''
'''
def parse_face_vertices(face_tokens):
    """Parse v/vt/vn into tuple for exact matching"""
    return tuple(face_tokens)

def extract_surface_faces_from_tetra(input_file, output_file):
    from collections import defaultdict

    face_count = defaultdict(int)  # To count occurrences
    face_map = defaultdict(list)   # To store full strings for each face

    with open(input_file, 'r') as f:
        lines = f.readlines()

    for line in lines:
        if line.startswith('f '):
            tokens = line.strip().split()[1:]
            if len(tokens) != 4:
                continue  # not a tetrahedron face
            v = tokens
            # All 4 faces of a tetrahedron (as tuples of 3)
            tri_faces = [
                tuple(sorted([v[0], v[1], v[2]])),
                tuple(sorted([v[0], v[1], v[3]])),
                tuple(sorted([v[0], v[2], v[3]])),
                tuple(sorted([v[1], v[2], v[3]])),
            ]
            for face in tri_faces:
                face_count[face] += 1
                face_map[face] = face  # Keep original order for output
        else:
            # Store original non-face lines to write back later
            face_map[line] = line

    with open(output_file, 'w') as f:
        for line in lines:
            if not line.startswith('f '):
                f.write(line)

        for face, count in face_count.items():
            if count == 1:  # only surface face appears once
                f.write("f " + ' '.join(face) + "\n")

# Example usage
extract_surface_faces_from_tetra('rose_1_modify.obj', 'rose_1_modify_modify.obj')
'''
'''
def quad_to_tris(input_file, output_file):
    with open(input_file, 'r') as fin, open(output_file, 'w') as fout:
        for line in fin:
            if line.startswith('f '):
                tokens = line.strip().split()
                verts = tokens[1:]
                if len(verts) == 4:
                    # Split quad into two triangles: [0,1,2] and [0,2,3]
                    tri1 = f"f {verts[0]} {verts[1]} {verts[2]}\n"
                    tri2 = f"f {verts[0]} {verts[2]} {verts[3]}\n"
                    fout.write(tri1)
                    fout.write(tri2)
                else:
                    # Already triangle or something else, keep as-is
                    fout.write(line)
            else:
                # Non-face line, write unchanged
                fout.write(line)

# Example usage
quad_to_tris('rose_2.obj', 'rose_2_modify.obj')
'''

def quad_to_tris(input_file, output_file):
    """Convert quad faces to triangles"""
    with open(input_file, 'r') as fin, open(output_file, 'w') as fout:
        for line in fin:
            if line.startswith('f '):
                tokens = line.strip().split()
                verts = tokens[1:]
                if len(verts) == 4:
                    tri1 = f"f {verts[0]} {verts[1]} {verts[2]}\n"
                    tri2 = f"f {verts[0]} {verts[2]} {verts[3]}\n"
                    fout.write(tri1)
                    fout.write(tri2)
                else:
                    fout.write(line)
            else:
                fout.write(line)

def process_vertices(input_path, output_path):
    """Offset vertex positions (x + 60, z + 80)"""
    with open(input_path, 'r') as infile, open(output_path, 'w') as outfile:
        for line in infile:
            if line.startswith('v '):
                parts = line.strip().split()
                x, y, z = float(parts[1])*30+40, float(parts[3])*30+40, float(parts[2])*30+30
                outfile.write(f"v {x} {y} {z}\n")
            else:
                outfile.write(line)

def adjust_face_indices(input_path, output_path, offset=0):
    """Subtract offset from vertex/texture/normal indices in face definitions"""
    with open(input_path, 'r') as infile, open(output_path, 'w') as outfile:
        for line in infile:
            if line.startswith('f '):
                parts = line.strip().split()
                new_parts = ['f']
                for triplet in parts[1:]:
                    v_t_n = triplet.split('/')
                    v = str(int(v_t_n[0]) - offset) if len(v_t_n) > 0 and v_t_n[0] else ''
                    t = str(int(v_t_n[1]) - offset) if len(v_t_n) > 1 and v_t_n[1] else ''
                    n = str(int(v_t_n[2]) - offset) if len(v_t_n) > 2 and v_t_n[2] else ''
                    new_triplet = '/'.join([v, t, n])
                    new_parts.append(new_triplet)
                outfile.write(' '.join(new_parts) + '\n')
            else:
                outfile.write(line)

# === Pipeline Execution ===

# Step 1: Convert quads to triangles
quad_to_tris('sphere_obj.obj', 'step1_triangulated.obj')

# Step 2: Offset vertices
process_vertices('step1_triangulated.obj', 'step2_offset_vertices.obj')

# Step 3: Adjust face indices
adjust_face_indices('step2_offset_vertices.obj', 'sphere_obj_processed.obj')

print("✅ Processing complete. Output: diamond_processed.obj")

