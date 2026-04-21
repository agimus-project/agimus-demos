import numpy as np
import struct


class PlyProcessor:
    _PLY_TYPE_TO_NUMPY = {
        "char": np.int8,
        "uchar": np.uint8,
        "short": np.int16,
        "ushort": np.uint16,
        "int": np.int32,
        "uint": np.uint32,
        "float": np.float32,
        "double": np.float64,
        "int8": np.int8,
        "uint8": np.uint8,
        "int16": np.int16,
        "uint16": np.uint16,
        "int32": np.int32,
        "uint32": np.uint32,
        "float32": np.float32,
        "float64": np.float64,
    }

    _PLY_TYPE_TO_STRUCT = {
        "char": "b",
        "uchar": "B",
        "short": "h",
        "ushort": "H",
        "int": "i",
        "uint": "I",
        "float": "f",
        "double": "d",
        "int8": "b",
        "uint8": "B",
        "int16": "h",
        "uint16": "H",
        "int32": "i",
        "uint32": "I",
        "float32": "f",
        "float64": "d",
    }

    def __init__(self):
        self.vertices = None  # Nx3 float
        self.normals = None  # Nx3 float
        self.colors = None  # Nx3 int
        self.faces = None  # Mx3 int
        self.vertex_count = 0
        self.face_count = 0
        self.header_lines = []

    def load_ply(self, file_path):
        print(f"Loading PLY from {file_path}...")
        with open(file_path, "rb") as f:
            raw = f.read()

        header_end = raw.find(b"end_header")
        if header_end < 0:
            raise ValueError("Invalid PLY file: missing end_header")
        payload_offset = raw.find(b"\n", header_end)
        if payload_offset < 0:
            raise ValueError("Invalid PLY file: malformed header termination")
        payload_offset += 1

        header_text = raw[:payload_offset].decode("ascii", errors="replace")
        self.header_lines = [line.strip() for line in header_text.splitlines()]
        fmt, elements = self._parse_header(self.header_lines)

        self.vertex_count = next(
            (element["count"] for element in elements if element["name"] == "vertex"),
            0,
        )
        self.face_count = next(
            (element["count"] for element in elements if element["name"] == "face"),
            0,
        )
        print(
            f"Header parsed. Format: {fmt}, Vertices: {self.vertex_count}, Faces: {self.face_count}"
        )

        payload = memoryview(raw)[payload_offset:]
        if fmt == "ascii":
            text = payload.tobytes().decode("utf-8", errors="replace")
            self._load_ascii_payload(text)
        elif fmt == "binary_little_endian":
            self._load_binary_payload(payload, elements, endianness="<")
        elif fmt == "binary_big_endian":
            self._load_binary_payload(payload, elements, endianness=">")
        else:
            raise ValueError(f"Unsupported PLY format: {fmt}")

        self.vertex_count = len(self.vertices)
        self.face_count = len(self.faces)
        print("PLY loaded successfully.")

    def _parse_header(self, header_lines):
        fmt = None
        elements = []
        current_element = None

        for line in header_lines:
            tokens = line.split()
            if not tokens:
                continue
            if tokens[0] == "format" and len(tokens) >= 2:
                fmt = tokens[1]
            elif tokens[0] == "element" and len(tokens) >= 3:
                current_element = {
                    "name": tokens[1],
                    "count": int(tokens[2]),
                    "properties": [],
                }
                elements.append(current_element)
            elif tokens[0] == "property" and current_element is not None:
                if tokens[1] == "list" and len(tokens) >= 5:
                    current_element["properties"].append(
                        {
                            "kind": "list",
                            "name": tokens[4],
                            "count_type": tokens[2],
                            "item_type": tokens[3],
                        }
                    )
                elif len(tokens) >= 3:
                    current_element["properties"].append(
                        {
                            "kind": "scalar",
                            "name": tokens[2],
                            "type": tokens[1],
                        }
                    )

        if fmt is None:
            raise ValueError("Invalid PLY header: missing format")
        return fmt, elements

    def _load_ascii_payload(self, text):
        lines = text.splitlines()
        current_line = 0

        self.vertices = np.zeros((self.vertex_count, 3), dtype=np.float32)
        self.normals = np.zeros((self.vertex_count, 3), dtype=np.float32)
        self.colors = np.zeros((self.vertex_count, 3), dtype=np.uint8)

        print("Parsing ASCII vertices...")
        if self.vertex_count > 0:
            first_vertex_line = lines[current_line].strip()
            parts = first_vertex_line.split()

            is_3_line_format = False
            if len(parts) == 3 and current_line + 1 < len(lines):
                parts_next = lines[current_line + 1].strip().split()
                if len(parts_next) == 3:
                    is_3_line_format = True

            if is_3_line_format:
                print("Detected 3-line vertex format.")
                for i in range(self.vertex_count):
                    self.vertices[i] = [float(p) for p in lines[current_line].split()]
                    current_line += 1
                    self.normals[i] = [float(p) for p in lines[current_line].split()]
                    current_line += 1
                    self.colors[i] = [int(p) for p in lines[current_line].split()[:3]]
                    current_line += 1
            else:
                print("Assuming standard 1-line vertex format.")
                for i in range(self.vertex_count):
                    parts = lines[current_line].split()
                    self.vertices[i] = [float(p) for p in parts[0:3]]
                    if len(parts) >= 6:
                        self.normals[i] = [float(p) for p in parts[3:6]]
                    if len(parts) >= 9:
                        self.colors[i] = [int(p) for p in parts[6:9]]
                    current_line += 1

        print("Parsing ASCII faces...")
        faces = []
        for _ in range(self.face_count):
            parts = lines[current_line].split()
            current_line += 1
            if not parts:
                continue
            n_indices = int(parts[0])
            indices = [int(p) for p in parts[1 : 1 + n_indices]]
            if len(indices) < 3:
                continue
            v0 = indices[0]
            for k in range(1, len(indices) - 1):
                faces.append([v0, indices[k], indices[k + 1]])

        self.faces = np.asarray(faces, dtype=np.int32)

    def _load_binary_payload(self, payload, elements, endianness):
        print("Parsing binary payload...")
        offset = 0
        vertices = None
        normals = None
        colors = None
        faces = []

        for element in elements:
            name = element["name"]
            count = element["count"]
            properties = element["properties"]

            if name == "vertex":
                if any(prop["kind"] != "scalar" for prop in properties):
                    raise ValueError("Unsupported binary PLY vertex list property.")

                dtype_fields = []
                for prop in properties:
                    np_dtype = np.dtype(
                        self._PLY_TYPE_TO_NUMPY[prop["type"]]
                    ).newbyteorder(endianness)
                    dtype_fields.append((prop["name"], np_dtype))
                vertex_dtype = np.dtype(dtype_fields)
                vertices_raw = np.frombuffer(
                    payload, dtype=vertex_dtype, count=count, offset=offset
                )
                offset += vertex_dtype.itemsize * count

                x_name = (
                    "x"
                    if "x" in vertices_raw.dtype.names
                    else vertices_raw.dtype.names[0]
                )
                y_name = (
                    "y"
                    if "y" in vertices_raw.dtype.names
                    else vertices_raw.dtype.names[1]
                )
                z_name = (
                    "z"
                    if "z" in vertices_raw.dtype.names
                    else vertices_raw.dtype.names[2]
                )
                vertices = np.column_stack(
                    [vertices_raw[x_name], vertices_raw[y_name], vertices_raw[z_name]]
                ).astype(np.float32)

                if {"nx", "ny", "nz"}.issubset(vertices_raw.dtype.names):
                    normals = np.column_stack(
                        [vertices_raw["nx"], vertices_raw["ny"], vertices_raw["nz"]]
                    ).astype(np.float32)
                else:
                    normals = np.zeros((count, 3), dtype=np.float32)

                if {"red", "green", "blue"}.issubset(vertices_raw.dtype.names):
                    colors = np.column_stack(
                        [
                            vertices_raw["red"],
                            vertices_raw["green"],
                            vertices_raw["blue"],
                        ]
                    ).astype(np.uint8)
                else:
                    colors = np.zeros((count, 3), dtype=np.uint8)

            elif name == "face":
                for _ in range(count):
                    parsed = {}
                    for prop in properties:
                        if prop["kind"] == "scalar":
                            fmt = self._PLY_TYPE_TO_STRUCT[prop["type"]]
                            value = struct.unpack_from(
                                f"{endianness}{fmt}", payload, offset
                            )[0]
                            offset += struct.calcsize(fmt)
                            parsed[prop["name"]] = value
                        else:
                            count_fmt = self._PLY_TYPE_TO_STRUCT[prop["count_type"]]
                            item_fmt = self._PLY_TYPE_TO_STRUCT[prop["item_type"]]
                            n_items = struct.unpack_from(
                                f"{endianness}{count_fmt}", payload, offset
                            )[0]
                            offset += struct.calcsize(count_fmt)
                            if n_items > 0:
                                values = struct.unpack_from(
                                    f"{endianness}{n_items}{item_fmt}",
                                    payload,
                                    offset,
                                )
                            else:
                                values = ()
                            offset += struct.calcsize(item_fmt) * n_items
                            parsed[prop["name"]] = values

                    indices = parsed.get("vertex_indices")
                    if indices is None:
                        for value in parsed.values():
                            if isinstance(value, tuple):
                                indices = value
                                break
                    if indices is None or len(indices) < 3:
                        continue
                    v0 = int(indices[0])
                    for k in range(1, len(indices) - 1):
                        faces.append([v0, int(indices[k]), int(indices[k + 1])])
            else:
                offset = self._skip_binary_element(
                    payload, offset, count, properties, endianness
                )

        if vertices is None:
            vertices = np.zeros((0, 3), dtype=np.float32)
        if normals is None:
            normals = np.zeros((len(vertices), 3), dtype=np.float32)
        if colors is None:
            colors = np.zeros((len(vertices), 3), dtype=np.uint8)

        self.vertices = vertices
        self.normals = normals
        self.colors = colors
        self.faces = np.asarray(faces, dtype=np.int32)

    def _skip_binary_element(self, payload, offset, count, properties, endianness):
        for _ in range(count):
            for prop in properties:
                if prop["kind"] == "scalar":
                    fmt = self._PLY_TYPE_TO_STRUCT[prop["type"]]
                    offset += struct.calcsize(fmt)
                else:
                    count_fmt = self._PLY_TYPE_TO_STRUCT[prop["count_type"]]
                    item_fmt = self._PLY_TYPE_TO_STRUCT[prop["item_type"]]
                    n_items = struct.unpack_from(
                        f"{endianness}{count_fmt}", payload, offset
                    )[0]
                    offset += struct.calcsize(count_fmt)
                    offset += struct.calcsize(item_fmt) * n_items
        return offset

    def upsample_mesh(self):
        print("Upsampling mesh (Loop subdivision)...")

        edge_to_new_vertex = {}
        new_vertices_list = []
        new_colors_list = []
        new_faces_list = []

        next_vertex_id = self.vertex_count

        def get_edge_midpoint(v1_idx, v2_idx):
            edge_key = tuple(sorted((v1_idx, v2_idx)))
            if edge_key in edge_to_new_vertex:
                return edge_to_new_vertex[edge_key]

            nonlocal next_vertex_id
            v1 = self.vertices[v1_idx]
            v2 = self.vertices[v2_idx]
            mid_point = (v1 + v2) * 0.5

            c1 = self.colors[v1_idx].astype(float)
            c2 = self.colors[v2_idx].astype(float)
            mid_color = ((c1 + c2) * 0.5).astype(np.uint8)

            new_vertices_list.append(mid_point)
            new_colors_list.append(mid_color)

            new_idx = next_vertex_id
            edge_to_new_vertex[edge_key] = new_idx
            next_vertex_id += 1
            return new_idx

        for i in range(self.face_count):
            v1, v2, v3 = self.faces[i]

            a = get_edge_midpoint(v1, v2)
            b = get_edge_midpoint(v2, v3)
            c = get_edge_midpoint(v3, v1)

            new_faces_list.append([v1, a, c])
            new_faces_list.append([v2, b, a])
            new_faces_list.append([v3, c, b])
            new_faces_list.append([a, b, c])

        if len(new_vertices_list) > 0:
            self.vertices = np.vstack([self.vertices, np.array(new_vertices_list)])
            self.colors = np.vstack([self.colors, np.array(new_colors_list)])
            self.normals = np.zeros((len(self.vertices), 3), dtype=np.float32)

        self.faces = np.array(new_faces_list, dtype=np.int32)
        self.vertex_count = len(self.vertices)
        self.face_count = len(self.faces)

        print(
            f"Upsampling complete. New vertices: {self.vertex_count}, New faces: {self.face_count}"
        )

    def refine_mesh(self, iterations=3, lambda_param=0.6):
        print(f"Refining mesh ({iterations} iterations)...")

        adj = [set() for _ in range(self.vertex_count)]
        for f in self.faces:
            v1, v2, v3 = f
            adj[v1].add(v2)
            adj[v1].add(v3)
            adj[v2].add(v1)
            adj[v2].add(v3)
            adj[v3].add(v1)
            adj[v3].add(v2)

        for it in range(iterations):
            new_verts = np.copy(self.vertices)
            for i in range(self.vertex_count):
                if len(adj[i]) == 0:
                    continue
                neighbors = list(adj[i])
                centroid = np.mean(self.vertices[neighbors], axis=0)
                new_verts[i] = self.vertices[i] + lambda_param * (
                    centroid - self.vertices[i]
                )
            self.vertices = new_verts
            print(f"Iteration {it + 1} complete.")

        print("Recomputing normals...")
        new_normals = np.zeros_like(self.normals)

        v0 = self.vertices[self.faces[:, 0]]
        v1 = self.vertices[self.faces[:, 1]]
        v2 = self.vertices[self.faces[:, 2]]

        cross = np.cross(v1 - v0, v2 - v0)
        face_normals = cross

        for i in range(self.face_count):
            fn = face_normals[i]
            for idx in self.faces[i]:
                new_normals[idx] += fn

        norms = np.linalg.norm(new_normals, axis=1, keepdims=True)
        norms[norms == 0] = 1.0
        self.normals = new_normals / norms

        print("Refinement complete.")

    def get_closest_vertex(self, point):
        dists = np.linalg.norm(self.vertices - point, axis=1)
        idx = np.argmin(dists)
        return idx, self.vertices[idx], self.normals[idx]

    def compute_trajectory(self, center_point, radius, num_points=100):
        print(
            f"Computing trajectory centered at {center_point} with radius {radius}..."
        )
        idx, v_c, n_c = self.get_closest_vertex(center_point)
        # print(f"Closest surface point: {v_c}, Normal: {n_c}")

        z_axis = n_c
        if abs(z_axis[0]) < 0.9:
            x_axis = np.cross(z_axis, [1, 0, 0])
        else:
            x_axis = np.cross(z_axis, [0, 1, 0])
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)

        trajectory_poses = []

        for i in range(num_points):
            theta = 2 * np.pi * i / num_points
            p_plane = v_c + radius * (np.cos(theta) * x_axis + np.sin(theta) * y_axis)
            v_idx, v_surf, n_surf = self.get_closest_vertex(p_plane)

            # Orientation: Z points INTO surface (opposite to normal)
            # Because standard convention for this demo seems to be:
            # desired_force = [0, 0, -F] means force along -Z.
            # If Z points OUT (normal), then -Z points IN.
            # BUT, in polishing_trajectory.py, f[2] = -desired_force.
            # And forces={measurement_frame: center_pose * f}
            # If center_pose Z aligns with normal (OUT), then -F on Z pushes IN. Correct.

            z_tool = -n_surf  # Z points INTO surface (opposite to Normal)

            # Tangent direction for Y (velocity)
            tangent_plane = -np.sin(theta) * x_axis + np.cos(theta) * y_axis
            tangent_surf = tangent_plane - np.dot(tangent_plane, n_surf) * n_surf
            norm_tan = np.linalg.norm(tangent_surf)
            if norm_tan > 1e-6:
                y_tool = tangent_surf / norm_tan
            else:
                y_tool = np.cross(z_tool, [1, 0, 0])
                if np.linalg.norm(y_tool) < 1e-6:
                    y_tool = np.cross(z_tool, [0, 1, 0])
                y_tool /= np.linalg.norm(y_tool)

            x_tool = np.cross(y_tool, z_tool)

            R = np.column_stack((x_tool, y_tool, z_tool))
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = v_surf

            trajectory_poses.append(T)

        return trajectory_poses
