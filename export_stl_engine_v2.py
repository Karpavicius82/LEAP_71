# -*- coding: utf-8 -*-
import math
import struct
import os

# === PURE PYTHON MINIMAL STL GENERATOR (NO DEPENDENCIES) ===
# Binary STL exporter + voxel-face mesher.
#
# This script keeps the same "pure python" approach as export_stl.py,
# but extends the SDF to a more realistic converging-diverging duct
# with end rings and internal channels that connect into collector cavities.
#
# Scope: geometric modeling only (no combustion / injector / propellant guidance).


# -------------------------------
# Helpers
# -------------------------------

def clamp(x, a, b):
    return a if x < a else (b if x > b else x)


def lerp(a, b, t):
    return a + (b - a) * t


def smoothstep(edge0, edge1, x):
    if edge0 == edge1:
        return 0.0
    t = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


# -------------------------------
# Profile: converging-diverging duct (piecewise smooth)
# -------------------------------

def get_engine_profile(z, L=15.0):
    """Geometric profile R(z) for a converging-diverging duct.

    z in [0..L]. Numbers are only shape controls.
    """
    z = clamp(z, 0.0, L)

    # Radii
    r_chamber = 4.0
    r_throat = 1.2
    r_exit = 4.2

    # Key positions
    z_chamber_end = 3.0
    z_throat = 6.0
    z_exit = L

    if z <= z_chamber_end:
        return r_chamber

    if z <= z_throat:
        t = smoothstep(z_chamber_end, z_throat, z)
        return lerp(r_chamber, r_throat, t * t)

    t = smoothstep(z_throat, z_exit, z)
    return lerp(r_throat, r_exit, t ** 1.5)


def get_wall_thickness(z, L=15.0):
    """Wall thickness with end thickening (manifold-like rings)."""
    base = 1.2
    ring_extra = 1.0

    ring_len = 1.6
    t_in = 1.0 - smoothstep(0.0, ring_len, z)
    t_out = smoothstep(L - ring_len, L, z)
    return base + ring_extra * max(t_in, t_out)


# -------------------------------
# SDF blocks
# -------------------------------

def sdf_annulus(dist_center, r_inner, r_outer):
    """SDF of a solid annulus region: r_inner <= r <= r_outer."""
    return max(dist_center - r_outer, r_inner - dist_center)


def sdf_z_band(z, z0, half_h):
    """SDF of a slab/band around z0 with half-height half_h."""
    return abs(z - z0) - half_h


# -------------------------------
# Main SDF
# -------------------------------

def sdf_engine_v2(x, y, z):
    """Negative => inside SOLID."""
    L = 15.0
    z = clamp(z, 0.0, L)

    dist_center = math.sqrt(x * x + y * y)

    r_target = get_engine_profile(z, L=L)
    thickness = get_wall_thickness(z, L=L)

    # Shell (solid): r_target <= r <= r_target+thickness
    wall_sdf = sdf_annulus(dist_center, r_target, r_target + thickness)

    # Explicit end rings (helps CAD feel)
    flange = 1.0
    ring_half_h = 0.7
    z_in_ring = 0.9
    z_out_ring = L - 0.9

    ring_in_sdf = max(
        sdf_annulus(dist_center, r_target, r_target + thickness + flange),
        sdf_z_band(z, z_in_ring, ring_half_h),
    )
    ring_out_sdf = max(
        sdf_annulus(dist_center, r_target, r_target + thickness + flange),
        sdf_z_band(z, z_out_ring, ring_half_h),
    )

    # Solid body = union(shell, rings)
    body_sdf = min(wall_sdf, ring_in_sdf, ring_out_sdf)

    # ---------------------------
    # Internal voids: channels + collector cavities
    # ---------------------------

    # Collector cavities: annular void slabs inside the wall near ends
    void_band_thick = 0.55
    void_half_h = 0.55

    r_void_center = r_target + thickness * 0.55
    void_radial = abs(dist_center - r_void_center) - void_band_thick
    void_inlet = max(void_radial, sdf_z_band(z, z_in_ring, void_half_h))
    void_outlet = max(void_radial, sdf_z_band(z, z_out_ring, void_half_h))

    # Helical channels that merge into the collector cavities near ends
    channel_sdf = 1e9

    base_branches = 12
    extra_branches = 24

    z_branch = 7.0
    branch_on = smoothstep(z_branch, z_branch + 1.2, z)  # 0..1

    z_join = 1.6
    t_in = 1.0 - smoothstep(0.0, z_join, z)
    t_out = smoothstep(L - z_join, L, z)
    join = max(t_in, t_out)

    base_r_chan = 0.35

    # Base channels
    for i in range(base_branches):
        angle = (float(i) / base_branches) * 2.0 * math.pi + (z * 0.12)

        r_midwall = r_target + thickness * 0.50
        r_center = lerp(r_midwall, r_void_center, join)

        # Small waviness (vascular-like)
        r_center += 0.12 * math.sin(3.0 * angle + 0.25 * z)

        cx = math.cos(angle) * r_center
        cy = math.sin(angle) * r_center

        r_chan = base_r_chan * (1.0 + 0.85 * join)
        c_dist = math.sqrt((x - cx) ** 2 + (y - cy) ** 2) - r_chan
        channel_sdf = min(channel_sdf, c_dist)

    # Extra channels (turn on after z_branch)
    for i in range(extra_branches):
        angle = (float(i) / extra_branches) * 2.0 * math.pi + (z * 0.12) + (math.pi / extra_branches)

        r_midwall = r_target + thickness * 0.52
        r_center = lerp(r_midwall, r_void_center, join)
        r_center += 0.10 * math.sin(5.0 * angle + 0.18 * z)

        cx = math.cos(angle) * r_center
        cy = math.sin(angle) * r_center

        r_chan = (base_r_chan * 0.75) * branch_on * (1.0 + 0.6 * join)
        c_dist = math.sqrt((x - cx) ** 2 + (y - cy) ** 2) - r_chan
        channel_sdf = min(channel_sdf, c_dist)

    # Union of void volumes
    void_sdf = min(channel_sdf, void_inlet, void_outlet)

    # Solid minus voids
    return max(body_sdf, -void_sdf)


# -------------------------------
# STL + voxel-face meshing
# -------------------------------

def write_stl_binary(triangles, out_path):
    with open(out_path, 'wb') as f:
        f.write(struct.pack('80s', b'SDF Voxel Export'))
        f.write(struct.pack('<I', len(triangles)))
        for tri in triangles:
            f.write(struct.pack('<3f', 0.0, 0.0, 0.0))
            for v in tri:
                f.write(struct.pack('<3f', float(v[0]), float(v[1]), float(v[2])))
            f.write(struct.pack('<H', 0))


def generate_voxel_mesh(res_xy=70, res_z=120):
    """Voxel-face meshing with separate XY and Z resolutions."""
    L = 15.0

    # Bounding box (tune if you change radii)
    x_min, x_max = -9.0, 9.0
    y_min, y_max = -9.0, 9.0
    z_min, z_max = 0.0, L

    step_x = (x_max - x_min) / float(res_xy)
    step_y = (y_max - y_min) / float(res_xy)
    step_z = (z_max - z_min) / float(res_z)

    inside = set()

    print("Skaiciuojamas SDF laukas (res_xy=%d, res_z=%d)..." % (res_xy, res_z))
    for zi in range(res_z):
        z = z_min + zi * step_z
        for yi in range(res_xy):
            y = y_min + yi * step_y
            for xi in range(res_xy):
                x = x_min + xi * step_x
                if sdf_engine_v2(x, y, z) < 0.0:
                    inside.add((xi, yi, zi))

    print("Generuojami tinklelio trikampiai...")
    triangles = []

    def in_bounds(ix, iy, iz):
        return (0 <= ix < res_xy) and (0 <= iy < res_xy) and (0 <= iz < res_z)

    faces = [
        (+1, 0, 0, [(1,0,0), (1,1,0), (1,1,1), (1,0,1)]),
        (-1, 0, 0, [(0,0,0), (0,0,1), (0,1,1), (0,1,0)]),
        (0, +1, 0, [(0,1,0), (0,1,1), (1,1,1), (1,1,0)]),
        (0, -1, 0, [(0,0,0), (1,0,0), (1,0,1), (0,0,1)]),
        (0, 0, +1, [(0,0,1), (1,0,1), (1,1,1), (0,1,1)]),
        (0, 0, -1, [(0,0,0), (0,1,0), (1,1,0), (1,0,0)]),
    ]

    for (xi, yi, zi) in inside:
        for dx, dy, dz, face_verts in faces:
            nxi, nyi, nzi = xi + dx, yi + dy, zi + dz
            if (not in_bounds(nxi, nyi, nzi)) or ((nxi, nyi, nzi) not in inside):
                v = []
                for fx, fy, fz in face_verts:
                    v.append((
                        x_min + (xi + fx) * step_x,
                        y_min + (yi + fy) * step_y,
                        z_min + (zi + fz) * step_z,
                    ))
                triangles.append([v[0], v[1], v[2]])
                triangles.append([v[0], v[2], v[3]])

    return triangles


if __name__ == "__main__":
    import time

    timestamp = int(time.time() % 100000)
    out_file = "engine_sdf_%d.stl" % timestamp

    print("=== PURE PYTHON STL EXPORTER (engine_v2 geometry) ===")
    try:
        tris = generate_voxel_mesh(res_xy=70, res_z=120)
        print("Sukurta trikampiu: %d" % len(tris))
        write_stl_binary(tris, out_file)
        print("\nSEKME: Failas sukurtas: %s" % os.path.abspath(out_file))
        print("SolidWorks importas: File -> Open -> %s -> Options -> Import as Solid Body" % out_file)
    except Exception as e:
        print("\nKLAIDA gaminant faila: %s" % str(e))
