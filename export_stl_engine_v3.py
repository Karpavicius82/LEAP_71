# -*- coding: utf-8 -*-
import math
import struct
import os
import multiprocessing

# === PURE PYTHON MINIMAL STL GENERATOR (NO DEPENDENCIES) ===
# Binary STL exporter + voxel-face mesher with MULTIPROCESSING.
#
# This script extends the SDF to a more realistic converging-diverging duct
# with end rings (flanges) and internal channels that connect into collector cavities.
# Now built for faster execution on modern CPUs to allow high-res output.


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


def sdf_sphere(x, y, z, cx, cy, cz, r):
    """SDF of a sphere."""
    return math.sqrt((x - cx)**2 + (y - cy)**2 + (z - cz)**2) - r


def sdf_cylinder_y(x, y, z, cy_start, cy_end, r):
    """SDF of a cylinder aligned with Y axis."""
    dist_xz = math.sqrt(x*x + (z - 5.0)**2) # Assuming centered at Z for offset later
    # We will compute distance inside the sdf_engine_v2
    return dist_xz - r

# -------------------------------
# Profile: converging-diverging duct (piecewise smooth)
# -------------------------------

def get_engine_profile(z, L=15.0):
    """Geometric profile R(z) for a converging-diverging duct.

    z in [0..L].
    """
    z = clamp(z, 0.0, L)

    # Radii for production prototype
    r_chamber = 4.5
    r_throat = 1.0  # Higher contraction ratio
    r_exit = 6.0    # Larger bell nozzle expansion

    # Key positions
    z_chamber_start = 1.5   # Space for the closed dome at the top (z=0 to z=1.5 is the dome)
    z_chamber_end = 3.5
    z_throat = 6.0
    z_exit = L

    if z <= z_chamber_start:
        # Dome closing the top of the combustion chamber
        # Radius goes from 0 at z=0 to r_chamber at z=z_chamber_start
        # Using a spherical profile for pressure vessel
        dome_r = r_chamber
        dome_z_center = z_chamber_start
        if z < dome_z_center - dome_r:
            return 0.0  # Before dome starts
        
        # r = sqrt(R^2 - (z - z_center)^2)
        r_val = math.sqrt(max(0.0, dome_r**2 - (z - dome_z_center)**2))
        return r_val

    if z <= z_chamber_end:
        # Cylindrical/spherical chamber
        return r_chamber

    if z <= z_throat:
        # Converging section (smooth but steeper)
        t = smoothstep(z_chamber_end, z_throat, z)
        return lerp(r_chamber, r_throat, t * t)

    # Diverging section (Bell nozzle shape - parabolic expansion)
    t = smoothstep(z_throat, z_exit, z)
    # Using t^0.65 to create a bell-like outward curve rather than a cone
    return lerp(r_throat, r_exit, t ** 0.65)


def get_wall_thickness(z, L=15.0):
    """Wall thickness with heavy end thickening (Flanges for welding/bolting)."""
    base = 1.0       # Base wall
    flange_extra = 3.0 # Heavy flange thickness

    ring_len = 2.5
    t_in = 1.0 - smoothstep(0.0, ring_len, z)
    t_out = smoothstep(L - ring_len, L, z)
    
    # Thicker throat walls because of higher pressure/heat flux
    throat_z = 6.0
    t_throat = smoothstep(throat_z - 1.5, throat_z, z) * (1.0 - smoothstep(throat_z, throat_z + 2.0, z))
    
    return base + flange_extra * max(t_in, t_out) + 0.5 * t_throat


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
    
    # Ensure slightly larger evaluation bounds so dome is fully resolved safely
    z_eval = clamp(z, 0.0, L)

    dist_center = math.sqrt(x * x + y * y)

    r_target = get_engine_profile(z_eval, L=L)
    thickness = get_wall_thickness(z_eval, L=L)

    # Shell (solid): r_target <= r <= r_target+thickness
    wall_sdf = sdf_annulus(dist_center, r_target, r_target + thickness)
    
    # The dome is completely filled at the very top (z < 0.2) to close it
    if z_eval <= 0.2:
        wall_sdf = min(wall_sdf, dist_center - (r_target + thickness))

    # Explicit heavy end rings (Flanges)
    flange_protrusion = 1.5
    ring_half_h = 1.0
    z_in_ring = 1.2    # Inlet near the top dome (Injector head)
    z_out_ring = L - 1.2 # Outlet at the nozzle base

    ring_in_sdf = max(
        sdf_annulus(dist_center, r_target - 0.2, r_target + thickness + flange_protrusion),
        sdf_z_band(z_eval, z_in_ring, ring_half_h),
    )
    ring_out_sdf = max(
        sdf_annulus(dist_center, r_target - 0.2, r_target + thickness + flange_protrusion),
        sdf_z_band(z_eval, z_out_ring, ring_half_h),
    )
    
    # Solid body = union(shell, rings)
    body_sdf = min(wall_sdf, ring_in_sdf, ring_out_sdf)

    # Add physical pipes connecting to the manifolds
    pipe_radius_solid = 1.5
    pipe_radius_void = 1.0
    
    # Pipe 1 (Inlet at nozzle bell base) - pointing along +Y axis
    # Pipe exists around z = z_out_ring
    pipe1_dist_xz = math.sqrt(x*x + (z - z_out_ring)**2)
    # Cylinder SDF extending out from the ring
    pipe1_solid = max(pipe1_dist_xz - pipe_radius_solid, -y + (r_target + thickness))
    # Cap the cylinder length
    pipe1_solid = max(pipe1_solid, y - (r_target + thickness + 6.0)) 

    # Pipe 2 (Outlet at combustion dome injector) - pointing along -Y axis
    # Pipe exists around z = z_in_ring
    pipe2_dist_xz = math.sqrt(x*x + (z - z_in_ring)**2)
    pipe2_solid = max(pipe2_dist_xz - pipe_radius_solid, y + (r_target + thickness))
    # Cap length
    pipe2_solid = max(pipe2_solid, -y - (r_target + thickness + 6.0))
    
    body_sdf = min(body_sdf, pipe1_solid, pipe2_solid)

    # ---------------------------
    # Internal voids: channels + collector cavities + pipe hollows
    # ---------------------------

    # Collector cavities: annular void slabs inside the wall near ends
    # They act as fluid manifolds
    void_band_thick = 0.8
    void_half_h = 0.8

    r_void_center = r_target + thickness * 0.55
    void_radial = abs(dist_center - r_void_center) - void_band_thick
    
    # Inlet manifold (usually at the bottom/nozzle exit in regenerative)
    # Outlet manifold (at the top/combustion chamber for injection)
    void_inlet = max(void_radial, sdf_z_band(z_eval, z_in_ring, void_half_h))
    void_outlet = max(void_radial, sdf_z_band(z_eval, z_out_ring, void_half_h))
    
    # Pipe hollow cavities
    pipe1_void = max(pipe1_dist_xz - pipe_radius_void, -y + r_void_center)
    pipe2_void = max(pipe2_dist_xz - pipe_radius_void, y + r_void_center)
    
    # Helical channels that merge into the collector cavities near ends
    channel_sdf = 1e9

    base_branches = 16  # Increased channel count
    extra_branches = 32

    # Branching happens where perimeter expands
    z_branch = 8.5
    branch_on = smoothstep(z_branch, z_branch + 1.5, z_eval)  # 0..1

    z_join = 2.4
    t_in = 1.0 - smoothstep(0.0, z_join, z_eval)
    t_out = smoothstep(L - z_join, L, z_eval)
    join = max(t_in, t_out)

    base_r_chan = 0.35
    helix_pitch = 0.15 # Steeper spiral

    # Limit channels from cutting out of the bottom dome top
    # Channels start just smoothly tapering at z=1.0 to avoid breaking inner dome
    channel_taper_dome = smoothstep(0.8, 1.5, z_eval)

    # Base channels
    for i in range(base_branches):
        angle = (float(i) / base_branches) * 2.0 * math.pi + (z_eval * helix_pitch)

        r_midwall = r_target + thickness * 0.50
        r_center = lerp(r_midwall, r_void_center, join)

        # Small waviness to increase surface area mixing
        r_center += 0.08 * math.sin(4.0 * angle + 0.3 * z_eval)

        cx = math.cos(angle) * r_center
        cy = math.sin(angle) * r_center

        # Channels get wider at the manifolds
        r_chan = base_r_chan * (1.0 + 1.2 * join) * channel_taper_dome
        
        # Flattened channels for better heat transfer (ellipse distance approx)
        c_dist = math.sqrt((x - cx) ** 2 + ((y - cy) * 0.8) ** 2) - r_chan
        channel_sdf = min(channel_sdf, c_dist)

    # Extra channels (turn on after z_branch where bell expands)
    for i in range(extra_branches):
        angle = (float(i) / extra_branches) * 2.0 * math.pi + (z_eval * helix_pitch) + (math.pi / extra_branches)

        r_midwall = r_target + thickness * 0.50
        r_center = lerp(r_midwall, r_void_center, join)

        cx = math.cos(angle) * r_center
        cy = math.sin(angle) * r_center

        r_chan = (base_r_chan * 0.8) * branch_on * (1.0 + 1.2 * join) * channel_taper_dome
        c_dist = math.sqrt((x - cx) ** 2 + ((y - cy) * 0.8) ** 2) - r_chan
        channel_sdf = min(channel_sdf, c_dist)

    # Union of void volumes
    void_sdf = min(channel_sdf, void_inlet, void_outlet, pipe1_void, pipe2_void)

    # Solid minus voids
    return max(body_sdf, -void_sdf)


# -------------------------------
# Multiprocessing Worker for Meshing
# -------------------------------

def evaluate_slice(args):
    """Worker function to evaluate a single horizontal slice of the SDF."""
    zi, z, res_xy, x_min, x_max, y_min, y_max, step_x, step_y = args
    inside_pts = set()
    for yi in range(res_xy):
        y = y_min + yi * step_y
        for xi in range(res_xy):
            x = x_min + xi * step_x
            if sdf_engine_v2(x, y, z) < 0.0:
                inside_pts.add((xi, yi, zi))
    return inside_pts


# -------------------------------
# STL + voxel-face meshing
# -------------------------------

def write_stl_binary(triangles, out_path):
    with open(out_path, 'wb') as f:
        f.write(struct.pack('80s', b'SDF Voxel Export High-Res'))
        f.write(struct.pack('<I', len(triangles)))
        for tri in triangles:
            # Flat shading normal calculation can be 0,0,0 - most CAD recalculates it
            f.write(struct.pack('<3f', 0.0, 0.0, 0.0))
            for v in tri:
                f.write(struct.pack('<3f', float(v[0]), float(v[1]), float(v[2])))
            f.write(struct.pack('<H', 0))


def generate_voxel_mesh(res_xy=1000, res_z=2000):
    """Voxel-face meshing with separate XY and Z resolutions using Multiprocessing."""
    L = 15.0

    # Bounding box - widened for pipes extending outward
    x_min, x_max = -12.0, 12.0
    y_min, y_max = -12.0, 12.0
    z_min, z_max = 0.0, L + 0.5 

    step_x = (x_max - x_min) / float(res_xy)
    step_y = (y_max - y_min) / float(res_xy)
    step_z = (z_max - z_min) / float(res_z)

    inside = set()
    
    cores = max(1, multiprocessing.cpu_count() - 1)
    print(f"Skaičiuojamas SDF laukas (res_xy={res_xy}, res_z={res_z}) naudojant {cores} CPU branduolius...")
    
    # Prepare arguments for each Z slice
    worker_args = []
    for zi in range(res_z):
        z = z_min + zi * step_z
        worker_args.append((zi, z, res_xy, x_min, x_max, y_min, y_max, step_x, step_y))
    
    # Evaluate slices in parallel
    pool = multiprocessing.Pool(processes=cores)
    results = pool.map(evaluate_slice, worker_args)
    pool.close()
    pool.join()
    
    # Merge results
    for subset in results:
        inside.update(subset)

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
    import argparse
    
    # Pridėta argumentų analizė greitam testavimui
    parser = argparse.ArgumentParser(description='SDF Raketinio Variklio Generatorius (10x Rezoliucija)')
    parser.add_argument('--test', action='store_true', help='Generuoti \u201cmažesnės\u201d rezoliucijos modelį (600x1000)')
    parser.add_argument('--prod', action='store_true', help='Generuoti aukščiausios rezoliucijos gamybinį modelį (1500x3000)')
    args = parser.parse_args()

    timestamp = int(time.time() % 100000)
    out_file = "engine_sdf_prod_%d.stl" % timestamp

    print("=== PURE PYTHON STL EXPORTER (Gamybinis Prototipas + Uždarasis Ciklas) ===")
    
    # 10x Base Resolution
    res_x = 1000
    res_z = 2000
    if args.test:
        res_x, res_z = 600, 1000
        print("[TEST MODE] 600x1000 Rezoliucija")
    elif args.prod:
        res_x, res_z = 1500, 3000
        print("[PROD MODE] 1500x3000 Aukščiausias detalumas")
    else:
        print("[DEFAULT MODE] 1000x2000 Detalumas")
        
    start_time = time.time()
    
    try:
        # Prevent multiprocessing freeze on Windows
        multiprocessing.freeze_support()
        
        tris = generate_voxel_mesh(res_xy=res_x, res_z=res_z)
        print("Sukurta trikampių: %d" % len(tris))
        
        print("Saugoma į STL failą...")
        write_stl_binary(tris, out_file)
        
        elapsed = time.time() - start_time
        print("\nSĖKMĖ: Failas sukurtas per %.1f sekundes: %s" % (elapsed, os.path.abspath(out_file)))
        print("SolidWorks importas: File -> Open -> %s -> Options -> Import as Solid Body" % out_file)
    except Exception as e:
        print("\nKLAIDA gaminant failą: %s" % str(e))

