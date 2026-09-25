#!/usr/bin/env python3
"""Turn the team's CAD export of the real airframe into Gazebo assets.

    python3 scripts/cad_to_gazebo.py "Drone frame/drone.gltf" \
        src/aerothon_sim/sim_gazebo/models/aerothon_quad

Reads the Open CASCADE glTF of the full assembly (95 parts, 1.9 M triangles,
metres) and writes, into OUT:

    meshes/body.glb        every part except the propellers, decimated,
                           coloured by part, in the body frame (FLU)
    meshes/prop_<n>.glb    each propeller centred on its own hub, scaled to
                           the prop diameter actually flown
    meshes/camera.glb      the C270, centred on itself: it rides the tilt
                           servo, so it cannot be part of the body
    airframe.json          what the vehicle model is built from: motor hubs,
                           prop plane, sensor and payload mounts, the landing
                           gear's lowest point, collision boxes, and the mass,
                           centre of mass and inertia of the airframe

WHY DECIMATE. 1.9 M triangles is the CAD's fastener threads and connector
pins. Gazebo draws the vehicle for the GUI AND for the aircraft's own camera
every frame, on a host that already runs the simulator at a quarter of real
time; the stock Iris is ~10 k. Vertex clustering per part, with the cell
scaled to the part, keeps every part recognisable at a few thousand
triangles and leaves small parts untouched.

FRAMES. The CAD is -Z up and -X forward (props, GPS mast at -Z; landing gear
at +Z; camera and lidar at -X; "Front Left" parts at +Y). The body frame is
ROS/Gazebo FLU, so x' = -x, y' = y, z' = -z: a 180 deg turn about Y, a proper
rotation, so triangle winding is kept.

MASS. The CAD carries no masses. Parts are given the team's figures where
known (all-up 2.0 kg, 4S2P 9000 mAh Li-ion pack, 2312 motors) and nominal
datasheet ones otherwise; the frame takes the remainder to make the all-up
weight. Each part is a solid box of its CAD extent for the inertia.
"""

import json
import math
import os
import struct
import sys

import numpy as np

AUW_KG = 2.0
PROP_DIAMETER_M = 0.240              # 9.45 in: the 9450 a 2312 980 KV flies
CT = {5126: np.float32, 5125: np.uint32, 5123: np.uint16, 5121: np.uint8}
NC = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}
CAD_TO_FLU = np.diag([-1.0, 1.0, -1.0])

# name keyword -> (kg, rgb). First match wins; order matters.
PARTS = [
    ("Li ion Battery", 0.560, (0.10, 0.25, 0.65)),     # 4S2P 21700, 9000 mAh
    ("Propeller", 0.014, (0.08, 0.08, 0.09)),
    ("Motor Mount", 0.006, (0.12, 0.12, 0.13)),
    ("Motor", 0.058, (0.70, 0.70, 0.72)),              # Hobbywing 2312SL
    ("PIXHAWK", 0.095, (0.55, 0.57, 0.60)),            # Pixhawk 6X
    ("raspberry_pi_5", 0.050, (0.05, 0.45, 0.12)),
    ("Logitech c270", 0.075, (0.04, 0.04, 0.05)),
    ("LD06", 0.042, (0.20, 0.20, 0.22)),
    ("Here3", 0.049, (0.05, 0.05, 0.06)),
    ("GPS_mount", 0.020, (0.25, 0.25, 0.27)),
    ("DJI O4", 0.012, (0.10, 0.10, 0.11)),
    ("ESC", 0.020, (0.10, 0.10, 0.35)),                # SpeedyBee 35 A 4-in-1
    ("Power Distribution", 0.015, (0.55, 0.10, 0.10)),
    ("Receiver", 0.008, (0.15, 0.15, 0.15)),
    ("Landing Gear", 0.045, (0.10, 0.10, 0.10)),
    ("Dropping Mechanism", 0.110, (0.85, 0.55, 0.05)),
    ("Standoff", 0.004, (0.75, 0.60, 0.20)),
    ("Arm", None, (0.06, 0.06, 0.07)),                 # frame: the remainder
    ("plate", None, (0.06, 0.06, 0.07)),
]


def spec_of(name):
    for key, kg, rgb in PARTS:
        if key.lower() in name.lower():
            return key, kg, rgb
    return "other", 0.005, (0.35, 0.35, 0.37)


# ---------------------------------------------------------------- reading ---
def load(path):
    g = json.load(open(path))
    buf = np.fromfile(os.path.join(os.path.dirname(path), g["buffers"][0]["uri"]),
                      dtype=np.uint8)

    def acc(i):
        a = g["accessors"][i]
        bv = g["bufferViews"][a["bufferView"]]
        off = bv.get("byteOffset", 0) + a.get("byteOffset", 0)
        dt = np.dtype(CT[a["componentType"]])
        n = NC[a["type"]]
        stride = bv.get("byteStride", 0) or dt.itemsize * n
        raw = buf[off: off + stride * (a["count"] - 1) + dt.itemsize * n].tobytes()
        if stride == dt.itemsize * n:
            return np.frombuffer(raw, dtype=dt).reshape(a["count"], n)
        return np.lib.stride_tricks.as_strided(
            np.frombuffer(raw, dtype=dt), (a["count"], n),
            (stride, dt.itemsize)).copy()

    def local(n):
        if "matrix" in n:
            return np.array(n["matrix"], dtype=float).reshape(4, 4).T
        M = np.eye(4)
        if "rotation" in n:
            x, y, z, w = n["rotation"]
            M[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
        if "scale" in n:
            M[:3, :3] = M[:3, :3] @ np.diag(n["scale"])
        if "translation" in n:
            M[:3, 3] = n["translation"]
        return M

    parts = {}

    def walk(i, M, top):
        n = g["nodes"][i]
        W = M @ local(n)
        name = top or n.get("name", f"node{i}")
        if "mesh" in n:
            for p in g["meshes"][n["mesh"]]["primitives"]:
                v = acc(p["attributes"]["POSITION"]).astype(float)
                v = (W[:3, :3] @ v.T).T + W[:3, 3]
                t = (acc(p["indices"]).reshape(-1, 3).astype(np.int64)
                     if "indices" in p else np.arange(len(v)).reshape(-1, 3))
                parts.setdefault(name, []).append((v @ CAD_TO_FLU.T, t))
        for c in n.get("children", []):
            walk(c, W, name)

    for i in g["scenes"][0]["nodes"]:
        walk(i, np.eye(4), None)
    out = {}
    for name, lst in parts.items():
        vs, ts, base = [], [], 0
        for v, t in lst:
            vs.append(v)
            ts.append(t + base)
            base += len(v)
        out[name] = (np.vstack(vs), np.vstack(ts))
    return out


# ------------------------------------------------------------ decimation ---
def decimate(v, t, cell):
    """Vertex clustering: vertices in one cell merge to their mean."""
    if cell <= 0:
        return v, t
    key = np.floor(v / cell).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    n = inv.max() + 1
    nv = np.zeros((n, 3))
    np.add.at(nv, inv, v)
    nv /= np.bincount(inv, minlength=n)[:, None]
    nt = inv[t]
    keep = (nt[:, 0] != nt[:, 1]) & (nt[:, 1] != nt[:, 2]) & (nt[:, 0] != nt[:, 2])
    nt = nt[keep]
    # Two triangles collapsed onto the same three vertices are one surface.
    _, first = np.unique(np.sort(nt, axis=1), axis=0, return_index=True)
    return nv, nt[np.sort(first)]


def reduce_part(v, t, target):
    """Coarsen until at or below `target` triangles (or give up)."""
    if len(t) <= target:
        return v, t
    diag = float(np.linalg.norm(v.max(0) - v.min(0)))
    cell = diag / 400.0
    for _ in range(12):
        nv, nt = decimate(v, t, cell)
        if len(nt) <= target:
            return nv, nt
        cell *= 1.5
    return nv, nt


# --------------------------------------------------------------- writing ---
def write_glb(path, prims):
    """prims: [(name, verts, tris, rgb)] -> flat-shaded GLB, one mesh."""
    bin_ = bytearray()
    views, accs, gprims, mats = [], [], [], []

    def add(arr, target):
        nonlocal bin_
        while len(bin_) % 4:
            bin_ += b"\0"
        off = len(bin_)
        b = arr.tobytes()
        bin_ += b
        views.append({"buffer": 0, "byteOffset": off, "byteLength": len(b),
                      "target": target})
        return len(views) - 1

    for name, v, t, rgb in prims:
        tri = v[t]                                   # (n, 3, 3) de-indexed
        nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        ln = np.linalg.norm(nrm, axis=1, keepdims=True)
        nrm = np.where(ln > 0, nrm / np.maximum(ln, 1e-12), [0, 0, 1])
        pos = tri.reshape(-1, 3).astype(np.float32)
        nor = np.repeat(nrm, 3, axis=0).astype(np.float32)
        pv = add(pos, 34962)
        accs.append({"bufferView": pv, "componentType": 5126, "count": len(pos),
                     "type": "VEC3", "min": pos.min(0).tolist(),
                     "max": pos.max(0).tolist()})
        pa = len(accs) - 1
        nv = add(nor, 34962)
        accs.append({"bufferView": nv, "componentType": 5126, "count": len(nor),
                     "type": "VEC3"})
        mats.append({"name": name, "doubleSided": True,
                     "pbrMetallicRoughness": {"baseColorFactor": list(rgb) + [1.0],
                                              "metallicFactor": 0.1,
                                              "roughnessFactor": 0.7}})
        gprims.append({"attributes": {"POSITION": pa, "NORMAL": len(accs) - 1},
                       "material": len(mats) - 1})
    gl = {"asset": {"version": "2.0", "generator": "aerothon cad_to_gazebo.py"},
          "scene": 0, "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}],
          "meshes": [{"primitives": gprims}], "materials": mats,
          "accessors": accs, "bufferViews": views,
          "buffers": [{"byteLength": len(bin_)}]}
    js = json.dumps(gl).encode()
    js += b" " * ((4 - len(js) % 4) % 4)
    while len(bin_) % 4:
        bin_ += b"\0"
    with open(path, "wb") as f:
        f.write(struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(js) + 8 + len(bin_)))
        f.write(struct.pack("<II", len(js), 0x4E4F534A) + js)
        f.write(struct.pack("<II", len(bin_), 0x004E4942) + bytes(bin_))


def box_of(v):
    lo, hi = v.min(0), v.max(0)
    return ((lo + hi) / 2).tolist(), (hi - lo).tolist()


def main():
    src, out = sys.argv[1], sys.argv[2]
    os.makedirs(os.path.join(out, "meshes"), exist_ok=True)
    parts = load(src)

    # ---- mounts, measured ----
    def centre(key):
        vs = [v for n, (v, _) in parts.items() if key.lower() in n.lower()]
        return box_of(np.vstack(vs)) if vs else None

    motors = {}
    for n, (v, _) in parts.items():
        if n.endswith(" Motor"):
            c, s = box_of(v)
            motors[n.replace(" Motor", "")] = {"hub": c, "top": c[2] + s[2] / 2}
    props = {n.replace(" Propeller", ""): parts[n] for n in parts
             if n.endswith(" Propeller")}
    prop_z = float(np.mean([box_of(v)[0][2] for v, _ in props.values()]))
    cad_prop_d = float(np.mean([max(box_of(v)[1][:2]) for v, _ in props.values()]))
    allv = np.vstack([v for v, _ in parts.values()])
    gear = np.vstack([v for n, (v, _) in parts.items() if "Landing Gear" in n])

    # ---- body mesh ----
    prims, total = [], 0
    for n, (v, t) in parts.items():
        if n.endswith(" Propeller") or "Logitech c270" in n:
            continue
        key, _, rgb = spec_of(n)
        nv, nt = reduce_part(v, t, target=4000 if len(t) > 20000 else 2500)
        total += len(nt)
        prims.append((n, nv, nt, rgb))
    write_glb(os.path.join(out, "meshes", "body.glb"), prims)

    # ---- props: centred on their hub, scaled to the flown diameter ----
    k = PROP_DIAMETER_M / cad_prop_d
    prop_files = {}
    for corner, (v, t) in props.items():
        hub = motors[corner]["hub"]
        pv = v - np.array([hub[0], hub[1], prop_z])
        pv[:, :2] *= k
        nv, nt = reduce_part(pv, t, target=1500)
        fn = f"prop_{corner.lower().replace(' ', '_')}.glb"
        write_glb(os.path.join(out, "meshes", fn),
                  [(corner, nv, nt, spec_of("Propeller")[2])])
        prop_files[corner] = fn

    # ---- the camera: its own mesh, centred where it pivots ----
    cam_v, cam_t = parts[next(n for n in parts if "Logitech c270" in n)]
    cam_c = np.array(box_of(cam_v)[0])
    nv, nt = reduce_part(cam_v - cam_c, cam_t, target=3000)
    write_glb(os.path.join(out, "meshes", "camera.glb"),
              [("Logitech c270", nv, nt, spec_of("Logitech c270")[2])])

    # ---- mass properties ----
    masses = {}
    for n, (v, _) in parts.items():
        key, kg, _ = spec_of(n)
        masses[n] = kg
    fixed = sum(m for m in masses.values() if m is not None)
    frame_parts = [n for n, m in masses.items() if m is None]
    frame_area = sum(np.prod(sorted(box_of(parts[n][0])[1])[1:]) for n in frame_parts)
    for n in frame_parts:
        masses[n] = (AUW_KG - fixed) * np.prod(sorted(box_of(parts[n][0])[1])[1:]) / frame_area
    rotor_parts = [n for n in parts if n.endswith(" Propeller")]
    body = {n: m for n, m in masses.items() if n not in rotor_parts}
    M = sum(body.values())
    com = sum(np.array(box_of(parts[n][0])[0]) * m for n, m in body.items()) / M
    I = np.zeros((3, 3))
    for n, m in body.items():
        c, s = box_of(parts[n][0])
        a, b, cc = s
        Ib = m / 12.0 * np.diag([b * b + cc * cc, a * a + cc * cc, a * a + b * b])
        d = np.array(c) - com
        I += Ib + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))

    # ---- collision boxes: the core stack, each skid, the drop mechanism ----
    core = np.vstack([parts[n][0] for n in parts
                      if any(k2 in n for k2 in ("plate", "Battery", "PIXHAWK", "raspberry",
                                                 "ESC", "Receiver", "Power"))])
    lg = [box_of(parts[n][0]) for n in parts if "Landing Gear" in n]
    arms = [box_of(parts[n][0]) for n in parts if n.endswith(" Arm")]

    lay = {
        "source": os.path.basename(src),
        "frame": "FLU, metres, origin at the CAD origin (centre plate)",
        "auw_kg": AUW_KG, "body_mass_kg": M,
        "rotor_mass_kg": sum(masses[n] for n in rotor_parts) / len(rotor_parts),
        "com": com.tolist(),
        "inertia": {"ixx": I[0, 0], "iyy": I[1, 1], "izz": I[2, 2],
                    "ixy": I[0, 1], "ixz": I[0, 2], "iyz": I[1, 2]},
        "motors": motors, "prop_plane_z": prop_z,
        "prop_diameter_m": PROP_DIAMETER_M, "cad_prop_diameter_m": cad_prop_d,
        "prop_meshes": prop_files,
        "camera": centre("Logitech c270"), "lidar": centre("LD06"),
        "gps": centre("Here3"), "drop_mechanism": centre("Dropping Mechanism"),
        "gear_bottom_z": float(gear[:, 2].min()),
        # LD06: the scan window sits ~12 mm below the top of the unit.
        "lidar_scan_z": float(box_of(np.vstack([v for n, (v, _) in parts.items()
                                                 if "LD06" in n]))[0][2]
                              + box_of(np.vstack([v for n, (v, _) in parts.items()
                                                   if "LD06" in n]))[1][2] / 2 - 0.012),
        # The hook: high enough that a 0.08 m payload clears the ground with the
        # aircraft on its gear (1 cm), which puts it inside the mechanism.
        "hook_z": float(gear[:, 2].min()) + 0.08 + 0.012 + 0.011,
        "prop_top_z": float(max(v[:, 2].max() for v, _ in props.values())),
        "bounds": [allv.min(0).tolist(), allv.max(0).tolist()],
        "collision": {"core": box_of(core), "skids": lg, "arms": arms,
                      "drop": centre("Dropping Mechanism")},
        "body_triangles": int(total),
    }
    json.dump(lay, open(os.path.join(out, "airframe.json"), "w"), indent=1)
    print(f"body {total} triangles; props x{len(prop_files)}; mass {M:.3f} kg + rotors; "
          f"com {np.round(com, 3)}; I diag {np.round(np.diag(I), 5)}")


if __name__ == "__main__":
    main()
