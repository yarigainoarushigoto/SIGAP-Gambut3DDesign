#!/usr/bin/env python3
"""
Model 3D parametrik - Node Sensor IoT Gambut (PV + LoRa)
Gaya mengikuti Alat Ukur TMAT Lite: tiang galvanis + tripod, panel surya di atas,
kerangkeng kuning berisi box ABS IP65, sensor di sekitar tiang, sumur pantau di tanah.

Cara pakai:
    python3 model_node_sensor.py          -> menulis semua file ke folder ./output
Ubah angka di blok PARAMETER untuk mengubah desain (semua satuan mm, derajat).

Keluaran:
    node_sensor.glb   - Blender / Windows 3D Viewer / online viewer (warna + nama part)
    node_sensor.obj   - + .mtl, untuk SketchUp, Fusion 360, Tinkercad, FreeCAD (mesh)
    node_sensor.stl   - satu mesh gabungan (mis. untuk SolidWorks / slicer)
    node_sensor.scad  - solid CSG untuk OpenSCAD (bisa dibuka FreeCAD -> ekspor STEP)
    viewer_data.json  - data untuk viewer HTML
Sumbu: Z ke atas, Y ke belakang (depan alat menghadap -Y), titik nol = kaki tiang di muka tanah.
"""
import json, math, os, struct, sys
import numpy as np

# =====================================================================
# PARAMETER (mm)
# =====================================================================
P = dict(
    # Tiang & tripod (pipa galvanis 2" seperti brosur)
    pole_d=60.3, pole_top=2450, pole_bottom=-400,
    leg_d=48.3, leg_count=3, leg_angles=(15, 135, 255), leg_top_z=1040, leg_foot_r=760, leg_foot_z=-150,
    collar_z=1000, collar_h=100,
    # Kerangkeng + box (pusat kerangkeng di ketinggian cage_z)
    cage_w=440, cage_d=240, cage_h=460, cage_z=1550, cage_gap=30, wire_pitch=55, wire_t=4, cage_bar=20,
    box_w=360, box_d=160, box_h=380,
    # Panel surya 50 Wp mono
    pv_l=670, pv_w=540, pv_t=30, pv_tilt=12, pv_z=2250, pv_cells=(9, 4),
    # Rain gauge tipping bucket (diameter kolektor 200 mm, brosur BGT-ABS25)
    rg_d=216, rg_h=270,
    # Radiation shield DHT22
    dht_arm_z=1900, dht_arm_len=420, dht_plates=7,
    # Tanah gambut cutaway + sumur pantau
    soil_r=1100, soil_depth=1300, water_z=-650,
    well_x=480, well_y=-460, well_od=89, well_wall=3.5, well_top=300, well_bottom=-1250,
    probe_x=760, probe_z=-320,
    ground_rod=(-350, 320),
)

# =====================================================================
# Warna & material
# =====================================================================
C = dict(
    galv=("B9BFC5", 0.65, 0.40), yellow=("F2B800", 0.0, 0.45), abs=("ECEEEE", 0.0, 0.55),
    lid=("F7F8F8", 0.0, 0.5), orange=("EF7D1A", 0.0, 0.5), dark=("2A2D31", 0.0, 0.6),
    black=("1B1C1E", 0.0, 0.5), rubber=("151515", 0.0, 0.8), steel=("D3D7DB", 0.85, 0.30),
    alu=("AEB4BA", 0.8, 0.35), cell=("16243B", 0.2, 0.15), busbar=("8E9AA8", 0.7, 0.3),
    backsheet=("E6E6E6", 0.0, 0.7), white=("F3F3F0", 0.0, 0.6), pvc=("9AA3AB", 0.0, 0.55),
    pvccap=("6F7880", 0.0, 0.55), water=("3C8DD6", 0.1, 0.1), peat=("3F2F22", 0.0, 0.95),
    wetpeat=("2A1F17", 0.0, 0.9), litter=("5E5233", 0.0, 0.95), copper=("C77B4A", 0.8, 0.35),
    brass=("C9A227", 0.8, 0.35), gland=("5B6168", 0.0, 0.5), green=("1F7A3A", 0.0, 0.5),
    battery=("3A6EA5", 0.0, 0.5), pcb=("1D5C3A", 0.0, 0.5), blue=("2457A6", 0.0, 0.5),
    gy=("7DB83A", 0.0, 0.6), qr=("111111", 0.0, 0.6),
)

# =====================================================================
# Transformasi
# =====================================================================
def T(x=0, y=0, z=0):
    m = np.eye(4); m[:3, 3] = [x, y, z]; return m

def Rx(a):
    a = math.radians(a); c, s = math.cos(a), math.sin(a)
    m = np.eye(4); m[1, 1] = c; m[1, 2] = -s; m[2, 1] = s; m[2, 2] = c; return m

def Ry(a):
    a = math.radians(a); c, s = math.cos(a), math.sin(a)
    m = np.eye(4); m[0, 0] = c; m[0, 2] = s; m[2, 0] = -s; m[2, 2] = c; return m

def Rz(a):
    a = math.radians(a); c, s = math.cos(a), math.sin(a)
    m = np.eye(4); m[0, 0] = c; m[0, 1] = -s; m[1, 0] = s; m[1, 1] = c; return m

def align_z(d):
    """Matriks rotasi yang memutar sumbu +Z ke arah d."""
    d = np.asarray(d, float); d = d / np.linalg.norm(d)
    z = np.array([0, 0, 1.0]); v = np.cross(z, d); c = float(np.dot(z, d))
    m = np.eye(4)
    if np.linalg.norm(v) < 1e-9:
        if c < 0: m = Rx(180)
        return m
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    R = np.eye(3) + vx + vx @ vx * (1 / (1 + c))
    m[:3, :3] = R; return m

# =====================================================================
# Primitif (disimpan sebagai data agar bisa diekspor ke mesh dan ke SCAD)
# =====================================================================
class Prim:
    def __init__(self, kind, M, **a):
        self.kind, self.M, self.a = kind, M, a

PARTS = []      # list of dict(name, group, mat, prims)
ANCHORS = []    # label callout: (teks, [x,y,z])

def part(name, group, mat):
    p = dict(name=name, group=group, mat=mat, prims=[]); PARTS.append(p); return p

def box(p, sx, sy, sz, M=None):
    p["prims"].append(Prim("box", np.eye(4) if M is None else M, s=(sx, sy, sz)))

def cyl(p, r1, h, M=None, r2=None, seg=32, a0=0, a1=360, caps=True):
    p["prims"].append(Prim("cyl", np.eye(4) if M is None else M, r1=r1, r2=r1 if r2 is None else r2,
                           h=h, seg=seg, a0=a0, a1=a1, caps=caps))

def tube(p, ro, ri, h, M=None, seg=32, a0=0, a1=360):
    p["prims"].append(Prim("tube", np.eye(4) if M is None else M, ro=ro, ri=ri, h=h, seg=seg, a0=a0, a1=a1))

def sphere(p, r, M=None, seg=12):
    p["prims"].append(Prim("sphere", np.eye(4) if M is None else M, r=r, seg=seg))

def seg_cyl(p, p0, p1, r, seg=16):
    p0 = np.asarray(p0, float); p1 = np.asarray(p1, float)
    L = float(np.linalg.norm(p1 - p0))
    cyl(p, r, L, T(*p0) @ align_z(p1 - p0), seg=seg)

def cable(p, pts, r=3.2):
    for a, b in zip(pts[:-1], pts[1:]):
        seg_cyl(p, a, b, r, seg=10)
    for q in pts[1:-1]:
        sphere(p, r, T(*q), seg=8)

def xf(M, pt):
    return (M @ np.array([*pt, 1.0]))[:3]

# =====================================================================
# MODEL
# =====================================================================
def build():
    g = P
    pr = g["pole_d"] / 2

    # ---------------- Lingkungan: tanah gambut cutaway ----------------
    R, D, W = g["soil_r"], g["soil_depth"], g["water_z"]
    soil_top = part("Tanah - lapisan serasah", "Lingkungan", "litter")
    soil = part("Tanah gambut (zona tak jenuh)", "Lingkungan", "peat")
    wet = part("Tanah gambut jenuh air", "Lingkungan", "wetpeat")
    for (x0, x1, y0, y1) in [(-R, 0, -R, R), (0, R, 0, R)]:
        cx, cy, sx, sy = (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0
        box(soil_top, sx, sy, 30, T(cx, cy, -15))
        box(soil, sx, sy, -W - 30, T(cx, cy, (W - 30) / 2))
        box(wet, sx, sy, D + W, T(cx, cy, (-D + W) / 2))
    wl = part("Garis muka air tanah", "Lingkungan", "water")
    box(wl, 3, R, 10, T(1.5, -R / 2, W))
    box(wl, R, 3, 10, T(R / 2, -1.5, W))
    ANCHORS.append(("Muka air tanah", [0, -R * 0.75, W]))

    # ---------------- Struktur: tiang + tripod ----------------
    pole = part("Tiang utama galvanis 2in", "Struktur", "galv")
    cyl(pole, pr, g["pole_top"] - g["pole_bottom"], T(0, 0, g["pole_bottom"]), seg=40)
    cyl(pole, pr + 2, 6, T(0, 0, g["pole_top"]), seg=40)
    ANCHORS.append(("Tiang galvanis 2\"", [0, -pr, 700]))

    collar = part("Klem tripod", "Struktur", "galv")
    tube(collar, pr + 10, pr, g["collar_h"], T(0, 0, g["collar_z"]), seg=40)
    for i, a in enumerate(g["leg_angles"]):
        th = math.radians(a)
        top = np.array([(pr + 14) * math.cos(th), (pr + 14) * math.sin(th), g["leg_top_z"]])
        foot = np.array([g["leg_foot_r"] * math.cos(th), g["leg_foot_r"] * math.sin(th), g["leg_foot_z"]])
        box(collar, 30, 16, 70, T(*((pr + 10) * np.array([math.cos(th), math.sin(th), 0]) + [0, 0, g["collar_z"] + 50])) @ Rz(a))
        leg = part(f"Kaki tripod {i+1}", "Struktur", "galv")
        seg_cyl(leg, top, foot, g["leg_d"] / 2, seg=32)
        cyl(leg, g["leg_d"] / 2 + 1.5, 4, T(*top) @ align_z(foot - top), seg=32)
    ANCHORS.append(("Kaki tripod (pipa galvanis)", list(np.array([g["leg_foot_r"] * 0.55 * math.cos(math.radians(255)), g["leg_foot_r"] * 0.55 * math.sin(math.radians(255)), 400]))))

    # ---------------- Kerangkeng kuning ----------------
    cw, cd, ch, cz = g["cage_w"], g["cage_d"], g["cage_h"], g["cage_z"]
    yb = -(pr + g["cage_gap"])          # muka belakang kerangkeng
    yf = yb - cd                         # muka depan
    yc = (yb + yf) / 2
    bar, wt, pitch = g["cage_bar"], g["wire_t"], g["wire_pitch"]

    cage = part("Kerangkeng besi (kuning)", "Box & kerangkeng", "yellow")
    box(cage, cw, 4, ch, T(0, yb - 2, cz))                      # pelat belakang
    for sx in (-1, 1):
        for sz in (-1, 1):
            box(cage, bar, cd, bar, T(sx * (cw - bar) / 2, yc, cz + sz * (ch - bar) / 2))   # rusuk Y
            box(cage, cw, bar, bar, T(0, yb - bar / 2 if sx < 0 else yf + bar / 2, cz + sz * (ch - bar) / 2))
            box(cage, bar, bar, ch, T(sx * (cw - bar) / 2, yb - bar / 2 if sz < 0 else yf + bar / 2, cz))
    mesh = part("Jaring kerangkeng", "Box & kerangkeng", "yellow")
    nx = int(cw // pitch); nz = int(ch // pitch); ny = int(cd // pitch)
    for i in range(1, nx):            # muka depan: kawat vertikal
        x = -cw / 2 + i * cw / nx
        box(mesh, wt, wt, ch - bar, T(x, yf + bar / 2, cz))
        box(mesh, wt, cd - bar, wt, T(x, yc, cz + ch / 2 - bar / 2))     # atas
    for k in range(1, nz):            # muka depan: kawat horizontal + sisi
        z = cz - ch / 2 + k * ch / nz
        box(mesh, cw - bar, wt, wt, T(0, yf + bar / 2, z))
        for sx in (-1, 1):
            box(mesh, wt, cd - bar, wt, T(sx * (cw / 2 - bar / 2), yc, z))
    for j in range(1, ny + 1):        # sisi & atas: kawat arah Z / X
        y = yb - j * cd / (ny + 1)
        for sx in (-1, 1):
            box(mesh, wt, wt, ch - bar, T(sx * (cw / 2 - bar / 2), y, cz))
        box(mesh, cw - bar, wt, wt, T(0, y, cz + ch / 2 - bar / 2))
    for x in (-cw / 4, cw / 4):       # bawah: dua palang saja (jalur kabel)
        box(mesh, bar * 0.8, cd - bar, bar * 0.6, T(x, yc, cz - ch / 2 + bar / 2))

    hw = part("Engsel & hasp pintu", "Box & kerangkeng", "galv")
    for z in (cz - ch / 3, cz + ch / 3):
        cyl(hw, 7, 50, T(-cw / 2 - 3, yf - 3, z - 25), seg=16)
    box(hw, 26, 4, 50, T(cw / 2 - 6, yf - 4, cz))
    box(hw, 8, 20, 30, T(cw / 2 + 8, yf + 6, cz))
    lock = part("Gembok pengaman", "Box & kerangkeng", "brass")
    lx, ly, lz = cw / 2 + 8, yf - 4, cz - 45
    box(lock, 40, 18, 34, T(lx, ly, lz))
    cable(lock, [[lx - 12, ly, lz + 17], [lx - 12, ly, lz + 40], [lx + 12, ly, lz + 40], [lx + 12, ly, lz + 17]], r=3.5)
    ANCHORS.append(("Kerangkeng besi + gembok", [cw / 2, yf, cz + ch / 2 - 60]))

    br = part("Bracket U-bolt box", "Box & kerangkeng", "galv")
    for dz in (-150, 150):
        tube(br, pr + 6, pr + 0.2, 30, T(0, 0, cz + dz - 15), seg=32)
        box(br, 60, -yb - pr + 4, 30, T(0, (yb + (-pr)) / 2, cz + dz))

    # ---------------- Box ABS IP65 ----------------
    bw, bd, bh = g["box_w"], g["box_d"], g["box_h"]
    yb2 = yb - 10; ylid = yb2 - bd + 17; yfront = yb2 - bd
    body = part("Box ABS IP65 - badan", "Box & kerangkeng", "abs")
    db, ybc, wt3 = bd - 17, (yb2 + ylid) / 2, 3      # badan berongga, tebal dinding 3 mm
    box(body, bw, wt3, bh, T(0, yb2 - wt3 / 2, cz))
    for s_ in (-1, 1):
        box(body, wt3, db, bh, T(s_ * (bw / 2 - wt3 / 2), ybc, cz))
        box(body, bw - 2 * wt3, db, wt3, T(0, ybc, cz + s_ * (bh / 2 - wt3 / 2)))
    seam = part("Box ABS - garis seal", "Box & kerangkeng", "dark")
    box(seam, bw + 1, 2, bh + 1, T(0, ylid, cz))
    lid = part("Box ABS IP65 - tutup", "Box & kerangkeng", "lid")
    box(lid, bw, 16, bh, T(0, (ylid + yfront) / 2, cz))
    scr = part("Baut tutup box", "Box & kerangkeng", "steel")
    for sx in (-1, 1):
        for sz in (-1, 1):
            cyl(scr, 6, 3, T(sx * (bw / 2 - 18), yfront, cz + sz * (bh / 2 - 18)) @ Rx(90), seg=16)
    lab = part("Label oranye", "Box & kerangkeng", "orange")
    box(lab, 220, 1.5, 120, T(0, yfront - 0.8, cz + 95))
    qr = part("Label - QR & teks", "Box & kerangkeng", "white")
    box(qr, 70, 1.5, 70, T(-60, yfront - 1.6, cz + 95))
    for i in range(3):
        box(qr, 90, 1.5, 9, T(45, yfront - 1.6, cz + 120 - i * 22))
    qrd = part("Label - pola QR", "Box & kerangkeng", "qr")
    rng = np.random.default_rng(7)
    cell = 70 / 9
    for i in range(9):
        for j in range(9):
            finder = (i < 3 and j < 3) or (i > 5 and j < 3) or (i < 3 and j > 5)
            on = (finder and not (i % 6 == 1 and j % 6 == 1)) or (not finder and rng.random() < 0.45)
            if on:
                box(qrd, cell, 1.5, cell, T(-60 - 35 + cell * (i + 0.5), yfront - 2.4, cz + 95 + 35 - cell * (j + 0.5)))
    btn = part("Saklar on/off", "Box & kerangkeng", "dark")
    cyl(btn, 9, 8, T(bw / 2, yfront + 40, cz - 120) @ Ry(90), seg=20)
    ANCHORS.append(("Box ABS IP65 (ESP32 LoRa, MPPT, LiFePO4)", [-bw / 2 + 30, yfront, cz - 40]))

    gl = part("Cable gland", "Box & kerangkeng", "gland")
    gxs = [-140, -70, 0, 70, 140]
    gy = (yb2 + ylid) / 2
    zb = cz - bh / 2
    for x in gxs:
        cyl(gl, 13, 8, T(x, gy, zb) @ Rx(180), seg=6)
        cyl(gl, 10, 18, T(x, gy, zb - 8) @ Rx(180), r2=7, seg=20)

    ant = part("Antena LoRa 915 MHz 3 dBi", "Box & kerangkeng", "black")
    ax, ay = 137, gy
    zt = cz + bh / 2
    cyl(ant, 8, 12, T(ax, ay, zt), seg=16)
    cyl(ant, 7, 22, T(ax, ay, zt + 12), seg=16)
    cyl(ant, 6.5, 200, T(ax, ay, zt + 34), r2=5, seg=16)
    sphere(ant, 5, T(ax, ay, zt + 234))
    ANCHORS.append(("Antena LoRa 920-923 MHz", [ax, ay, zt + 200]))

    # Isi box (terlihat bila tutup disembunyikan)
    ins_y = yb2 - 3
    plate = part("Isi - pelat dudukan", "Isi box", "orange")
    box(plate, bw - 30, 3, bh - 30, T(0, ins_y - 1.5, cz))
    bat = part("Isi - baterai LiFePO4 12,8V 20Ah", "Isi box", "battery")
    box(bat, 181, 77, 167, T(-80, ins_y - 3 - 38.5, cz - 80))
    term = part("Isi - terminal baterai", "Isi box", "steel")
    for dx in (-60, 60):
        cyl(term, 7, 10, T(-80 + dx, ins_y - 3 - 38.5, cz - 80 + 83.5), seg=12)
    mppt = part("Isi - MPPT 20A", "Isi box", "black")
    box(mppt, 150, 40, 100, T(-75, ins_y - 3 - 20, cz + 105))
    ttgo = part("Isi - TTGO LoRa32 V2.1 (ESP32+SX1276)", "Isi box", "pcb")
    box(ttgo, 54, 12, 26, T(95, ins_y - 3 - 6, cz + 135))
    box(ttgo, 22, 3, 13, T(85, ins_y - 3 - 13, cz + 138))
    mods = part("Isi - buck, step-up 24V, ADS1115, MAX485", "Isi box", "blue")
    box(mods, 60, 18, 50, T(95, ins_y - 3 - 9, cz + 60))
    box(mods, 43, 14, 21, T(95, ins_y - 3 - 7, cz + 5))
    box(mods, 28, 8, 18, T(70, ins_y - 3 - 4, cz - 40))
    box(mods, 28, 8, 18, T(120, ins_y - 3 - 4, cz - 40))
    fuse = part("Isi - fuse/MCB + terminal", "Isi box", "white")
    box(fuse, 120, 35, 30, T(95, ins_y - 3 - 17, cz - 110))

    # ---------------- Panel surya ----------------
    L, Wp, tp, tilt = g["pv_l"], g["pv_w"], g["pv_t"], g["pv_tilt"]
    py = -pr - 20 - (Wp / 2) * math.cos(math.radians(tilt))
    Mpv = T(0, py, g["pv_z"]) @ Rx(tilt)
    fr = part("Panel surya - frame aluminium", "Energi", "alu")
    for sy in (-1, 1):
        box(fr, L, 30, tp, Mpv @ T(0, sy * (Wp / 2 - 15), -tp / 2))
        box(fr, 30, Wp - 60, tp, Mpv @ T(sy * (L / 2 - 15), 0, -tp / 2))
    cells = part("Panel surya - sel mono", "Energi", "cell")
    box(cells, L - 60, Wp - 60, 4, Mpv @ T(0, 0, -5))
    bus = part("Panel surya - garis sel", "Energi", "busbar")
    ncx, ncy = g["pv_cells"]
    for i in range(1, ncx):
        box(bus, 2, Wp - 60, 1, Mpv @ T(-(L - 60) / 2 + i * (L - 60) / ncx, 0, -2.6))
    for j in range(1, ncy):
        box(bus, L - 60, 2, 1, Mpv @ T(0, -(Wp - 60) / 2 + j * (Wp - 60) / ncy, -2.6))
    bs = part("Panel surya - backsheet", "Energi", "backsheet")
    box(bs, L - 60, Wp - 60, 2, Mpv @ T(0, 0, -tp + 6))
    jb = part("Panel surya - junction box", "Energi", "black")
    jb_local = (0, Wp / 2 - 90, -tp + 5 - 12)
    box(jb, 110, 70, 24, Mpv @ T(*jb_local))
    ANCHORS.append(("Panel surya 50 Wp mono", list(xf(Mpv, (L / 2 - 120, -Wp / 4, 0)))))

    rail = part("Bracket panel (rel + lengan)", "Energi", "galv")
    for sx in (-1, 1):
        box(rail, 30, Wp - 40, 20, Mpv @ T(sx * 200, 0, -tp - 10))
    tube(rail, pr + 8, pr + 0.2, 140, T(0, 0, g["pv_z"] - 160), seg=32)
    for sx in (-1, 1):
        back = xf(Mpv, (sx * 200, Wp / 2 - 40, -tp - 20))
        mid = xf(Mpv, (sx * 200, -Wp / 6, -tp - 20))
        seg_cyl(rail, [sx * 25, -pr - 4, g["pv_z"] - 30], back, 11, seg=12)
        seg_cyl(rail, [sx * 25, -pr - 4, g["pv_z"] - 150], mid, 11, seg=12)

    # ---------------- Rain gauge di puncak tiang ----------------
    zt = g["pole_top"]
    rgb = part("Bracket rain gauge", "Sensor", "steel")
    tube(rgb, pr + 6, pr + 0.2, 90, T(0, 0, zt - 80), seg=32)
    cyl(rgb, 130, 4, T(0, 0, zt + 10), seg=48)
    rr, rh = g["rg_d"] / 2, g["rg_h"]
    rg = part("Rain gauge tipping bucket", "Sensor", "black")
    zr = zt + 14
    for a in (90, 210, 330):
        box(rg, 34, 20, 26, T(118 * math.cos(math.radians(a)), 118 * math.sin(math.radians(a)), zr + 13) @ Rz(a))
    tube(rg, rr + 17, rr - 8, 8, T(0, 0, zr + 18), seg=64)
    cyl(rg, rr, rh, T(0, 0, zr + 26), seg=64, caps=False)
    cyl(rg, rr - 2, 4, T(0, 0, zr + 26), seg=64)
    tube(rg, rr + 1, rr - 6, 12, T(0, 0, zr + 26 + rh - 12), seg=64)
    fun = part("Rain gauge - corong", "Sensor", "dark")
    cyl(fun, 14, 90, T(0, 0, zr + 26 + rh - 100), r2=rr - 6, seg=64, caps=False)
    cyl(fun, 40, 3, T(0, 0, zr + 26 + rh - 30), seg=32)
    ANCHORS.append(("Rain gauge tipping bucket Ø200", [rr * 0.7, -rr * 0.7, zr + rh * 0.6]))

    # ---------------- DHT22 + radiation shield ----------------
    za = g["dht_arm_z"]; al = g["dht_arm_len"]
    arm = part("Lengan DHT22", "Struktur", "galv")
    tube(arm, pr + 6, pr + 0.2, 40, T(0, 0, za - 20), seg=32)
    seg_cyl(arm, [pr + 5, 0, za], [al, 0, za], 12, seg=12)
    sh = part("Radiation shield DHT22", "Sensor", "white")
    zs = za - 22
    cyl(sh, 72, 6, T(al, 0, zs), seg=40)
    for i in range(1, g["dht_plates"]):
        z0 = zs - i * 20
        cyl(sh, 72, 14, T(al, 0, z0), r2=42, seg=40, caps=False)
    for a in (0, 120, 240):
        seg_cyl(sh, [al + 60 * math.cos(math.radians(a)), 60 * math.sin(math.radians(a)), zs - g["dht_plates"] * 20],
                [al + 60 * math.cos(math.radians(a)), 60 * math.sin(math.radians(a)), zs], 3, seg=8)
    ANCHORS.append(("Suhu + RH (DHT22 + radiation shield)", [al + 72, 0, zs - 60]))

    # ---------------- Sumur pantau + sensor muka air ----------------
    wx, wy = g["well_x"], g["well_y"]
    ro = g["well_od"] / 2; ri = ro - g["well_wall"]
    a0 = 31; a1 = 211                     # separuh pipa yang menjauh dari kamera (cutaway)
    wellp = part("Sumur pantau pipa PVC 3in", "Sensor", "pvc")
    tube(wellp, ro, ri, g["well_top"], T(wx, wy, 0), seg=40)
    tube(wellp, ro, ri, -g["well_bottom"], T(wx, wy, g["well_bottom"]), seg=40, a0=a0, a1=a1)
    slots = part("Sumur pantau - slot perforasi", "Sensor", "dark")
    for z in np.arange(g["well_bottom"] + 60, -450, 70):
        tube(slots, ro + 0.6, ro - 0.5, 6, T(wx, wy, z), seg=40, a0=a0 + 12, a1=a1 - 12)
    cap = part("Sumur pantau - tutup", "Sensor", "pvccap")
    cyl(cap, ro + 4, 40, T(wx, wy, g["well_top"]), seg=40)
    cyl(cap, 6, 4, T(wx + 22, wy, g["well_top"] + 40), seg=12)
    wat = part("Air di sumur", "Sensor", "water")
    cyl(wat, ri - 0.5, W - g["well_bottom"] - 5, T(wx, wy, g["well_bottom"] + 5), seg=40, a0=a0, a1=a1)
    ANCHORS.append(("Sumur pantau PVC 3\" (cutaway)", [wx + ro, wy, -250]))

    hs = part("Sensor hydrostatic 4-20 mA", "Sensor", "steel")
    zs0 = g["well_bottom"] + 30
    cyl(hs, 13, 150, T(wx, wy, zs0), seg=24)
    cyl(hs, 13, 14, T(wx, wy, zs0), r2=13, seg=24)
    cyl(hs, 9, 10, T(wx, wy, zs0 - 10), r2=13, seg=24)
    ANCHORS.append(("Sensor level hydrostatic 4-20 mA", [wx + 13, wy, zs0 + 75]))
    fl = part("Float switch (batas bawah)", "Sensor", "steel")
    fx, fy = wx - 18, wy + 10
    cyl(fl, 4, 200, T(fx, fy, W - 330), seg=12)
    tube(fl, 18, 5, 36, T(fx, fy, W - 260), seg=24)
    ANCHORS.append(("Float switch (level rendah)", [fx - 18, fy, W - 240]))

    # ---------------- Soil sensor SEN0600 di dinding cutaway ----------------
    sx_, sz_ = g["probe_x"], g["probe_z"]
    soilp = part("Soil sensor RS485 (SEN0600)", "Sensor", "black")
    box(soilp, 44, 70, 15, T(sx_, -35, sz_) @ Ry(90))
    rods = part("Soil sensor - jarum stainless", "Sensor", "steel")
    for dz in (-14, 0, 14):
        seg_cyl(rods, [sx_, -1, sz_ + dz], [sx_, 70, sz_ + dz], 1.6, seg=8)
    ANCHORS.append(("Soil moisture RS485 (SEN0600)", [sx_, -70, sz_]))

    # ---------------- Grounding ----------------
    gx, gyy = g["ground_rod"]
    gr = part("Grounding rod tembaga", "Lingkungan", "copper")
    cyl(gr, 8, 1100, T(gx, gyy, -900), seg=16)
    tube(gr, 14, 8, 22, T(gx, gyy, 110), seg=16)

    # ---------------- Kabel ----------------
    cab = part("Kabel sensor & daya", "Kabel", "rubber")
    zg = zb - 26

    def pole_pt(ang, z, r=pr + 5):
        return [r * math.cos(math.radians(ang)), r * math.sin(math.radians(ang)), z]

    def to_gland(x, ang):
        return [pole_pt(ang, 1150), [x * 0.6, (gy + yb) / 2, zg - 60], [x, gy, zg - 30], [x, gy, zg]]

    # hydrostatic + float -> gland 0
    cable(cab, [[wx, wy, g["well_top"] + 40], [wx - 40, wy + 30, g["well_top"] + 110], [250, -240, 230],
                [60, -80, 160], pole_pt(-95, 150)] + to_gland(0, -95))
    seg_cyl(cab, [wx, wy, zs0 + 150], [wx, wy, g["well_top"] + 40], 2.8, seg=8)
    seg_cyl(cab, [fx, fy, W - 130], [fx + 10, fy - 5, g["well_top"]], 2.4, seg=8)
    # soil sensor -> gland -70
    cable(cab, [[sx_, -70, sz_], [sx_, -60, sz_ + 40], [sx_, -12, -40], [sx_ - 20, 25, 12], [220, 45, 14],
                [70, 40, 40], pole_pt(10, 120)] + to_gland(-70, 10))
    # DHT22 -> gland -140
    cable(cab, [[al - 30, 0, zs - 20], [al - 120, -10, za - 60], [pr + 30, -20, za - 140], pole_pt(-20, 1700)] +
          to_gland(-140, -20), r=2.6)
    # rain gauge -> gland 70 (turun di sisi belakang tiang)
    cable(cab, [[0, 125, zr + 18], [0, pr + 30, zt - 60], pole_pt(80, zt - 120), pole_pt(80, 1250),
                pole_pt(30, 1180)] + to_gland(70, 30)[1:], r=2.6)
    # PV -> gland 140
    jbw = xf(Mpv, (0, Wp / 2 - 90, -tp - 10))
    cable(cab, [list(jbw), [jbw[0] + 40, jbw[1] + 20, jbw[2] - 80], pole_pt(110, g["pv_z"] - 200),
                pole_pt(110, 1260), pole_pt(60, 1190)] + to_gland(140, 60)[1:], r=3.6)
    # grounding
    gcab = part("Kabel grounding", "Kabel", "gy")
    cable(gcab, [[gx, gyy, 121], [gx * 0.5, gyy * 0.5, 220], pole_pt(130, 300)], r=3)

# =====================================================================
# MESH dari primitif
# =====================================================================
def mesh_box(s):
    sx, sy, sz = [v / 2 for v in s]
    V, N, F = [], [], []
    faces = [((1, 0, 0), [(1, -1, -1), (1, 1, -1), (1, 1, 1), (1, -1, 1)]),
             ((-1, 0, 0), [(-1, 1, -1), (-1, -1, -1), (-1, -1, 1), (-1, 1, 1)]),
             ((0, 1, 0), [(1, 1, -1), (-1, 1, -1), (-1, 1, 1), (1, 1, 1)]),
             ((0, -1, 0), [(-1, -1, -1), (1, -1, -1), (1, -1, 1), (-1, -1, 1)]),
             ((0, 0, 1), [(-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)]),
             ((0, 0, -1), [(-1, 1, -1), (1, 1, -1), (1, -1, -1), (-1, -1, -1)])]
    for n, q in faces:
        b = len(V)
        for c in q:
            V.append((c[0] * sx, c[1] * sy, c[2] * sz)); N.append(n)
        F += [(b, b + 1, b + 2), (b, b + 2, b + 3)]
    return np.array(V, float), np.array(N, float), np.array(F, int)

def _angles(seg, a0, a1):
    full = abs((a1 - a0) - 360) < 1e-6
    n = max(3, int(round(seg * (a1 - a0) / 360)))
    t = np.radians(np.linspace(a0, a1, n + 1))
    return t, full

def mesh_cyl(r1, r2, h, seg, a0, a1, caps):
    t, full = _angles(seg, a0, a1)
    V, N, F = [], [], []
    slope = (r1 - r2) / h if h else 0
    # sisi
    b = 0
    for k, a in enumerate(t):
        c, s = math.cos(a), math.sin(a)
        n = np.array([c, s, slope]); n /= np.linalg.norm(n)
        V += [(r1 * c, r1 * s, 0), (r2 * c, r2 * s, h)]; N += [n, n]
    for k in range(len(t) - 1):
        i = 2 * k
        F += [(i, i + 2, i + 3), (i, i + 3, i + 1)]
    if caps:
        for z, r, nz in ((0, r1, -1), (h, r2, 1)):
            if r <= 0: continue
            c0 = len(V); V.append((0, 0, z)); N.append((0, 0, nz))
            for a in t:
                V.append((r * math.cos(a), r * math.sin(a), z)); N.append((0, 0, nz))
            for k in range(len(t) - 1):
                F.append((c0, c0 + 1 + k, c0 + 2 + k) if nz > 0 else (c0, c0 + 2 + k, c0 + 1 + k))
        if not full:
            for a, sgn in ((t[0], -1), (t[-1], 1)):
                c, s = math.cos(a), math.sin(a)
                n = (-s * sgn, c * sgn, 0)
                q = [(0, 0, 0), (r1 * c, r1 * s, 0), (r2 * c, r2 * s, h), (0, 0, h)]
                bb = len(V); V += q; N += [n] * 4
                F += [(bb, bb + 1, bb + 2), (bb, bb + 2, bb + 3)]
    return np.array(V, float), np.array(N, float), np.array(F, int)

def mesh_tube(ro, ri, h, seg, a0, a1):
    t, full = _angles(seg, a0, a1)
    V, N, F = [], [], []
    def strip(r, sign):
        b = len(V)
        for a in t:
            c, s = math.cos(a), math.sin(a)
            V.extend([(r * c, r * s, 0), (r * c, r * s, h)]); N.extend([(sign * c, sign * s, 0)] * 2)
        for k in range(len(t) - 1):
            i = b + 2 * k
            F.extend([(i, i + 2, i + 3), (i, i + 3, i + 1)] if sign > 0 else [(i, i + 3, i + 2), (i, i + 1, i + 3)])
    strip(ro, 1); strip(ri, -1)
    for z, nz in ((0, -1), (h, 1)):
        b = len(V)
        for a in t:
            c, s = math.cos(a), math.sin(a)
            V.extend([(ri * c, ri * s, z), (ro * c, ro * s, z)]); N.extend([(0, 0, nz)] * 2)
        for k in range(len(t) - 1):
            i = b + 2 * k
            F.extend([(i, i + 1, i + 3), (i, i + 3, i + 2)] if nz > 0 else [(i, i + 3, i + 1), (i, i + 2, i + 3)])
    if not full:
        for a, sgn in ((t[0], -1), (t[-1], 1)):
            c, s = math.cos(a), math.sin(a)
            n = (-s * sgn, c * sgn, 0)
            q = [(ri * c, ri * s, 0), (ro * c, ro * s, 0), (ro * c, ro * s, h), (ri * c, ri * s, h)]
            bb = len(V); V += q; N += [n] * 4
            F += [(bb, bb + 1, bb + 2), (bb, bb + 2, bb + 3)]
    return np.array(V, float), np.array(N, float), np.array(F, int)

def mesh_sphere(r, seg):
    V, N, F = [], [], []
    rings = max(4, seg // 2)
    for i in range(rings + 1):
        th = math.pi * i / rings
        for j in range(seg + 1):
            ph = 2 * math.pi * j / seg
            n = (math.sin(th) * math.cos(ph), math.sin(th) * math.sin(ph), math.cos(th))
            V.append(tuple(r * np.array(n))); N.append(n)
    for i in range(rings):
        for j in range(seg):
            a = i * (seg + 1) + j; b = a + seg + 1
            F += [(a, b, b + 1), (a, b + 1, a + 1)]
    return np.array(V, float), np.array(N, float), np.array(F, int)

def prim_mesh(pm):
    a = pm.a
    if pm.kind == "box": V, N, F = mesh_box(a["s"])
    elif pm.kind == "cyl": V, N, F = mesh_cyl(a["r1"], a["r2"], a["h"], a["seg"], a["a0"], a["a1"], a["caps"])
    elif pm.kind == "tube": V, N, F = mesh_tube(a["ro"], a["ri"], a["h"], a["seg"], a["a0"], a["a1"])
    elif pm.kind == "sphere": V, N, F = mesh_sphere(a["r"], a["seg"])
    M = pm.M
    V = (M[:3, :3] @ V.T).T + M[:3, 3]
    N = (M[:3, :3] @ N.T).T
    N /= np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-12)
    return V, N, F

def part_mesh(p):
    Vs, Ns, Fs, off = [], [], [], 0
    for pm in p["prims"]:
        V, N, F = prim_mesh(pm)
        Vs.append(V); Ns.append(N); Fs.append(F + off); off += len(V)
    return np.vstack(Vs), np.vstack(Ns), np.vstack(Fs)

# =====================================================================
# EKSPOR
# =====================================================================
def hex2rgb(h):
    return [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]

def zup_to_yup(V):
    return np.stack([V[:, 0], V[:, 2], -V[:, 1]], axis=1)

def export_glb(meshes, path):
    bin_ = bytearray(); bviews, accs, gmeshes, nodes, mats = [], [], [], [], []
    mat_index = {}
    def add_view(data, target):
        while len(bin_) % 4: bin_.append(0)
        off = len(bin_); bin_.extend(data)
        bviews.append(dict(buffer=0, byteOffset=off, byteLength=len(data), target=target))
        return len(bviews) - 1
    groups = {}
    for p, (V, N, F) in meshes:
        m = p["mat"]
        if m not in mat_index:
            col, met, rough = C[m]
            mat = dict(name=m, pbrMetallicRoughness=dict(baseColorFactor=hex2rgb(col) + [1.0],
                       metallicFactor=met, roughnessFactor=rough), doubleSided=True)
            mat_index[m] = len(mats); mats.append(mat)
        Vy = (zup_to_yup(V) * 0.001).astype(np.float32)
        Ny = zup_to_yup(N).astype(np.float32)
        I = F.astype(np.uint32).ravel()
        vp = add_view(Vy.tobytes(), 34962); vn = add_view(Ny.tobytes(), 34962); vi = add_view(I.tobytes(), 34963)
        accs.append(dict(bufferView=vp, componentType=5126, count=len(Vy), type="VEC3",
                         min=Vy.min(0).tolist(), max=Vy.max(0).tolist()))
        accs.append(dict(bufferView=vn, componentType=5126, count=len(Ny), type="VEC3"))
        accs.append(dict(bufferView=vi, componentType=5125, count=len(I), type="SCALAR"))
        a = len(accs)
        gmeshes.append(dict(name=p["name"], primitives=[dict(attributes=dict(POSITION=a - 3, NORMAL=a - 2),
                                                             indices=a - 1, material=mat_index[m])]))
        nodes.append(dict(name=p["name"], mesh=len(gmeshes) - 1))
        groups.setdefault(p["group"], []).append(len(nodes) - 1)
    roots = []
    for gname, kids in groups.items():
        nodes.append(dict(name=gname, children=kids)); roots.append(len(nodes) - 1)
    nodes.append(dict(name="Node Sensor Gambut (PV + LoRa)", children=roots))
    gltf = dict(asset=dict(version="2.0", generator="model_node_sensor.py"), scene=0,
                scenes=[dict(nodes=[len(nodes) - 1])], nodes=nodes, meshes=gmeshes, materials=mats,
                accessors=accs, bufferViews=bviews, buffers=[dict(byteLength=len(bin_))])
    js = json.dumps(gltf, ensure_ascii=False).encode("utf-8")
    while len(js) % 4: js += b" "
    while len(bin_) % 4: bin_.append(0)
    with open(path, "wb") as f:
        f.write(struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(js) + 8 + len(bin_)))
        f.write(struct.pack("<II", len(js), 0x4E4F534A)); f.write(js)
        f.write(struct.pack("<II", len(bin_), 0x004E4942)); f.write(bin_)

def safe(n):
    return "".join(ch if ch.isalnum() else "_" for ch in n)

def export_obj(meshes, path):
    mtl = os.path.splitext(path)[0] + ".mtl"
    with open(mtl, "w") as f:
        for k, (col, met, rough) in C.items():
            r, g_, b = hex2rgb(col)
            f.write(f"newmtl {k}\nKd {r:.4f} {g_:.4f} {b:.4f}\nKa 0 0 0\nKs {0.2+met*0.5:.3f} {0.2+met*0.5:.3f} {0.2+met*0.5:.3f}\nNs {int((1-rough)*200)+10}\nd 1\nillum 2\n\n")
    with open(path, "w") as f:
        f.write("# Node Sensor Gambut (PV + LoRa) - satuan mm, sumbu Y ke atas\n")
        f.write(f"mtllib {os.path.basename(mtl)}\n")
        vo = 1
        for p, (V, N, F) in meshes:
            Vy, Ny = zup_to_yup(V), zup_to_yup(N)
            f.write(f"o {safe(p['group'])}__{safe(p['name'])}\nusemtl {p['mat']}\n")
            f.write("".join(f"v {x:.2f} {y:.2f} {z:.2f}\n" for x, y, z in Vy))
            f.write("".join(f"vn {x:.4f} {y:.4f} {z:.4f}\n" for x, y, z in Ny))
            f.write("".join(f"f {a+vo}//{a+vo} {b+vo}//{b+vo} {c+vo}//{c+vo}\n" for a, b, c in F))
            vo += len(V)

def export_stl(meshes, path):
    tris = []
    for p, (V, N, F) in meshes:
        tris.append(V[F])
    T_ = np.concatenate(tris).astype(np.float32)
    n = np.cross(T_[:, 1] - T_[:, 0], T_[:, 2] - T_[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(T_), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"] = n; rec["v"] = T_
    with open(path, "wb") as f:
        f.write(b"Node Sensor Gambut (PV+LoRa) mm Z-up".ljust(80, b" "))
        f.write(struct.pack("<I", len(T_))); f.write(rec.tobytes())

def scad_matrix(M):
    rows = ",".join("[" + ",".join(f"{v:.5g}" for v in M[i]) + "]" for i in range(4))
    return f"multmatrix([{rows}])"

def export_scad(path):
    out = ["// Node Sensor Gambut (PV + LoRa) - diekspor dari model_node_sensor.py",
           "// Satuan mm, Z ke atas. Untuk mengubah dimensi secara parametrik, edit PARAMETER di",
           "// model_node_sensor.py lalu jalankan ulang; file ini versi solid CSG (butuh OpenSCAD 2019.05+).",
           "// Setiap part adalah satu module; matikan dengan memberi tanda // di baris pemanggilnya.", ""]
    calls = []
    for p in PARTS:
        mod = safe(p["name"]).lower()
        col = hex2rgb(C[p["mat"]][0])
        calls.append(f'color([{col[0]:.3f},{col[1]:.3f},{col[2]:.3f}]) {mod}();   // {p["group"]}: {p["name"]}')
        out.append(f"module {mod}() {{")
        for pm in p["prims"]:
            a = pm.a; m = scad_matrix(pm.M)
            if pm.kind == "box":
                out.append(f"  {m} cube([{a['s'][0]:.3f},{a['s'][1]:.3f},{a['s'][2]:.3f}], center=true);")
            elif pm.kind == "sphere":
                out.append(f"  {m} sphere(r={a['r']:.3f}, $fn={a['seg']});")
            else:
                ang = a["a1"] - a["a0"]
                if pm.kind == "cyl":
                    r1, r2, h = a["r1"], a["r2"], a["h"]
                    if a["caps"]:
                        prof = f"[[0,0],[{r1:.3f},0],[{r2:.3f},{h:.3f}],[0,{h:.3f}]]"
                    else:
                        t = 1.5
                        prof = f"[[{max(r1-t,0):.3f},0],[{r1:.3f},0],[{r2:.3f},{h:.3f}],[{max(r2-t,0):.3f},{h:.3f}]]"
                else:
                    ro, ri, h = a["ro"], a["ri"], a["h"]
                    prof = f"[[{ri:.3f},0],[{ro:.3f},0],[{ro:.3f},{h:.3f}],[{ri:.3f},{h:.3f}]]"
                if abs(ang - 360) < 1e-6:
                    out.append(f"  {m} rotate_extrude($fn={a['seg']}) polygon({prof});")
                else:
                    out.append(f"  {m} rotate([0,0,{a['a0']:.3f}]) rotate_extrude(angle={ang:.3f}, $fn={a['seg']}) polygon({prof});")
        out.append("}\n")
    out.append("// ===== Rakitan =====")
    out += calls
    with open(path, "w") as f:
        f.write("\n".join(out) + "\n")

def export_viewer(meshes, path):
    S = 4.0   # 1 unit = 0.25 mm
    pos, nor, col, parts = [], [], [], []
    first = 0
    for p, (V, N, F) in meshes:
        tri = V[F].reshape(-1, 3); ntri = N[F].reshape(-1, 3)
        pos.append(np.round(tri * S).astype(np.int16)); nor.append(np.round(ntri * 127).astype(np.int8))
        parts.append(dict(name=p["name"], group=p["group"], mat=p["mat"], first=first, count=len(tri),
                          color=C[p["mat"]][0], metal=C[p["mat"]][1]))
        first += len(tri)
    import base64
    P16 = np.concatenate(pos); N8 = np.concatenate(nor)
    data = dict(scale=1 / S, parts=parts, anchors=[dict(text=t, p=[float(v) for v in q]) for t, q in ANCHORS],
                pos=base64.b64encode(P16.tobytes()).decode(), nor=base64.b64encode(N8.tobytes()).decode(),
                vcount=int(len(P16)))
    with open(path, "w") as f:
        json.dump(data, f)
    return len(P16) // 3

if __name__ == "__main__":
    outdir = sys.argv[1] if len(sys.argv) > 1 else "output"
    os.makedirs(outdir, exist_ok=True)
    build()
    meshes = [(p, part_mesh(p)) for p in PARTS]
    export_glb(meshes, os.path.join(outdir, "node_sensor.glb"))
    export_obj(meshes, os.path.join(outdir, "node_sensor.obj"))
    export_stl(meshes, os.path.join(outdir, "node_sensor.stl"))
    export_scad(os.path.join(outdir, "node_sensor.scad"))
    nt = export_viewer(meshes, os.path.join(outdir, "viewer_data.json"))
    print(f"{len(PARTS)} part, {nt} segitiga -> {outdir}/")
