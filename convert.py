import numpy as np
import trimesh
import cadquery as cq
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopoDS import TopoDS
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopLoc import TopLoc_Location
from OCP.BRep import BRep_Tool
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder

SRC = '/home/hatch/workspace/user/files/Catbot_____1body.step'
OUT = '/home/hatch/workspace/catbot-walk/catbot.glb'

# Colors from the STEP's own COLOUR_RGB definitions (sRGB)
YELLOW = (0.910, 0.678, 0.137)   # プラスチック - 光沢(黄) #E8AC22
RED    = (0.769, 0.208, 0.153)   # プラスチック - 光沢(赤) #C43527
STEEL  = (0.627, 0.627, 0.627)   # 鋼 - サテン #9F9F9F

def srgb_to_linear(c):
    return tuple(v / 12.92 if v <= 0.04045 else ((v + 0.055) / 1.055) ** 2.4 for v in c)

shape = cq.importers.importStep(SRC).val().wrapped
BRepMesh_IncrementalMesh(shape, 0.08, False, 0.35, True)

verts, tris, cols = [], [], []
stats = {'yellow': 0, 'red': 0, 'steel': 0}
red_pts = []
exp = TopExp_Explorer(shape, TopAbs_FACE)
while exp.More():
    face = TopoDS.Face_s(exp.Current())
    props = GProp_GProps()
    BRepGProp.SurfaceProperties_s(face, props)
    cen = props.CentreOfMass()
    cx, cy, cz = cen.X(), cen.Y(), cen.Z()

    # --- color decision (geometry rules, see analysis) ---
    col, kind = YELLOW, 'yellow'
    if cy < -0.25 and 35.0 <= cz <= 50.0:
        col, kind = RED, 'red'                      # eye discs on the head front (-Y)
    else:
        ad = BRepAdaptor_Surface(face)
        if ad.GetType() == GeomAbs_Cylinder:
            cyl = ad.Cylinder()
            ax = cyl.Axis().Direction()
            if abs(ax.Z()) > 0.99 and cyl.Radius() <= 6.0 and 29.0 <= cz <= 34.0:
                col, kind = STEEL, 'steel'          # neck ring
    stats[kind] += 1

    loc = TopLoc_Location()
    tri = BRep_Tool.Triangulation_s(face, loc)
    if tri is not None:
        trsf = loc.Transformation()
        nodes = []
        for i in range(1, tri.NbNodes() + 1):
            pnt = tri.Node(i).Transformed(trsf)
            nodes.append((pnt.X(), pnt.Y(), pnt.Z()))
        base = len(verts)
        rev = face.Orientation() == TopAbs_REVERSED
        for i in range(1, tri.NbTriangles() + 1):
            a, b, c = tri.Triangle(i).Get()
            tris.append((a - 1 + base, c - 1 + base, b - 1 + base) if rev else (a - 1 + base, b - 1 + base, c - 1 + base))
        verts.extend(nodes)
        lin = srgb_to_linear(col)
        rgba = (int(lin[0] * 255), int(lin[1] * 255), int(lin[2] * 255), 255)
        cols.extend([rgba] * len(nodes))
        if kind == 'red':
            red_pts.extend(nodes)
    exp.Next()

verts = np.array(verts, dtype=np.float64)
tris = np.array(tris, dtype=np.int64)
cols = np.array(cols, dtype=np.uint8)
print('vertices:', len(verts), 'triangles:', len(tris), 'faces by color:', stats)
rp = np.array(red_pts)
print('red centroid: x %.2f y %.2f z %.2f  (front = -Y if y is the min side)' % (rp[:, 0].mean(), rp[:, 1].mean(), rp[:, 2].mean()))

# footprint center -> origin (feet bottom stays z=0)
bb_min, bb_max = verts.min(axis=0), verts.max(axis=0)
verts[:, 0] -= (bb_min[0] + bb_max[0]) / 2.0
verts[:, 1] -= (bb_min[1] + bb_max[1]) / 2.0
print('recentered: x shift %.2f, y shift %.2f' % (-(bb_min[0] + bb_max[0]) / 2.0, -(bb_min[1] + bb_max[1]) / 2.0))

mesh = trimesh.Trimesh(vertices=verts, faces=tris, process=False)
mesh.visual.vertex_colors = cols
mesh.export(OUT)
import os
print('GLB bytes:', os.path.getsize(OUT))
