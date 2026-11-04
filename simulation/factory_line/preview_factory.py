"""Software layout preview from the exported USD; does not require a GPU."""
import os
os.environ.setdefault('MPLCONFIGDIR', '/tmp/acme-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from pxr import Usd, UsdGeom
from build_factory import OUT, COLORS
from factory_process import STACK

stage = Usd.Stage.Open(str(OUT/'acme_factory.usdc'))
cache = UsdGeom.XformCache(0)
fig = plt.figure(figsize=(17, 10), facecolor='#121b25')
ax = fig.add_subplot(111, projection='3d', computed_zorder=False)
ax.set_facecolor('#121b25')
faces, colors = [], []

def cube(center, size, color):
    x, y, z = center
    w, d, h = np.array(size)/2
    points = np.array([[x+i*w, y+j*d, z+k*h] for i,j,k in
                       [(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),(-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]])
    for indices in ([0,1,2,3],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]):
        faces.append(points[indices])
        colors.append(color)

for prim in stage.Traverse():
    path = str(prim.GetPath())
    if not prim.IsA(UsdGeom.Cube) or 'Prototypes' in path or '/Building/' in path:
        continue
    xform = cache.GetLocalToWorldTransform(prim)
    pos = xform.ExtractTranslation()
    size = [np.linalg.norm(np.array(xform[i])[:3]) for i in range(3)]
    from pxr import UsdShade
    mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
    color = COLORS.get(str(mat.GetPath()).split('/')[-1], COLORS['steel'])
    cube(pos, size, color)
# Simplify the stack interiors for the software preview only. USD contains individual instances.
for a in range(6):
    for b in range(6):
        if (a, b) == STACK:
            continue
        cube((.7+a*1.04, .47+b*.73, 1.0555), (.98, .67, 1.311), COLORS['B'])
        for col in range(4):
            for row in range(10):
                cube((.7+a*1.04+(col-1.5)*.245,.47+b*.73+(row-4.5)*.067,1.7455),(.242,.064,.069),COLORS['ABC'[(a+b+col+row)%3]])
collection = Poly3DCollection(faces, facecolors=colors, edgecolors=(0,0,0,.13), linewidths=.15, zsort='average')
ax.add_collection3d(collection)
# Real robot mesh previews, sampled to keep software plotting light.
for prim in stage.Traverse():
    if not prim.IsA(UsdGeom.Mesh):
        continue
    mesh = UsdGeom.Mesh(prim)
    points = np.array(mesh.GetPointsAttr().Get())
    transform = np.array(cache.GetLocalToWorldTransform(prim))
    points = np.c_[points,np.ones(len(points))] @ transform
    indices = np.array(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1,3)[::8]
    color = tuple(mesh.GetDisplayColorAttr().Get()[0])
    ax.add_collection3d(Poly3DCollection(points[indices,:3], facecolors=color, edgecolors='none'))
for label, pos in [('01  KILN CART + G1', (1,1,3)), ('02  INSPECTION', (3.2,6.5,2.6)),
                   ('03  FR3 REJECT REMOVAL', (8,8,2.4)), ('04  C1 LAYER COMPLETION',(13,6.5,2.6)),
                   ('05  G2 PALLETISING / SHIPPING',(13.5,9,3.5)), ('06  SCRAP BELTS / CRUSHER',(14,1.5,2.0))]:
    ax.text(*pos,label,color='white',fontsize=8,zorder=100)
ax.set_xlim(-2,23)
ax.set_ylim(-5.8,13)
ax.set_zlim(0,6)
ax.set_box_aspect((25,19,6))
ax.view_init(elev=36,azim=-65)
ax.set_axis_off()
fig.text(.07,.93,'ACME  /  BRICK SORTING FACTORY',color='white',fontsize=23,weight='bold')
fig.text(.07,.89,'Isaac Sim USD · 28,800-brick kiln cart · 4 FR3 reject removal · C1 layer completion · 560-brick pallets',color='#94adbf',fontsize=12)
fig.text(.07,.06,'Software layout preview • stack interiors simplified • motion and vision are conceptual',color='#94adbf',fontsize=10)
fig.subplots_adjust(left=0,right=1,bottom=.07,top=.88)
fig.savefig(OUT/'factory_preview.png',dpi=150,facecolor=fig.get_facecolor())
print(OUT/'factory_preview.png')
