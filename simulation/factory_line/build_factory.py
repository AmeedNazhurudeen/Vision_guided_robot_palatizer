"""Build a portable Isaac Sim USD using the locally installed USD libraries.

This is a deterministic process animation, not a validated dynamics simulation.
"""
import csv
import json
import math
from collections import Counter
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'output'
FPS = 30
COLORS = {'A': (.63, .24, .14), 'B': (.48, .15, .09),
          'C': (.34, .10, .065), 'X': (.85, .65, .43), 'D': (.17, .045, .03),
          'steel': (.16, .21, .25), 'belt': (.055, .065, .075),
          'yellow': (.96, .61, .045), 'wood': (.46, .30, .16),
          'floor': (.26, .29, .31), 'blue': (.035, .30, .52)}
BRICK = (.245, .067, .069)


def build():
    OUT.mkdir(exist_ok=True)
    stage = Usd.Stage.CreateNew(str(OUT / '.acme_factory.build.usdc'))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    stage.SetTimeCodesPerSecond(FPS)
    stage.SetStartTimeCode(0)
    world = UsdGeom.Xform.Define(stage, '/World')
    stage.SetDefaultPrim(world.GetPrim())
    stage.GetRootLayer().customLayerData = {
        'description': 'ACME dehacking line: G1, inspection, FR3 reject removal, SP1 separator to lanes A/B, C1/C2 layer completion, G2 palletising; authored motion, no validated robot dynamics',
        'source': 'UTUM Brick Coloration.pdf and ACME sim/layout_v5/components_v5.csv',
    }
    physics = UsdPhysics.Scene.Define(stage, '/World/Physics')
    physics.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    physics.CreateGravityMagnitudeAttr(9.81)
    materials = {}
    for name, color in COLORS.items():
        mat = UsdShade.Material.Define(stage, '/World/Looks/' + name)
        shader = UsdShade.Shader.Define(stage, str(mat.GetPath()) + '/Shader')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(color)
        shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.75)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
        materials[name] = mat

    def box(path, pos, size, color='steel', collision=False):
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1)
        cube.AddTranslateOp().Set(Gf.Vec3d(*pos))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(materials[color])
        if collision:
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        return cube

    def animate(obj, points):
        op = obj.GetOrderedXformOps()[0]
        for seconds, p in points:
            op.Set(Gf.Vec3d(*p), seconds * FPS)

    def camera(path, eye, target, focal=30):
        cam = UsdGeom.Camera.Define(stage, path)
        mat = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse()
        cam.AddTransformOp().Set(mat)
        cam.CreateFocalLengthAttr(focal)
        cam.CreateClippingRangeAttr(Gf.Vec2f(.02, 500))
        return cam

    box('/World/Building/Floor', (10, 4, -.12), (31, 25, .24), 'floor', True)
    # Open roof and front walls keep the process visible in the overview camera.
    box('/World/Building/BackWall', (10, 14.5, 3.5), (31, .2, 7), 'steel')
    for i, x in enumerate((-4, 4, 12, 20, 25)):
        box(f'/World/Building/Column{i}', (x, 14.3, 3.5), (.22, .22, 7))
        box(f'/World/Building/Rafter{i}', (x, 4, 7), (.18, 21, .24))
    rows = list(csv.DictReader((ROOT / 'assets/components_v5.csv').open()))
    from factory_process import revised_rows, STACK
    rows = revised_rows(rows)
    equipment = {}
    for row in rows:
        ident, kind = row['id'], row['kind']
        x, y, z, sx, sy, sz = [float(row[k]) / 1000 for k in ('x', 'y', 'z', 'sx', 'sy', 'sz')]
        parent = '/World/Equipment/' + ident
        prim = UsdGeom.Xform.Define(stage, parent).GetPrim()
        prim.SetCustomDataByKey('name', row['name'])
        prim.SetCustomDataByKey('kind', kind)
        equipment[ident] = (x, y, z, sx, sy, sz)
        if kind == 'fence':
            # Extend south to enclose the v5 pusher/rail entry; entry remains conceptual.
            for j, (xx, yy, ww, dd) in enumerate(((x, -6, sx, .04), (x, 13, sx, .04), (-1.5, 3.5, .04, 19), (22, 3.5, .04, 19))):
                for h in (.3, 1, 1.9):
                    box(f'{parent}/Rail{j}_{int(h*10)}', (xx, yy, h), (ww, dd, .035), 'yellow')
            continue
        if kind == 'crane':
            continue  # Detailed, synchronised gantries are built with the material flow.
        if kind == 'rail':
            for j, dx in enumerate((-2.8, 2.8)):
                box(f'{parent}/Rail{j}', (x+dx, y, .075), (.10, sy, .15))
            continue
        if kind == 'light':
            # Four light strips leave the inspection camera's optical path open.
            for j, (dx, dy, w, d) in enumerate(((-.6, 0, .06, .9), (.6, 0, .06, .9), (0, -.45, 1.2, .06), (0, .45, 1.2, .06))):
                box(f'{parent}/Strip{j}', (x+dx, y+dy, z), (w, d, .04), 'yellow')
            continue
        if kind in ('conveyor', 'roller', 'table'):
            top = z + sz
            box(parent+'/Bed', (x, y, top-.06), (sx, sy, .12), 'belt', True)
            for j, (dx, dy) in enumerate(((-sx*.4, -sy*.3), (sx*.4, sy*.3))):
                box(f'{parent}/Leg{j}', (x+dx, y+dy, top/2), (.055, .055, top))
            if kind == 'conveyor':
                axis = 0 if sx > sy else 1
                for side in (-1, 1):
                    p = [x, y, top+.015]
                    p[1-axis] += side*((sy if axis == 0 else sx)/2+.02)
                    box(f'{parent}/Edge{side+1}', p, (sx, .025, .07) if axis == 0 else (.025, sy, .07), 'blue')
            continue
        if kind == 'pallet':
            for j in range(7):
                box(f'{parent}/Board{j}', (x, y-sy/2+(j+.5)*sy/7, .125), (sx, sy/8, .05), 'wood', True)
            for j in (-1, 0, 1):
                box(f'{parent}/Runner{j+1}', (x+j*sx*.38, y, .05), (.10, sy, .10), 'wood')
            continue
        color = 'yellow' if kind in ('sensor', 'pusher', 'gate') else 'blue' if kind in ('robot', 'cabinet', 'machine', 'crusher') else 'steel'
        box(parent+'/Body', (x, y, z+sz/2), (sx, sy, sz), color, kind not in ('zone', 'camera'))

    # Full car: 36 stacks x 20 layers x 40 bricks, less the stack being dehacked (animated).
    inst = UsdGeom.PointInstancer.Define(stage, '/World/KilnInventory')
    prototypes = []
    for shade in 'ABC':
        proto = box('/World/KilnInventory/Prototypes/'+shade, (0, 0, 0), BRICK, shade)
        prototypes.append(proto.GetPath())
    inst.CreatePrototypesRel().SetTargets(prototypes)
    positions, indices = [], []
    for a in range(6):
        for b in range(6):
            for h in range(20):
                if (a, b) == STACK:
                    continue
                for col in range(4):
                    for row in range(10):
                        positions.append((.7+a*1.04+(col-1.5)*.245, .47+b*.73+(row-4.5)*.067, .4+(h+.5)*.069))
                        indices.append((a+b+h+col+row) % 3)
    inst.CreatePositionsAttr(positions)
    inst.CreateProtoIndicesAttr(indices)

    add_fr3_visuals(stage)
    from factory_process import build_process
    process = build_process(stage, box, animate, FPS)
    duration = process.pop('duration')
    stage.SetEndTimeCode(duration * FPS)
    from factory_process import finish_hardware
    finish_hardware(stage,process["supports"])
    dome = UsdLux.DomeLight.Define(stage, '/World/Lighting/Ambient')
    dome.CreateIntensityAttr(650)
    sun = UsdLux.DistantLight.Define(stage, '/World/Lighting/Sun')
    sun.CreateIntensityAttr(2200)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(315, 25, 25))
    # One overhead camera + diffuse light above each FR3 station, looking straight down at the layer.
    from robot_manipulation import STATIONS, BELT_Y
    for s, x in enumerate(STATIONS):
        panel = UsdLux.RectLight.Define(stage, f'/World/Lighting/Station{s+1}')
        panel.AddTranslateOp().Set(Gf.Vec3d(x, BELT_Y, 2.30))
        panel.CreateWidthAttr(1.0)
        panel.CreateHeightAttr(.8)
        panel.CreateIntensityAttr(900)
        inspect = UsdGeom.Camera.Define(stage, '/World/Cameras/Inspection' + ('' if s == 0 else '2'))
        inspect.AddTranslateOp().Set(Gf.Vec3d(x, BELT_Y, 2.26))
        inspect.CreateFocalLengthAttr(18)
        inspect.CreateHorizontalApertureAttr(22)
        inspect.CreateVerticalApertureAttr(16)
        inspect.CreateClippingRangeAttr(Gf.Vec2f(.01, 100))
    camera('/World/Cameras/Overview', (26, -20, 22), (8, 4.5, 0), 30)
    camera('/World/Cameras/PickStation', (9.3, 1.0, 4.6), (9.3, 5.8, .85), 22)
    camera('/World/Cameras/GantryLoading', (7.5, -2.5, 6.5), (1.9, 4.6, 1.3), 26)
    camera('/World/Cameras/Palletising', (19, 2.4, 5.2), (13.6, 6.4, .9), 24)
    camera('/World/Cameras/Separator', (13.9, 3.3, 3.1), (12.1, 5.8, .8), 20)
    stage.GetRootLayer().Save()
    moving = len(process['ledger'])
    report = {'static_cart_bricks': len(positions), 'animated_bricks': moving,
              'input_total': len(positions)+moving, 'brick_dimensions_m': BRICK,
              'duration_seconds': duration, 'classification': 'synthetic shade metadata, not camera inference',
              **process}
    (OUT/'flow_report.json').write_text(json.dumps(report, indent=2))
    (OUT / '.acme_factory.build.usdc').replace(OUT / 'acme_factory.usdc')
    print(json.dumps({k:v for k,v in report.items() if k not in ('ledger', 'manipulation', 'handling', 'supports', 'c1',
                                                                    'pair_index_seconds', 'donor_left')}, indent=2))


def add_fr3_visuals(stage):
    """Convert installed Menagerie mesh data to self-contained USD, using MuJoCo FK."""
    import mujoco
    from robot_manipulation import MODEL, ROBOTS
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    data.qpos[:7] = [0, -.5, 0, -2, 0, 1.5, .785]
    mujoco.mj_forward(model, data)
    for robot in ROBOTS:
        root = UsdGeom.Xform.Define(stage, f"/World/Equipment/{robot['name']}/FR3Visual")
        root.AddTranslateOp().Set(Gf.Vec3d(*robot['base']))
        root.AddRotateZOp().Set(robot['yaw'])
        root.GetPrim().SetCustomDataByKey('status', 'Visual rest pose only; not a PhysX articulation')
        for g in range(model.ngeom):
            if model.geom_group[g] != 2:
                continue
            mid = model.geom_dataid[g]
            va, vn = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
            fa, fn = model.mesh_faceadr[mid], model.mesh_facenum[mid]
            vertices = model.mesh_vert[va:va+vn]
            rotation = data.geom_xmat[g].reshape(3, 3)
            vertices = vertices @ rotation.T + data.geom_xpos[g]
            mesh = UsdGeom.Mesh.Define(stage, str(root.GetPath())+f'/Mesh{g}')
            mesh.CreatePointsAttr(vertices.tolist())
            mesh.CreateFaceVertexCountsAttr([3]*fn)
            mesh.CreateFaceVertexIndicesAttr(model.mesh_face[fa:fa+fn].flatten().tolist())
            mesh.CreateSubdivisionSchemeAttr('none')
            mat_id = model.geom_matid[g]
            color = model.mat_rgba[mat_id, :3] if mat_id >= 0 else model.geom_rgba[g, :3]
            mesh.CreateDisplayColorAttr([Gf.Vec3f(*map(float, color))])


if __name__ == '__main__':
    build()
