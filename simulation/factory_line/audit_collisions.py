"""Check the recorded FR3s against each other, their tools, every brick and the fixed equipment.

Uses the supplied FR3 collision meshes and exact box geometry, not visual guesses.
"""
import copy
import json
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from pxr import Usd,UsdGeom
from build_factory import OUT
from robot_manipulation import MODEL,TOOL_LENGTH,CYCLE,ROBOTS
from validate_factory import load_motion,MOVING


def collision_model(stage,report):
    tree = ET.parse(MODEL)
    root = tree.getroot()
    root.find('compiler').set('meshdir',str(MODEL.parent/'assets'))
    source = copy.deepcopy(root.find('worldbody')[0])
    world = root.find('worldbody')
    world.clear()
    for tag in ('actuator','keyframe','contact'):
        node = root.find(tag)
        if node is not None:
            root.remove(node)
    contacts = ET.SubElement(root,'contact')
    for robot in ROBOTS:
        body = copy.deepcopy(source)
        prefix = robot['name']+'_'
        body.set('pos',' '.join(map(str,robot['base'])))
        body.set('euler',f"0 0 {np.radians(robot['yaw'])}")
        for node in body.iter():
            if node.get('name'):
                node.set('name',prefix+node.get('name'))
        link7 = next(n for n in body.iter('body') if n.get('name')==prefix+'fr3_link7')
        for label,length,half,radius in [('stem',TOOL_LENGTH/2,TOOL_LENGTH/2,.022),('cup',TOOL_LENGTH-.006,.006,.027)]:
            ET.SubElement(link7,'geom',name=prefix+label,type='cylinder',size=f'{radius} {half}',
                          pos=f'0 0 { .107+length}',mass='0',contype='1',conaffinity='1')
        world.append(body)
        ET.SubElement(contacts,'exclude',body1=prefix+'fr3_link0',body2=prefix+'fr3_link1')
    cache = UsdGeom.XformCache(0)
    fixed_paths = {}
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not prim.IsA(UsdGeom.Cube) or any(part in path for part in ('/Flow/','/KilnInventory/','/FR3Visual/',*MOVING)):
            continue
        mat = cache.GetLocalToWorldTransform(prim)
        pos = mat.ExtractTranslation()
        half = np.linalg.norm(np.array(mat)[:3,:3],axis=1)/2
        name = 'Fixed'+str(len(fixed_paths))
        ET.SubElement(world,'geom',name=name,type='box',size=' '.join(map(str,half)),pos=' '.join(map(str,pos)),contype='2',conaffinity='1')
        fixed_paths[name] = path
    for entry in report['ledger']:
        n = entry['brick']
        body = ET.SubElement(world,'body',name=f'BrickBody{n}',mocap='true',pos='0 0 -100')
        ET.SubElement(body,'geom',name=f'Brick{n}',type='box',size='.1225 .0335 .0345',contype='4',conaffinity='1')
    model = mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
    return model,fixed_paths


def moving_machinery_clear(stage,duration):
    """Cranes, the gantry and pallets are not in the MuJoCo scene; prove they never enter an FR3 workspace."""
    # FR3 reach 0.855 m from the shoulder, plus the 0.24 m tool and a carried brick's half diagonal.
    shoulders,reach = [np.array(r['base'])+[0,0,.333] for r in ROBOTS],.855+TOOL_LENGTH+.13
    parts = [p for p in stage.Traverse() if p.IsA(UsdGeom.Cube) and any(m in str(p.GetPath()) for m in MOVING)]
    nearest = 1e9
    for t in np.arange(0,duration,1.):
        cache = UsdGeom.XformCache(t*30)
        for prim in parts:
            mat = cache.GetLocalToWorldTransform(prim)
            c,h = np.array(mat.ExtractTranslation()),np.linalg.norm(np.array(mat)[:3,:3],axis=1)/2
            for shoulder in shoulders:
                gap = np.linalg.norm(np.maximum(np.abs(shoulder-c)-h,0))
                nearest = min(nearest,gap)
                if gap < reach:
                    raise AssertionError(f'{prim.GetPath()} enters an FR3 workspace at {t:.0f}s ({gap:.2f} m)')
    return len(parts),nearest


def run():
    report = json.loads((OUT/'flow_report.json').read_text())
    stage = Usd.Stage.Open(str(OUT/'acme_factory.usdc'))
    times,positions = load_motion(stage,report)
    model,fixed_paths = collision_model(stage,report)
    data = mujoco.MjData(model)
    manipulation = report['manipulation']
    home = np.array(manipulation['home_q_rad'])
    plans = {tuple(np.round(p['pick_local_m'],6)):np.array(p['q_rad']) for p in manipulation['plans']}
    names = [mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,n) or '' for n in range(model.ngeom)]
    order = [entry['brick'] for entry in report['ledger']]
    brick_mocap = [model.body_mocapid[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,f'BrickBody{i}')] for i in order]
    gone = [entry.get('consumed_seconds',1e9) for entry in report['ledger']]
    tasks = {r['name']:[] for r in ROBOTS}
    for task in manipulation['tasks']:
        tasks[task['robot']].append((round(task['start_seconds']*30),plans[tuple(np.round(task['plan'],6))]))
    frames = sorted({start+k for items in tasks.values() for start,q in items for k in range(len(q))})
    worst,checked = 0.,0
    for frame in frames:
        seconds = frame/30
        q = []
        for robot in ROBOTS:
            active = [(start,plan) for start,plan in tasks[robot['name']] if start <= frame < start+len(plan)]
            q.append(active[0][1][frame-active[0][0]] if active else home)
        data.qpos[:] = np.concatenate(q)
        here = positions[:,min(frame,positions.shape[1]-1)].astype(float)
        here[np.array(gone) <= seconds] = (0,0,-100)  # crushed
        data.mocap_pos[brick_mocap] = here
        mujoco.mj_forward(model,data)
        checked += 1
        for contact in data.contact:
            a,b = names[contact.geom1],names[contact.geom2]
            if not (a.startswith('R') or b.startswith('R')):
                continue
            depth = max(0.,-contact.dist)
            # Mounting contact at the bolted base is intentional.
            if a.startswith('Fixed') or b.startswith('Fixed'):
                fixed,robot = (a,b) if a.startswith('Fixed') else (b,a)
                if robot.endswith('fr3_link0_collision') and fixed_paths[fixed]==f"/World/Equipment/{robot.split('_')[0]}/Body":
                    continue
            worst = max(worst,depth)
            if depth>.001:
                detail = {'seconds':seconds,'depth_m':depth,'geometry_a':fixed_paths.get(a,a),'geometry_b':fixed_paths.get(b,b)}
                (OUT/'collision_failure.json').write_text(json.dumps(detail,indent=2))
                raise AssertionError(detail)
    parts,nearest = moving_machinery_clear(stage,report['duration_seconds'])
    result = {'passed':True,'sample_hz':30,'robots':len(ROBOTS),'robot_scene_states_checked':checked,'max_penetration_m':worst,
              'representation':'4 FR3 collision meshes, vacuum cylinders, fixed equipment boxes, 800 animated brick boxes',
              'moving_machine_parts_kept_out_of_fr3_workspace':parts,'nearest_moving_part_to_fr3_reach_m':round(nearest-(.855+TOOL_LENGTH+.13),3),
              'intentional_exclusion':'bolted robot base mounting contact',
              'limitations':'kinematic sampled geometry audit; not contact dynamics or machinery certification'}
    (OUT/'collision_report.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    run()
