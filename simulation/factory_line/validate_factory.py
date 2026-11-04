"""Audit every animated brick at 30 Hz: support, attachments, overlaps, layer completion and pallets."""
import json
from collections import Counter, defaultdict
import numpy as np
from pxr import Usd, UsdGeom
from build_factory import OUT, BRICK
from factory_process import STACK, Z, PALLET_LAYERS

HALF = np.array(BRICK)/2
SPIN_HALF = np.array([.1268,.1268,.0345])   # a brick at any yaw, while a robot swings it above the belts


def keyed(op,times):
    keys = np.array(op.GetTimeSamples())/30
    values = np.array([op.Get(t*30) for t in keys])
    return np.column_stack([np.interp(times,keys,values[:,axis]) for axis in range(3)])


def load_motion(stage,report):
    times = np.arange(round(report['duration_seconds']*30)+1)/30
    positions = np.empty((len(report['ledger']),len(times),3),np.float32)
    for n,entry in enumerate(report['ledger']):
        prim = stage.GetPrimAtPath(f"/World/Flow/Brick{entry['brick']:03}")
        positions[n] = keyed(UsdGeom.Xformable(prim).GetOrderedXformOps()[0],times)
    return times,positions


def kiln_stacks():
    """Static kiln-car stacks as boxes (the stack being dehacked is animated brick by brick)."""
    return [((.7+a*1.04,.47+b*.73,.4+.69),(.49,.335,.69)) for a in range(6) for b in range(6) if (a,b) != STACK]


MOVING = ('/Bridge/','/Trolley/','/Spreader/','/TelescopicMast','/Pallets/')


def assembly(path):
    """Parts of one assembly may touch or interpenetrate (posts into beams, deck corners); different ones may not."""
    if path.startswith('kiln'):
        return 'kiln'
    parts = path.split('/')
    if parts[2] == 'Equipment':
        return 'camera' if parts[3].startswith(('CAM','LIGHT')) else parts[3]
    if parts[2] == 'Transport':
        return parts[3].split('_')[0].replace('Down','')
    return parts[2]


def equipment_clearance(stage,times):
    """No machine stands in another: fixed boxes pairwise, and crane/gantry/pallet parts swept over the run."""
    cache = UsdGeom.XformCache(0)
    fixed,moving = [],[]
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not prim.IsA(UsdGeom.Cube) or any(p in path for p in ('/Flow/','/KilnInventory/','/Building/')):
            continue
        if prim.GetParent().GetCustomDataByKey('kind') == 'zone':
            continue  # floor markings
        if any(p in path for p in MOVING):
            moving.append(prim)
            continue
        mat = cache.GetLocalToWorldTransform(prim)
        fixed.append((path,np.array(mat.ExtractTranslation()),np.linalg.norm(np.array(mat)[:3,:3],axis=1)/2))
    fixed += [(f'kiln stack {c}',np.array(c),np.array(h)) for c,h in kiln_stacks()]
    for i,(pa,ca,ha) in enumerate(fixed):
        for pb,cb,hb in fixed[i+1:]:
            if assembly(pa) != assembly(pb) and np.all(np.abs(ca-cb) < ha+hb-.002):
                raise AssertionError(f'{pa} stands in {pb}')
    samples = 0
    for t in np.arange(0,times[-1],.2):
        cache = UsdGeom.XformCache(t*30)
        for prim in moving:
            path = str(prim.GetPath())
            mat = cache.GetLocalToWorldTransform(prim)
            c,h = np.array(mat.ExtractTranslation()),np.linalg.norm(np.array(mat)[:3,:3],axis=1)/2
            for pb,cb,hb in fixed:
                if assembly(path) != assembly(pb) and np.all(np.abs(c-cb) < h+hb-.002):
                    raise AssertionError(f'{path} hits {pb} at {t:.1f}s')
            samples += 1
    return {'fixed_equipment_pairs_checked':len(fixed)*(len(fixed)-1)//2,'moving_machine_part_samples':samples}


def validate():
    report = json.loads((OUT/'flow_report.json').read_text())
    stage = Usd.Stage.Open(str(OUT/'acme_factory.usdc'))
    assert stage and UsdGeom.GetStageMetersPerUnit(stage)==1
    assert stage.GetEndTimeCode()==report['duration_seconds']*30
    inst = UsdGeom.PointInstancer(stage.GetPrimAtPath('/World/KilnInventory'))
    assert len(inst.GetPositionsAttr().Get())==28000
    assert report['input_total']==28800 and report['animated_bricks']==800
    ledger = report['ledger']
    assert sum(report['destinations'].values())==800
    times,positions = load_motion(stage,report)
    n_bricks = len(ledger)
    # A brick consumed in the crusher no longer exists for overlap and obstacle checks.
    active = np.ones((n_bricks,len(times)),bool)
    for n,entry in enumerate(ledger):
        if 'consumed_seconds' in entry:
            active[n,times>=entry['consumed_seconds']-1e-6] = False
    spin = np.zeros_like(active)
    surfaces = report['supports']
    support_samples,head_samples = 0,0
    for n,(entry,pos) in enumerate(zip(ledger,positions)):
        assert np.linalg.norm(pos[-1]-entry['final_position_m'])<1e-5
        for segment in entry['segments']:
            mask = (times>=segment['start']-1e-8)&(times<segment['end']-1e-8)
            pp = pos[mask].astype(float)
            if not len(pp):
                continue
            support = segment['support']
            if support=='surface':
                supported = np.ones(len(pp),dtype=bool)
                for dx,dy in [(-.1225,-.0335),(-.1225,.0335),(.1225,-.0335),(.1225,.0335)]:
                    corner = np.zeros(len(pp),dtype=bool)
                    for surface in surfaces:
                        corner |= ((np.abs(pp[:,0]+dx-surface['x'])<=surface['sx']/2+3e-5)&
                                   (np.abs(pp[:,1]+dy-surface['y'])<=surface['sy']/2+3e-5)&
                                   (np.abs(pp[:,2]-.0345-surface['top'])<3e-5))
                    supported &= corner
                if not np.all(supported):
                    k = np.flatnonzero(~supported)[0]
                    raise AssertionError(f"Unsupported brick {entry['brick']} at {times[mask][k]:.3f}s, {pp[k]}, segment {segment}")
                support_samples += len(pp)
            elif support=='cart':
                assert np.max(np.abs(pp-pp[0]))<1e-6, f"Brick {entry['brick']} moves while still on the kiln car"
            elif support in ('G1','G2','C1','C2'):
                head = keyed(UsdGeom.Xformable(stage.GetPrimAtPath(f'/World/Equipment/{support}/Spreader')).GetOrderedXformOps()[0],times[mask])
                offset = pp-head
                assert np.max(np.abs(offset-offset[0]))<2e-5, f"Brick {entry['brick']} slips from {support}"
                assert np.max(np.abs(offset[:,2]+.0345))<2e-5
                head_samples += len(pp)
            elif support.startswith('pallet:'):
                body = keyed(UsdGeom.Xformable(stage.GetPrimAtPath(f"/World/Pallets/P{support.split(':')[1]}")).GetOrderedXformOps()[0],times[mask])
                offset = pp-body
                assert np.max(np.abs(offset-offset[0]))<2e-5, f"Brick {entry['brick']} slips on its pallet"
                assert np.all(np.abs(offset[0,:2])<=[.51-.1225+1e-4,.355-.0335+1e-4]), 'Brick overhangs its pallet'
                level = (offset[0,2]-.15-.0345)/.069
                assert abs(level-round(level))<1e-4 and 0<=round(level)<PALLET_LAYERS
            elif support.startswith('R'):
                spin[n,mask] = pp[:,2] > Z+.069
            elif support!='crusher':
                raise AssertionError('Unknown support '+support)

    # Every bad brick is removed by an FR3 and crushed; every palletised layer is complete.
    tasks = report['manipulation']['tasks']
    bad = {e['brick'] for e in ledger if e['bad']}
    assert {t['brick'] for t in tasks}==bad and len(tasks)==len(bad)
    assert all(e['destination']=='crusher' for e in ledger if e['bad'])
    assert not any(e['bad'] for e in ledger if e['destination']!='crusher')
    for task in tasks:
        brick = stage.GetPrimAtPath(f"/World/Flow/Brick{task['brick']:03}")
        cup = stage.GetPrimAtPath(f"/World/Equipment/{task['robot']}/FR3Visual/SuctionCup")
        relative = []
        for frame in range(round(task['attach_seconds']*30),round(task['release_seconds']*30)+1):
            cache = UsdGeom.XformCache(frame)
            relative.append(np.array(cache.GetLocalToWorldTransform(brick)*cache.GetLocalToWorldTransform(cup).GetInverse()))
        relative = np.array(relative)
        assert np.max(np.abs(relative-relative[0]))<1e-5, f"Brick {task['brick']} slips from {task['robot']}"
    # North and south FR3s of one station never work inside the layer at the same time.
    windows = defaultdict(list)
    for task in tasks:
        windows[task['station']].append(tuple(task['zone_seconds']))
    for station,items in windows.items():
        items.sort()
        assert all(a[1]<=b[0]+1e-6 for a,b in zip(items,items[1:])), f'Station {station} robots share the layer'
    g1 = [h for h in report['handling'] if h['machine']=='G1']
    g2 = [h for h in report['handling'] if h['machine']=='G2']
    assert len(g1)==report['layers_in'] and all(len(h['bricks'])==40 for h in g1)
    assert len(g2)==report['layers_out'] and all(len(h['bricks'])==40 for h in g2)
    for n,pallet in enumerate(report['pallets']):
        assert pallet['bricks']==40*pallet['layers']
        assert pallet['layers']<PALLET_LAYERS or pallet['shipped_seconds'] is not None
    on_pallet = Counter(e['destination'] for e in ledger)['pallet']
    assert on_pallet==40*len(g2) and len(report['donor_left'])==report['destinations'].get('donor table',0)

    # Brick-brick overlap: each frame, every brick that moved is tested against every brick present.
    worst = 0.
    previous = positions[:,0]
    order = np.arange(n_bricks)
    for frame in range(len(times)):
        here = positions[:,frame]
        moved = order[active[:,frame]&(np.any(np.abs(here-previous)>1e-7,axis=1)|(frame==0))]
        previous = here
        if not len(moved):
            continue
        others = order[active[:,frame]]
        half_m = np.where(spin[moved,frame,None],SPIN_HALF,HALF)
        half_o = np.where(spin[others,frame,None],SPIN_HALF,HALF)
        delta = np.abs(here[moved,None,:]-here[None,others,:])
        overlap = np.all(delta<half_m[:,None,:]+half_o[None,:,:]-.0005,axis=2)
        overlap[moved[:,None]==others[None,:]] = False
        if np.any(overlap):
            i,j = np.argwhere(overlap)[0]
            raise AssertionError(f'Brick overlap: {ledger[moved[i]]["brick"]}/{ledger[others[j]]["brick"]} at {times[frame]:.3f}s: {delta[i,j]}')

    # Bricks must not pass through fixed machinery, walls, guards, kiln stacks or the moving C1/feeder parts.
    obstacles = [(f'/World/KilnInventory/stack{c}',np.array(c),np.array(h)) for c,h in kiln_stacks()]
    cache = UsdGeom.XformCache(0)
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not prim.IsA(UsdGeom.Cube) or any(part in path for part in ('/Flow/','/KilnInventory/','/FR3Visual/','/Pallets/',
                                                                    '/Bridge/','/Trolley/','/Spreader/','/TelescopicMast')):
            continue
        mat = cache.GetLocalToWorldTransform(prim)
        obstacles.append((path,np.array(mat.ExtractTranslation()),np.linalg.norm(np.array(mat)[:3,:3],axis=1)/2))
    chunk = 300
    lo = np.stack([positions[:,s:s+chunk].min(axis=1) for s in range(0,len(times),chunk)],1)-SPIN_HALF
    hi = np.stack([positions[:,s:s+chunk].max(axis=1) for s in range(0,len(times),chunk)],1)+SPIN_HALF
    for path,center,half in obstacles:
        candidate = np.argwhere(np.all(hi>center-half,axis=2)&np.all(lo<center+half,axis=2))
        for n,c in candidate:
            sl = slice(c*chunk,min((c+1)*chunk,len(times)))
            brick_half = np.where(spin[n,sl,None],SPIN_HALF,HALF)
            hit = np.all(np.abs(positions[n,sl]-center)<half+brick_half-.0005,axis=1)&active[n,sl]
            if np.any(hit):
                k = np.flatnonzero(hit)[0]
                raise AssertionError(f"Brick {ledger[n]['brick']} intersects {path} at {times[sl][k]:.3f}s, {positions[n,sl][k]}")

    # SP1: the paddle never cuts into a brick, and every layer is moved sideways only by the paddle pushing it.
    row = {e['brick']:n for n,e in enumerate(ledger)}
    paddle = keyed(UsdGeom.Xformable(stage.GetPrimAtPath('/World/Equipment/SP1/Spreader')).GetOrderedXformOps()[0],times)
    paddle_half = np.array([.43,.01,.025])
    near = np.flatnonzero(paddle[:,2] < .869+.025)     # paddle down at brick height
    for frame in near:
        live = active[:,frame]
        hit = np.all(np.abs(positions[live,frame]-paddle[frame])<paddle_half+HALF-.0005,axis=1)
        assert not np.any(hit), f'SP1 paddle cuts into a brick at {times[frame]:.3f}s'
    jobs = [h for h in report['handling'] if h['machine']=='SP1']
    assert len(jobs)==report['layers_in'] and [j['lane'] for j in jobs]==['A','B']*(len(jobs)//2)
    for job in jobs:
        n = [row[i] for i in job['bricks']]
        start,end = job['push_seconds']
        mask = (times>=start)&(times<=end)
        side = 1 if job['direction']=='north' else -1
        face = (positions[n][:,mask,1].min(axis=0)-HALF[1]) if side>0 else (positions[n][:,mask,1].max(axis=0)+HALF[1])
        gap = side*(face-paddle[mask,1])-.01
        assert np.all(gap>-1e-4) and np.all(gap<.004), f'SP1 loses contact with layer pushed to lane {job["lane"]}'
        assert np.all(paddle[mask,2]<.869), 'SP1 pushes with the paddle raised'
    separator_samples = len(near)

    # G1's lifting frame reaches down between full stacks without touching them.
    head = keyed(UsdGeom.Xformable(stage.GetPrimAtPath('/World/Equipment/G1/Spreader')).GetOrderedXformOps()[0],times)
    frame_parts = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath('/World/Equipment/G1/Spreader')):
        if prim.IsA(UsdGeom.Cube):
            ops = UsdGeom.Xformable(prim).GetOrderedXformOps()
            frame_parts.append((np.array(ops[0].Get()),np.array(ops[1].Get())/2))
    for center,half in kiln_stacks():
        for local,part in frame_parts:
            hit = np.all(np.abs(head+local-center)<np.array(half)+part,axis=1)
            assert not np.any(hit), f'G1 frame hits kiln stack at {center} at {times[np.flatnonzero(hit)[0]]:.2f}s'

    clearance = equipment_clearance(stage,times)

    import mujoco
    from robot_manipulation import MODEL, VELOCITY_LIMIT
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    manipulation = report['manipulation']
    home = np.array(manipulation['home_q_rad'])
    for plan in manipulation['plans']:
        q = np.array(plan['q_rad'])
        assert np.all(q>=model.jnt_range[:,0]) and np.all(q<=model.jnt_range[:,1])
        assert np.max(np.abs(np.diff(q,axis=0))*30/VELOCITY_LIMIT)<=1, 'FR3 joint velocity limit exceeded'
        assert np.max(np.abs(q[0]-home))<1e-10 and np.max(np.abs(q[-1]-home))<1e-10, 'Cycles must start and end at home'
    assert manipulation['max_self_penetration_m']<.0005
    result = {'passed':True,'audit_hz':30,'brick_positions_checked':int(active.sum()),
              'supported_surface_samples':support_samples,'crane_and_gantry_carry_samples':head_samples,
              'fr3_reject_picks':len(tasks),'separator_pushes':len(jobs),'separator_paddle_samples':separator_samples,'layers_in':report['layers_in'],'complete_layers_palletised':len(g2),
              'pallets':report['pallets'],'fixed_obstacles_checked':len(obstacles),
              'max_fr3_joint_velocity_fraction':manipulation['max_joint_velocity_fraction_of_limit'],
              **clearance,
              'brick_overlap_tolerance_m':.0005}
    (OUT/'validation_report.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
    return stage,report,times,positions


if __name__=='__main__':
    validate()
