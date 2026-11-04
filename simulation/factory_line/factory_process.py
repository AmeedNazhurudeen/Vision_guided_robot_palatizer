"""Dehacking line: kiln-car crane, FR3 reject removal on layer pairs, two fill lanes and palletising.

Flow (zoned layer belt LB along y=5.8, layers handled in pairs):
  G1 lifts layers from the kiln car onto LB. Two layers index together: the leading one to FR3 station 2,
  the trailing one to FR3 station 1, so both robot pairs work at the same time (80 bricks on the belt).
  An overhead camera above each station images the layer (shade, defects, pose) before the picks.
  FR3s on both sides lift the bad bricks onto the north/south scrap belts, which run beside LB to the crusher.
  After the FR3 stations the pair runs onto the split table at the end of LB, where the overhead two-way
  separator SP1 lowers a paddle behind each layer: it pushes the leading layer north onto fill lane A and the
  trailing layer south onto fill lane B, then lifts clear. On each lane a cartesian
  gantry (C1, C2) fills the holes with good bricks from that lane's donor layer. Crane G2 sets whichever
  lane's complete 40-brick layer is ready first on the pallet (14 layers = 560 bricks); a full pallet
  rolls out to shipping. When a donor runs short, the arriving layer becomes that lane's next donor.
"""
import math
from collections import defaultdict
import numpy as np
from pxr import Gf, UsdGeom
from robot_manipulation import Manipulators, ROBOTS, STATIONS, BELT_Y, BASE_DY, PLACE_DY, CYCLE, ATTACH, RELEASE, local_slot

YB = BELT_Y
Z = .8345                     # brick centre on any belt (belt top .8)
TOP = .869                    # brick top on any belt; also the lifting-head plane
LAYERS = 20                   # one complete 10 x 4 x 20 kiln-car stack = 800 bricks
STACK = (1,5)                 # kiln-car stack (a, b) being dehacked, top layer first
REJECT_RATE = .10             # ASSUMED: the brief gives no reject rate
SEED = 11
BELT_V,SHIFT_V,SCRAP_V,PALLET_V = .8,.4,.4,.3
HOIST,TRAVEL = .6,.8          # crane hoist and bridge/trolley speeds, m/s
C_XY,C_Z = 1.5,.8             # cartesian gantry axis speeds, m/s
IMAGE = 1.5                   # overhead camera image + pose estimate at each FR3 station, s
DROP,STAGE,SPLIT,LB_END = 2.0,4.2,12.1,11.5
LANES = {'A':6.2,'B':5.4}     # fill lanes after the split; A takes the leading layer of each pair
GANTRY = {'A':'C1','B':'C2'}
PARK_Y = {'A':7.0,'B':4.6}    # gantry bridges park over the scrap belts while G2 works their lane
FILL,DONOR,LANE_END = 13.15,14.45,15.0
G1Z,G2Z,CZ = 2.6,2.2,1.10     # head-plane travel heights: G1 clears the full stacks, G2 the gantry frame
SCRAP_Y = {1:YB+BASE_DY+PLACE_DY,-1:YB-BASE_DY-PLACE_DY}
SCRAP_TURN = {1:15.9,-1:15.45}  # x of each scrap belt's leg down to the crusher
CRUSHER = (15.65,2.9)
LEG_END = 3.8
PALLET = (FILL,7.75)
DISPENSER_X,SHIP_X = 11.3,16.6
PALLET_LAYERS = 14
# Separator SP1: overhead beam across the split table, carriage along y, paddle on a vertical rod.
SP_X,SP_LEN,SP_T,SP_H = 12.13,.86,.02,.05   # paddle x (stays >1.225 m from the station-2 FR3 shoulders), size
SP_DOWN,SP_UP = .8+.006+SP_H/2,TOP+.03+SP_H/2  # paddle centre: 6 mm over the table / 30 mm over the bricks
SP_BEAM,SP_POSTS = 1.40,(4.45,7.15)
SP_TRAVEL,SP_ZV,SP_GAP = .8,.3,.003            # empty carriage m/s, rod m/s, paddle-to-brick contact gap
CAMERA_POSTS = (7.0,9.3,11.3)  # station camera frame: posts clear of the FR3 swing and the G1/G2 columns


def revised_rows(rows):
    """Keep the v5 kiln car, G1 and layer belt; the front camera, lanes, formers and old pallets are replaced."""
    keep = {'KC1','RL1','PU1','G1','LB','S1','S2','S3','S4','CAB','FNC','FKZ'}
    result = []
    for original in rows:
        r = dict(original)
        name = r['id']
        if name not in keep:
            continue
        def pose(x=None,y=None,z=None,sx=None,sy=None,sz=None):
            for k,v in zip(('x','y','z','sx','sy','sz'),(x,y,z,sx,sy,sz)):
                if v is not None:
                    r[k] = str(v*1000)
        if name == 'KC1':
            pose(z=.15,sz=.25)  # deck rides on the rails instead of passing through them
        if name == 'LB':
            pose(x=(.9+LB_END)/2,sx=LB_END-.9)
            r['name'] = 'Layer belt, zoned: drop x2000, staging x4200, FR3 stations x8200/x10400; split table + separator x12100'
        if name == 'S2':
            pose(x=STAGE)
            r['name'] = 'Layer sensor: staging position'
        if name == 'S3':
            pose(x=7.6)
        if name == 'S4':
            pose(x=11.0)
            r['name'] = 'Layer sensor: belt exit to the split table'
        if name == 'CAB':
            pose(x=5.0,y=8.6)
        if name == 'FKZ':
            pose(x=SHIP_X+.4,y=PALLET[1],sx=1.8,sy=1.6)
            r['name'] = 'Shipping: full-pallet pick-up'
        result.append(r)
    for robot in ROBOTS:
        x,y,_ = robot['base']
        result.append({'id':robot['name'],'kind':'robot','x':str(x*1000),'y':str(y*1000),'z':'0',
                       'sx':'300','sy':'300','sz':'900',
                       'name':f"Robot {robot['name']}: Franka Research 3 + vacuum tool, station {robot['station']+1}, "
                              f"{'north' if robot['side']>0 else 'south'}; removes bad bricks"})
    return result


class Track:
    def __init__(self,point):
        self.points = [(0,tuple(point))]
        self.segments = []

    def add(self,t,p,support):
        old = self.points[-1][0]
        if t < old-1e-7:
            raise ValueError(f'Time went backwards: {old} -> {t}')
        if t > old+1e-7:
            self.segments.append({'start':old,'end':t,'support':support})
        self.points.append((float(t),tuple(map(float,p))))

    @property
    def last(self):
        return self.points[-1]


def slot_offset(col,row):
    return (col-1.5)*.245,(row-4.5)*.067


def build_process(stage,box,animate,fps):
    rng = np.random.default_rng(SEED)
    supports,handling = [],[]

    def surface(path,x,y,w,d,top=.8,color='belt'):
        box(path,(x,y,top-.04),(w,d,.08),color,True)
        supports.append({'path':path,'x':x,'y':y,'sx':w,'sy':d,'top':top})

    supports.append({'path':'/World/Equipment/LB/Bed','x':(.9+LB_END)/2,'y':YB,'sx':LB_END-.9,'sy':.7,'top':.8})
    surface('/World/Transport/SplitTable',(LB_END+FILL-.55)/2,YB,FILL-.55-LB_END,1.5,color='steel')
    for lane,y in LANES.items():
        surface(f'/World/Transport/Lane{lane}',(FILL-.55+LANE_END)/2,y,LANE_END-FILL+.55,.7)
    for side in (1,-1):
        name = 'North' if side > 0 else 'South'
        start,turn,y = 7.6,SCRAP_TURN[side],SCRAP_Y[side]
        surface(f'/World/Transport/Scrap{name}',(start+turn+.18)/2,y,turn+.18-start,.3)
        surface(f'/World/Transport/Scrap{name}Down',turn,(y+.15+LEG_END)/2,.36,y+.15-LEG_END)
    cx,cy = CRUSHER
    box('/World/Crusher/Body',(cx,cy,.35),(1.6,1.0,.7),'blue',True)
    for j,(pos,size) in enumerate((((cx,cy-.5,.85),(1.6,.04,.3)),((cx-.8,cy,.85),(.04,1.0,.3)),((cx+.8,cy,.85),(.04,1.0,.3)))):
        box(f'/World/Crusher/HopperWall{j}',pos,size,'steel')
    box('/World/Crusher/Drive',(cx,cy-.85,.3),(.6,.5,.6),'steel')
    box('/World/Crusher/ClayReturn',(cx+3.35,cy,.35),(5.0,.4,.08),'belt')
    for side in (-1,1):
        box(f'/World/PalletLine/Rail{side+1}',((DISPENSER_X+SHIP_X+1.2)/2,PALLET[1]+side*.48,.03),(SHIP_X+1.2-DISPENSER_X+.6,.05,.06),'steel')
    for j,(dx,dy) in enumerate(((-.58,-.43),(-.58,.43),(.58,-.43),(.58,.43))):
        box(f'/World/PalletLine/DispenserPost{j}',(DISPENSER_X+dx,PALLET[1]+dy,.85),(.06,.06,1.7),'yellow')
    box('/World/PalletLine/DispenserTop',(DISPENSER_X,PALLET[1],1.72),(1.22,.92,.04),'yellow')
    # Overhead camera + ring light directly above each FR3 station's pick position.
    root = '/World/Equipment/CAMF'
    UsdGeom.Xform.Define(stage,root).GetPrim().SetCustomDataByKey('name','Station camera frame: one camera above each FR3 pair')
    for i,x in enumerate(CAMERA_POSTS):
        for j,y in enumerate((4.45,7.15)):
            box(f'{root}/Post{i}{j}',(x,y,1.225),(.1,.1,2.45),'yellow')
        box(f'{root}/CrossBeam{i}',(x,5.8,2.45),(.1,2.8,.1),'yellow')
    box(f'{root}/Spine',(sum(CAMERA_POSTS[::2])/2,YB,2.45),(CAMERA_POSTS[-1]-CAMERA_POSTS[0],.1,.1),'yellow')
    for s,x in enumerate(STATIONS):
        box(f'{root}/Camera{s+1}',(x,YB,2.33),(.14,.14,.14),'blue')
        for j,(dx,dy,w,d) in enumerate(((-.35,0,.05,.6),(.35,0,.05,.6),(0,-.3,.7,.05),(0,.3,.7,.05))):
            box(f'{root}/Light{s+1}_{j}',(x+dx,YB+dy,2.37),(w,d,.04),'yellow')

    def crane(name,cx,cy,span,travel,head_points):
        """Bridge + trolley + telescopic mast + 40 independent suction zones."""
        root = '/World/Equipment/'+name
        for side in (-1,1):
            for j,yy in enumerate((cy-travel/2,cy+travel/2)):
                box(f'{root}/Column{side+1}{j}',(cx+side*span/2,yy,2.5),(.18,.18,5),'yellow')
            box(f'{root}/Runway{side+1}',(cx+side*span/2,cy,5),( .22,travel,.25),'yellow')
        bridge = UsdGeom.Xform.Define(stage,root+'/Bridge')
        bridge.AddTranslateOp().Set(Gf.Vec3d(cx,cy,5.15))
        for side in (-1,1):
            box(f'{root}/Bridge/Girder{side+1}',(0,side*.23,0),(span,.16,.3),'yellow')
        trolley = UsdGeom.Xform.Define(stage,root+'/Trolley')
        trolley.AddTranslateOp().Set(Gf.Vec3d(cx,cy,4.95))
        box(root+'/Trolley/Carriage',(0,0,0),(.7,.65,.22),'blue')
        box(root+'/Trolley/DriveMotor',(.4,0,0),(.17,.32,.24),'steel')
        mast = box(root+'/TelescopicMast',(cx,cy,3),(.12,.12,2.8),'steel')
        head = UsdGeom.Xform.Define(stage,root+'/Spreader')
        head.AddTranslateOp().Set(Gf.Vec3d(cx,cy,3))
        # The frame stays inside the 1.04 x 0.73 m stack pitch so it can reach down between full stacks.
        for side in (-1,1):
            box(f'{root}/Spreader/LongBeam{side+1}',(0,side*.29,.18),(1.0,.065,.09),'blue')
            box(f'{root}/Spreader/CrossBeam{side+1}',(side*.46,0,.18),(.065,.72,.09),'blue')
        box(root+'/Spreader/CrossTie',(0,0,.18),(.065,.72,.09),'blue')
        for col in range(4):
            for row in range(10):
                dx,dy = slot_offset(col,row)
                box(f'{root}/Spreader/Stem{col}_{row}',(dx,dy,.09),(.025,.025,.15),'steel')
                pad = UsdGeom.Cylinder.Define(stage,f'{root}/Spreader/Pad{col}_{row}')
                pad.CreateRadiusAttr(.025)
                pad.CreateHeightAttr(.012)
                pad.AddTranslateOp().Set(Gf.Vec3d(dx,dy,.006))
                pad.CreateDisplayColorAttr([Gf.Vec3f(.05,.6,.7)])
        animate(head,head_points)
        animate(bridge,[(t,(cx,p[1],5.15)) for t,p in head_points])
        animate(trolley,[(t,(p[0],p[1],4.95)) for t,p in head_points])
        animate(mast,[(t,(p[0],p[1],(4.82+p[2]+.23)/2)) for t,p in head_points])
        scale = mast.GetOrderedXformOps()[1]
        for t,p in head_points:
            scale.Set(Gf.Vec3f(.12,.12,4.82-(p[2]+.23)),t*fps)

    # ------------------------------------------------------------------ bricks: one full stack
    a,b = STACK
    stack_x,stack_y = .7+a*1.04,.47+b*.73
    tracks,entries,layers = {},[],[]
    for layer in range(LAYERS):
        h = 19-layer
        bricks = {}
        for col in range(4):
            for row in range(10):
                index = layer*40+col*10+row
                dx,dy = slot_offset(col,row)
                bad = rng.random() < REJECT_RATE
                shade = str(rng.choice(['X','D'])) if bad else 'ABC'[rng.integers(3)]
                initial = (stack_x+dx,stack_y+dy,.4+(h+.5)*.069)
                brick = box(f'/World/Flow/Brick{index:03}',initial,(.245,.067,.069),shade)
                brick.GetPrim().SetCustomDataByKey('shade',shade)
                brick.GetPrim().SetCustomDataByKey('route','crusher' if bad else 'pallet')
                tracks[index] = Track(initial)
                bricks[(col,row)] = index
                entries.append({'brick':index,'layer':layer,'stack_level':h,'slot':[col,row],'shade':shade,'bad':bad})
        layers.append({'index':layer,'h':h,'bricks':bricks})
    ledger = {e['brick']:e for e in entries}

    def move_layer(bricks,t,x,y=YB,support='surface'):
        """Record every brick of a layer (slot -> index) at layer centre (x, y) at time t."""
        for (col,row),index in bricks.items():
            dx,dy = slot_offset(col,row)
            tracks[index].add(t,(x+dx,y+dy,Z),support)

    # ------------------------------------------------------------------ G1: kiln car -> LB drop
    g1 = [(0,(stack_x,stack_y,G1Z))]
    g1_free = 0.
    travel_g1 = math.hypot(DROP-stack_x,YB-stack_y)/TRAVEL

    def g1_drop(layer,touchdown):
        """Lift the stack's top layer and set it on the LB drop zone no earlier than `touchdown`."""
        nonlocal g1_free
        top = .4+(layer['h']+1)*.069
        start = g1_free
        t1 = start+(G1Z-top)/HOIST
        t2 = t1+1.0
        t3 = t2+(G1Z-top)/HOIST
        t4 = t3+travel_g1
        t4w = max(t4,touchdown-(G1Z-TOP)/HOIST)
        t5 = t4w+(G1Z-TOP)/HOIST
        t6 = t5+1.0
        t7 = t6+(G1Z-TOP)/HOIST
        t8 = t7+travel_g1
        g1.extend([(start,(stack_x,stack_y,G1Z)),(t1,(stack_x,stack_y,top)),(t2,(stack_x,stack_y,top)),
                   (t3,(stack_x,stack_y,G1Z)),(t4,(DROP,YB,G1Z)),(t4w,(DROP,YB,G1Z)),(t5,(DROP,YB,TOP)),
                   (t6,(DROP,YB,TOP)),(t7,(DROP,YB,G1Z)),(t8,(stack_x,stack_y,G1Z))])
        g1_free = t8
        for (col,row),index in layer['bricks'].items():
            dx,dy = slot_offset(col,row)
            tr = tracks[index]
            if t2 > 0:
                tr.add(t2,tr.last[1],'cart')
            for tt,(x,y,z) in ((t3,(stack_x,stack_y,G1Z)),(t4,(DROP,YB,G1Z)),(t4w,(DROP,YB,G1Z)),(t5,(DROP,YB,TOP)),(t6,(DROP,YB,TOP))):
                tr.add(tt,(x+dx,y+dy,z-.0345),'G1')
        handling.append({'machine':'G1','layer':layer['index'],'stack_level':layer['h'],'bricks':sorted(layer['bricks'].values()),
                         'attach_seconds':t2,'release_seconds':t6,'busy_seconds':t8-start-(t4w-t4)})
        return t6

    # ------------------------------------------------------------------ FR3 stations
    robots = Manipulators(stage,fps)
    robot_free = defaultdict(float)
    zone_windows = defaultdict(list)       # station -> [(start,end)] layer-envelope occupancy
    releases = {1:[],-1:[]}                 # scrap belt -> [(x,t)]

    def scrap_conflict(x,t,side):
        """Two bricks on one scrap belt (same speed) must stay a brick length + 120 mm apart while one is set down."""
        for x2,t2 in releases[side]:
            (xa,ta),(xb,tb) = ((x2,t2),(x,t)) if t2 <= t else ((x,t),(x2,t2))
            for tau in np.linspace(tb-.6,tb,7):
                if tau >= ta and abs(xa+SCRAP_V*(tau-ta)-xb) < .365:
                    return True
        return False

    def to_crusher(index,t,side):
        """Scrap belt run from the release point to the crusher; the brick is consumed in the hopper."""
        tr = tracks[index]
        x,y,_ = tr.last[1]
        turn,yy = SCRAP_TURN[side],SCRAP_Y[side]
        t = t+(turn-x)/SCRAP_V
        tr.add(t,(turn,yy,Z),'surface')
        t += (yy-(LEG_END+.05))/SCRAP_V
        tr.add(t,(turn,LEG_END+.05,Z),'surface')
        consumed = t
        t += (LEG_END+.05-CRUSHER[1])/SCRAP_V
        tr.add(t,(turn,CRUSHER[1],Z),'crusher')
        tr.add(t+.6,(turn,CRUSHER[1],.35),'crusher')
        # A single time sample would hold for the whole run: keep the brick visible until it drops in.
        visibility = UsdGeom.Imageable(stage.GetPrimAtPath(f'/World/Flow/Brick{index:03}')).GetVisibilityAttr()
        visibility.Set('inherited',(t+.5)*fps)
        visibility.Set('invisible',(t+.6)*fps)
        ledger[index].update({'destination':'crusher','consumed_seconds':consumed})

    def station_picks(layer,station,t_arrive):
        """Image the layer, then remove all its bad bricks: north FR3 rows 5-9, south FR3 rows 0-4."""
        x = STATIONS[station]
        t_image = t_arrive+IMAGE
        for index in layer['bricks'].values():
            ledger[index].update({'classified_seconds':t_image,'camera':station+1})
        leave = t_image
        jobs = []
        for side in (1,-1):
            mine = sorted(slot for slot,index in layer['bricks'].items()
                          if ledger[index]['bad'] and (slot[1] >= 5) == (side > 0))
            robot = next(i for i,r in enumerate(ROBOTS) if r['station'] == station and r['side'] == side)
            jobs += [(k,robot,side,slot) for k,slot in enumerate(mine)]
        jobs.sort()  # alternate north and south robots
        for _,robot,side,(col,row) in jobs:
            index = layer['bricks'][(col,row)]
            zone = robots.plan(local_slot(ROBOTS[robot],col,row))[0]['zone']
            start = math.ceil(max(t_image,robot_free[robot])*fps)/fps
            while (any(start+zone[0] < e and s < start+zone[1] for s,e in zone_windows[station])
                   or scrap_conflict(x,start+RELEASE,side)):
                start += 1/fps
            tracks[index].add(start+ATTACH,tracks[index].last[1],'surface')
            brick = stage.GetPrimAtPath(f'/World/Flow/Brick{index:03}')
            for t,p in robots.remove(brick,index,robot,start,col,row):
                tracks[index].add(t,p,ROBOTS[robot]['name'])
            robot_free[robot] = start+CYCLE
            zone_windows[station].append((start+zone[0],start+zone[1]))
            releases[side].append((x,start+RELEASE))
            to_crusher(index,start+RELEASE,side)
            ledger[index].update({'robot':ROBOTS[robot]['name'],'station':station+1})
            del layer['bricks'][(col,row)]
            leave = max(leave,start+zone[1])
        return leave

    # ------------------------------------------------------------------ C1/C2 gantries and G2 crane
    gantries = {lane:[(0,(DONOR,PARK_Y[lane],CZ))] for lane in LANES}
    donors = {lane:{} for lane in LANES}
    g2 = [(0,(FILL,LANES['A'],G2Z))]
    g2_free = 0.
    pallets = []
    c_moves = []

    def pallet(n):
        while len(pallets) <= n:
            k = len(pallets)
            x0 = PALLET[0] if k == 0 else DISPENSER_X
            root = UsdGeom.Xform.Define(stage,f'/World/Pallets/P{k}')
            root.AddTranslateOp().Set(Gf.Vec3d(x0,PALLET[1],0))
            for j in range(7):
                box(f'/World/Pallets/P{k}/Board{j}',(0,-.355+(j+.5)*.71/7,.125),(1.02,.71/8,.05),'wood')
            for j in (-1,0,1):
                box(f'/World/Pallets/P{k}/Runner{j+1}',(j*1.02*.38,0,.05),(.10,.71,.10),'wood')
            pallets.append({'path':f'/World/Pallets/P{k}','points':[(0,(x0,PALLET[1],0))],'layers':[],
                            'ready':0. if k == 0 else None})
        return pallets[n]

    def c_transfer(lane,t,index,dst):
        """Move one brick with the lane's single-cup cartesian head; returns the time the head is back up."""
        points = gantries[lane]
        src = tracks[index].last[1]
        p0 = points[-1][1]
        t = max(t,points[-1][0])
        up_s,down_s = (src[0],src[1],CZ),(src[0],src[1],TOP)
        up_d,down_d = (dst[0],dst[1],CZ),(dst[0],dst[1],TOP)
        lower = (CZ-TOP)/C_Z
        t1 = t+math.dist(p0,up_s)/C_XY
        t2,t3 = t1+lower,t1+lower+.3
        t4 = t3+lower
        t5 = t4+math.dist(up_s,up_d)/C_XY
        t6,t7 = t5+lower,t5+lower+.3
        t8 = t7+lower
        points.extend([(t,p0),(t1,up_s),(t2,down_s),(t3,down_s),(t4,up_s),(t5,up_d),(t6,down_d),(t7,down_d),(t8,up_d)])
        tr = tracks[index]
        tr.add(t3,src,'surface')
        for tt,p in ((t4,up_s),(t5,up_d),(t6,down_d),(t7,down_d)):
            tr.add(tt,(p[0],p[1],p[2]-.0345),GANTRY[lane])
        c_moves.append({'gantry':GANTRY[lane],'brick':index,'grip_seconds':t3,'release_seconds':t7})
        return t8

    def c_park(lane,t):
        points = gantries[lane]
        p0 = points[-1][1]
        t = max(t,points[-1][0])
        park = (DONOR,PARK_Y[lane],CZ)
        points.extend([(t,p0),(t+math.dist(p0,park)/C_XY,park)])
        return points[-1][0]

    def fill(lane,layer,t):
        """Fill the holes from the lane's donor. Returns (complete, time) - ready for G2, or departed as donor."""
        y = LANES[lane]
        donor = donors[lane]
        holes = sorted(set((c,r) for c in range(4) for r in range(10))-set(layer['bricks']),key=lambda s:(-s[1],s[0]))
        complete = len(donor) >= len(holes)
        for slot in holes:
            if not donor:
                break
            dx,dy = slot_offset(*slot)
            target = (FILL+dx,y+dy,Z)
            # Nearest donor brick first keeps the gantry strokes short.
            source = min(donor,key=lambda s:math.dist(tracks[donor[s]].last[1],target))
            index = donor.pop(source)
            t = c_transfer(lane,t,index,target)
            tracks[index].add(t,target,'surface')
            layer['bricks'][slot] = index
        t = c_park(lane,t)
        if complete:
            return True,t
        # Too few donor bricks: this layer rolls onto the donor table and becomes the lane's donor.
        move_layer(layer['bricks'],t,FILL,y)
        move_layer(layer['bricks'],t+(DONOR-FILL)/BELT_V,DONOR,y)
        donor.update(layer['bricks'])
        for index in donor.values():
            ledger[index]['donor'] = lane
        handling.append({'machine':'lane '+lane,'action':'layer to donor table','seconds':t,'bricks':len(donor)})
        return False,t+1.1/BELT_V

    def g2_lift(lane,layer,t):
        """G2 lifts a complete layer from fill lane `lane` onto the current pallet. Returns the lift-off time."""
        nonlocal g2_free
        y = LANES[lane]
        n = sum(len(p['layers']) for p in pallets)//PALLET_LAYERS
        pal = pallet(n)
        k = len(pal['layers'])
        t0 = max(t,g2_free)
        ta = t0+abs(g2[-1][1][1]-y)/TRAVEL
        tb = ta+(G2Z-TOP)/HOIST
        tc = tb+1.0
        td = tc+(G2Z-TOP)/HOIST
        te = max(td+(PALLET[1]-y)/TRAVEL,pal['ready'])
        level = .15+(k+1)*.069
        tf = te+(G2Z-level)/HOIST
        tg = tf+1.0
        th = tg+(G2Z-level)/HOIST
        g2.extend([(t0,g2[-1][1]),(ta,(FILL,y,G2Z)),(tb,(FILL,y,TOP)),(tc,(FILL,y,TOP)),(td,(FILL,y,G2Z)),
                   (te,(PALLET[0],PALLET[1],G2Z)),(tf,(PALLET[0],PALLET[1],level)),(tg,(PALLET[0],PALLET[1],level)),
                   (th,(PALLET[0],PALLET[1],G2Z))])
        g2_free = th
        move_layer(layer['bricks'],tc,FILL,y)
        for (col,row),index in layer['bricks'].items():
            dx,dy = slot_offset(col,row)
            tr = tracks[index]
            for tt,(x,yy,z) in ((td,(FILL,y,G2Z)),(te,(PALLET[0],PALLET[1],G2Z)),(tf,(PALLET[0],PALLET[1],level)),(tg,(PALLET[0],PALLET[1],level))):
                tr.add(tt,(x+dx,yy+dy,z-.0345),'G2')
            tr.add(th,tr.last[1],f'pallet:{n}')
            ledger[index].update({'destination':'pallet','pallet':n,'pallet_layer':k,'lane':lane})
        pal['layers'].append({'bricks':sorted(layer['bricks'].values()),'release_seconds':tg})
        handling.append({'machine':'G2','lane':lane,'pallet':n,'pallet_layer':k,'bricks':sorted(layer['bricks'].values()),
                         'attach_seconds':tc,'release_seconds':tg,'busy_seconds':th-t0})
        if len(pal['layers']) == PALLET_LAYERS:
            # Full pallet (10 x 4 x 14 = 560) rolls out to shipping; the dispenser feeds the next one in.
            ship = th+(SHIP_X-PALLET[0])/PALLET_V
            pal['points'] += [(th,(PALLET[0],PALLET[1],0)),(ship,(SHIP_X,PALLET[1],0))]
            pal['shipped_seconds'] = ship
            for item in pal['layers']:
                for index in item['bricks']:
                    x,yy,z = tracks[index].last[1]
                    tracks[index].add(th,(x,yy,z),f'pallet:{n}')
                    tracks[index].add(ship,(x+SHIP_X-PALLET[0],yy,z),f'pallet:{n}')
            nxt = pallet(n+1)
            nxt['ready'] = th+(PALLET[0]-DISPENSER_X)/PALLET_V
            nxt['points'] += [(th,(DISPENSER_X,PALLET[1],0)),(nxt['ready'],(PALLET[0],PALLET[1],0))]
        return td

    # ------------------------------------------------------------------ SP1 separator on the split table
    sp = [(0,(SP_X,YB,SP_UP))]
    sp_jobs = []

    def separate(layer,t_arrive,lane):
        """Lower the paddle behind the layer and push it onto `lane`; returns the time the paddle is back up."""
        y = LANES[lane]
        side = 1 if y > YB else -1          # +1: push north (paddle on the south face)
        rows = [r for _,r in layer['bricks']]
        face = (min(rows) if side > 0 else max(rows)-9)*.067   # touch the outermost brick row present
        y0 = YB+face-side*(.335+SP_T/2+SP_GAP)
        y1 = y0+(y-YB)
        t0,p0 = sp[-1]
        tm = t0+abs(p0[1]-y0)/SP_TRAVEL     # raised paddle travels over the layer while it arrives
        tl = max(tm,t_arrive)
        td = tl+(SP_UP-SP_DOWN)/SP_ZV
        tp = td+abs(y-YB)/SHIFT_V
        tu = tp+(SP_UP-SP_DOWN)/SP_ZV
        sp.extend([(tm,(SP_X,y0,SP_UP)),(tl,(SP_X,y0,SP_UP)),(td,(SP_X,y0,SP_DOWN)),
                   (tp,(SP_X,y1,SP_DOWN)),(tu,(SP_X,y1,SP_UP))])
        move_layer(layer['bricks'],td,SPLIT)
        move_layer(layer['bricks'],tp,SPLIT,y)
        sp_jobs.append({'machine':'SP1','lane':lane,'direction':'north' if side > 0 else 'south',
                        'bricks':sorted(layer['bricks'].values()),'paddle_y':[y0,y1],
                        'push_seconds':[td,tp],'busy_seconds':(tm-t0)+(tu-tl)})
        return tu

    # ------------------------------------------------------------------ schedule, one layer pair at a time
    drop_free = stage_free = st1_free = st2_free = split_free = 0.
    lane_free = {lane:0. for lane in LANES}
    pair_times,dwell = [],defaultdict(list)
    for k in range(0,LAYERS,2):
        lead,trail = layers[k],layers[k+1]
        # Lead layer: drop, then on to staging so the trailing layer can be dropped behind it.
        t = g1_drop(lead,drop_free)
        go = max(t,stage_free)
        move_layer(lead['bricks'],go,DROP)
        t_stage = go+(STAGE-DROP)/BELT_V
        move_layer(lead['bricks'],t_stage,STAGE)
        t = g1_drop(trail,go+1.1/BELT_V)
        # Index the pair: lead to station 2, trailing to station 1, both 6.2 m.
        t_idx = max(t_stage,t,st1_free,st2_free)
        move_layer(lead['bricks'],t_idx,STAGE)
        move_layer(trail['bricks'],t_idx,DROP)
        arrive = t_idx+(STATIONS[0]-DROP)/BELT_V
        move_layer(lead['bricks'],arrive,STATIONS[1])
        move_layer(trail['bricks'],arrive,STATIONS[0])
        drop_free = stage_free = t_idx+1.1/BELT_V
        pair_times.append(t_idx)
        ready_lead = station_picks(lead,1,arrive)
        ready_trail = station_picks(trail,0,arrive)
        dwell['station2'].append(ready_lead-arrive)
        dwell['station1'].append(ready_trail-arrive)
        arrivals = {}
        for layer,station,lane,ready in ((lead,1,'A',ready_lead),(trail,0,'B',ready_trail)):
            x0 = STATIONS[station]
            # The trailing layer follows the lead through station 2; the split table takes one layer at a time.
            depart = max(ready,split_free-(SPLIT-x0)/BELT_V,st2_free if station == 0 else 0.)
            move_layer(layer['bricks'],depart,x0)
            t = depart+(SPLIT-x0)/BELT_V
            move_layer(layer['bricks'],t,SPLIT)
            if station == 1:
                st2_free = depart+1.1/BELT_V
            else:
                st1_free = depart+1.1/BELT_V
            y = LANES[lane]
            t = separate(layer,t,lane)
            move_layer(layer['bricks'],t,SPLIT,y)
            t = max(t,lane_free[lane])
            move_layer(layer['bricks'],t,SPLIT,y)
            split_free = t+1.1/BELT_V
            arrivals[lane] = t+(FILL-SPLIT)/BELT_V
            move_layer(layer['bricks'],arrivals[lane],FILL,y)
        # Fill both lanes, then G2 serves whichever lane's layer is complete first.
        ready = {}
        for layer,lane in ((lead,'A'),(trail,'B')):
            complete,t = fill(lane,layer,arrivals[lane])
            dwell['fill '+lane].append(t-arrivals[lane])
            if complete:
                ready[lane] = (t,layer)
            else:
                lane_free[lane] = t
        for lane,(t,layer) in sorted(ready.items(),key=lambda item:item[1][0]):
            lane_free[lane] = g2_lift(lane,layer,t)

    # ------------------------------------------------------------------ author motion
    end = max(max(tr.last[0] for tr in tracks.values()),g1[-1][0],g2[-1][0],
              *(p[-1][0] for p in gantries.values()),*(p['points'][-1][0] for p in pallets))
    duration = math.ceil(end+10)
    crane('G1',3.3,3.0,7.9,7.5,g1+[(duration,g1[-1][1])])
    crane('G2',FILL,6.5,3.2,4.2,g2+[(duration,g2[-1][1])])
    separator(stage,box,animate,sp+[(duration,sp[-1][1])],fps)
    gantry_frame(stage,box)
    for lane,points in gantries.items():
        gantry(stage,box,animate,GANTRY[lane],points+[(duration,points[-1][1])],fps)
    for pal in pallets:
        animate(UsdGeom.Xformable(stage.GetPrimAtPath(pal['path'])),pal['points']+[(duration,pal['points'][-1][1])])
    robots.finish(duration)
    for index,tr in tracks.items():
        tr.add(duration,tr.last[1],tr.segments[-1]['support'] if tr.segments else 'cart')
        animate(UsdGeom.Xformable(stage.GetPrimAtPath(f'/World/Flow/Brick{index:03}')),tr.points)
        ledger[index]['final_position_m'] = list(tr.last[1])
        ledger[index]['segments'] = tr.segments
        if 'destination' not in ledger[index]:
            ledger[index]['destination'] = 'donor table' if ledger[index].get('donor') else 'unknown'

    # ------------------------------------------------------------------ production rate
    g1_jobs = [h for h in handling if h['machine'] == 'G1']
    g2_jobs = [h for h in handling if h['machine'] == 'G2']
    handling += sp_jobs
    span = g1_jobs[-1]['release_seconds']-g1_jobs[0]['release_seconds']
    busy = defaultdict(float)
    for task in robots.tasks:
        busy[task['robot']] += CYCLE
    for move in c_moves:
        busy[move['gantry']] += move['release_seconds']-move['grip_seconds']+1.0
    window = (g1_jobs[0]['release_seconds'],g2_jobs[-1]['release_seconds'])
    utilisation = {'G1 crane':sum(h['busy_seconds'] for h in g1_jobs)/(window[1]-window[0]),
                   'G2 crane':sum(h['busy_seconds'] for h in g2_jobs)/(window[1]-window[0]),
                   'SP1 separator':sum(h['busy_seconds'] for h in sp_jobs)/(window[1]-window[0]),
                   **{r['name']:busy[r['name']]/(window[1]-window[0]) for r in ROBOTS},
                   **{g:busy[g]/(window[1]-window[0]) for g in GANTRY.values()}}
    production = {
        'brief_bricks_per_hour':[1500,3000],'reject_rate_assumed':REJECT_RATE,
        'input_bricks_per_hour_measured':round(80*3600/float(np.mean(np.diff(pair_times)))),
        'good_bricks_palletised_per_hour_measured':round(40*3600*(len(g2_jobs)-1)/(g2_jobs[-1]['release_seconds']-g2_jobs[0]['release_seconds'])),
        'pair_cycle_seconds':{'mean':round(float(np.mean(np.diff(pair_times))),2),'max':round(float(np.max(np.diff(pair_times))),2)},
        'utilisation':{k:round(v,3) for k,v in utilisation.items()},
        'bottleneck':max(utilisation,key=utilisation.get),
        'fr3_picks':{r['name']:sum(t['robot'] == r['name'] for t in robots.tasks) for r in ROBOTS},
        'gantry_moves':{g:sum(m['gantry'] == g for m in c_moves) for g in GANTRY.values()},
        'separator_pushes':{lane:sum(j['lane'] == lane for j in sp_jobs) for lane in LANES},
        'layer_dwell_seconds':{k:{'mean':round(float(np.mean(v)),2),'max':round(float(np.max(v)),2)} for k,v in dwell.items()},
    }
    counts = defaultdict(int)
    for e in entries:
        counts[e['destination']] += 1
    both = [t for t in robots.tasks if t['station'] == 2 and any(u['station'] == 1 and abs(u['attach_seconds']-t['attach_seconds']) < 2 for u in robots.tasks)]
    review = {'gantry_layer_lift':g1_jobs[0]['attach_seconds']+2,
              'parallel_robots':(both or robots.tasks)[len(both or robots.tasks)//3]['attach_seconds']+.5,
              'separator_push':float(np.mean(sp_jobs[2]['push_seconds'])),
              'pallet_layer_lift':g2_jobs[len(g2_jobs)//2]['attach_seconds']+3,
              'pallet_to_shipping':pallets[0].get('shipped_seconds',duration)-4}
    return {'duration':duration,'destinations':dict(counts),
            'layers_in':LAYERS,'layers_out':len(g2_jobs),
            'pallets':[{'pallet':n,'layers':len(p['layers']),'bricks':sum(len(l['bricks']) for l in p['layers']),
                        'shipped_seconds':p.get('shipped_seconds')} for n,p in enumerate(pallets)],
            'donor_left':sorted(i for d in donors.values() for i in d.values()),
            'motion':'G1 dehacks one stack; layer pairs at two FR3 stations with overhead cameras; separator SP1 pushes '
                     'the lead layer to fill lane A and the trailing layer to lane B; G2 palletises whichever lane is ready first',
            'production':production,'review_seconds':review,
            'manipulation':robots.report(),'c1':c_moves,'handling':handling,'supports':supports,
            'pair_index_seconds':pair_times,'ledger':entries}


def separator(stage,box,animate,points,fps):
    """Two-way pusher over the split table: beam across y, carriage, vertical rod and a paddle at brick height."""
    root = '/World/Equipment/SP1'
    UsdGeom.Xform.Define(stage,root).GetPrim().SetCustomDataByKey(
        'name','Separator SP1: overhead two-way pusher; lead layer -> lane A (north), trailing layer -> lane B (south)')
    for j,y in enumerate(SP_POSTS):
        box(f'{root}/Post{j}',(SP_X,y,SP_BEAM/2),(.1,.1,SP_BEAM),'yellow')
    box(root+'/Beam',(SP_X,sum(SP_POSTS)/2,SP_BEAM+.06),(.12,SP_POSTS[1]-SP_POSTS[0]+.1,.12),'yellow')
    trolley = UsdGeom.Xform.Define(stage,root+'/Trolley')
    trolley.AddTranslateOp().Set(Gf.Vec3d(SP_X,points[0][1][1],SP_BEAM-.06))
    box(root+'/Trolley/Carriage',(0,0,0),(.2,.16,.12),'blue')
    rod = box(root+'/TelescopicMast',(SP_X,points[0][1][1],1.0),(.04,.04,.3),'steel')
    head = UsdGeom.Xform.Define(stage,root+'/Spreader')
    head.AddTranslateOp().Set(Gf.Vec3d(*points[0][1]))
    box(root+'/Spreader/Paddle',(0,0,0),(SP_LEN,SP_T,SP_H),'blue')
    box(root+'/Spreader/Stiffener',(0,0,SP_H/2+.015),(SP_LEN,.04,.03),'steel')
    animate(head,points)
    animate(trolley,[(t,(SP_X,p[1],SP_BEAM-.06)) for t,p in points])
    bottom = SP_BEAM-.12                   # carriage underside
    animate(rod,[(t,(SP_X,p[1],(bottom+p[2]+SP_H/2+.03)/2)) for t,p in points])
    scale = rod.GetOrderedXformOps()[1]
    for t,p in points:
        scale.Set(Gf.Vec3f(.04,.04,bottom-(p[2]+SP_H/2+.03)),t*fps)


def gantry_frame(stage,box):
    """Shared frame for the two fill-lane gantries: rails along y outside the scrap belts."""
    root = '/World/Equipment/C1'
    UsdGeom.Xform.Define(stage,root).GetPrim().SetCustomDataByKey('name','Cartesian layer-completion gantries C1 (lane A) and C2 (lane B)')
    for i,x in enumerate(FRAME_X):
        for j,y in enumerate(FRAME_Y):
            box(f'{root}/Post{i}{j}',(x,y,.75),(.1,.1,1.5),'yellow')
        box(f'{root}/Rail{i}',(x,sum(FRAME_Y)/2,1.5),(.1,FRAME_Y[1]-FRAME_Y[0]+.1,.1),'yellow')


FRAME_X,FRAME_Y = (FILL-.55,LANE_END+.05),(4.45,7.15)


def gantry(stage,box,animate,name,points,fps):
    """Cartesian XYZ gantry: bridge along x riding the y rails, carriage, Z rod and a single vacuum cup."""
    root = '/World/Equipment/'+name
    UsdGeom.Xform.Define(stage,root)
    bridge = UsdGeom.Xform.Define(stage,root+'/Bridge')
    bridge.AddTranslateOp().Set(Gf.Vec3d(sum(FRAME_X)/2,points[0][1][1],1.5))
    box(root+'/Bridge/Beam',(0,0,.08),(FRAME_X[1]-FRAME_X[0]-.1,.1,.08),'blue')
    trolley = UsdGeom.Xform.Define(stage,root+'/Trolley')
    trolley.AddTranslateOp().Set(Gf.Vec3d(*points[0][1][:2],1.5))
    box(root+'/Trolley/Carriage',(0,0,0),(.18,.18,.14),'blue')
    rod = box(root+'/TelescopicMast',(points[0][1][0],points[0][1][1],1.2),(.03,.03,.4),'steel')
    head = UsdGeom.Xform.Define(stage,root+'/Spreader')
    head.AddTranslateOp().Set(Gf.Vec3d(*points[0][1]))
    cup = UsdGeom.Cylinder.Define(stage,root+'/Spreader/Cup')
    cup.CreateRadiusAttr(.03)
    cup.CreateHeightAttr(.012)
    cup.AddTranslateOp().Set(Gf.Vec3d(0,0,.006))
    cup.CreateDisplayColorAttr([Gf.Vec3f(.05,.6,.7)])
    animate(head,points)
    animate(bridge,[(t,(sum(FRAME_X)/2,p[1],1.5)) for t,p in points])
    animate(trolley,[(t,(p[0],p[1],1.5)) for t,p in points])
    animate(rod,[(t,(p[0],p[1],(1.43+p[2]+.012)/2)) for t,p in points])
    scale = rod.GetOrderedXformOps()[1]
    for t,p in points:
        scale.Set(Gf.Vec3f(.03,.03,1.43-(p[2]+.012)),t*fps)


def finish_hardware(stage,supports):
    """Ground the transport decks and connect every vacuum stem to its lifting frame."""
    from pxr import UsdShade
    def solid(path,pos,size,color='steel'):
        if stage.GetPrimAtPath(path):
            return
        cube = UsdGeom.Cube.Define(stage,path)
        cube.CreateSizeAttr(1)
        cube.AddTranslateOp().Set(Gf.Vec3d(*map(float,pos)))
        cube.AddScaleOp().Set(Gf.Vec3f(*map(float,size)))
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(UsdShade.Material(stage.GetPrimAtPath('/World/Looks/'+color)))
    placed = []
    for surface in supports:
        path = surface['path']
        if not path.startswith('/World/Transport/'):
            continue
        x,y,w,d,top = [surface[k] for k in ('x','y','sx','sy','top')]
        long_x = w>=d
        length = w if long_x else d
        width = d if long_x else w
        for j,distance in enumerate(np.linspace(-length/2+.07,length/2-.07,max(2,math.ceil(length/2)+1))):
            for side in (-1,1):
                xx = x+distance if long_x else x+side*max(.05,width/2-.05)
                yy = y+side*max(.05,width/2-.05) if long_x else y+distance
                if any(abs(xx-px) < .16 and abs(yy-py) < .16 for px,py in placed):
                    continue  # a corner junction already has a leg here
                placed.append((xx,yy))
                solid(path+f'_Leg{j}_{side+1}',(xx,yy,(top-.08)/2),(.05,.05,top-.08))
                solid(path+f'_Foot{j}_{side+1}',(xx,yy,.015),(.13,.13,.03))
    for crane in ('G1','G2'):
        root = '/World/Equipment/'+crane
        solid(root+'/Trolley/MastCollar',(0,0,-.145),(.26,.26,.12),'blue')
        solid(root+'/Spreader/CentralMount',(0,0,.23),(.22,.22,.10),'blue')
        for col in range(4):
            solid(root+f'/Spreader/VacuumManifold{col}',((col-1.5)*.245,0,.18),(.04,.67,.05),'blue')
            for row in range(10):
                stem = UsdGeom.Xformable(stage.GetPrimAtPath(root+f'/Spreader/Stem{col}_{row}'))
                stem.GetOrderedXformOps()[1].Set(Gf.Vec3f(.025,.025,.156))
