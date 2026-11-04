"""FR3 reject removal: Cartesian IK pick-and-place of bad bricks; recorded kinematics, not contact dynamics."""
from pathlib import Path
import numpy as np
import mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from pxr import Gf, UsdGeom

MODEL = Path(__file__).resolve().parent / 'assets' / 'franka_fr3' / 'fr3.xml'      # MuJoCo Menagerie FR3, kept in this folder
TOOL_LENGTH = .24
BELT_Y = 5.8
BASE_DY = .60          # base axis to layer-belt centre line (reach study: rows 4-9 reachable from one side)
BASE_Z = .9
PLACE_DY = .40         # base axis to scrap-belt centre line, away from the layer belt
STATIONS = (8.2, 10.4)
# Two stations, one FR3 each side. A one-sided FR3 cannot reach the far rows, so north robots take
# rows 5-9 and south robots rows 0-4. South robots are yawed 180 degrees and reuse the same local plans.
ROBOTS = [{'name':f'R{2*s+k+1}','station':s,'side':side,'base':(x,BELT_Y+side*BASE_DY,BASE_Z),'yaw':0 if side>0 else 180}
          for s,x in enumerate(STATIONS) for k,side in enumerate((1,-1))]
CYCLE = 8.0
ATTACH, RELEASE = 1.4, 5.0
DOWN = np.diag([1.,-1.,-1.])
TOP = .869-BASE_Z      # brick top relative to the robot base, on either belt
# libfranka FR3 joint velocity limits, rad/s.
VELOCITY_LIMIT = np.array([2.62,2.62,2.62,2.62,5.26,4.18,5.26])


def quaternion(matrix):
    x,y,z,w = Rotation.from_matrix(matrix).as_quat()
    return Gf.Quatd(float(w),Gf.Vec3d(float(x),float(y),float(z)))


def yaw_matrix(degrees):
    return Rotation.from_euler('z',degrees,degrees=True).as_matrix()


def local_slot(robot,col,row):
    """Brick top-face centre in the robot's base frame for layer slot (col,row) at the robot's station."""
    world = np.array([(col-1.5)*.245,BELT_Y+(row-4.5)*.067-robot['base'][1],0.])
    local = yaw_matrix(-robot['yaw'])@world
    return np.array([round(local[0],6),round(local[1],6),TOP])


class Manipulators:
    def __init__(self, stage, fps):
        self.stage,self.fps = stage,fps
        self.model = mujoco.MjModel.from_xml_path(str(MODEL))
        self.data = mujoco.MjData(self.model)
        self.site = mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_SITE,'attachment_site')
        self.lo,self.hi = self.model.jnt_range[:,0]+.001,self.model.jnt_range[:,1]-.001
        self.tasks,self.links,self.tools,self.plans = [],[],[],{}
        self.max_error,self.max_self_penetration,self.max_velocity_ratio = 0.,0.,0.
        self.home_q = np.array([-np.pi/2,-.3,0,-2.2,0,1.9,.785])
        for _ in range(8):  # fixed point: re-solving home from home must not move the arm
            self.home_q = self.solve(np.array([0,-.35,.45]),self.home_q)
        self.forward(self.home_q)
        self.home = self.record(0.,self.home_q)
        for robot in ROBOTS:
            root = stage.GetPrimAtPath(f"/World/Equipment/{robot['name']}/FR3Visual")
            root.SetCustomDataByKey('status','Cartesian IK reject removal; kinematic vacuum attachment')
            xf = UsdGeom.Xformable(root)
            xf.GetOrderedXformOps()[0].Set(Gf.Vec3d(*robot['base']))
            xf.GetOrderedXformOps()[1].Set(robot['yaw'])
            ops = []
            for g in range(self.model.ngeom):
                if self.model.geom_group[g] != 2:
                    continue
                mesh = UsdGeom.Mesh(stage.GetPrimAtPath(str(root.GetPath())+f'/Mesh{g}'))
                mid = self.model.geom_dataid[g]
                va,vn = self.model.mesh_vertadr[mid],self.model.mesh_vertnum[mid]
                mesh.GetPointsAttr().Set(self.model.mesh_vert[va:va+vn].tolist())
                ops.append((g,mesh.AddTranslateOp(),mesh.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)))
            self.links.append(ops)
            tools = []
            for name,radius,height,color in [('VacuumTool',.022,TOOL_LENGTH,(.08,.12,.16)),('SuctionCup',.027,.012,(.1,.7,.8))]:
                cyl = UsdGeom.Cylinder.Define(stage,str(root.GetPath())+'/'+name)
                cyl.CreateRadiusAttr(radius)
                cyl.CreateHeightAttr(height)
                cyl.CreateDisplayColorAttr([Gf.Vec3f(*color)])
                tools.append((cyl.AddTranslateOp(),cyl.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)))
            self.tools.append(tools)
            self.sample(len(self.tools)-1,0,self.home)

    def forward(self,q):
        self.data.qpos[:] = q
        mujoco.mj_forward(self.model,self.data)
        return self.data.site_xpos[self.site].copy(),self.data.site_xmat[self.site].reshape(3,3).copy()

    def penetration(self):
        return max([max(0.,-c.dist) for c in self.data.contact]+[0.])

    def solve(self,target,seed,yaw=0.):
        goal = yaw_matrix(yaw)@DOWN
        def error(q,weight=.001):
            p,r = self.forward(q)
            # Continuity with the previous frame, and joints 3 and 5 kept near zero (away from the
            # shoulder singularity where joints 1 and 3 align and the solution slides between them).
            return np.r_[6*(p+r[:,2]*TOOL_LENGTH-target),Rotation.from_matrix(goal@r.T).as_rotvec(),
                         weight*(q-seed),3*weight*q[[2,4]]]
        angle = np.arctan2(target[1],target[0])
        seeds = [seed,np.array([angle,.6,0,-1.8,0,2.2,.785]),np.array([angle,.2,0,-2.4,0,2.6,.785])]
        rng = np.random.default_rng(31)
        seeds += [rng.uniform(self.lo,self.hi) for _ in range(25)]
        for initial in seeds:
            # A strongly regularised first pass stays on the previous null-space branch; the second is exact.
            initial = least_squares(error,np.clip(initial,self.lo,self.hi),bounds=(self.lo,self.hi),
                                    max_nfev=60,kwargs={'weight':.1}).x
            fit = least_squares(error,initial,bounds=(self.lo,self.hi),
                                max_nfev=160,ftol=1e-8,xtol=1e-8,gtol=1e-8)
            residual = error(fit.x)
            penetration = self.penetration()
            if np.linalg.norm(residual[:3])/6 < 2e-5 and np.linalg.norm(residual[3:6]) < 2e-4 and penetration < .0005:
                self.max_error = max(self.max_error,float(np.linalg.norm(residual[:3])/6))
                self.max_self_penetration = max(self.max_self_penetration,float(penetration))
                return fit.x
        raise RuntimeError(f'No accurate, self-collision-free IK solution for {target}; last residual {residual}, penetration {penetration}')

    def record(self,t,q):
        p,r = self.forward(q)
        geom = [(g,self.data.geom_xpos[g].copy(),self.data.geom_xmat[g].reshape(3,3).copy()) for g in range(self.model.ngeom) if self.model.geom_group[g]==2]
        return {'seconds':t,'q':q.copy(),'flange':p,'rotation':r,'geoms':geom}

    def plan(self,pick):
        """One reject cycle in the base frame: home, pick, swing round the pedestal, scrap belt, home."""
        key = tuple(pick)
        if key in self.plans:
            return self.plans[key]
        # Keys are cylindrical about the base axis (angle, radius, height) so the swing keeps its radius
        # instead of cutting past the base axis. The swing goes round the +x side (joint 1 cannot pass
        # -180 degrees); the tool yaw turns 180 degrees with it so the wrist never unwinds, and a brick
        # turned end for end lands with the same footprint.
        angle,radius = np.degrees(np.arctan2(pick[1],pick[0])),np.hypot(pick[0],pick[1])
        mid = (max(.45,.5*(radius+PLACE_DY)),.30)
        # Separate channels: angle and yaw make one smooth 2.6 s swing; radius and height pass a mid point.
        spin = [(0,-90,0),(.8,angle,0),(1.8,angle,0),(4.4,90,180),(5.4,90,180),(7.0,0,90),(8,-90,0)]
        lift = [(0,.35,.45),(.8,radius,TOP+.12),(1.2,radius,TOP),(ATTACH,radius,TOP),(1.8,radius,TOP+.12),
                (3.1,*mid),(4.4,PLACE_DY,TOP+.12),(4.8,PLACE_DY,TOP),(RELEASE,PLACE_DY,TOP),
                (5.4,PLACE_DY,TOP+.15),(7.0,.45,.35),(8,.35,.45)]
        def channel(keys,t):
            for (a,*ka),(b,*kb) in zip(keys,keys[1:]):
                if a <= t <= b+1e-9:
                    u = np.clip((t-a)/(b-a),0,1)
                    u = u*u*(3-2*u)
                    return np.array(ka)*(1-u)+np.array(kb)*u
        result,seed,previous,zone = [],self.home_q,None,[]
        keys = spin
        for frame in range(round(CYCLE*self.fps)+1):
            t = frame/self.fps
            (theta,yaw),(r,z) = channel(spin,t),channel(lift,t)
            target = np.array([r*np.cos(np.radians(theta)),r*np.sin(np.radians(theta)),z])
            # Inside the layer envelope (over the belt, low): north and south robots never share it.
            if abs(target[1]+BASE_DY) < .385 and z < TOP+.25:
                zone.append(t)
            if t <= keys[-2][0]+1e-9:
                q = self.home_q if frame == 0 else self.solve(target,seed,yaw)
                return_start = q.copy()
            else:
                # Joint-space return closes the cycle on the exact home configuration.
                u = np.clip((t-keys[-2][0])/(CYCLE-keys[-2][0]),0,1)
                q = return_start*(1-u*u*(3-2*u))+self.home_q*u*u*(3-2*u)
                self.forward(q)
                if self.penetration() > .0005:
                    raise RuntimeError('Home-return interpolation self-collides')
            if previous is not None:
                if np.max(np.abs(q-previous)) > .25:
                    raise RuntimeError(f'IK branch jump at {t}: {q-previous}')
                self.max_velocity_ratio = max(self.max_velocity_ratio,float(np.max(np.abs(q-previous)*self.fps/VELOCITY_LIMIT)))
            result.append(self.record(t,q))
            seed,previous = q,q
        if self.max_velocity_ratio > 1:
            raise RuntimeError(f'Reject cycle for {pick} exceeds FR3 joint velocity limits ({self.max_velocity_ratio:.2f}x)')
        result[0]['zone'] = (min(zone)-.2,max(zone)+.2)
        self.plans[key] = result
        return result

    def sample(self,robot,seconds,record):
        tc = round(seconds*self.fps)
        for (_,trans,orient),(g,pos,rot) in zip(self.links[robot],record['geoms']):
            trans.Set(Gf.Vec3d(*map(float,pos)),tc)
            orient.Set(quaternion(rot),tc)
        for ops,offset in zip(self.tools[robot],(TOOL_LENGTH/2,TOOL_LENGTH-.006)):
            ops[0].Set(Gf.Vec3d(*map(float,record['flange']+record['rotation'][:,2]*offset)),tc)
            ops[1].Set(quaternion(record['rotation']),tc)

    def remove(self,brick,index,robot_index,start,col,row):
        """Record one reject pick. Returns the carried brick centre path, attach and release times."""
        robot = ROBOTS[robot_index]
        base,yaw = np.array(robot['base']),yaw_matrix(robot['yaw'])
        samples = self.plan(local_slot(robot,col,row))
        xf = UsdGeom.Xformable(brick)
        translate,scale = xf.GetOrderedXformOps()
        orient = xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
        xf.SetXformOpOrder([translate,orient,scale])
        orient.Set(Gf.Quatd(1))
        orient.Set(Gf.Quatd(1),0)
        attach = samples[round(ATTACH*self.fps)]
        pick = local_slot(robot,col,row)-[0,0,.0345]
        relative = attach['rotation'].T@(pick-attach['flange'])
        relative_rot = attach['rotation'].T@yaw.T
        path = []
        for record in samples:
            t = record['seconds']
            self.sample(robot_index,start+t,record)
            if ATTACH-1e-9 <= t <= RELEASE+1e-9:
                center = base+yaw@(record['flange']+record['rotation']@relative)
                path.append((start+t,center.tolist()))
                orient.Set(quaternion(yaw@record['rotation']@relative_rot),round((start+t)*self.fps))
        self.tasks.append({'brick':index,'robot':robot['name'],'station':robot['station']+1,'slot':[col,row],
                           'plan':list(local_slot(robot,col,row)),
                           'start_seconds':start,'attach_seconds':start+ATTACH,'release_seconds':start+RELEASE,
                           'zone_seconds':[start+samples[0]['zone'][0],start+samples[0]['zone'][1]],'pick_center_m':path[0][1],'place_center_m':path[-1][1]})
        brick.GetPrim().SetCustomDataByKey('robot',robot['name'])
        return path

    def finish(self,duration):
        for robot in range(len(ROBOTS)):
            self.sample(robot,duration,self.home)

    def report(self):
        return {'mode':'FR3 reject removal from both sides of the layer belt',
                'robots':ROBOTS,'contact_dynamics':False,'self_collision_check':'MuJoCo collision hulls at 30 Hz',
                'max_self_penetration_m':self.max_self_penetration,'max_ik_position_error_m':self.max_error,
                'max_joint_velocity_fraction_of_limit':self.max_velocity_ratio,'cycle_seconds':CYCLE,
                'home_q_rad':self.home_q.tolist(),'tasks':self.tasks,
                'plans':[{'pick_local_m':list(key),'q_rad':[s['q'].tolist() for s in plan]} for key,plan in self.plans.items()]}
