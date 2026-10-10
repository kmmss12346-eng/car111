#!/usr/bin/env python3
"""Live LD14P -> nominal field map -> stable cylinder candidates -> A* preview.
No automatic localization and no motor control in this file.
The vehicle pose and lidar mount transform must be measured.
Press R before a new survey to clear confirmed obstacles.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import time
from datetime import datetime
import numpy as np
from ld14p_scan import Stream, cartesian, list_ports
from obstacles import detect, Tracker
from field_map_stage2 import (plan, segment_free, expanded_rects, STEP, FIELD_MM,
                              YELLOW_RECTS, TEMP_ZONE, ROUGH_ZONE, START_RECTS, POINTS)
ROOT=Path(__file__).resolve().parent


def validate_config(c):
    if c.get('pose_and_mount_measured') is not True:
        raise ValueError('Measure vehicle pose and lidar mounting parameters, then set pose_and_mount_measured to true')
    fields=('car_x_mm','car_y_mm','car_yaw_deg','lidar_forward_mm','lidar_left_mm',
            'lidar_yaw_deg','cylinder_diameter_mm','margin_mm','sdk_y_sign')
    for k in fields:
        if isinstance(c.get(k),bool) or not isinstance(c.get(k),(int,float)) or not math.isfinite(c[k]):
            raise ValueError(k+' requires a measured/confirmed numeric value')
    for k in ('car_length_mm','car_width_mm'):
        v=c.get(k,290 if k=='car_length_mm' else 260)
        if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or not 0<v<=FIELD_MM:
            raise ValueError(k+' must be positive finite millimeters')
    if c['sdk_y_sign'] not in (-1,1): raise ValueError('sdk_y_sign must be +1 or -1')
    if c['cylinder_diameter_mm']<=0 or c['margin_mm']<0: raise ValueError('diameter must be positive and margin must be non-negative')
    if not 0<=c['car_x_mm']<=FIELD_MM or not 0<=c['car_y_mm']<=FIELD_MM:
        raise ValueError('vehicle center is outside the field')
    guard=c.get('boundary_guard_mm',0)
    if isinstance(guard,bool) or not isinstance(guard,(int,float)) or not math.isfinite(guard) or not 0<=guard<1200:
        raise ValueError('boundary_guard_mm must be in [0,1200)')
    pad=c.get('exclusion_pad_mm',0)
    if isinstance(pad,bool) or not isinstance(pad,(int,float)) or not math.isfinite(pad) or not 0<=pad<=100:
        raise ValueError('exclusion_pad_mm must be in [0,100]')
    for zone in c.get('cylinder_exclusion_zones',[]):
        r=zone.get('rect_mm',[])
        if len(r)!=4 or not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in r):
            raise ValueError('invalid exclusion rect_mm')
        if not 0<=r[0]<r[2]<=FIELD_MM or not 0<=r[1]<r[3]<=FIELD_MM:
            raise ValueError('exclusion rectangle outside field')
    for zone in c.get('cylinder_exclusion_capsules',[]):
        s=zone.get('segment_mm',[])
        radius=zone.get('radius_mm')
        if len(s)!=4 or not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in s):
            raise ValueError('invalid exclusion segment_mm')
        if isinstance(radius,bool) or not isinstance(radius,(int,float)) or not math.isfinite(radius) or radius<=0:
            raise ValueError('invalid exclusion radius_mm')
    return c


def rotate(xy, angle):
    a=math.radians(angle); co,si=math.cos(a),math.sin(a)
    return np.asarray(xy,dtype=float).reshape(-1,2) @ np.array([[co,si],[-si,co]])


def to_map(xy,c):
    points=np.asarray(xy,dtype=float).reshape(-1,2).copy()
    points[:,1]*=c['sdk_y_sign']
    body=rotate(points,c['lidar_yaw_deg'])+[c['lidar_forward_mm'],c['lidar_left_mm']]
    return rotate(body,c['car_yaw_deg'])+[c['car_x_mm'],c['car_y_mm']]


def body_corners(c):
    length=c.get('car_length_mm',290)/2
    width=c.get('car_width_mm',260)/2
    return [(-length,-width),(length,-width),(length,width),(-length,width)]


def body_half_bound(c):
    # Conservative square envelope for the legacy axis-aligned planner.
    return max(c.get('car_length_mm',290),c.get('car_width_mm',260))/2


def make_path(c,obstacles,goal):
    # Stage2 supports an axis-aligned square vehicle only.
    heading_error=abs((c['car_yaw_deg']+45)%90-45)
    if heading_error>1e-6:
        return [],'HEADING NOT AXIS-ALIGNED'
    margin=c['margin_mm']+body_half_bound(c)-150
    start=(c['car_x_mm'],c['car_y_mm'])
    rects=expanded_rects(obstacles,margin)
    if not segment_free(start,start,rects,margin):
        return [],'START BLOCKED / MARGIN'
    # Connect a non-grid pose to nearby valid grid nodes and validate the real segment.
    xs={math.floor(start[0]/STEP)*STEP,math.ceil(start[0]/STEP)*STEP}
    ys={math.floor(start[1]/STEP)*STEP,math.ceil(start[1]/STEP)*STEP}
    choices=sorted(((x,y) for x in xs for y in ys),key=lambda p:math.dist(start,p))
    for node in choices:
        if segment_free(start,node,rects,margin):
            path,status=plan(node,goal,obstacles,margin)
            if path: return ([start]+path if node!=start else path),'PARTIAL MAP: PREVIEW ONLY'
    return [],'NO PATH / GOAL BLOCKED'


def field_points(xy,c):
    """Reject near-body returns before reference correction; missing is unknown."""
    from reference_correction import apply
    world=to_map(xy,c)
    body=rotate(world-[c['car_x_mm'],c['car_y_mm']],-c['car_yaw_deg'])
    pad=float(c.get('self_echo_pad_mm',120))
    external=~((np.abs(body[:,0])<=c.get('car_length_mm',290)/2+pad)&
               (np.abs(body[:,1])<=c.get('car_width_mm',260)/2+pad))
    world=apply(world,c)
    keep=(external&(world[:,0]>=0)&(world[:,0]<=FIELD_MM)&
                   (world[:,1]>=0)&(world[:,1]<=FIELD_MM))
    return world[keep]


def inside_field_points(polar,c):
    """原始极坐标点里，换算到地图后落在场地内、且不是车身自身回波的那些点（保持原格式）。"""
    polar=[p for p in polar if p[1]>0]          # 先去掉无回波(距离0)的点，否则和 cartesian() 的结果对不上号
    if not polar:
        return []
    world=to_map(cartesian(polar),c)
    body=rotate(world-[c['car_x_mm'],c['car_y_mm']],-c['car_yaw_deg'])
    pad=float(c.get('self_echo_pad_mm',120))
    own=((np.abs(body[:,0])<=c.get('car_length_mm',290)/2+pad)&
         (np.abs(body[:,1])<=c.get('car_width_mm',260)/2+pad))
    edge=float(c.get('scan_field_margin_mm',0))   # 正数=比场地边线再往里收
    keep=(~own&(world[:,0]>=edge)&(world[:,0]<=FIELD_MM-edge)&
               (world[:,1]>=edge)&(world[:,1]<=FIELD_MM-edge))
    return [p for p,k in zip(polar,keep) if k]


def accumulated_detections(frames,c,link_mm=45.0,min_frames=3,min_points=3,extra_width_mm=40.0,report=None):
    """把一次扫描里所有帧(车不动)的场内回波叠在一起再找障碍物。
    黑色圆柱离得远时每帧只回1~2个点，单帧识别不出来，叠加12帧后就够了。
    远处放宽(2026-10-05)：离雷达 >= far_detect_mm(默认1800) 的一团，只要 >= far_min_frames(默认2) 帧就算，
    点数要求不变；只在调用者用默认的 3 帧门槛时放宽(第二站配对用的严格门槛不受影响)。"""
    far_mm=float(c.get('far_detect_mm',1800));far_f=int(c.get('far_min_frames',2))
    pts=[];fid=[]
    for k,f in enumerate(frames):
        inside=inside_field_points(f.get('points',[]),c)
        if inside:
            w=to_map(cartesian(inside),c);pts.append(w);fid+=[k]*len(w)
    if not pts:
        return [],np.empty((0,2))
    P=np.vstack(pts);F=np.array(fid)
    n=len(P);seen=np.zeros(n,bool);out=[];wide=np.zeros(n,bool)
    d2=((P[:,None,:]-P[None,:,:])**2).sum(-1)<=link_mm**2
    lidar=to_map([[0,0]],c)[0];r=float(c['cylinder_diameter_mm'])/2
    for i in range(n):
        if seen[i]:
            continue
        stack=[i];seen[i]=True;idx=[]
        while stack:
            j=stack.pop();idx.append(j)
            nb=np.where(d2[j]&~seen)[0];seen[nb]=True;stack.extend(nb.tolist())
        Q=P[idx];nf=len(set(F[idx].tolist()))
        m=Q.mean(axis=0);u=m-lidar;dist=float(np.linalg.norm(u))
        width=float(np.linalg.norm(np.ptp(Q,axis=0)))
        need_f=min(min_frames,far_f) if (dist>=far_mm and min_frames<=3) else min_frames   # 远处放宽帧数
        why=''
        if len(idx)<min_points or nf<need_f:why=f'点太少(要>={min_points}点、>={need_f}帧)'
        elif width>2*r+extra_width_mm:why=f'太宽{width:.0f}mm，不像圆柱'
        elif dist<1e-6:why='距离异常'
        cen=m+(u/dist*0.8*r if dist>1e-6 else 0)   # 回波在圆柱靠雷达的一面，中心往后推一点
        if not why and roi_class(cen[0],cen[1],c)!='interior':why='在不识别区域内'
        if why.startswith('太宽'):
            wide[idx]=True          # 墙、桌腿排这类大物体：不算障碍物，也不画出来
        if report is not None:
            report.append(dict(x=float(m[0]),y=float(m[1]),count=len(idx),frames=nf,dist=dist,width=width,why=why))
        if why:
            continue
        out.append(dict(x=float(cen[0]),y=float(cen[1]),count=len(idx),frames=nf))
    return out,P[~wide]


def _distance_to_segment(px,py,x0,y0,x1,y1):
    vx,vy=x1-x0,y1-y0
    wx,wy=px-x0,py-y0
    vv=vx*vx+vy*vy
    if vv<=1e-12:
        return math.hypot(px-x0,py-y0)
    t=max(0.0,min(1.0,(wx*vx+wy*vy)/vv))
    qx,qy=x0+t*vx,y0+t*vy
    return math.hypot(px-qx,py-qy)


def roi_class(x,y,c):
    guard=float(c.get('boundary_guard_mm',0))
    if not guard<=x<=FIELD_MM-guard or not guard<=y<=FIELD_MM-guard:
        windows=c.get('boundary_target_windows',[])
        if not any(math.hypot(x-w['x_mm'],y-w['y_mm'])<=w['radius_mm'] for w in windows):
            return 'boundary_uncertain'

    pad=float(c.get('exclusion_pad_mm',0))

    for z in c.get('cylinder_exclusion_zones',[]):
        x0,y0,x1,y1=map(float,z['rect_mm'])
        if x0-pad<=x<=x1+pad and y0-pad<=y<=y1+pad:
            return 'work_zone'

    for z in c.get('cylinder_exclusion_capsules',[]):
        x0,y0,x1,y1=map(float,z['segment_mm'])
        radius=float(z['radius_mm'])+pad
        if _distance_to_segment(x,y,x0,y0,x1,y1)<=radius:
            return 'work_zone'

    return 'interior'


class Mapping:
    """Stable latched obstacle map with non-flickering confirmed positions."""
    def __init__(self,c):
        self.c=c
        self.tracker=Tracker()
        self.latched=[]
        self.last_count=-1
        self.points=np.empty((0,2))
        self.next_id=1

    @property
    def planner_obstacles(self):
        return [[o['x'],o['y'],o['radius']] for o in self.latched]

    def clear(self):
        self.tracker.clear()
        self.latched=[]
        self.last_count=-1
        self.points=np.empty((0,2))
        self.next_id=1

    def ingest(self,frame,count):
        if count==self.last_count:
            return self.latched

        if self.last_count>=0 and (count<=self.last_count or count-self.last_count>2):
            self.tracker.clear()

        self.last_count=count
        self.points=field_points(cartesian(frame['points']),self.c)

        # 只把落在场地正方形以内（按雷达当前位置换算）的回波拿去找障碍物，
        # 场外的人、桌子、墙不会参与聚类，也就不会被当成障碍物。
        inside=inside_field_points(frame['points'],self.c)
        detections=self.tracker.update(
            detect(inside,self.c['cylinder_diameter_mm'])
        )

        nominal_radius=float(self.c['cylinder_diameter_mm'])/2.0

        for d in detections:
            if not d.get('stable',False):
                continue

            xy=field_points([(d['x'],d['y'])],self.c)
            if len(xy)==0:
                continue

            x,y=map(float,xy[0])

            if roi_class(x,y,self.c)!='interior':
                continue

            best=None
            best_dist=float('inf')
            for o in self.latched:
                dist=math.hypot(o['x']-x,o['y']-y)
                if dist<best_dist:
                    best_dist=dist
                    best=o

            match_limit=max(45.0,nominal_radius*2.0)

            if best is not None and best_dist<=match_limit:
                alpha=0.35
                best['x']=(1.0-alpha)*best['x']+alpha*x
                best['y']=(1.0-alpha)*best['y']+alpha*y
                best['seen']+=1
                best['count']=int(d.get('count',best['count']))
                best['kind']=d.get('kind',best['kind'])
            else:
                self.latched.append({
                    'id':self.next_id,
                    'x':x,
                    'y':y,
                    'radius':nominal_radius,
                    'seen':1,
                    'count':int(d.get('count',0)),
                    'kind':d.get('kind','candidate'),
                })
                self.next_id+=1

        return self.latched


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port')
    ap.add_argument('--list',action='store_true')
    ap.add_argument('--model',choices=('LD14P_2300HZ','LD14P_4000HZ'),default='LD14P_4000HZ')
    ap.add_argument('--config',default=str(ROOT/'map_config_start2_roi.json'))
    ap.add_argument('--range-mm',type=int,default=4000,help='local display only')
    args=ap.parse_args()
    if args.range_mm<=0: ap.error('range-mm must be positive')
    if args.list: list_ports();return 0
    if not args.port: ap.error('Use --list first, then provide the actual serial port with --port')
    if not Path(args.port).exists(): ap.error('serial port does not exist')
    if not (ROOT/'scan_bridge').is_file(): ap.error('run bash build.sh first')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle,Polygon,Circle
    fig,world=plt.subplots(figsize=(12,8))
    fig.subplots_adjust(bottom=.22,top=.88)
    fig.suptitle('LIVE MAP v2 - CURRENT OBSERVATIONS | STATIONARY CAR ONLY | NO MOTOR CONTROL',fontsize=12)
    world.set(xlim=(0,2400),ylim=(0,2400),xlabel='MAP X / mm',ylabel='MAP Y / mm')
    world.set_aspect('equal');world.set_facecolor('#d4d6d9')
    for r in YELLOW_RECTS+[TEMP_ZONE,ROUGH_ZONE]:
        world.add_patch(Rectangle((r[0],r[1]),r[2]-r[0],r[3]-r[1],facecolor='#fff0a6' if r in YELLOW_RECTS else 'white',edgecolor='gray'))
    for zone,r in START_RECTS.items():
        world.add_patch(Rectangle((r[0],r[1]),300,300,facecolor='#ddeafe',edgecolor='#6684aa'))
        world.text(r[0]+150,r[1]+150,f'S{zone}',ha='center')
    for name,p in POINTS.items():
        world.plot(*p,'.',color='navy');world.annotate(name,p,xytext=(3,4),textcoords='offset points',fontsize=7)
    mapped=world.scatter([],[],s=3,c='#4995b5',alpha=.6)
    route,=world.plot([],[],'--',c='#153be6',lw=2)
    car=Polygon(np.zeros((4,2)),fill=False,edgecolor='green',lw=2);world.add_patch(car);car.set_visible(False)
    info=fig.text(.5,.12,'',ha='center',fontsize=10)
    fig.text(.5,.035,'L: load measured config | M: invalidate pose after moving car | R: reset tracking\n3: QR | 4: raw | 5: temp | 6: rough | S: save | Q: quit\nGreen: fitted / Orange: uncertain / Gray: work zone or boundary / Blue: returns | Preview only.',ha='center',fontsize=9)
    state={'mapping':None,'path':[],'goal':POINTS['QR_SCAN'],'seq':None,'lastplan':0,'fresh':False,'markers':[],'roi_artists':[],'reason':'CONFIG REQUIRED'}

    def reset_artists():
        for a in state['markers']:a.remove()
        state['markers']=[]
        mapped.set_offsets(np.empty((0,2)));route.set_data([],[]);state['path']=[]

    def load():
        reset_artists();car.set_visible(False);state['mapping']=None
        for a in state['roi_artists']:a.remove()
        state['roi_artists']=[]
        try:
            c=validate_config(json.loads(Path(args.config).read_text()))
            state['mapping']=Mapping(c)
            guard=c.get('boundary_guard_mm',0)
            if guard:
                edge=Rectangle((guard,guard),FIELD_MM-2*guard,FIELD_MM-2*guard,
                               fill=False,edgecolor='#d97706',linestyle='--',lw=1)
                world.add_patch(edge);state['roi_artists'].append(edge)
            for z in c.get('cylinder_exclusion_zones',[]):
                x0,y0,x1,y1=z['rect_mm']
                patch=Rectangle((x0,y0),x1-x0,y1-y0,fc='gray',alpha=.16,hatch='//')
                world.add_patch(patch);state['roi_artists'].append(patch)
            for z in c.get('cylinder_exclusion_capsules',[]):
                x0,y0,x1,y1=map(float,z['segment_mm'])
                rr=float(z['radius_mm'])
                line,=world.plot([x0,x1],[y0,y1],color='gray',lw=max(1.0,rr*0.12),alpha=.35)
                state['roi_artists'].append(line)
                for cx,cy in ((x0,y0),(x1,y1)):
                    cap=Circle((cx,cy),rr,fc='gray',ec='gray',alpha=.12,hatch='//')
                    world.add_patch(cap);state['roi_artists'].append(cap)
            corners=rotate(body_corners(c),c['car_yaw_deg'])+[c['car_x_mm'],c['car_y_mm']]
            car.set_xy(corners);car.set_visible(True)
            state['reason']='STATIC MEASURED POSE - NOT LOCALIZATION'
            print('Loaded stationary calibration. Press M after moving the vehicle.',flush=True)
        except (OSError,ValueError,TypeError) as e:
            state['reason']='CONFIG REQUIRED';print('Map overlay unavailable:',e,flush=True)
        state['lastplan']=0

    stream=Stream(args.port,args.model)
    def update():
        frame,received,count=stream.snapshot()
        fresh=frame is not None and time.monotonic()-received<=1 and stream.proc.poll() is None
        state['fresh']=fresh

        if not fresh:
            world.set_title('NO FRESH SCAN - LAST CONFIRMED OBSTACLES HELD',color='red')
            info.set_text('No fresh lidar frame. Confirmed candidates are held until R resets them.')
            fig.canvas.draw_idle()
            return

        state['seq']=frame['seq']

        m=state['mapping']
        if m is None:
            world.set_title('CONFIG REQUIRED: NO MAP OVERLAY',color='red')
            info.set_text('Load a measured pose with L.')
        else:
            changed=count!=m.last_count
            m.ingest(frame,count)
            mapped.set_offsets(m.points)

            if changed:
                for a in state['markers']:
                    a.remove()
                state['markers']=[]

                for o in m.latched:
                    x,y,r=o['x'],o['y'],o['radius']
                    dot=Circle((x,y),r,color='black',zorder=6)
                    world.add_patch(dot)
                    state['markers'].append(dot)

                    half=body_half_bound(m.c)+m.c['margin_mm']+r
                    pad=Rectangle((x-half,y-half),2*half,2*half,
                                  fc='red',alpha=.10,ec='none')
                    world.add_patch(pad)
                    state['markers'].append(pad)

                    state['markers'].append(
                        world.annotate(
                            f"C{o['id']} ({x:.0f},{y:.0f}) n={o['count']} seen={o['seen']}",
                            (x,y),xytext=(4,5),textcoords='offset points',
                            fontsize=7,color='black'
                        )
                    )

            if changed or time.monotonic()-state['lastplan']>=1:
                state['path'],state['reason']=make_path(
                    m.c,m.planner_obstacles,state['goal']
                )
                route.set_data(*zip(*state['path'])) if state['path'] else route.set_data([],[])
                state['lastplan']=time.monotonic()

            world.set_title(state['reason'],fontsize=10)
            info.set_text(
                f"CONFIRMED obstacles={len(m.latched)} | field returns={len(m.points)} | "
                f"frame={frame['seq']} | age={time.monotonic()-received:.2f}s\n"
                "Stable candidates are latched across missed frames. Press R before a new survey.\n"
                "Gray/hatched regions and the boundary guard are excluded from cylinder identity."
            )

        fig.canvas.draw_idle()

    def key(event):
        k=(event.key or '').lower()
        if k in ('q','escape'):plt.close(fig)
        elif k=='l':load();update()
        elif k=='m':
            state['mapping']=None;reset_artists();car.set_visible(False);update()
        elif k=='r' and state['mapping']:
            state['mapping']=Mapping(state['mapping'].c);reset_artists();state['lastplan']=0;update()
        elif k in ('3','4','5','6'):
            state['goal']=POINTS[{'3':'QR_SCAN','4':'RAW_STOP','5':'TEMP_STOP','6':'ROUGH_STOP'}[k]];state['lastplan']=0;update()
        elif k=='s':
            update()
            if not state['fresh'] or state['mapping'] is None:
                print('Map not saved: fresh scan or valid calibration is missing.');return
            m=state['mapping'];out=ROOT/'map_captures';out.mkdir(exist_ok=True)
            name='map_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            payload={'drive_ready':False,'pose_source':'manual_stationary','map_completeness':'partial_unknown_unobserved',
                     'config':m.c,'candidate_obstacles_mm':m.planner_obstacles,'confirmed_detections':m.latched,'preview_path_mm':state['path'],'goal_mm':state['goal'],'scan_seq':state['seq']}
            (out/(name+'.json')).write_text(json.dumps(payload,indent=2))
            fig.savefig(out/(name+'.png'),dpi=140);print('Saved map preview:',out/(name+'.json'),flush=True)
    if hasattr(fig.canvas.manager,'key_press_handler_id'):fig.canvas.mpl_disconnect(fig.canvas.manager.key_press_handler_id)
    fig.canvas.mpl_connect('key_press_event',key)
    timer=fig.canvas.new_timer(interval=200);timer.add_callback(update)
    fig.canvas.mpl_connect('close_event',lambda e:timer.stop())
    try:
        load();update();timer.start();plt.show()
    except KeyboardInterrupt:pass
    finally:timer.stop();stream.close()
    return 0

if __name__=='__main__':raise SystemExit(main())
