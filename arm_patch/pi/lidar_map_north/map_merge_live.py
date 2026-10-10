#!/usr/bin/env python3
"""两次扫描合并 + 全程A*路线 + 一键行驶（v10）。
雷达识别障碍物(两次扫描合并)；1010：每个停车点再用雷达重定位(和出发时的扫描对齐，reloc.py)，修正车位后再做任务。
比赛：race = 屏上选启停区 -> 车放好自动扫描、预先规划 -> 屏上按 START 一键跑完(开跑后不碰电脑)。"""
import os
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[key]='1'
import argparse,copy,json,math,queue,re,threading,time
from datetime import datetime
from pathlib import Path
import numpy as np
from lidar_map_live import (Mapping,validate_config,roi_class,rotate,make_path,body_corners,body_half_bound,FIELD_MM,to_map,accumulated_detections,
    YELLOW_RECTS,TEMP_ZONE,ROUGH_ZONE,START_RECTS,POINTS)
from ld14p_scan import Stream
from reference_correction import applicable, description, metadata
from merge_stations import History,moved_config,second_scan_config
from pose_fix import calibrate_mount,refine_second
from route_plan import plan_mission,route_stats,Motion
from stm32_link import Stm32Link,FakeLink
from auto_run import (run_mission,Abort,apply_zone,zone_snapshot,zone_restore,mission_names,race_flow,run_bg,park_arm,quit_park,
    home_route_legs,home_approach,Nav,firmware_caps,zone_request,DEFAULT_SECOND_MOVES)
import reloc
ROOT=Path(__file__).resolve().parent

SIM=None   # 桌面模拟时由 --sim 设置

class GyroReader:
    """后台读 STM32 从串口发来的 'YAW100 数值'（角度×100，逆时针为正）。读不到就当没有。"""
    def __init__(self,port,baud=115200):
        self.value=None;self.t=0.0;self.ok=False;self.err=''
        try:
            import serial
            self.s=serial.Serial(port,baud,timeout=0.2);self.ok=True
            threading.Thread(target=self._run,daemon=True).start()
        except Exception as e:
            self.err=str(e)
    def _run(self):
        buf=b''
        while True:
            try:
                buf+=self.s.read(64)
            except Exception:
                time.sleep(.2);continue
            while b'\n' in buf:
                line,buf=buf.split(b'\n',1)
                parts=line.decode('ascii','ignore').split()
                if len(parts)==2 and parts[0]=='YAW100':
                    try:self.value=int(parts[1]);self.t=time.monotonic()
                    except ValueError:pass
    def fresh(self,max_age=1.0):
        return self.value is not None and time.monotonic()-self.t<=max_age

def add_accumulated(m):
    """单帧没识别出来、但多帧叠加后成团的回波，也算障碍物。显示也改成叠加后的全部场内回波。"""
    found,allpts=accumulated_detections(m.raw_frames,m.c)
    if len(allpts):m.points=allpts
    r=float(m.c['cylinder_diameter_mm'])/2
    for d in found:
        if any(math.hypot(o['x']-d['x'],o['y']-d['y'])<=60 for o in m.latched):
            continue
        m.latched.append({'id':getattr(m,'next_id',len(m.latched)+1),'x':d['x'],'y':d['y'],'radius':r,
                          'seen':d['frames'],'count':d['count'],'kind':'accumulated'})
        m.next_id=getattr(m,'next_id',0)+1

def remap(m,c):
    """1010：同一次扫描的原始帧按另一个车位(雷达测到的实际车位)重新识别障碍物，障碍物登记在对的位置上。"""
    m2=Mapping(c);m2.raw_frames=m.raw_frames
    for n,f in enumerate(m.raw_frames,1):m2.ingest(f,n)
    add_accumulated(m2)
    return m2

_STREAM={'s':None,'key':None,'lock':threading.Lock()}

def get_stream(port,model):
    """v12：雷达只打开一次，之后一直在后台转、一直收数据；每次扫描直接取最新的帧，不用再等雷达启动。"""
    st=_STREAM['s']
    if st is not None and (st.proc.poll() is not None or _STREAM['key']!=(port,model)):
        try:st.close()
        except Exception:pass
        st=None
    fresh=st is None
    if fresh:
        st=Stream(port,model);_STREAM['s']=st;_STREAM['key']=(port,model)
    return st,fresh

def close_stream():
    st=_STREAM['s'];_STREAM['s']=None
    if st is not None:
        try:st.close()
        except Exception:pass

def acquire(config,port,model,stop,frames=12,true_pose=None,settle_s=0.3,quiet=False):
    m=Mapping(config);m.raw_frames=[]
    t_start=time.monotonic()
    with _STREAM['lock']:
        if SIM is not None:
            stream=SIM;SIM.set_true_pose(true_pose or (config['car_x_mm'],config['car_y_mm'],config['car_yaw_deg']));fresh=False
        else:
            stream,fresh=get_stream(port,model)
        # 刚打开的雷达等1秒让它转稳；已经在转的只等 settle_s 秒(车刚停下，让车身晃动消失)
        after=time.monotonic()+(1.0 if fresh else settle_s);deadline=time.monotonic()+15
        last=-1;n=0;seq=None
        while not stop.is_set():
            now=time.monotonic()
            if now>deadline:
                if SIM is None:close_stream()     # 雷达可能卡住了：关掉，下次重新打开
                raise ValueError('扫描超时，本次不合并；之前地图保留')
            if stream.proc.poll() is not None:
                if SIM is None:close_stream()
                raise ValueError('雷达驱动已退出（下次扫描会自动重新打开）')
            f,received,count=stream.snapshot()
            if f is not None and received>after and 0<=now-received<=1 and count>last:
                m.raw_frames.append(copy.deepcopy(f));m.ingest(f,count);last=count;seq=f['seq'];n+=1
                if not quiet:print(f'收集 {n}/{frames} 帧',flush=True)
                if n>=frames:
                    add_accumulated(m)
                    if not quiet:print(f'  扫描用时 {time.monotonic()-t_start:.1f} 秒（{"雷达刚打开" if fresh else "雷达已在转"}，{frames} 帧）',flush=True)
                    return m,seq
            time.sleep(.02)
        raise ValueError('本次扫描已取消')

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port',default='/dev/ttyACM0')
    ap.add_argument('--model',default='LD14P_4000HZ',choices=['LD14P_4000HZ','LD14P_2300HZ'])
    ap.add_argument('--config',default=str(ROOT/'map_config_start2_roi.json'))
    ap.add_argument('--sim',action='store_true',help='桌面模拟：不接雷达，用配置里的 sim_obstacles')
    ap.add_argument('--gyro-port',default='/dev/serial0',help='（旧）只读 YAW100 的串口；有 --stm-port 时不用\n')
    ap.add_argument('--stop-wait',type=float,default=3.0,help='每个停车点停多久(秒)，代替抓取/放置等任务')
    ap.add_argument('--stm-port',default='/dev/serial0',help='和 STM32 通信的串口（指令+YAW100）；none=不连 STM32')
    ap.add_argument('--zone',type=int,choices=(1,2),default=None,help='启停区(不写就用配置里的 start_zone；比赛时在屏上选)')
    args=ap.parse_args()
    raw_cfg=json.loads(Path(args.config).read_text(encoding='utf-8'))
    raw_cfg.setdefault('second_relative_moves',dict(DEFAULT_SECOND_MOVES))
    zone_snap=zone_snapshot(raw_cfg)              # 配置文件里和启停区有关的原值：写回文件时恢复它，文件内容不跟着切换走
    zs=apply_zone(raw_cfg,int(args.zone or raw_cfg.get('start_zone',2) or 2),zone_snap)
    config=validate_config(raw_cfg)
    global SIM
    if args.sim:
        from sim_lidar import SimStream
        SIM=SimStream(config,[tuple(o) for o in raw_cfg.get('sim_obstacles',[])],raw_cfg.get('raw_center_mm',[1200,2480]),
                      raw_cfg.get('raw_radius_mm',150),config.get('car_length_mm',290),config.get('car_width_mm',260))
    else:
        if not (ROOT/'scan_bridge').is_file():ap.error('请先运行 bash build.sh')
        if not Path(args.port).exists():ap.error('雷达串口不存在')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle,Polygon,Circle
    fig,world=plt.subplots(figsize=(12,8));fig.subplots_adjust(bottom=.24,top=.88)
    fig.suptitle('1010 | RACE: TOUCH ZONE + START ON SCREEN | LIDAR RELOC AT EVERY STOP | PREHOME',fontsize=11)
    world.set(xlim=(0,2400),ylim=(0,2400),xlabel='MAP X / mm',ylabel='MAP Y / mm')
    world.set_aspect('equal');world.set_facecolor('#d4d6d9')
    for r in YELLOW_RECTS+[TEMP_ZONE,ROUGH_ZONE]:
        world.add_patch(Rectangle((r[0],r[1]),r[2]-r[0],r[3]-r[1],facecolor='#fff0a6' if r in YELLOW_RECTS else 'white',edgecolor='gray'))
    for zone,r in START_RECTS.items():
        world.add_patch(Rectangle((r[0],r[1]),300,300,facecolor='#ddeafe',edgecolor='#6684aa'))
        world.text(r[0]+150,r[1]+150,f'S{zone}',ha='center')
    for name,p in POINTS.items():
        world.plot(*p,'.',color='navy');world.annotate(name,p,xytext=(3,4),textcoords='offset points',fontsize=7)
    guard=config.get('boundary_guard_mm',0)
    if guard:world.add_patch(Rectangle((guard,guard),FIELD_MM-2*guard,FIELD_MM-2*guard,fill=False,edgecolor='#d97706',linestyle='--',lw=1))
    for z in config.get('cylinder_exclusion_zones',[]):
        x0,y0,x1,y1=z['rect_mm'];world.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,fc='gray',alpha=.16,hatch='//'))
    for z in config.get('cylinder_exclusion_capsules',[]):
        x0,y0,x1,y1=map(float,z['segment_mm']);rr=float(z['radius_mm'])
        world.plot([x0,x1],[y0,y1],color='gray',lw=max(1.,rr*.12),alpha=.35)
        for cx,cy in ((x0,y0),(x1,y1)):world.add_patch(Circle((cx,cy),rr,fc='gray',ec='gray',alpha=.12,hatch='//'))
    mapped=world.scatter([],[],s=3,c='#4995b5',alpha=.6)
    route,=world.plot([],[],'--',c='#153be6',lw=2)
    car=Polygon(np.zeros((4,2)),fill=False,edgecolor='green',lw=2);world.add_patch(car)
    heading,=world.plot([],[],color='green',lw=2)
    lidar_mark,=world.plot([],[],'o',color='#f97316',ms=10,mec='black',zorder=9)
    lidar_txt=world.text(0,0,'LIDAR',color='#c2410c',fontsize=8,fontweight='bold',zorder=9)
    front_txt=world.text(0,0,'FRONT',color='green',fontsize=8,fontweight='bold',zorder=9)
    info=fig.text(.5,.13,'',ha='center',fontsize=10)
    status=fig.text(.5,.095,'',ha='center',fontsize=9)
    fig.text(.5,.03,'Terminal: race | go | zone 1/2 | park | abort | calib X Y | scan | second | route | pose X Y YAW | home [check] | list | pts | reset | save | q\nBlue line = full A* mission route preview. Unobserved space is UNKNOWN.',ha='center',fontsize=9)
    history=History();markers=[];commands=queue.Queue();results=queue.Queue();stop=threading.Event()
    link=None
    if args.sim:link=FakeLink()
    elif args.stm_port!='none':link=Stm32Link(args.stm_port)
    if link is not None and not link.ok:
        print('STM32 串口打不开（',link.err,'），go/drive 不能用，scan/second 照常。',flush=True);link=None
    gyro=link if link is not None else (GyroReader(args.gyro_port) if (args.gyro_port!='none' and not args.sim) else None)
    if gyro is not None and not gyro.ok:print('陀螺仪串口打不开（',gyro.err,'），第二站角度按设定60°计算',flush=True)
    state=dict(config=config,busy=False,worker=None,path=[],reason='NOT SCANNED',goal=POINTS['QR_SCAN'],yaw_scan1=None,
               legs=[],plan_start=None,calib=None,mode='scan',done_count=0,err_count=0,confirm=None,abort=False,auto=False,
               aborted=False,hooks=None,gui_done=0,preplan=None,ref=None,ref_key=None,ref1=None,home_routes={},home_gen=0,
               route_ref=None,stm_target_yaw=None,legs_partial=False,race=False)
    plan_lock=threading.Lock()
    rel=raw_cfg['second_relative_moves']          # 切换启停区时原地改，这里一直是同一个 dict
    sim_err=raw_cfg.get('sim_second_pose_error',[0,0,0])
    log=lambda m:print(m,flush=True)
    def file_cfg():
        """写回配置文件的内容：和启停区有关的键保持文件里原来的值(切换启停区只在内存里)。"""
        out=copy.deepcopy(raw_cfg);zone_restore(out,zone_snap);return out
    def mission_cfg():
        mc=dict(raw_cfg);mc.update(car_length_mm=config.get('car_length_mm',290),car_width_mm=config.get('car_width_mm',260))
        return mc
    def select_zone(z):
        """切换启停区(只在还没扫描/重新开始时)：起点车位、START 停车点、PREHOME、第二站动作一次改完，并清空历史。"""
        s=apply_zone(raw_cfg,z,zone_snap)
        state['preplan_gen']=state.get('preplan_gen',0)+1
        history.objects.clear();history.views.clear()
        state.update(plan_start=None,replan_from=None,preplan=None,ref=None,ref_key=None,ref1=None,legs=[],path=[],reason='NOT SCANNED',
                     plan_key=None,plan_obj_key=None,route_ref=None,home_routes={},legs_partial=False)
        state['config']=validate_config(dict(state['config'],car_x_mm=s['car'][0],car_y_mm=s['car'][1],car_yaw_deg=s['car'][2]))
        redraw()
        print(f"启停区 {s['zone']}：起点 ({s['car'][0]:.0f},{s['car'][1]:.0f}) 车头 {s['car'][2]:.0f}°；回家停 {s['start_name']} "
              f"({s['start_stop'][0]:.0f},{s['start_stop'][1]:.0f})"+(f"，回家前先停 PREHOME ({s['prehome'][0]:.0f},{s['prehome'][1]:.0f}) 定位" if s['prehome'] else '')
              +f"；第二站 左移{rel.get('left_mm')} 前进{rel.get('forward_mm')} 顺时针{rel.get('cw_deg')}°",flush=True)
        return s
    def obj_key():
        return (len(history.views),tuple((round(o['x']),round(o['y'])) for o in history.objects))
    def is_weak(o):
        return o.get('seen',99)<int(raw_cfg.get('weak_seen',3)) or o.get('count',99)<int(raw_cfg.get('weak_count',6))
    def _plan_route(cur=None):
        if not history.views:
            state['legs'],state['path'],state['reason']=[],[],'NOT SCANNED';return
        c=state['config']
        if cur is None:cur=(c['car_x_mm'],c['car_y_mm'],c['car_yaw_deg'])
        d=float(raw_cfg.get('turn_pivot_back_mm',0))
        state['start_pivot']=None
        rf=state.get('replan_from')
        if rf is not None:                     # v14：站点扫描发现新障碍物：从这个停车点规划剩下的路线
            state['plan_start']=(float(rf[0][0]),float(rf[0][1]),float(rf[0][2]));state['turn_back']=0
        if state['plan_start'] is None:
            # 车现在的车头可能是斜的(第二站)：先绕转轴原地转回起点车头方向，转轴不动
            yaw_now=float(cur[2]);yaw0=float(history.views[0]['config']['car_yaw_deg'])
            lft=0.0
            a=math.radians(yaw_now);ux,uy=math.cos(a),math.sin(a)
            px=cur[0]-d*ux-lft*uy;py=cur[1]-d*uy+lft*ux
            b=math.radians(yaw0);ux,uy=math.cos(b),math.sin(b)
            state['plan_start']=(px+d*ux+lft*uy,py+d*uy-lft*ux,yaw0)
            state['start_pivot']=(px,py)
            state['turn_back']=((yaw0-yaw_now+180)%360)-180
            state['route_ref']=(float(cur[0]),float(cur[1]),float(cur[2]))
        # v14：看得不清楚的圆柱(点少/帧少)位置可能偏好几厘米，规划时把它当成更大的圆柱，路线离它更远
        weak_extra=float(raw_cfg.get('weak_obstacle_extra_mm',80))
        weak=[o for o in history.objects if is_weak(o)]
        if weak:print('  看得不清楚的圆柱(规划时多留%.0fmm)：'%weak_extra+'，'.join(f"C{o['id']}({o['x']:.0f},{o['y']:.0f}) {o.get('count','?')}点/{o.get('seen','?')}帧" for o in weak),flush=True)
        t_plan=time.monotonic()
        mission=mission_names(raw_cfg)                 # 1010：START 换成本区的，回家前加 PREHOME
        if rf is not None:mission=list(rf[1])
        state['sealed']=[];state['lane_fallback']=False
        def attempt(mission):
            # v14：先按大余量规划(离障碍物远，绕过去)；规划不出来再逐步减小余量
            base=float(raw_cfg.get('margin_mm',60))
            tries=[(base,weak_extra)]+[(m,e) for m,e in ((40,40),(20,0)) if m<base]
            for k,(mg,ex) in enumerate(tries):
                obs=[(o['x'],o['y'],o['radius']+(ex if is_weak(o) else 0)) for o in history.objects]
                legs,why,pl_=plan_mission(dict(mission_cfg(),margin_mm=mg),obs,state['plan_start'],mission,start_pivot=state.get('start_pivot'))
                if why=='OK':
                    if k:print(f'  ⚠ 按障碍物余量{base:.0f}mm规划不出来，降到{mg:.0f}mm(看不清的圆柱多留{ex:.0f}mm)才规划出来',flush=True)
                    break
            return legs,why,pl_,mg,obs
        try:
            legs,why,pl_,mg,obs=attempt(mission)
            if why!='OK' and 'PREHOME' in mission:
                print(f'  ⚠ 带 PREHOME 规划不出来({why})：去掉 PREHOME 再规划(回启停区前在离启停区 {raw_cfg.get("home_approach_mm",800)}mm 内定位)',flush=True)
                legs,why,pl_,mg,obs=attempt([s for s in mission if s!='PREHOME'])
            state['strafe_fallback']=bool(getattr(pl_,'fallback',False))
            state['lane_fallback']=bool(getattr(pl_,'lane_fallback',False))
            state['sealed']=[] if state['lane_fallback'] else list(getattr(pl_,'sealed',[]))   # v11：整段封死的车道
            state['box']=getattr(pl_,'box',None)                                                 # v13：车身外框(含雷达)
            state['plan_margin']=mg;state['plan_obs']=obs
        except ValueError as e:
            legs,why=[],str(e)
        state['legs']=legs;state['reason']='ROUTE OK' if why=='OK' else why
        state['legs_partial']=rf is not None;state['plan_obj_key']=obj_key()
        print(f'  规划用时 {time.monotonic()-t_plan:.1f} 秒',flush=True)
        pts=[state['plan_start'][:2]]
        for L in legs:pts+=[p[:2] for p in L['poses']]
        state['path']=pts if legs else []
    def plan_route(cur=None):
        with plan_lock:_plan_route(cur)
    def start_home_routes():
        """1010：在后台给每个停车点预先算好"从这里直接回家"的路线(时间不够时用，车上现场规划太慢)。"""
        state['home_gen']+=1;gen=state['home_gen'];routes={};state['home_routes']=routes
        legs=list(state['legs']);stops=raw_cfg.get('stops') or {}
        if not legs or not str(legs[-1]['stop']).upper().startswith('START'):return
        tail=([legs[-2]['stop']] if len(legs)>1 and legs[-2]['stop']=='PREHOME' else [])+[legs[-1]['stop']]
        cfg_=dict(mission_cfg(),margin_mm=state.get('plan_margin',raw_cfg.get('margin_mm',60)))
        obs=list(state.get('plan_obs') or [])
        goals=[]
        for L in legs:
            if str(L['stop']).upper().startswith(('START','PREHOME')):continue
            key=(L['stop'],round(L['goal'][0]),round(L['goal'][1]))
            if key not in [k for k,_ in goals]:goals.append((key,tuple(L['goal'])))
        def work():
            t0=time.monotonic()
            for key,goal in goals:
                if state['home_gen']!=gen:return
                r=home_route_legs(cfg_,obs,goal,tail)
                if r:routes[key]=r
            print(f'  (后台算好了 {len(routes)}/{len(goals)} 个停车点直接回家的路线，{time.monotonic()-t0:.1f} 秒)',flush=True)
        threading.Thread(target=work,daemon=True).start()
    def print_route():
        if not state['legs']:print('没有路线：',state['reason'],flush=True);return
        names={'F':('前进','后退'),'S':('左移','右移')}
        ps=state['plan_start'];tb=state.get('turn_back',0) or 0
        st=route_stats(state['legs'],raw_cfg)
        print(f'全程路线（启停区 {raw_cfg.get("start_zone",2)}；转弯绕车中心(转轴偏后{raw_cfg.get("turn_pivot_back_mm",0)}mm)；预计行驶 {st["time"]:.0f} 秒：前进/后退 {st["F"]}mm，横移 {st["S"]}mm，转弯 {st["turns"]} 次）：',flush=True)
        if state.get('box'):
            b_=state['box'];print(f'  规划用的车身外框(含雷达)：前后各{b_[1]:.0f}mm，右边{-b_[2]:.0f}mm，左边(雷达那边){b_[3]:.0f}mm（从车中心算）',flush=True)
        if state.get('sealed'):
            print('  有障碍物、整段封死不走的车道：'+'，'.join(f'x{r[0]}~{r[2]} y{r[1]}~{r[3]}' for r in state['sealed']),flush=True)
        if state.get('lane_fallback'):
            print('  ⚠ 把有障碍物的车道整段封死以后到不了，已改成只绕开障碍物本身：路线会从障碍物旁边过！',flush=True)
        if abs(tb)>=0.5 and ps:print(f"  先原地{'逆时针' if tb>0 else '顺时针'}转{abs(tb):.0f}°，车头回到{ps[2]:.0f}°，车中心到 ({ps[0]:.0f},{ps[1]:.0f})",flush=True)
        for L in state['legs']:
            txt=[]
            for k,v in L['cmds']:
                if k=='R':txt.append(('逆时针转' if v>0 else '顺时针转')+f'{abs(v)}°')
                else:txt.append(names[k][0 if v>0 else 1]+f'{abs(v)}mm')
            print(f"  到 {L['stop']}：",'，'.join(txt) if txt else '原地',flush=True)
            if L.get('note'):print('    '+L['note'],flush=True)
        if state['reason']!='ROUTE OK':print('  后面的路线失败：',state['reason'],flush=True)
        if state.get('strafe_fallback') and state['legs']:
            print('  ⚠ 配置里限制了横移长度(strafe_max_mm_per_leg)，限制下到不了，已放开限制：路线里有长距离横移。',flush=True)
    def print_objects():
        if not history.objects:print('当前没有障碍物候选。',flush=True);return
        for o in history.objects:
            print(f"  C{o['id']}: ({o['x']:.0f}, {o['y']:.0f})  第{o['anchor_view']}次扫描发现  看到{o['seen']}次"+('  [多个靠近，需确认]' if o.get('ambiguous') else ''),flush=True)
    def redraw():
        for a in markers:a.remove()
        markers.clear()
        c=state['config'];x,y=c['car_x_mm'],c['car_y_mm'];yaw=c['car_yaw_deg']
        car.set_xy(rotate(body_corners(c),yaw)+[x,y])
        end=rotate([(120,0)],yaw)[0]+[x,y];heading.set_data([x,end[0]],[y,end[1]])
        front_txt.set_position((end[0]+15,end[1]+15))
        lp=to_map([[0,0]],c)[0];lidar_mark.set_data([lp[0]],[lp[1]]);lidar_txt.set_position((lp[0]-150,lp[1]+40))
        points=[p for v in history.views for p in v['field_returns']]
        mapped.set_offsets(np.asarray(points).reshape(-1,2))
        for o in list(history.objects):
            x,y,r=o['x'],o['y'],o['radius'];dot=Circle((x,y),r,color='black',zorder=6);world.add_patch(dot);markers.append(dot)
            half=max(body_half_bound(c),float(raw_cfg.get('plan_body_mm',300) or 0)/2)+c['margin_mm']+r
            pad=Rectangle((x-half,y-half),2*half,2*half,fc='red',alpha=.10,ec='none');world.add_patch(pad);markers.append(pad)
            markers.append(world.annotate(f"C{o['id']} ({x:.0f},{y:.0f}) n={o['count']} seen={o['seen']}"+(' ?' if o.get('ambiguous') else ''),(x,y),xytext=(4,5),textcoords='offset points',fontsize=7))
        # v12：只有两次扫描都做完才自动规划；地图没变就不重新规划。1010：一键流程运行中由行驶线程在后台规划，这里不规划(不卡界面)
        if not state['auto']:
            if len(history.views)>=2:
                key=(len(history.views),tuple((round(o['x']),round(o['y'])) for o in history.objects),state['plan_start'])
                if key!=state.get('plan_key'):
                    plan_route();state['plan_key']=key
            elif history.views:
                state['legs'],state['path'],state['reason']=[],[],'ONE SCAN (route after 2nd scan)'
            else:
                plan_route()
        path=list(state['path'])
        route.set_data(*zip(*path)) if path else route.set_data([],[])
        for r in (state.get('sealed') or []):      # v11：整段封死的车道
            box=Rectangle((r[0],r[1]),r[2]-r[0],r[3]-r[1],fc='red',alpha=.18,ec='red',hatch='xx',lw=1,zorder=3)
            world.add_patch(box);markers.append(box)
        world.set_title(f"ZONE {raw_cfg.get('start_zone',2)} | {state['reason']}",fontsize=10,color='red' if state['reason']!='ROUTE OK' else 'black')
        info.set_text(f"STOPPED views={len(history.views)} | candidates={len(history.objects)} | pose=({c['car_x_mm']:.0f}, {c['car_y_mm']:.0f}, {c['car_yaw_deg']:.1f} deg)\nOriginal v4 exclusion regions retained. Unobserved space is UNKNOWN.")
        fig.canvas.draw_idle()
    def save():
        if state['busy']:raise ValueError('请等待扫描完成')
        out=ROOT/'map_captures';out.mkdir(exist_ok=True)
        base=out/('merged_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        payload=dict(reference_correction=metadata(),route_start=state['plan_start'],route_status=state['reason'],start_zone=raw_cfg.get('start_zone',2),
                     route_legs=[dict(stop=L['stop'],goal=L['goal'],cmds=L['cmds']) for L in state['legs']],drive_ready=False,pose_source='external_manual',merge_policy='earliest_anchor_wins',merge_radius_mm=history.merge_radius_mm,map_completeness='partial_unknown_unobserved',config=state['config'],views=history.views,confirmed_detections=history.objects,candidate_obstacles_mm=[[o['x'],o['y'],o['radius']] for o in history.objects],preview_path_mm=state['path'],goal_mm=state['goal'])
        base.with_suffix('.json').write_text(json.dumps(payload,indent=2))
        fig.savefig(base.with_suffix('.png'),dpi=100)
        print('已保存：',base,flush=True)
    def launch(c,source,mode='scan'):
        if state['busy']:raise ValueError('正在扫描，不要移动；等待完成后再输入')
        c=copy.deepcopy(validate_config(c));state['busy']=True;state['mode']=mode
        true_pose=None
        if SIM is not None and source=='second_relative_default':
            true_pose=(c['car_x_mm']+sim_err[0],c['car_y_mm']+sim_err[1],c['car_yaw_deg']+sim_err[2])
        print('参考校正：'+description(c),flush=True)
        status.set_text('CAPTURING: keep chassis STILL. Terminal shows progress.');fig.canvas.draw_idle()
        def worker():
            try:
                m,seq=acquire(c,args.port,args.model,stop,frames=int(raw_cfg.get('scan_frames',12)),true_pose=true_pose,settle_s=float(raw_cfg.get('scan_settle_s',0.3)));results.put(('done',m,seq,source))
            except Exception as e:results.put(('error',str(e)))
        state['worker']=threading.Thread(target=worker,daemon=True);state['worker'].start()
    # ---------------------------------------------------------------- 1010 雷达重定位
    def view_pose(v):
        c=v['config'];return (float(c['car_x_mm']),float(c['car_y_mm']),float(c['car_yaw_deg']))
    def reference(only_first=False):
        """出发时的参考点云(第一次 + 第二次扫描，换到出发时的车身坐标) + 最近点表；地图变了才重建。"""
        vs=[v for v in history.views[:1 if only_first else 2] if v.get('raw_frames')]
        if not vs:return None
        key=tuple((id(v),view_pose(v)) for v in vs);slot='ref1' if only_first else 'ref'
        if state.get(slot+'_key')==key and state.get(slot) is not None:return state[slot]
        t0=time.monotonic()
        ref=reloc.build_reference([(v['raw_frames'],view_pose(v)) for v in vs],
                                  lambda fr,n,c=vs[0]['config']:reloc.body_points(fr,c,max_n=n),cfg=raw_cfg)
        for i,p_old,p_new,r in getattr(ref,'align_notes',[]):
            if p_new is None:print(f'  (第二次扫描和第一次对不上({r["why"]})：参考点云按记下的第二站车位合并)',flush=True)
            else:
                d=math.hypot(p_new[0]-p_old[0],p_new[1]-p_old[1]);a=reloc.wrap180(p_new[2]-p_old[2])
                print(f'  第二站雷达对齐：实际车位 ({p_new[0]:.0f},{p_new[1]:.0f}) 车头{p_new[2]:.1f}°，和记下的差 {d:.0f}mm、{a:+.1f}°',flush=True)
                if d>float(raw_cfg.get('second_warn_mm',30)) or abs(a)>2.0:
                    print(f'  ⚠ 第二站：按障碍物校正的车位和雷达整圈对齐的差得多({d:.0f}mm、{a:+.1f}°)。路线起点按雷达对齐的算；'
                          '第二次扫描登记的障碍物位置可能偏了，看一眼地图(启停区 1 第一次用时尤其要看)',flush=True)
                if not only_first:state['second_icp']=tuple(p_new)
        state[slot]=ref;state[slot+'_key']=key
        print(f'  (雷达定位参考点云：{len(ref.points)} 个点，建表 {time.monotonic()-t0:.2f} 秒)',flush=True)
        return ref
    def relocalize(plan_pose,frames=8,use_init=True,frames_data=None,max_shift=None,only_first=False):
        """在车现在的位置扫 frames 帧(或用 frames_data)，和参考点云对齐，得到车的实际位置(世界坐标)。
        修正量 > reloc_confirm_mm 时再扫一次，两次一致才信。返回 reloc.relocalize 的 dict。"""
        bad=lambda why:dict(ok=False,why=why,pose=None,dx=0.0,dy=0.0,dyaw=0.0,rms=0.0,inlier=0.0,t=0.0)
        if SIM is not None:return bad('桌面模拟不做雷达定位')
        if not raw_cfg.get('reloc_enabled',True):return bad('配置里关掉了(reloc_enabled=false)')
        try:ref=reference(only_first)
        except Exception as e:return bad(f'建参考点云出错：{e!r}')
        if ref is None:return bad('没有出发时的扫描数据')
        c0=history.views[0]['config'];box=[frames_data];npts=int(raw_cfg.get('reloc_points',600))
        def meas(pp):
            try:
                if box[0] is not None:fr,box[0]=box[0],None
                else:fr=acquire(c0,args.port,args.model,stop,frames=frames,settle_s=float(raw_cfg.get('reloc_settle_s',0.3)),quiet=True)[0].raw_frames
                return ref.measure(reloc.body_points(fr,c0,max_n=npts),pp,raw_cfg,use_init=use_init,max_shift=max_shift)
            except Exception as e:
                return dict(ok=False,why=f'扫描出错：{e}',pose=None,t=0.0)
        return reloc.relocalize(meas,plan_pose,raw_cfg)
    def still_check():
        """race 等 START 时：车还在第一次扫描的位置吗(雷达看)？"""
        if not history.views:return None
        c0=history.views[0]['config'];p0=view_pose(history.views[0])
        r=relocalize(p0,frames=6,only_first=True,max_shift=400)
        if not r.get('ok'):return None
        d=math.hypot(r['dx'],r['dy'])
        moved=d>float(raw_cfg.get('race_moved_mm',20)) or abs(r['dyaw'])>float(raw_cfg.get('race_moved_deg',1.0))
        return dict(moved=moved,why=f'雷达看到车挪了 {d:.0f}mm、{r["dyaw"]:+.1f}°')
    def obstacles():
        return [(float(o['x']),float(o['y']),float(o['radius'])) for o in list(history.objects)]
    def predicted_second():
        """开跑前预先规划用：按第二站动作推算第二站车位(配置有 second_pose_mm 就用它)。"""
        c0=history.views[0]['config']
        c=moved_config(c0,float(rel['left_mm']),float(rel['forward_mm']),float(rel['cw_deg']))
        sp=raw_cfg.get('second_pose_mm')
        if sp:return (float(sp[0]),float(sp[1]),float(sp[2]))
        return (float(c['car_x_mm']),float(c['car_y_mm']),float(c['car_yaw_deg']))
    def try_reuse_preplan():
        """race：第二次扫描没看到新障碍物、第二站车位和推算的差不多，就直接用开跑前规划好的路线(不再花十几秒规划)。"""
        pp=state.get('preplan')
        if not pp or pp.get('status')!='ok' or len(history.views)<2 or state.get('replan_from') is not None:return False
        c=state['config'];pr=pp['pred']
        dp=math.hypot(c['car_x_mm']-pr[0],c['car_y_mm']-pr[1])
        lim=float(raw_cfg.get('preplan_reuse_mm',40))
        new=[o for o in history.objects if all(math.hypot(o['x']-q[0],o['y']-q[1])>float(raw_cfg.get('preplan_obs_mm',60)) for q in pp['obs'])]
        if dp>lim or new:
            print(f'  开跑前规划的路线不能直接用('+(f'第二站和推算的差 {dp:.0f}mm' if dp>lim else '')+('，' if dp>lim and new else '')
                  +(f'第二次扫描多了 {len(new)} 个障碍物' if new else '')+')：重新规划(手臂会轻摆，避开 15 秒规则)……',flush=True)
            return False
        state.update(legs=pp['legs'],reason='ROUTE OK',turn_back=pp['turn_back'],plan_start=pp['plan_start'],start_pivot=pp['start_pivot'],
                     route_ref=pp['pred'],path=pp['path'],plan_margin=pp['margin'],plan_obs=pp['plan_obs'],legs_partial=False)
        state['plan_obj_key']=obj_key()
        print(f'  第二次扫描没有新障碍物、第二站差 {dp:.0f}mm：直接用开跑前规划好的路线。',flush=True)
        return True
    class Ctx:
        tick=None
        def _tick(self):
            if self.tick is not None:self.tick()
        def _gui(self,cmd,timeout=10.0):
            """让界面线程做一件事(动地图/历史只在界面线程里做)，等它做完。"""
            before=state['gui_done'];commands.put(cmd);t=time.monotonic()
            while state['gui_done']==before:
                if time.monotonic()-t>timeout:raise Abort(f'界面没有响应({cmd})')
                time.sleep(0.05)
        def relocalize(self,plan_pose,frames=8,use_init=True,frames_data=None,max_shift=None):
            self._tick();return relocalize(plan_pose,frames,use_init,frames_data,max_shift)
        def obstacles(self):return obstacles()
        def route_start(self):
            """路线起点：ref = 规划时按的第二站车位；est = 第二站实际车位 + STM32 现在要保持的车头(开跑时 HOME 的车头 - 第二站顺时针转的角度)。"""
            c=state['config'];ref=state.get('route_ref') or (c['car_x_mm'],c['car_y_mm'],c['car_yaw_deg'])
            t0=state.get('stm_target_yaw');p2=state.get('second_icp')     # 第二站雷达对齐出来的实际车位(有就用它)
            x,y=(p2[0],p2[1]) if (p2 and t0 is not None) else (c['car_x_mm'],c['car_y_mm'])
            return dict(est=(x,y,t0 if t0 is not None else c['car_yaw_deg']),ref=tuple(ref),yaw_real=c['car_yaw_deg'])
        def home_route(self,name,goal):
            if goal is None:return None
            return state['home_routes'].get((name,round(goal[0]),round(goal[1])))
        def station_scan(self,stop_name,pose,remaining,est=None):
            """v14：在停车点上再扫一次。1010：先用这次扫描的帧做雷达定位，按测到的实际车位登记新障碍物；
            有新障碍物就从这个停车点重新规划剩下的路线(后台规划，手臂轻摆)。"""
            pose=tuple(float(v) for v in pose[:3]);est=tuple(float(v) for v in (est or pose)[:3])
            c=copy.deepcopy(state['config']);c.update(car_x_mm=est[0],car_y_mm=est[1],car_yaw_deg=est[2])
            print(f'站点扫描({stop_name})：车停稳，扫一圈(看有没有之前没看到的障碍物，同时雷达定位)……',flush=True)
            try:
                m,seq=run_bg(lambda:acquire(c,args.port,args.model,stop,frames=int(raw_cfg.get('station_frames',raw_cfg.get('scan_frames',12))),true_pose=est,
                                            settle_s=float(raw_cfg.get('scan_settle_s',0.3)),quiet=True),self.tick,lambda:state['abort'])
            except Abort:raise
            except Exception as e:
                print('  站点扫描失败：',e,'，按原路线继续。',flush=True);return None
            r=relocalize(est,int(raw_cfg.get('reloc_frames',8)),frames_data=m.raw_frames) if SIM is None else None
            if r and r.get('ok'):
                p=r['pose']
                if math.hypot(p[0]-est[0],p[1]-est[1])>10 or abs(r['dyaw'])>0.5:
                    c2=dict(c,car_x_mm=p[0],car_y_mm=p[1],car_yaw_deg=p[2]);m=remap(m,c2)   # 障碍物按实际车位登记
            mr=float(raw_cfg.get('station_new_mm',200))
            seen=list(m.latched)
            new=[d for d in seen if all(math.hypot(d['x']-o['x'],d['y']-o['y'])>mr for o in history.objects)]
            res=dict(stop=stop_name,new=[(round(d['x']),round(d['y'])) for d in new],replanned=False,legs=[],reason='ROUTE OK',reloc=r)
            if not new:
                print(f'  站点扫描({stop_name})：看到 {len(seen)} 个圆柱，都是已知的，按原路线继续。',flush=True)
                return res
            print(f'  站点扫描({stop_name})：发现 {len(new)} 个新障碍物：'+'，'.join(f"({d['x']:.0f},{d['y']:.0f}) {d.get('count','?')}点/{d.get('seen','?')}帧" for d in new)+'。加进地图，从这里重新规划剩下的路线……',flush=True)
            state['commit_req']=(m,'station_'+stop_name,seq);self._gui('__commit')
            state['replan_from']=(pose,list(remaining));state['plan_start']=None
            run_bg(plan_route,self.tick,lambda:state['abort'])
            print_route();commands.put('__redraw')
            res.update(replanned=True,legs=list(state['legs']),reason=state['reason'])
            if state['reason']=='ROUTE OK':start_home_routes()
            return res
        def _wait(self,key,before,timeout=60.0):
            t=time.monotonic()
            while time.monotonic()-t<timeout:
                if state['abort']:raise Abort('收到 abort')
                if state['err_count']!=state['err_before']:raise Abort('扫描失败，见上面的提示')
                if state[key]!=before and not state['busy']:return
                self._tick()
                time.sleep(0.1)
            raise Abort('等待扫描超时')
        def scan(self):
            state['err_before']=state['err_count'];before=state['done_count']
            commands.put('__scan');self._wait('done_count',before)
        def second(self):
            state['err_before']=state['err_count'];before=state['done_count']
            commands.put('__second');self._wait('done_count',before)
        def plan(self):
            """1010：规划在后台线程里做(不卡界面)，等的时候手臂轻摆；race 时开跑前规划好的路线能用就直接用。"""
            try:run_bg(reference,self.tick,lambda:state['abort'])          # 雷达定位的参考点云(顺便把第二站和第一次扫描对齐)
            except Abort:raise
            except Exception as e:print('  (建雷达定位参考点云出错：',repr(e),')',flush=True)
            cur_ok=state['legs'] and state['reason']=='ROUTE OK' and not state.get('legs_partial') and state.get('plan_obj_key')==obj_key()
            if not cur_ok and not try_reuse_preplan():
                state['plan_start']=None;state['replan_from']=None
                print('规划全程路线(后台)……',flush=True)
                run_bg(plan_route,self.tick,lambda:state['abort'])
                commands.put('__redraw')
                if state['reason']=='ROUTE OK':start_home_routes()
            elif not state['home_routes']:start_home_routes()
            return state.get('turn_back',0) or 0,list(state['legs']),state['reason']
        def print_route(self):print_route()
        def ask(self,msg):
            print(msg,flush=True);state['confirm']=None
            while state['confirm'] is None:
                if state['abort']:return False
                time.sleep(0.1)
            return bool(state['confirm'])
        def aborted(self):return state['abort']
        # ---- race(开跑前，在 race 线程里)
        def zone_select(self,z):self._gui(f'__zone {int(z)}')
        def gyro_yaw(self):
            if gyro is None or not gyro.fresh(1.0):return None
            return gyro.value/100.0
        def preplan_start(self):
            """第一次扫描以后，按推算的第二站车位和第一次扫描的障碍物在后台先规划一遍(开跑后第二次扫描没变化就直接用)。"""
            state['preplan_gen']=state.get('preplan_gen',0)+1;gen=state['preplan_gen']   # 车又被挪了/重新选区：旧的后台规划结果作废
            state['preplan']=dict(status='run')
            def work():
                try:
                    pred=predicted_second()
                    with plan_lock:
                        seen=[(o['x'],o['y']) for o in history.objects]
                        state['plan_start']=None;state['replan_from']=None
                        _plan_route(pred)
                        pp=dict(status='ok' if state['reason']=='ROUTE OK' else state['reason'],legs=list(state['legs']),
                                turn_back=state.get('turn_back',0),plan_start=state['plan_start'],start_pivot=state.get('start_pivot'),
                                pred=pred,path=list(state['path']),margin=state.get('plan_margin'),plan_obs=list(state.get('plan_obs') or []),
                                obs=seen)
                        if pp['status']=='ok':print_route()
                        state['plan_start']=None
                    if state.get('preplan_gen')!=gen:return
                    state['preplan']=pp;commands.put('__redraw')
                    if pp['status']=='ok':start_home_routes()
                except Exception as e:
                    if state.get('preplan_gen')==gen:state['preplan']=dict(status=f'出错：{e!r}')
            threading.Thread(target=work,daemon=True).start()
        def preplan_state(self):
            pp=state.get('preplan');return None if not pp else pp.get('status')
        def still_check(self):return still_check()
    def save_params(updates):
        """把运动参数存进配置的 stm32_params(第一次改之前把原配置备份成 .json.bak)。树莓派每次 go/drive 都会把它们发给 STM32。"""
        p=Path(args.config)
        bak=p.with_suffix('.json.bak')
        if not bak.exists():bak.write_text(p.read_text(encoding='utf-8'),encoding='utf-8')
        sp_=dict(raw_cfg.get('stm32_params',{}));sp_.update(updates);raw_cfg['stm32_params']=sp_
        p.write_text(json.dumps(file_cfg(),indent=2,ensure_ascii=False),encoding='utf-8')
    synced=[False]
    def sync_now(force=False):
        """把配置里的 stm32_params 发给 STM32(STM32 断电重启后参数会回到编译进去的默认值)。每次运行程序第一次动车前自动做一次。"""
        if link is None or (synced[0] and not force):return
        sp_=raw_cfg.get('stm32_params') or {}
        bad=link.sync_params(sp_,log=lambda m:print(m,flush=True))
        synced[0]=True
        if sp_:print(f'已把配置里的 {len(sp_)} 个运动参数发给 STM32'+(f'（{len(bad)} 个失败）' if bad else '')+'。',flush=True)
    def make_hooks():
        if not (raw_cfg.get('mission_cfg') or {}).get('enabled',False):return None
        # 机械臂任务钩子：QR 读码、RAW 抓取、ROUGH/TEMP 放置、START 显示统计(见 mission_hooks.py)
        from mission_hooks import MissionHooks
        try:
            from mission_cli import release
            release()                                   # 测试命令(mtest/vcal…)占着的摄像头先放掉
        except Exception:pass
        h=MissionHooks(raw_cfg,log=log)
        print('已启用机械臂任务钩子(mission_cfg.enabled=true)。',flush=True)
        return h
    def mission_body(first_scan,scanned=False):
        hooks=None
        try:
            hooks=make_hooks();state['hooks']=hooks or state.get('hooks')
            if first_scan:
                # STM32 开跑时 HOME：要保持的车头 = 出发时的车头；第二站顺时针转 cw 度以后 = 出发车头 - cw
                state['stm_target_yaw']=float(state['config']['car_yaw_deg'])-float(rel.get('cw_deg',60))
                state['legs']=[];state['plan_obj_key']=None;state['second_icp']=None   # 不用上一次的路线
                if not scanned:state['preplan']=None;state['home_routes']={}          # go：没有开跑前的预先规划
            run_mission(Ctx(),link,log=log,first_scan=first_scan,stop_wait=args.stop_wait,hooks=hooks,cfg=raw_cfg,scanned=scanned)
        except Abort as e:
            print('★ 已停止：',e,flush=True)
            after_error(hooks)
        except Exception as e:
            print('★ 出错停止：',repr(e),flush=True)
            after_error(hooks)
        finally:
            if hooks is not None:
                try:hooks.close()                               # 释放摄像头，下一次 go 才能再打开
                except Exception:pass
            state['replan_from']=None;state['plan_start']=None;state['stm_target_yaw']=None
            if state.get('legs_partial'):state['legs']=[];state['reason']='用过站点重新规划：drive 前先 route'
    def after_error(hooks):
        """出错停下(不是急停)：收臂、升降停 60mm(手里可能有物料时 hooks.park 会跳过)。急停后不自动动。"""
        if state['abort']:
            print('  急停后不自动收臂。确认车旁没人后输入 park，把升降停到 60mm。',flush=True);return
        if hooks is not None and hasattr(hooks,'park') and raw_cfg.get('park_after_error',True):
            try:park_arm(link,hooks,log)
            except Exception as e:print('  收臂失败：',repr(e),flush=True)
        else:print('  要把升降停到 60mm：输入 park。',flush=True)
    def mission_thread(first_scan):
        try:mission_body(first_scan)
        finally:state['auto']=False
    def race_thread():
        try:
            state['race']=True                              # 在等屏上选区/按 START(终端 zone 1|2 可以代替屏上选区)
            def start_run(z,scanned):
                state['race']=False;mission_body(True,scanned)
            st=race_flow(Ctx(),link,log,raw_cfg,start_run)
            if st=='nozone':print('（race 没开始：用 zone 1 / zone 2 选区，go 开始）',flush=True)
        except Abort as e:print('★ 比赛流程已停止：',e,flush=True)
        except Exception as e:print('★ 比赛流程出错：',repr(e),flush=True)
        finally:state['auto']=False;state['race']=False
    def command(line):
        import unicodedata
        line=unicodedata.normalize('NFKC',line).replace('\u3000',' ')   # 中文输入法打出的全角字母/空格也能认
        parts=line.strip().split();k=parts[0].lower() if parts else 'scan'
        if len(parts)==1 and re.fullmatch(r'\d{3}(\+\d{3}){3}',parts[0]):
            parts=['mcode',parts[0]];k='mcode'      # 直接输入任务码(例如 652+312+526+231) = mcode 任务码
        # ---- 行驶/比赛线程让界面线程做的事
        if k.startswith('__'):
            try:
                if k=='__scan':launch(state['config'],'unchanged_external_pose')
                elif k=='__second':command_second([])
                elif k=='__zone':select_zone(int(parts[1]))
                elif k=='__commit':
                    m_,src_,seq_=state.pop('commit_req');history.commit(m_,src_,seq_);redraw()
                    try:save()
                    except Exception as e:print('保存失败：',e,flush=True)
                elif k=='__redraw':redraw()
            except Exception as e:
                print('未执行：',e,flush=True)
                if k in ('__scan','__second'):state['err_count']+=1     # 等扫描的那边马上知道失败了
            finally:
                if k not in ('__scan','__second'):state['gui_done']+=1
            return
        if k=='q':plt.close(fig);return
        if k in ('y','yes'):state['confirm']=True;return
        if k in ('n','no'):state['confirm']=False;return
        if k=='abort':
            state['abort']=True;state['aborted']=True
            if link is not None:link.abort()
            print('收到 abort：已给 STM32 发急停(!)，路线停止发送。急停后不会自动收臂；确认车旁没人后输入 park 把升降停到 60mm。',flush=True);return
        if k=='ping':
            if link is None:raise ValueError('没有连接 STM32（--stm-port）')
            print('PING ->',link.ping(),flush=True);return
        if k=='sync':
            if link is None:raise ValueError('没有连接 STM32（--stm-port）')
            sync_now(force=True);return
        if k=='get':
            if link is None:raise ValueError('没有连接 STM32（--stm-port）')
            sync_now()
            ps,rep_=link.get_params()
            if ps is None:raise ValueError('读参数失败：'+str(rep_))
            sp_=raw_cfg.get('stm32_params',{})
            print('STM32 现在的运动参数（* 表示配置里存了、每次启动会自动发给 STM32）：',flush=True)
            for n_,v_ in ps.items():print(f"  {'*' if n_ in sp_ else ' '} {n_:7s} = {v_:g}",flush=True)
            return
        if k in ('set','tune'):
            if link is None:raise ValueError('没有连接 STM32（--stm-port）')
            if len(parts)!=3:raise ValueError('格式：set 名字 数值，例如 set SSACC 250（输入 get 看全部参数）')
            n_=parts[1].upper();v_=float(parts[2])
            sync_now()
            ok_,rep_=link.set_param(n_,v_)
            if not ok_:raise ValueError(f'STM32 拒绝：{rep_}')
            save_params({n_:v_});print(f'已设置 {n_}={v_:g}，并存进配置(以后每次启动自动发给 STM32)。',flush=True);return
        if k=='fix':
            # 距离校准：fix F 1000 985 = 命令走了1000mm，尺子量实际只走了985mm。算出新的 脉冲/mm 发给 STM32 并存进配置
            if link is None:raise ValueError('没有连接 STM32（--stm-port）')
            names={'F':'FPPM','B':'BPPM','L':'LPPM','R':'RPPM'}
            if len(parts)==4 and parts[1].upper() in ('DL','DR'):
                # 横移时往前后漂的校准：fix DL 600 40 = 左移600mm后，车往后偏了40mm(往前偏就填负数)；DR 是右移
                n_='DRL' if parts[1].upper()=='DL' else 'DRR';dist_=float(parts[2]);back=float(parts[3])
                if dist_<=0 or abs(back)>0.2*dist_:raise ValueError('距离要是正数，偏后/偏前的毫米数不能超过横移距离的20%')
                sync_now()
                ps,rep_=link.get_params()
                if ps is None or n_ not in ps:raise ValueError('读 STM32 参数失败：'+str(rep_))
                old=ps[n_];new=old-back/dist_
                ok_,rep_=link.set_param(n_,new)
                if not ok_:raise ValueError(f'STM32 拒绝：{rep_}')
                save_params({n_:round(new,5)});print(f'{n_}: {old:.4f} → {new:.4f}，已存进配置。再横移一次同样的距离验证。',flush=True);return
            if len(parts)!=4 or parts[1].upper() not in names:raise ValueError('格式：fix F 1000 985（F前进 B后退 L左移 R右移；命令的距离 实际量到的距离）；横移前后漂：fix DL 600 40（左移600后偏后40mm，偏前填负数；DR是右移）')
            n_=names[parts[1].upper()];cmd_d=float(parts[2]);real=float(parts[3])
            if cmd_d<=0 or real<=0:raise ValueError('距离要是正数')
            sync_now()
            ps,rep_=link.get_params()
            if ps is None or n_ not in ps:raise ValueError('读 STM32 参数失败：'+str(rep_))
            old=ps[n_];new=old*cmd_d/real
            if not 0.8<new/old<1.25:raise ValueError(f'相差太大({real:.0f} 对 {cmd_d:.0f})，先检查是不是量错了、或是车打滑')
            ok_,rep_=link.set_param(n_,new)
            if not ok_:raise ValueError(f'STM32 拒绝：{rep_}')
            save_params({n_:round(new,4)});print(f'{n_}: {old:.4f} → {new:.4f}（命令{cmd_d:.0f}mm、实际{real:.0f}mm），已存进配置。再走一次同样的距离验证。',flush=True);return
        if k=='park':
            # 1010：收臂、升降停到 60mm(下次开机编码器能认出高度)
            if link is None:raise ValueError('没有连接 STM32（--stm-port）')
            if state['auto'] or state['busy']:raise ValueError('正在运行或扫描，等结束再用')
            state['busy']=True
            def pworker():
                try:
                    if park_arm(link,state.get('hooks'),log,force=True):state['aborted']=False
                finally:state['busy']=False
            threading.Thread(target=pworker,daemon=True).start();return
        if k=='home':
            # v12：回家对准测试。车停在出发扫描过的地方附近，home = 测出偏差并小步修回去；home check = 只测不动
            if link is None:raise ValueError('没有连接 STM32（--stm-port）')
            if state['auto'] or state['busy']:raise ValueError('正在运行或扫描，等结束再用')
            if not history.views:raise ValueError('先在出发位置 scan 一次(作为对准的基准)，再挪车测试')
            chk=len(parts)>1 and parts[1].lower() in ('check','c','test')
            state['busy']=True;state['abort']=False
            def hworker():
                try:
                    p0=view_pose(history.views[0]);fr=int(raw_cfg.get('home_fix_frames',12))
                    if chk:
                        r=relocalize(p0,fr)
                        if r.get('ok'):print(f"回家对准(只测)：离出发位置 x{r['dx']:+.0f} y{r['dy']:+.0f}mm 车头{r['dyaw']:+.1f}°（{r['inlier']*100:.0f}%的点对上，残差{r['rms']:.1f}mm，ICP {r['t']:.2f}秒）",flush=True)
                        else:print('回家对准(只测)：对不上：',r.get('why'),flush=True)
                        return
                    sync_now()
                    ok_,rep_=link.home()                  # 把现在的车头记为要保持的方向
                    if not ok_:print('HOME 失败：',rep_,flush=True);return
                    nav=Nav(dict(est=p0,ref=p0))
                    res=home_approach(Ctx(),link,log,nav,p0,raw_cfg,firmware_caps(link,raw_cfg),obstacles(),aborted=lambda:state['abort'])
                    print('回家对准：',res['status'],flush=True)
                except Exception as e:print('回家对准出错：',repr(e),flush=True)
                finally:state['busy']=False
            threading.Thread(target=hworker,daemon=True).start();return
        if k=='cal':
            # 横移校准：车左边留够空地(默认 800mm)，车会不纠偏地左移再右移，测出车自己的偏转模型，结果自动存进配置
            if link is None:raise ValueError('没有连接 STM32（--stm-port）')
            if state['auto']:raise ValueError('一键流程正在运行；输入 abort 可以中止')
            sync_now()
            dist=int(float(parts[1])) if len(parts)>1 else 800
            sp_=int(float(parts[2])) if len(parts)>2 else int(raw_cfg.get('strafe_speed_rpm',170))
            print(f'横移校准：车会不纠偏地向左走 {dist}mm 再向右走回来(车头会转歪一些)，速度 {sp_}。车周围留出 1 米空地……',flush=True)
            def cworker():
                try:
                    res,rep_=link.cal(dist,sp_)
                    if res is None:print('校准失败：',rep_,flush=True);return
                    save_params(res)
                    print('校准完成，已生效并存进配置：'+'  '.join(f'{a}={b:.5f}' for a,b in res.items()),flush=True)
                finally:state['busy']=False
            state['busy']=True;threading.Thread(target=cworker,daemon=True).start();return
        if k in ('send','mv'):
            # 单条测试：send F 300 / send S -150 / send R -90，发一条等 DONE
            if link is None:raise ValueError('没有连接 STM32（--stm-port）')
            if state['auto']:raise ValueError('一键流程正在运行；输入 abort 可以中止')
            if len(parts) not in (3,4) or parts[1].upper() not in ('F','S','R'):raise ValueError('格式：send F 300 / send S -150 / send R -90 / send F 100 60(慢速)')
            sync_now()
            c,v=parts[1].upper(),int(float(parts[2]));sp=int(float(parts[3])) if len(parts)==4 else None
            if c=='R' and not -180<=v<=180:raise ValueError('R 的角度要在 -180~180')
            if c!='R' and not -2500<=v<=2500:raise ValueError('F/S 的距离要在 -2500~2500mm')
            if sp is None and c in ('F','S'):sp=int(raw_cfg.get('route_speed_rpm',220) if c=='F' else raw_cfg.get('strafe_speed_rpm',170))
            t0=time.monotonic();print(f'发送 {c} {v}'+(f' 速度{sp}' if sp else '')+' …',flush=True)
            ok_,rep_=link.move(c,v,sp);print(f'{c} {v} ->',rep_,f'  (往返用时{time.monotonic()-t0:.1f}秒)',flush=True);return
        if k in ('arm','vcal','vclaw','vmask','mtest','vdbg','qr','mcode','mot','gtest','rtest','vwatch','pcal'):
            # 机械臂/视觉测试命令、mot(看电机驱动器状态)，见 mission_cli.py 开头的说明
            # STM32 重启(断电、重新烧录)后参数回到编译进去的默认值：第一次用这些命令前也把配置里存的参数(SPOW、CLWO…)发一遍
            sync_now()
            from mission_cli import handle_cli
            handle_cli(k,parts,link=link,raw_cfg=raw_cfg,state=state,log=lambda m:print(m,flush=True));return
        if k in ('go','drive','p2','race'):
            if link is None:raise ValueError('没有连接 STM32（--stm-port），不能用 '+k)
            if state['auto']:raise ValueError('一键流程正在运行；输入 abort 可以中止')
            if state.get('busy'):raise ValueError('测试命令(vclaw/gtest/vcal…)还在运行，等它结束再用 '+k)
            if k=='p2':
                sync_now()
                print('第二站动作 ->',link.second_move(None if raw_cfg.get('second_use_p2',False) else rel,log=lambda m:print(m,flush=True)),flush=True);return
            if k=='race':
                # 1010 比赛：屏上选启停区 -> 车放好自动扫描、预先规划 -> 屏上按 START 开跑(开跑后不再碰电脑)
                state['abort']=False;state['aborted']=False;state['confirm']=None;state['auto']=True
                threading.Thread(target=race_thread,daemon=True).start();return
            if k=='go' and history.views:raise ValueError('go 要从头开始：先 reset 清空历史，并把车放回起点')
            if k=='go':
                # 上一次跑完 reset 以后 state['config'] 还是第二站/站点扫描的车位：第一次扫描要按本区的起点车位算
                state['config']=validate_config(dict(state['config'],car_x_mm=float(raw_cfg['car_x_mm']),car_y_mm=float(raw_cfg['car_y_mm']),car_yaw_deg=float(raw_cfg['car_yaw_deg'])))
            if k=='drive' and len(history.views)<2:raise ValueError('drive 只按已规划好的路线行驶，先完成两次扫描')
            if k=='drive' and (state.get('legs_partial') or not state['legs']):raise ValueError('现在没有完整的路线(上一次 go 在 QR 重新规划过)：先输入 route 重新规划全程')
            state['abort']=False;state['aborted']=False;state['confirm']=None;state['auto']=True
            threading.Thread(target=mission_thread,args=(k=='go',),daemon=True).start();return
        if k in ('3','4','5','6'):
            if state['busy']:raise ValueError('请等待扫描完成')
            state['goal']=POINTS[{'3':'QR_SCAN','4':'RAW_STOP','5':'TEMP_STOP','6':'ROUGH_STOP'}[k]];redraw();return
        if k in ('save','s'):save();return
        if k=='zone' and state.get('race') and len(parts)==2 and parts[1] in ('1','2'):
            # race 还在等屏上选区：从终端选(和在屏上按一样，桌上调试或屏的触摸不好用时)；START 还是要在屏上按
            ok_,rep_,_=zone_request(link,'ZONE '+parts[1])
            print(f'已让屏幕选启停区 {parts[1]}' if ok_ else f'STM32 不认 ZONE {parts[1]}：{rep_}',flush=True);return
        if state['busy']:raise ValueError('正在扫描，请等待完成')
        if k=='list':print_objects();return
        if state['auto'] and k!='pts':raise ValueError('一键流程正在运行(输入 abort 可以中止)，等结束再用 '+k)
        if k=='zone':
            # 1010：切换启停区(只在第一次扫描之前；比赛时在屏上选，race 会自动切换)
            if len(parts)!=2 or parts[1] not in ('1','2'):raise ValueError('格式：zone 1 或 zone 2')
            if history.views:raise ValueError('切换启停区要在第一次扫描之前：先 reset 清空历史')
            select_zone(int(parts[1]));return
        if k=='reset':
            history.objects.clear();history.views.clear();state.update(plan_start=None,replan_from=None,preplan=None,legs=[],plan_obj_key=None,legs_partial=False)
            redraw();print('已清空历史；scan 在当前已知车位重新扫描。',flush=True);return
        if k=='merge' and len(parts)==2:
            history.set_radius(float(parts[1]));print('去重距离：',history.merge_radius_mm,'mm',flush=True);return
        if k=='pts':
            if not history.views:raise ValueError('先扫描')
            for vi,vw in enumerate(history.views,1):
                rep=[];accumulated_detections(vw.get('raw_frames',[]),vw['config'],report=rep)
                print(f'第{vi}次扫描：场地内回波 {sum(r["count"] for r in rep)} 个点，{len(rep)} 团',flush=True)
                for r in sorted(rep,key=lambda r:-r['count']):
                    print(f"  ({r['x']:.0f},{r['y']:.0f}) 离雷达{r['dist']:.0f}mm  {r['count']}点/{r['frames']}帧  "+('→ 认作障碍物' if not r['why'] else '→ 没认：'+r['why']),flush=True)
            return
        if k=='route':state['plan_start']=None;state['replan_from']=None;plan_route();state['plan_key']=None;redraw();print_route();return
        if k=='pose' and len(parts)==4:
            if not history.views:raise ValueError('先扫描再设置规划起点')
            state['plan_start']=tuple(float(v) for v in parts[1:]);state['turn_back']=0;redraw();print_route();return
        if k=='calib':
            if len(parts)==2 and parts[1].lower()=='save':
                if not state['calib']:raise ValueError('还没有标定结果，先 calib X Y ...')
                bak=Path(args.config).with_suffix('.json.bak');bak.write_text(Path(args.config).read_text(encoding='utf-8'),encoding='utf-8')
                raw_cfg.update({k2:state['calib'][k2] for k2 in ('lidar_yaw_deg','lidar_forward_mm','lidar_left_mm')})
                Path(args.config).write_text(json.dumps(file_cfg(),indent=2,ensure_ascii=False),encoding='utf-8')
                state['config']=validate_config(dict(state['config'],**{k2:state['calib'][k2] for k2 in ('lidar_yaw_deg','lidar_forward_mm','lidar_left_mm')}))
                print('已写入配置（原文件备份为 .json.bak）。请 q 退出后重新 bash run_live.sh 再扫描。',flush=True);return
            if history.views:raise ValueError('标定要在第一次 scan 之前、车停在起点时做；已扫描请先 reset')
            if len(parts)<3 or len(parts)%2!=1:raise ValueError('格式：calib X1 Y1 [X2 Y2 ...]（圆柱的真实地图坐标mm）')
            state['calib_known']=[(float(parts[i]),float(parts[i+1])) for i in range(1,len(parts),2)]
            print('标定扫描：车停在起点不要动……',flush=True)
            launch(state['config'],'calibration','calib');return
        if k=='second':command_second(parts);return
        if k=='scan':launch(state['config'],'unchanged_external_pose');return
        if k=='r' and len(parts)==4:
            if not history.views:raise ValueError('请先完成第一次扫描')
            launch(moved_config(state['config'],*map(float,parts[1:])),'user_entered_displacement');return
        if k=='a' and len(parts)==4:
            if history.views:raise ValueError('修改初始车位前请先输入 reset 清空历史')
            c=copy.deepcopy(state['config']);c.update(zip(('car_x_mm','car_y_mm','car_yaw_deg'),map(float,parts[1:])))
            launch(c,'user_entered_absolute_pose');return
        raise ValueError('命令：race(比赛：屏上选区、按 START) / go(一键全流程) / zone 1|2 / park / drive / abort / ping / mot(电机驱动器状态) / mot en / gtest 颜色号(夹取测试) / send F 300 / get / set 名字 数值 / sync / fix F 1000 985 / cal / p2 / home [check] / calib X Y [X Y..] / calib save / scan / second / second X Y yaw 30 / route / pose X Y 角度 / list / pts / reset / save / q')
    def command_second(parts):
        if len(history.views)!=1:raise ValueError('second 要求已有且仅有第一次扫描')
        if len(parts)<=1:
            c=moved_config(history.views[0]['config'],float(rel['left_mm']),float(rel['forward_mm']),float(rel['cw_deg']))
            print(f"按设定动作推算第二站：左移{rel['left_mm']}、前进{rel['forward_mm']}、顺时针{rel['cw_deg']}° → ({c['car_x_mm']:.0f},{c['car_y_mm']:.0f}) 车头{c['car_yaw_deg']:.0f}°",flush=True)
            sp=raw_cfg.get('second_pose_mm')
            if sp:
                c=dict(c,car_x_mm=float(sp[0]),car_y_mm=float(sp[1]),car_yaw_deg=float(sp[2]))
                print(f"  按实测标定的第二站位置：({sp[0]},{sp[1]}) 车头{sp[2]}°（配置 second_pose_mm）",flush=True)
            state['second_yaw_gyro']=False
            if gyro is not None and raw_cfg.get('second_use_gyro',False) and gyro.fresh() and state['yaw_scan1'] is not None:
                d=(gyro.value-state['yaw_scan1'])/100.0
                yaw=(history.views[0]['config']['car_yaw_deg']+d+180)%360-180
                if abs(d+float(rel['cw_deg']))<=15:
                    c['car_yaw_deg']=yaw;state['second_yaw_gyro']=True
                    print(f'  陀螺仪实测两次扫描之间车头转了 {d:+.1f}°，第二站车头按 {yaw:.1f}° 计算(校正时不再改角度)',flush=True)
                else:
                    print(f'  陀螺仪读数变化 {d:+.1f}° 和设定差太多（可能中途清零了），不用它',flush=True)
            elif gyro is not None and raw_cfg.get('second_use_gyro',False):
                print('  没收到陀螺仪 YAW100，车头按上面的值计算',flush=True)
            launch(c,'second_relative_default');return
        if len(parts)==5:
            c=second_scan_config(history.views[0]['config'],float(parts[1]),float(parts[2]),parts[3],float(parts[4]))
        else:raise ValueError('格式：second X Y cw/ccw/yaw 角度，例如 second 2120 200 cw 60')
        print('第二次车位：',c['car_x_mm'],c['car_y_mm'],c['car_yaw_deg'],'度（不含自动校正）',flush=True)
        launch(c,'user_entered_second_absolute_xy_and_heading')
    def poll():
        dirty=False
        while not results.empty():
            item=results.get();state['busy']=False
            if item[0]=='done' and state['mode']=='calib':
                _,m,seq,source=item;known=state['calib_known']
                det=[(d['x'],d['y']) for d in m.latched]
                pairs=[]
                for kx,ky in known:
                    if det:
                        d=min(det,key=lambda p:math.hypot(p[0]-kx,p[1]-ky))
                        if math.hypot(d[0]-kx,d[1]-ky)<=400:pairs.append((d,(kx,ky)))
                if not pairs:print('标定失败：已知位置400mm内没扫到圆柱。检查圆柱是否在雷达能看到的一侧、车是否停在起点。',flush=True)
                else:
                    L=to_map([[0,0]],m.c)[0]
                    r=calibrate_mount(m.c,[p for p,_ in pairs],[q for _,q in pairs],L);state['calib']=r
                    for (d,q) in pairs:print(f'  扫到({d[0]:.0f},{d[1]:.0f}) ↔ 真实({q[0]:.0f},{q[1]:.0f})，差{math.hypot(d[0]-q[0],d[1]-q[1]):.0f}mm',flush=True)
                    print(f"标定结果：安装角 {m.c['lidar_yaw_deg']:.2f} → {r['lidar_yaw_deg']:.2f}°；雷达位置 前{r['lidar_forward_mm']:.0f} 左{r['lidar_left_mm']:.0f}mm；拟合残差{r['residual_mm']:.0f}mm",flush=True)
                    print('  只有1个圆柱时只修角度。确认没问题输入 calib save 写入配置。',flush=True)
                state['mode']='scan';continue
            if item[0]=='done':
                _,m,seq,source=item
                if history.views:
                    fix,msg=refine_second(m,history.objects,accumulated_detections,roi_class,to_map,trust_yaw=bool(state.get('second_yaw_gyro')));print(msg,flush=True)
                    if fix:print(f"  校正后第二站：({m.c['car_x_mm']:.0f},{m.c['car_y_mm']:.0f}) 车头{m.c['car_yaw_deg']:.1f}°",flush=True)
                if not history.views and gyro is not None and gyro.fresh():state['yaw_scan1']=gyro.value
                history.commit(m,source,seq);state['config']=m.c
                status.set_text('PAUSED: scan merged. You may move; enter ACTUAL displacement in terminal.')
                state['plan_start']=None
                redraw();print(f'扫描完成，已合并 {len(history.views)} 次，障碍物 {len(history.objects)} 个：',flush=True);print_objects()
                if len(history.views)>=2 and not state['auto']:print_route()
                try:save()
                except Exception as e:print('保存失败：',e,flush=True)
                state['done_count']+=1
            else:status.set_text('SCAN FAILED: previous map retained. See terminal.');print(item[1],flush=True);dirty=True;state['err_count']+=1
        while not commands.empty():
            try:command(commands.get())
            except Exception as e:print('未执行：',e,flush=True)
        if dirty:fig.canvas.draw_idle()
    def terminal():
        while not stop.is_set():
            try:line=input()
            except (EOFError,OSError):return
            commands.put(line)
    def key(e):
        k=(e.key or '').lower()
        if k in ('3','4','5','6','s','q'):commands.put(k)
    if hasattr(fig.canvas.manager,'key_press_handler_id'):fig.canvas.mpl_disconnect(fig.canvas.manager.key_press_handler_id)
    fig.canvas.mpl_connect('key_press_event',key)
    timer=fig.canvas.new_timer(interval=100);timer.add_callback(poll)
    def close(e):stop.set();timer.stop()
    fig.canvas.mpl_connect('close_event',close)
    c_=state['config']
    print(f"启停区 {zs['zone']}：起点车中心=({c_['car_x_mm']:.0f}, {c_['car_y_mm']:.0f})，车头 {c_['car_yaw_deg']:.0f}°（90=朝北，180=朝西）；雷达 前{config['lidar_forward_mm']:.1f} 左{config['lidar_left_mm']:.1f}mm。",flush=True)
    print('比赛：输入 race(或配置 race_auto=true 开机自动)：屏上选启停区 1/2 → 车放好不动，自动扫描、预先规划，屏上显示 READY → 按屏上 START 开跑，之后不用碰电脑。',flush=True)
    print('调试：zone 1 / zone 2 切换启停区；go = 一键全流程(不再问 y)；park = 收臂、升降停 60mm；abort = 急停(急停后不自动收臂)。',flush=True)
    print('（可选）标定：起点放好车，在雷达看得到的一侧放已知圆柱，输入 calib X1 Y1 X2 Y2 …，满意后 calib save。',flush=True)
    print('分步：1) 车停在起点输入 scan   2) 车自己走完 左移/前进/顺时针 后停稳，输入 second   3) route 重新规划，pose X Y 角度 改规划起点。q退出。',flush=True)
    print('1010：每个停车点先用雷达定位再修正；回家前停在 PREHOME(离启停区 250mm)定位，最后几步慢速、按实际车身限幅，不出场地。',flush=True)
    print('回家对准测试：出发位置 scan → 手动挪车 → home(修) 或 home check(只测)。',flush=True)
    print('v10 运动调试：send F 1000 = 单条指令；fix F 1000 985 = 距离校准；cal = 横移校准；get / set 名字 数值 = 看/改 STM32 参数；sync = 把配置里的参数重新发给 STM32。',flush=True)
    if args.sim:print('【桌面模拟模式】',flush=True)
    threading.Thread(target=terminal,daemon=True).start();redraw()
    if raw_cfg.get('race_auto',False) and link is not None:commands.put('race')
    try:timer.start();plt.show()
    except KeyboardInterrupt:pass
    finally:
        stop.set();timer.stop()
        try:quit_park(link,state.get('hooks'),log,running=state['auto'],aborted=state['aborted'],enabled=bool(raw_cfg.get('park_on_quit',True)) and not args.sim)
        except Exception as e:print('退出时收臂出错：',repr(e),flush=True)
        if state['worker'] is not None:state['worker'].join(timeout=6)
        close_stream()
    return 0
if __name__=='__main__':raise SystemExit(main())
