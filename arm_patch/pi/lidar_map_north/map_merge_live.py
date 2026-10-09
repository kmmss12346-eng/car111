#!/usr/bin/env python3
"""两次扫描合并 + 全程A*路线 + 一键行驶（v10）。
路线只靠 STM32 的陀螺仪和电机脉冲，没有雷达位置修正；雷达只用来识别障碍物(两次扫描合并)。"""
import os
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[key]='1'
import argparse,copy,json,math,queue,threading,time
from datetime import datetime
from pathlib import Path
import numpy as np
from lidar_map_live import (Mapping,validate_config,roi_class,rotate,make_path,body_corners,body_half_bound,FIELD_MM,to_map,accumulated_detections,
    YELLOW_RECTS,TEMP_ZONE,ROUGH_ZONE,START_RECTS,POINTS)
from ld14p_scan import Stream
from reference_correction import applicable, description, metadata
from merge_stations import History,moved_config,second_scan_config
from pose_fix import relocalize,apply_fix,calibrate_mount,refine_second
from route_plan import plan_mission,route_stats,Motion
from stm32_link import Stm32Link,FakeLink
from auto_run import run_mission,Abort
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
                    print(f'  扫描用时 {time.monotonic()-t_start:.1f} 秒（{"雷达刚打开" if fresh else "雷达已在转"}，{frames} 帧）',flush=True)
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
    args=ap.parse_args()
    raw_cfg=json.loads(Path(args.config).read_text(encoding='utf-8'))
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
    fig.suptitle('v14 | RESCAN AT QR + LIDAR LOCATE BEFORE START ZONE + LANES SEALED',fontsize=11)
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
    fig.text(.5,.03,'Terminal: calib X Y [X Y..] | calib save | scan | second | second X Y yaw DEG | route | pose X Y YAW | list | pts | reset | save | q\nBlue line = full A* mission route preview. Unobserved space is UNKNOWN.',ha='center',fontsize=9)
    history=History();markers=[];commands=queue.Queue();results=queue.Queue();stop=threading.Event()
    link=None
    if args.sim:link=FakeLink()
    elif args.stm_port!='none':link=Stm32Link(args.stm_port)
    if link is not None and not link.ok:
        print('STM32 串口打不开（',link.err,'），go/drive 不能用，scan/second 照常。',flush=True);link=None
    gyro=link if link is not None else (GyroReader(args.gyro_port) if (args.gyro_port!='none' and not args.sim) else None)
    if gyro is not None and not gyro.ok:print('陀螺仪串口打不开（',gyro.err,'），第二站角度按设定60°计算',flush=True)
    state=dict(config=config,busy=False,worker=None,path=[],reason='NOT SCANNED',goal=POINTS['QR_SCAN'],yaw_scan1=None,
               legs=[],plan_start=None,calib=None,mode='scan',done_count=0,err_count=0,confirm=None,abort=False,auto=False)
    rel=raw_cfg.get('second_relative_moves',{'left_mm':42,'forward_mm':83,'cw_deg':60})
    sim_err=raw_cfg.get('sim_second_pose_error',[0,0,0])
    def mission_cfg():
        mc=dict(raw_cfg);mc.update(car_length_mm=config.get('car_length_mm',290),car_width_mm=config.get('car_width_mm',260))
        return mc
    def plan_route():
        if not history.views:
            state['legs'],state['path'],state['reason']=[],[],'NOT SCANNED';return
        c=state['config']
        d=float(raw_cfg.get('turn_pivot_back_mm',0))
        state['start_pivot']=None
        rf=state.get('replan_from')
        if rf is not None:                     # v14：站点扫描发现新障碍物：从这个停车点规划剩下的路线
            state['plan_start']=(float(rf[0][0]),float(rf[0][1]),float(rf[0][2]));state['turn_back']=0
        if state['plan_start'] is None:
            # 车现在的车头可能是斜的(第二站)：先绕转轴原地转回起点车头方向，转轴不动
            yaw_now=float(c['car_yaw_deg']);yaw0=float(history.views[0]['config']['car_yaw_deg'])
            lft=0.0
            a=math.radians(yaw_now);ux,uy=math.cos(a),math.sin(a)
            px=c['car_x_mm']-d*ux-lft*uy;py=c['car_y_mm']-d*uy+lft*ux
            b=math.radians(yaw0);ux,uy=math.cos(b),math.sin(b)
            state['plan_start']=(px+d*ux+lft*uy,py+d*uy-lft*ux,yaw0)
            state['start_pivot']=(px,py)
            state['turn_back']=((yaw0-yaw_now+180)%360)-180
        # v14：看得不清楚的圆柱(点少/帧少)位置可能偏好几厘米，规划时把它当成更大的圆柱，路线离它更远
        weak_extra=float(raw_cfg.get('weak_obstacle_extra_mm',80))
        def is_weak(o):
            return o.get('seen',99)<int(raw_cfg.get('weak_seen',3)) or o.get('count',99)<int(raw_cfg.get('weak_count',6))
        weak=[o for o in history.objects if is_weak(o)]
        if weak:print('  看得不清楚的圆柱(规划时多留%.0fmm)：'%weak_extra+'，'.join(f"C{o['id']}({o['x']:.0f},{o['y']:.0f}) {o.get('count','?')}点/{o.get('seen','?')}帧" for o in weak),flush=True)
        t_plan=time.monotonic()
        mission=[('START2' if raw_cfg.get('start_zone',2)==2 else 'START1') if s=='START' else s for s in raw_cfg.get('mission',[])]
        if rf is not None:mission=list(rf[1])
        state['sealed']=[];state['lane_fallback']=False
        try:
            # v14：先按大余量规划(离障碍物远，绕过去)；规划不出来再逐步减小余量
            base=float(raw_cfg.get('margin_mm',60))
            tries=[(base,weak_extra)]+[(m,e) for m,e in ((40,40),(20,0)) if m<base]
            for k,(mg,ex) in enumerate(tries):
                obs=[(o['x'],o['y'],o['radius']+(ex if is_weak(o) else 0)) for o in history.objects]
                legs,why,pl_=plan_mission(dict(mission_cfg(),margin_mm=mg),obs,state['plan_start'],mission,start_pivot=state.get('start_pivot'))
                if why=='OK':
                    if k:print(f'  ⚠ 按障碍物余量{base:.0f}mm规划不出来，降到{mg:.0f}mm(看不清的圆柱多留{ex:.0f}mm)才规划出来',flush=True)
                    break
            state['strafe_fallback']=bool(getattr(pl_,'fallback',False))
            state['lane_fallback']=bool(getattr(pl_,'lane_fallback',False))
            state['sealed']=[] if state['lane_fallback'] else list(getattr(pl_,'sealed',[]))   # v11：整段封死的车道
            state['box']=getattr(pl_,'box',None)                                                 # v13：车身外框(含雷达)
        except ValueError as e:
            legs,why=[],str(e)
        state['legs']=legs;state['reason']='ROUTE OK' if why=='OK' else why
        print(f'  规划用时 {time.monotonic()-t_plan:.1f} 秒',flush=True)
        pts=[state['plan_start'][:2]]
        for L in legs:pts+=[p[:2] for p in L['poses']]
        state['path']=pts if legs else []
    def print_route():
        if not state['legs']:print('没有路线：',state['reason'],flush=True);return
        names={'F':('前进','后退'),'S':('左移','右移')}
        ps=state['plan_start'];tb=state.get('turn_back',0) or 0
        st=route_stats(state['legs'],raw_cfg)
        print(f'全程路线（转弯绕车中心(转轴偏后{raw_cfg.get("turn_pivot_back_mm",0)}mm)；预计行驶 {st["time"]:.0f} 秒：前进/后退 {st["F"]}mm，横移 {st["S"]}mm，转弯 {st["turns"]} 次）：',flush=True)
        if state.get('box'):
            b_=state['box'];print(f'  规划用的车身外框(含雷达)：前后各{b_[1]:.0f}mm，右边{-b_[2]:.0f}mm，左边(雷达那边){b_[3]:.0f}mm（从车中心算）',flush=True)
        if state.get('sealed'):
            print('  有障碍物、整段封死不走的车道：'+'，'.join(f'x{r[0]}~{r[2]} y{r[1]}~{r[3]}' for r in state['sealed']),flush=True)
        if state.get('lane_fallback'):
            print('  ⚠ 把有障碍物的车道整段封死以后到不了，已改成只绕开障碍物本身：路线会从障碍物旁边过，按 y 之前看清楚！',flush=True)
        if abs(tb)>=0.5:print(f"  先原地{'逆时针' if tb>0 else '顺时针'}转{abs(tb):.0f}°，车头回到{ps[2]:.0f}°，车中心到 ({ps[0]:.0f},{ps[1]:.0f})",flush=True)
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
        for o in history.objects:
            x,y,r=o['x'],o['y'],o['radius'];dot=Circle((x,y),r,color='black',zorder=6);world.add_patch(dot);markers.append(dot)
            half=max(body_half_bound(c),float(raw_cfg.get('plan_body_mm',300) or 0)/2)+c['margin_mm']+r
            pad=Rectangle((x-half,y-half),2*half,2*half,fc='red',alpha=.10,ec='none');world.add_patch(pad);markers.append(pad)
            markers.append(world.annotate(f"C{o['id']} ({x:.0f},{y:.0f}) n={o['count']} seen={o['seen']}"+(' ?' if o.get('ambiguous') else ''),(x,y),xytext=(4,5),textcoords='offset points',fontsize=7))
        # v12：只有两次扫描都做完才自动规划(第一次扫描后规划全程没有意义，还白白多花几秒)；地图没变就不重新规划
        if len(history.views)>=2:
            key=(len(history.views),tuple((round(o['x']),round(o['y'])) for o in history.objects),state['plan_start'])
            if key!=state.get('plan_key'):
                plan_route();state['plan_key']=key
        elif history.views:
            state['legs'],state['path'],state['reason']=[],[],'ONE SCAN (route after 2nd scan)'
        else:
            plan_route()
        route.set_data(*zip(*state['path'])) if state['path'] else route.set_data([],[])
        for r in (state.get('sealed') or []):      # v11：整段封死的车道
            box=Rectangle((r[0],r[1]),r[2]-r[0],r[3]-r[1],fc='red',alpha=.18,ec='red',hatch='xx',lw=1,zorder=3)
            world.add_patch(box);markers.append(box)
        world.set_title(state['reason'],fontsize=10,color='red' if state['reason']!='ROUTE OK' else 'black')
        info.set_text(f"STOPPED views={len(history.views)} | candidates={len(history.objects)} | pose=({c['car_x_mm']:.0f}, {c['car_y_mm']:.0f}, {c['car_yaw_deg']:.1f} deg)\nOriginal v4 exclusion regions retained. Unobserved space is UNKNOWN.")
        fig.canvas.draw_idle()
    def save():
        if state['busy']:raise ValueError('请等待扫描完成')
        out=ROOT/'map_captures';out.mkdir(exist_ok=True)
        base=out/('merged_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        payload=dict(reference_correction=metadata(),route_start=state['plan_start'],route_status=state['reason'],
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
    def do_home_fix(link_,log=lambda m:print(m,flush=True),move=True,init=None,order=('S','F'),max_move=None,label='回家对准'):
        """v12/v14 回家对准：把现在扫到的一整圈点和出发时第一次扫描对齐(ICP)，算出车离出发位置差多少，小步修回去。
        init=(角度,前,左)：按路线推算的车位(相对出发位置)，车离启停区还有几十厘米时从这里开始对齐(v14 回家前定位)。
        order：平移的先后顺序(和原路线最后几步一样，免得压到别的地方)。max_move：第一步平移超过它就不信、不走。
        对不上(场地里变化太大、点太少)就不修，返回 False；在误差内/修好了返回 True。"""
        from home_fix import body_points,icp_align
        if not raw_cfg.get('home_fix_enabled',True):
            log(f'    {label}：配置里关掉了(home_fix_enabled=false)，不做');return False
        if SIM is not None:
            log(f'    {label}：桌面模拟不做');return False
        if not history.views or not history.views[0].get('raw_frames'):
            log(f'    {label}：没有出发时的扫描数据，不做');return False
        ref=history.views[0];c0=ref['config']
        refP=body_points(ref['raw_frames'],c0)
        tol=float(raw_cfg.get('home_fix_tol_mm',6));tol_a=float(raw_cfg.get('home_fix_tol_deg',1.0))
        rpm=int(raw_cfg.get('home_fix_rpm',60));rounds=int(raw_cfg.get('home_fix_rounds',3))
        sp_big={'F':int(raw_cfg.get('route_speed_rpm',150)),'S':int(raw_cfg.get('strafe_speed_rpm',140))}
        homed=False
        for rnd in range(1,rounds+2):              # 最多修 rounds 次，最后再测一次确认
            if state['abort']:log(f'      收到 abort，{label}停止');return False
            t0=time.monotonic()
            m,_=acquire(c0,args.port,args.model,stop,frames=int(raw_cfg.get('home_fix_frames',12)),settle_s=0.4,quiet=True)
            first=(rnd==1 and init is not None)
            res=icp_align(body_points(m.raw_frames,c0),refP,init=init if first else None)
            fx,ly,th=res['tx'],res['ty'],res['theta_deg']
            log(f"    {label}(第{rnd}次)：离出发位置 前后{fx:+.0f}mm 左右{ly:+.0f}mm 车头{th:+.1f}°"
                +(f"（推算是 前后{init[1]:+.0f} 左右{init[2]:+.0f} 车头{init[0]:+.1f}°）" if first else '')
                +f"（{res['inlier']*100:.0f}%的点对上，残差{res['rms']:.1f}mm，{time.monotonic()-t0:.1f}秒）")
            if not res['ok']:
                log(f"      和出发时的扫描对不上：{res['why']}。不按它修。");return False
            if abs(fx)<tol and abs(ly)<tol and abs(th)<tol_a:
                log('      已经在误差范围内。');return True
            if not move:return False
            if rnd>rounds:
                log(f'      修了{rounds}次还没进误差范围，停在这里。');return False
            if max_move is not None and rnd==1 and (abs(fx)>max_move or abs(ly)>max_move):
                log(f'      要走的距离超过 {max_move:.0f}mm，不像是对的，不按它走。');return False
            if not homed:
                ok_,rep_=link_.home()                  # 把现在的车头记为要保持的方向，后面的转弯、平移都从这里算
                if not ok_:log(f'      HOME 失败：{rep_}，不修');return False
                homed=True
            moves=[]
            if abs(th)>=tol_a:moves.append(('R',int(round(-th))))
            tr={'S':int(round(-ly)) if abs(ly)>=tol else 0,'F':int(round(-fx)) if abs(fx)>=tol else 0}
            for k_ in (list(order)+[q for q in ('S','F') if q not in order]):
                if tr.get(k_):moves.append((k_,tr[k_]))
            log('      修正：'+'，'.join({'R':'转','S':'横移','F':'前后'}[k_]+f' {v_:+d}' for k_,v_ in moves))
            for k_,v_ in moves:
                if v_==0:continue
                if state['abort']:log(f'      收到 abort，{label}停止');return False
                spd=None if k_=='R' else (sp_big[k_] if abs(v_)>150 else rpm)
                ok_,rep_=link_.move(k_,v_,spd)
                if not ok_:log(f'      {k_} {v_} 失败：{rep_}');return False
        return False
    class Ctx:
        def home_fix(self,link_,log):return do_home_fix(link_,log)
        def home_approach(self,link_,log,plan_pose,order,max_move):
            """v14：离启停区还有几十厘米时停下，用雷达定位，按实际位置走进启停区(不再照原路线最后几步走)。"""
            if not history.views:return False
            c0=history.views[0]['config'];x0,y0,p0=c0['car_x_mm'],c0['car_y_mm'],c0['car_yaw_deg']
            dx,dy=plan_pose[0]-x0,plan_pose[1]-y0;a=math.radians(-p0)
            init=(((plan_pose[2]-p0)+180)%360-180,dx*math.cos(a)-dy*math.sin(a),dx*math.sin(a)+dy*math.cos(a))
            return do_home_fix(link_,log,move=True,init=init,order=order,max_move=max_move,label='回家前定位')
        def station_scan(self,stop_name,pose,remaining):
            """v14：在停车点上再扫一次。发现新的障碍物就加进地图，从这里重新规划剩下的路线。"""
            state['station_req']=(stop_name,tuple(float(v) for v in pose[:3]),list(remaining));state['station_result']=None
            state['err_before']=state['err_count'];before=state['done_count']
            commands.put('__station')
            try:self._wait('done_count',before,timeout=180.0)
            except Abort:
                if state['abort']:raise
                print('  站点扫描失败(见上面的提示)，按原路线继续。',flush=True);return None
            return state['station_result']
        def _wait(self,key,before,timeout=60.0):
            t=time.monotonic()
            while time.monotonic()-t<timeout:
                if state['abort']:raise Abort('收到 abort')
                if state['err_count']!=state['err_before']:raise Abort('扫描失败，见上面的提示')
                if state[key]!=before and not state['busy']:return
                time.sleep(0.1)
            raise Abort('等待扫描超时')
        def scan(self):
            state['err_before']=state['err_count'];before=state['done_count']
            commands.put('scan');self._wait('done_count',before)
        def second(self):
            state['err_before']=state['err_count'];before=state['done_count']
            commands.put('second');self._wait('done_count',before)
        def plan(self):
            return state.get('turn_back',0) or 0,list(state['legs']),state['reason']
        def ask(self,msg):
            print(msg,flush=True);state['confirm']=None
            while state['confirm'] is None:
                if state['abort']:return False
                time.sleep(0.1)
            return bool(state['confirm'])
        def aborted(self):return state['abort']
    def save_params(updates):
        """把运动参数存进配置的 stm32_params(第一次改之前把原配置备份成 .json.bak)。树莓派每次 go/drive 都会把它们发给 STM32。"""
        p=Path(args.config)
        bak=p.with_suffix('.json.bak')
        if not bak.exists():bak.write_text(p.read_text(encoding='utf-8'),encoding='utf-8')
        sp_=dict(raw_cfg.get('stm32_params',{}));sp_.update(updates);raw_cfg['stm32_params']=sp_
        p.write_text(json.dumps(raw_cfg,indent=2,ensure_ascii=False),encoding='utf-8')
    synced=[False]
    def sync_now(force=False):
        """把配置里的 stm32_params 发给 STM32(STM32 断电重启后参数会回到编译进去的默认值)。每次运行程序第一次动车前自动做一次。"""
        if link is None or (synced[0] and not force):return
        sp_=raw_cfg.get('stm32_params') or {}
        bad=link.sync_params(sp_,log=lambda m:print(m,flush=True))
        synced[0]=True
        if sp_:print(f'已把配置里的 {len(sp_)} 个运动参数发给 STM32'+(f'（{len(bad)} 个失败）' if bad else '')+'。',flush=True)
    def mission_thread(first_scan):
        hooks=None
        try:
            if (raw_cfg.get('mission_cfg') or {}).get('enabled',False):
                # 机械臂任务钩子：QR 读码、RAW 抓取、ROUGH/TEMP 放置、START 显示统计(见 mission_hooks.py)；计时从这里开始
                from mission_hooks import MissionHooks
                try:
                    from mission_cli import release
                    release()                                   # 测试命令(mtest/vcal…)占着的摄像头先放掉
                except Exception:pass
                hooks=MissionHooks(raw_cfg,log=lambda m:print(m,flush=True))
                print('已启用机械臂任务钩子(mission_cfg.enabled=true)。',flush=True)
            run_mission(Ctx(),link,log=lambda m:print(m,flush=True),first_scan=first_scan,stop_wait=args.stop_wait,hooks=hooks,cfg=raw_cfg)
        except Abort as e:print('★ 已停止：',e,flush=True)
        except Exception as e:print('★ 出错停止：',repr(e),flush=True)
        finally:
            if hooks is not None:
                try:hooks.close()                               # 释放摄像头，下一次 go 才能再打开
                except Exception:pass
            state['auto']=False;state['replan_from']=None;state['plan_start']=None
    def command(line):
        import unicodedata
        line=unicodedata.normalize('NFKC',line).replace('\u3000',' ')   # 中文输入法打出的全角字母/空格也能认
        parts=line.strip().split();k=parts[0].lower() if parts else 'scan'
        if k=='q':plt.close(fig);return
        if k in ('y','yes'):state['confirm']=True;return
        if k in ('n','no'):state['confirm']=False;return
        if k=='abort':
            state['abort']=True
            if link is not None:link.abort()
            print('收到 abort：已给 STM32 发急停(!)，路线停止发送。',flush=True);return
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
        if k=='__station':
            # v14：go 跑到停车点后由行驶线程发来；车位按这个停车点算(不用雷达改车位)
            stop_,pose_,rem_=state['station_req']
            c=copy.deepcopy(state['config']);c.update(car_x_mm=pose_[0],car_y_mm=pose_[1],car_yaw_deg=pose_[2])
            print(f'站点扫描({stop_})：车停稳，扫一圈看有没有之前没看到的障碍物……',flush=True)
            launch(c,'station_'+stop_);return
        if k=='home':
            # v12：回家对准测试。车停在出发扫描过的地方附近，home = 测出偏差并小步修回去；home check = 只测不动
            if link is None:raise ValueError('没有连接 STM32（--stm-port）')
            if state['auto'] or state['busy']:raise ValueError('正在运行或扫描，等结束再用')
            if not history.views:raise ValueError('先在出发位置 scan 一次(作为对准的基准)，再挪车测试')
            chk=len(parts)>1 and parts[1].lower() in ('check','c','test')
            state['busy']=True;state['abort']=False
            def hworker():
                try:do_home_fix(link,move=not chk)
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
        if k in ('arm','vcal','vclaw','vmask','mtest','vdbg','qr','mcode','mot','gtest','rtest'):
            # 机械臂/视觉测试命令、mot(看电机驱动器状态)，见 mission_cli.py 开头的说明
            # STM32 重启(断电、重新烧录)后参数回到编译进去的默认值：第一次用这些命令前也把配置里存的参数(SPOW、CLWO…)发一遍
            sync_now()
            from mission_cli import handle_cli
            handle_cli(k,parts,link=link,raw_cfg=raw_cfg,state=state,log=lambda m:print(m,flush=True));return
        if k in ('go','drive','p2'):
            if link is None:raise ValueError('没有连接 STM32（--stm-port），不能用 '+k)
            if state['auto']:raise ValueError('一键流程正在运行；输入 abort 可以中止')
            if state.get('busy'):raise ValueError('测试命令(vclaw/gtest/vcal…)还在运行，等它结束再用 '+k)
            if k=='p2':
                sync_now()
                print('第二站动作 ->',link.second_move(None if raw_cfg.get('second_use_p2',False) else rel,log=lambda m:print(m,flush=True)),flush=True);return
            if k=='go' and history.views:raise ValueError('go 要从头开始：先 reset 清空历史，并把车放回起点')
            if k=='drive' and len(history.views)<2:raise ValueError('drive 只按已规划好的路线行驶，先完成两次扫描')
            state['abort']=False;state['confirm']=None;state['auto']=True
            threading.Thread(target=mission_thread,args=(k=='go',),daemon=True).start();return
        if k in ('3','4','5','6'):
            if state['busy']:raise ValueError('请等待扫描完成')
            state['goal']=POINTS[{'3':'QR_SCAN','4':'RAW_STOP','5':'TEMP_STOP','6':'ROUGH_STOP'}[k]];redraw();return
        if k in ('save','s'):save();return
        if state['busy']:raise ValueError('正在扫描，请等待完成')
        if k=='reset':
            history.objects.clear();history.views.clear();state['plan_start']=None;state['replan_from']=None;redraw();print('已清空历史；scan 在当前已知车位重新扫描。',flush=True);return
        if k=='merge' and len(parts)==2:
            history.set_radius(float(parts[1]));print('去重距离：',history.merge_radius_mm,'mm',flush=True);return
        if k=='list':print_objects();return
        if k=='pts':
            if not history.views:raise ValueError('先扫描')
            for vi,vw in enumerate(history.views,1):
                rep=[];accumulated_detections(vw.get('raw_frames',[]),vw['config'],report=rep)
                print(f'第{vi}次扫描：场地内回波 {sum(r["count"] for r in rep)} 个点，{len(rep)} 团',flush=True)
                for r in sorted(rep,key=lambda r:-r['count']):
                    print(f"  ({r['x']:.0f},{r['y']:.0f}) 离雷达{r['dist']:.0f}mm  {r['count']}点/{r['frames']}帧  "+('→ 认作障碍物' if not r['why'] else '→ 没认：'+r['why']),flush=True)
            return
        if k=='route':plan_route();state['plan_key']=None;redraw();print_route();return
        if k=='pose' and len(parts)==4:
            if not history.views:raise ValueError('先扫描再设置规划起点')
            state['plan_start']=tuple(float(v) for v in parts[1:]);state['turn_back']=0;redraw();print_route();return
        if k=='calib':
            if len(parts)==2 and parts[1].lower()=='save':
                if not state['calib']:raise ValueError('还没有标定结果，先 calib X Y ...')
                bak=Path(args.config).with_suffix('.json.bak');bak.write_text(Path(args.config).read_text(encoding='utf-8'),encoding='utf-8')
                raw_cfg.update({k2:state['calib'][k2] for k2 in ('lidar_yaw_deg','lidar_forward_mm','lidar_left_mm')})
                Path(args.config).write_text(json.dumps(raw_cfg,indent=2,ensure_ascii=False),encoding='utf-8')
                state['config']=validate_config(dict(state['config'],**{k2:state['calib'][k2] for k2 in ('lidar_yaw_deg','lidar_forward_mm','lidar_left_mm')}))
                print('已写入配置（原文件备份为 .json.bak）。请 q 退出后重新 bash run_live.sh 再扫描。',flush=True);return
            if history.views:raise ValueError('标定要在第一次 scan 之前、车停在起点时做；已扫描请先 reset')
            if len(parts)<3 or len(parts)%2!=1:raise ValueError('格式：calib X1 Y1 [X2 Y2 ...]（圆柱的真实地图坐标mm）')
            state['calib_known']=[(float(parts[i]),float(parts[i+1])) for i in range(1,len(parts),2)]
            print('标定扫描：车停在起点不要动……',flush=True)
            launch(state['config'],'calibration','calib');return
        if k=='second':
            if len(history.views)!=1:raise ValueError('second 要求已有且仅有第一次扫描')
            if len(parts)==1:
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
            elif len(parts)==5:
                c=second_scan_config(history.views[0]['config'],float(parts[1]),float(parts[2]),parts[3],float(parts[4]))
            else:raise ValueError('格式：second X Y cw/ccw/yaw 角度，例如 second 2120 200 cw 60')
            print('第二次车位：',c['car_x_mm'],c['car_y_mm'],c['car_yaw_deg'],'度（不含自动校正）',flush=True)
            launch(c,'user_entered_second_absolute_xy_and_heading');return
        if k=='scan':launch(state['config'],'unchanged_external_pose');return
        if k=='r' and len(parts)==4:
            if not history.views:raise ValueError('请先完成第一次扫描')
            launch(moved_config(state['config'],*map(float,parts[1:])),'user_entered_displacement');return
        if k=='a' and len(parts)==4:
            if history.views:raise ValueError('修改初始车位前请先输入 reset 清空历史')
            c=copy.deepcopy(state['config']);c.update(zip(('car_x_mm','car_y_mm','car_yaw_deg'),map(float,parts[1:])))
            launch(c,'user_entered_absolute_pose');return
        raise ValueError('命令：go(一键全流程) / drive / abort / ping / mot(电机驱动器状态) / mot en / gtest 颜色号(夹取测试) / send F 300 / get / set 名字 数值 / sync / fix F 1000 985 / cal / p2 / calib X Y [X Y..] / calib save / scan / second / second X Y yaw 30 / route / pose X Y 角度 / list / pts / reset / save / q')
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
            if item[0]=='done' and str(item[3]).startswith('station_'):
                _,m,seq,source=item
                stop_,pose_,rem_=state['station_req']
                mr=float(raw_cfg.get('station_new_mm',200))
                seen=[d for d in m.latched]
                new=[d for d in seen if all(math.hypot(d['x']-o['x'],d['y']-o['y'])>mr for o in history.objects)]
                res=dict(stop=stop_,new=[(round(d['x']),round(d['y'])) for d in new],replanned=False,legs=[],reason='ROUTE OK')
                if not new:
                    print(f'  站点扫描({stop_})：看到 {len(seen)} 个圆柱，都是已知的，按原路线继续。',flush=True)
                else:
                    print(f'  站点扫描({stop_})：发现 {len(new)} 个新障碍物：'+'，'.join(f"({d['x']:.0f},{d['y']:.0f}) {d.get('count','?')}点/{d.get('seen','?')}帧" for d in new)+'。加进地图，从这里重新规划剩下的路线……',flush=True)
                    history.commit(m,source,seq)
                    state['replan_from']=(tuple(pose_),list(rem_));state['plan_start']=None
                    redraw();print_route()
                    res.update(replanned=True,legs=list(state['legs']),reason=state['reason'])
                    try:save()
                    except Exception as e:print('保存失败：',e,flush=True)
                state['station_result']=res;state['done_count']+=1
                continue
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
                if len(history.views)>=2:print_route()
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
    print(f"起点：车中心=({config['car_x_mm']}, {config['car_y_mm']})，车头 {config['car_yaw_deg']}°（90=朝北）；雷达 前{config['lidar_forward_mm']} 左{config['lidar_left_mm']}mm。",flush=True)
    print('（可选）标定：起点放好车，在雷达看得到的一侧放已知圆柱，输入 calib X1 Y1 X2 Y2 …，满意后 calib save。',flush=True)
    print('一键：车放起点停稳，输入 go（扫描→STM32走第二站→再扫描→规划→确认后按路线行驶）。',flush=True)
    print('分步：1) 车停在起点输入 scan   2) 车自己走完 左移/前进/顺时针 后停稳，输入 second',flush=True)
    print('3) 自动用共同障碍物校正第二站，合并后显示全程路线；route 重新打印，pose X Y 角度 改规划起点。q退出。',flush=True)
    print('v14：到 QR 停车点再扫一次(有新障碍物就重新规划)；回启停区前先用雷达定位，再按实际位置走进去。',flush=True)
    print('v12：回到启停区后自动“回家对准”(和出发时的扫描对齐，小步修回去)。测试：出发位置 scan → 手动挪车 → home(修) 或 home check(只测)。',flush=True)
    print('v10 运动调试：send F 1000 = 单条指令；fix F 1000 985 = 距离校准；cal = 横移校准；get / set 名字 数值 = 看/改 STM32 参数；sync = 把配置里的参数重新发给 STM32；abort = 急停。',flush=True)
    if args.sim:print('【桌面模拟模式】',flush=True)
    threading.Thread(target=terminal,daemon=True).start();redraw()
    try:timer.start();plt.show()
    except KeyboardInterrupt:pass
    finally:
        stop.set();timer.stop()
        if state['worker'] is not None:state['worker'].join(timeout=6)
        close_stream()
    return 0
if __name__=='__main__':raise SystemExit(main())
