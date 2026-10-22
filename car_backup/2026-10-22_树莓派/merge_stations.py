"""Associate stopped observations in a fixed field; never estimate vehicle pose."""
import copy
import math
import numpy as np

def moved_config(config,left,forward,cw):
    from lidar_map_live import validate_config
    if not all(math.isfinite(v) for v in (left,forward,cw)):
        raise ValueError('位移和角度必须是有限数值')
    c=copy.deepcopy(config);a=math.radians(c['car_yaw_deg'])
    c['car_x_mm']+=forward*math.cos(a)-left*math.sin(a)
    c['car_y_mm']+=forward*math.sin(a)+left*math.cos(a)
    # 原地转：车绕“车中心往后 turn_pivot_back_mm”的点转（实测），不是绕车中心
    d=float(c.get('turn_pivot_back_mm',0))
    px=c['car_x_mm']-d*math.cos(a);py=c['car_y_mm']-d*math.sin(a)
    c['car_yaw_deg']=(c['car_yaw_deg']-cw+180)%360-180
    b=math.radians(c['car_yaw_deg'])
    c['car_x_mm']=px+d*math.cos(b);c['car_y_mm']=py+d*math.sin(b)
    return validate_config(c)

def second_scan_config(initial, x=2120., y=200., direction='cw', degrees=60.):
    """Absolute chassis XY, relative turn from first scan heading."""
    from lidar_map_live import validate_config
    if direction.lower() not in ('cw','ccw','yaw'):
        raise ValueError('方向填 cw 顺时针、ccw 逆时针，或 yaw 绝对朝向')
    if not all(math.isfinite(v) for v in (x,y,degrees)):
        raise ValueError('坐标和角度必须是有限数值')
    if direction.lower()!='yaw' and not 0<=degrees<=360:
        raise ValueError('旋转角度必须在0到360度之间')
    c=copy.deepcopy(initial)
    yaw=degrees if direction.lower()=='yaw' else c['car_yaw_deg']+(-degrees if direction.lower()=='cw' else degrees)
    c.update(car_x_mm=x,car_y_mm=y,car_yaw_deg=(yaw+180)%360-180)
    return validate_config(c)

class History:
    def __init__(self, merge_radius_mm=200.):
        self.objects=[];self.views=[]
        self.set_radius(merge_radius_mm)
    def set_radius(self,value):
        if not math.isfinite(value) or not 0<value<=500:
            raise ValueError('去重距离需大于0且不超过500mm')
        if self.views:raise ValueError('请在第一次扫描前设置去重距离')
        self.merge_radius_mm=float(value)
    def commit(self,mapping,source,seq):
        incoming=copy.deepcopy(mapping.latched)
        # Existing map anchors never move. Within a new view, prefer strongest evidence.
        ordered=sorted(incoming,key=lambda d:(-d.get('seen',0),-d.get('count',0),d['x'],d['y']))
        associations=[]
        for d in ordered:
            obs=copy.deepcopy(d)
            near=[o for o in self.objects if math.hypot(d['x']-o['x'],d['y']-o['y'])<=self.merge_radius_mm]
            if near:
                o=min(near,key=lambda o:math.hypot(d['x']-o['x'],d['y']-o['y']))
                o['observations'].append(obs)
                o['seen']+=d.get('seen',0)
                if len(near)>1:o['ambiguous']=True
                associations.append(dict(anchor_id=o['id'],suppressed=obs,ambiguous=len(near)>1))
            else:
                d.update(id=len(self.objects)+1,observations=[obs],ambiguous=False,anchor_view=len(self.views)+1)
                self.objects.append(d)
        self.views.append(dict(config=copy.deepcopy(mapping.c),source=source,scan_seq=seq,
                               raw_frames=copy.deepcopy(getattr(mapping,'raw_frames',[])),detections=incoming,field_returns=mapping.points.tolist(),associations=associations,
                               merge_radius_mm=self.merge_radius_mm))
