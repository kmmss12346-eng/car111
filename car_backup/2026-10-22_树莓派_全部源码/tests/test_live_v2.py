"""Reproducible synthetic checks: no physical lidar needed."""
import json, math, os, sys, tempfile, time
from pathlib import Path
os.environ['MPLBACKEND']='Agg'
PACKAGE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PACKAGE))
import numpy as np
from obstacles import detect, Tracker
import lidar_map_live as app

def scene(centers, radius=25):
    result=[]
    for a in np.arange(0,360,.54):
        u=np.array([math.cos(math.radians(a)),math.sin(math.radians(a))])
        distances=[]
        for center in centers:
            c=np.array(center);along=u@c;disc=along**2-(c@c-radius**2)
            if disc>=0 and along>0: distances.append(along-math.sqrt(disc))
        result.append([float(a),float(min(distances)) if distances else 0,220,0])
    return result

c=dict(pose_and_mount_measured=True,car_x_mm=1200,car_y_mm=1200,car_yaw_deg=0,
       lidar_forward_mm=0,lidar_left_mm=0,lidar_yaw_deg=0,sdk_y_sign=1,cylinder_diameter_mm=50,margin_mm=20)
m=app.Mapping(c)
def frame(centers,n):return dict(points=scene(centers),seq=n,hz=6)
for n in range(1,4):m.ingest(frame([(500,0)],n),n)
assert len(m.current)==1 and m.current[0]['stable']
old_id=m.current[0]['id']
assert abs(m.current[0]['map_x']-1700)<2
# Movement larger than association gate must still display immediately.
m.ingest(frame([(700,150)],4),4)
assert len(m.current)==1 and abs(m.current[0]['map_x']-1900)<2
assert abs(m.current[0]['map_y']-1350)<2
assert m.current[0]['id']!=old_id and not m.current[0]['stable']
# Duplicate GUI refreshes cannot accumulate confirmations.
hits=m.current[0]['hits'];m.ingest(frame([(700,150)],4),4)
assert m.current[0]['hits']==hits
m.ingest(frame([],5),5)
assert m.current==[] and m.planner_obstacles==[] and len(m.points)==0
# ROI: outside field and inside body excluded; edges retained.
kept=app.field_points([(-1300,0),(0,0),(1200,0),(-1200,0),(0,1201)],c)
assert np.allclose(kept,[[2400,1200],[0,1200]])
# Calibration agreed with user's A/B measurements.
preset=json.loads((PACKAGE/'map_config_start2_initial.json').read_text())
assert np.allclose(app.to_map([[-306,-262],[-174,-409]],preset),[[1702.16,150],[1706.82,347.51]],atol=.1)
# 2-point far returns shown as surface only, not fake circle center.
pts=[[math.degrees(math.atan2(y,2000)),math.hypot(2000,y),200,0] for y in (-10,10)]
d=detect(pts,50);assert len(d)==1 and d[0]['sparse'] and d[0]['kind']=='surface_cluster'
assert abs(d[0]['x']-2000)<1e-6
pts80=[[math.degrees(math.atan2(y,2000)),math.hypot(2000,y),200,0] for y in (-40,40)]
assert not detect(pts80,50)
assert not detect([[float('nan'),500,0,0],[0,float('inf'),0,0]],50)
# Scan may have only one return: keep raw display, no invented candidate.
assert not detect([[0,2000,200,0]],50)
# Circle reception beyond 800 display range still works.
assert detect(scene([(1000,0)]),50)
print('PASS moving target / removal / repeated frame / ROI / calibration / sparse distinction / 800 not a detector limit')

import matplotlib.pyplot as plt
from matplotlib.backend_bases import KeyEvent
from matplotlib.backends.backend_agg import FigureCanvasAgg
class Proc:
    def poll(self):return None
class Stream:
    instance=None
    def __init__(self,*a):
        self.proc=Proc();self.count=1;self.stale=False;self.centers=[(500,0)];self.closed=False;Stream.instance=self
    def snapshot(self):return frame(self.centers,self.count),time.monotonic()-(2 if self.stale else 0),self.count
    def close(self):self.closed=True
class Timer:
    instance=None
    def __init__(self,*a,**k):self.fn=None;Timer.instance=self
    def add_callback(self,fn):self.fn=fn
    def start(self):pass
    def stop(self):pass
with tempfile.TemporaryDirectory() as temp:
    app.ROOT=Path(temp);(app.ROOT/'scan_bridge').touch()
    conf=app.ROOT/'map_config_start2_initial.json';conf.write_text(json.dumps(c))
    sys.argv=['lidar_map_live.py','--port','/dev/null','--config',str(conf)]
    app.Stream=Stream;FigureCanvasAgg.new_timer=Timer
    def show():
        f=plt.gcf();s=Stream.instance;world=f.axes[1]
        def labels():return [a.get_text() for a in world.texts if a.get_text().startswith('C')]
        assert len(labels())==1
        s.centers=[(700,150)];s.count+=1;Timer.instance.fn()
        assert len(labels())==1 and '(1900,1350)' in labels()[0],labels()
        s.centers=[];s.count+=1;Timer.instance.fn();assert not labels()
        s.centers=[(500,0),(0,500)];s.count+=1;Timer.instance.fn();assert len(labels())==2
        # Saving uses a new current snapshot, never stale latched list.
        f.canvas.callbacks.process('key_press_event',KeyEvent('key_press_event',f.canvas,key='s'))
        payload=json.loads(next((app.ROOT/'map_captures').glob('*.json')).read_text())
        assert len(payload['current_detections'])==2 and payload['drive_ready'] is False
        f.suptitle('SYNTHETIC REGRESSION TEST - NOT HARDWARE',color='red')
        f.savefig(app.ROOT/'synthetic_preview.png',dpi=100)
        s.stale=True;Timer.instance.fn();assert not labels()
        assert 'PATH WITHDRAWN' in world.get_title()
        assert len(world.collections[0].get_offsets())==0
        s.stale=False;s.count+=1;Timer.instance.fn();assert len(labels())==2
        f.canvas.callbacks.process('key_press_event',KeyEvent('key_press_event',f.canvas,key='m'))
        assert not labels() and 'CONFIG REQUIRED' in world.get_title()
        plt.close(f)
    plt.show=show
    assert app.main()==0 and Stream.instance.closed
print('PASS actual GUI callbacks: movement / disappearance / export / stale clear / resume / pose invalidation')
