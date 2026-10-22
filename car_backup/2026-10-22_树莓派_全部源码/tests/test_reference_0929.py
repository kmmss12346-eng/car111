import sys, json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import reference_correction as rc
from lidar_map_live import field_points, rotate, roi_class
c=json.loads((ROOT/'map_config_start2_roi.json').read_text())
point=np.array([[625.,174.]])
assert rc.profile(c)=='first_translation'
assert np.allclose(rc.apply(point,c),[[470,255]])
assert np.allclose(rc.apply(point,dict(c,reference_correction_enabled=False)),point)
assert np.allclose(rc.apply(point,dict(c,car_x_mm=2249)),point)
assert np.allclose(rc.apply(point,dict(c,lidar_left_mm=150)),point)
c2=dict(c,car_x_mm=2186,car_y_mm=241,car_yaw_deg=120)
assert rc.profile(c2)=='second_rigid'
assert np.allclose(rc.apply(rc.OBSERVED,c2),rc.OBSERVED@rc.ROT.T+rc.SHIFT)
assert np.allclose(rc.apply(rc.OBSERVED,c2),[[1208.08404313,1200],[2391.91595687,1200]])
# Exercise the actual point pipeline, not just the correction helper.
def sdk_from_map(world):
    body=rotate(np.asarray(world)-[c['car_x_mm'],c['car_y_mm']],-c['car_yaw_deg'])
    sdk=rotate(body-[c['lidar_forward_mm'],c['lidar_left_mm']],-c['lidar_yaw_deg'])
    sdk[:,1]*=c['sdk_y_sign']
    return sdk
assert np.allclose(field_points(sdk_from_map(point),c),[[470,255]])
assert roi_class(470,255,c)=='interior'
assert len(field_points(sdk_from_map([[2250,150]]),c))==0
assert c['car_x_mm']==2250 and c['car_y_mm']==150
print('PASS: first reference, point pipeline, ROI, self filter, pose/mount guards, disable, second unchanged')
