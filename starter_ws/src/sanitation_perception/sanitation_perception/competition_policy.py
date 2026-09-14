"""Pure input/target guards for the development RGB-D node; no truth access."""
import json
import math


def thresholds(value, classes):
    result=json.loads(value)
    if set(result)!=set(classes) or any(type(v) not in (int,float) or not math.isfinite(v) or not 0<=v<=1 for v in result.values()):
        raise ValueError('one finite [0,1] threshold is required for every class')
    return result


def stamp(message):
    s=message.header.stamp
    if s.sec<0 or not 0<=s.nanosec<1_000_000_000:raise ValueError('invalid source stamp')
    return s.sec+s.nanosec*1e-9


def validate_context(rgb,depth,info,max_skew):
    if not math.isfinite(max_skew) or max_skew<=0:raise ValueError('positive RGB-D skew limit required')
    if stamp(rgb)<=0:raise ValueError('zero RGB stamp would request latest TF')
    if not rgb.header.frame_id or len({rgb.header.frame_id,depth.header.frame_id,info.header.frame_id})!=1:
        raise ValueError('registered RGB depth and calibration frames must match')
    if min(rgb.width,rgb.height)<=0 or (rgb.width,rgb.height)!=(depth.width,depth.height) or (rgb.width,rgb.height)!=(info.width,info.height):
        raise ValueError('registered RGB-D calibration dimensions must match')
    if abs(stamp(rgb)-stamp(depth))>max_skew:raise ValueError('RGB-depth timestamp skew')
    if stamp(info)!=0 and abs(stamp(rgb)-stamp(info))>max_skew:raise ValueError('RGB-calibration timestamp skew')
    if depth.encoding not in ('32FC1','16UC1'):raise ValueError('unsupported depth units')
    if len(info.k)!=9 or not all(math.isfinite(x) for x in info.k) or min(info.k[0],info.k[4])<=0:raise ValueError('invalid intrinsics')


def in_ground_roi(xyz,bounds):
    if len(bounds)!=6 or not all(math.isfinite(v) for v in bounds) or any(bounds[i]>=bounds[i+1] for i in (0,2,4)):
        raise ValueError('ROI requires ordered finite xmin xmax ymin ymax zmin zmax')
    return len(xyz)==3 and all(math.isfinite(v) and bounds[2*i]<=v<=bounds[2*i+1] for i,v in enumerate(xyz))


def publishable(state,allow_tentative=False):
    return state in {'CONFIRMED','QUEUED','APPROACHING','CLEANING'} or (allow_tentative and state=='TENTATIVE')
