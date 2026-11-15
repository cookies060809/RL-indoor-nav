"""Reference scan -> robot-centred BEV. Port this contract to the ROS C++ node."""
import math
import numpy as np
from scipy.ndimage import distance_transform_edt
from numba import njit
from .scan import normalize_for_raster

def canonicalize_scan(ranges, angle_min, angle_increment, beams=360):
    """Nearest-ray resampling to [-pi,pi); unobserved angular sectors stay NaN.

    Preserves NaN/inf semantics. Use the same mapping in the C++ node.
    """
    ranges=np.asarray(ranges,dtype=np.float32)
    if len(ranges)<2 or angle_increment<=0:
        raise ValueError('Expected at least two rays and positive angle_increment')
    targets=-np.pi+np.arange(beams)*2*np.pi/beams
    output=np.full(beams,np.nan,dtype=np.float32); best=np.full(beams,np.inf)
    for turn in (-2*np.pi,0,2*np.pi):
        idx=np.rint((targets+turn-angle_min)/angle_increment).astype(int)
        valid=(idx>=0)&(idx<len(ranges))
        indices=np.clip(idx,0,len(ranges)-1)
        error=np.abs(angle_min+indices*angle_increment-(targets+turn))
        use=valid&(error<=angle_increment*0.501)&(error<best)
        output[use]=ranges[indices[use]]; best[use]=error[use]
    return output

@njit(cache=True)
def rasterize(ranges, n, extent, rmin, rmax, sx, sy, syaw, radius):
    """Rasterize a classified scan: hit values or +inf; unknown rays are NaN."""
    grid=np.full((n,n),0.5,dtype=np.float32)
    res=extent/n; half=extent/2
    hits=np.zeros((n,n),dtype=np.uint8)
    # Slab clipping handles square corners and sensor offset correctly.
    for i in range(len(ranges)):
        value=ranges[i]
        if math.isnan(value) or value<rmin: continue
        hit=math.isfinite(value) and value<=rmax
        length=min(value,rmax) if hit else rmax
        angle=syaw-math.pi+i*2*math.pi/len(ranges)
        dx,dy=math.cos(angle),math.sin(angle)
        near,far=0.0,length
        for k in range(2):
            origin=sx if k==0 else sy
            direction=dx if k==0 else dy
            if abs(direction)<1e-12:
                if origin < -half or origin >= half: far=-1
            else:
                a,b=(-half-origin)/direction,(half-origin)/direction
                near=max(near,min(a,b)); far=min(far,max(a,b))
        if far<near: continue
        # Visit cells using Bresenham, endpoints are clamped to valid interior.
        x0=min(n-1,max(0,int(math.floor((sx+near*dx+half)/res))))
        y0=min(n-1,max(0,int(math.floor((sy+near*dy+half)/res))))
        x1=min(n-1,max(0,int(math.floor((sx+far*dx+half)/res))))
        y1=min(n-1,max(0,int(math.floor((sy+far*dy+half)/res))))
        ddx,ddy=abs(x1-x0),-abs(y1-y0)
        stepx=1 if x0<x1 else -1; stepy=1 if y0<y1 else -1
        err=ddx+ddy
        while True:
            grid[y0,x0]=0.0
            if x0==x1 and y0==y1: break
            twice=2*err
            if twice>=ddy: err+=ddy; x0+=stepx
            if twice<=ddx: err+=ddx; y0+=stepy
        ex,ey=sx+length*dx,sy+length*dy
        if hit and -half<=ex<half and -half<=ey<half:
            col,row=int(math.floor((ex+half)/res)),int(math.floor((ey+half)/res))
            hits[row,col]=1
    for row in range(n):
        for col in range(n):
            x,y=(col+0.5)*res-half,(row+0.5)*res-half
            if x*x+y*y<radius*radius and hits[row,col]==0:
                grid[row,col]=0.0
            if hits[row,col]: grid[row,col]=1.0
    return grid

def scan_to_bev(ranges, cfg):
    o,s=cfg['observation'],cfg['sensor']
    classified=normalize_for_raster(ranges,s)
    occupancy=rasterize(classified,o['size'],o['extent'],s['range_min'],s['range_max'],s['x'],s['y'],s['yaw'],cfg['robot']['radius'])
    free=occupancy==0
    if np.all(free):
        distance=np.ones_like(occupancy)
    else:
        distance=np.minimum(distance_transform_edt(free)*o['extent']/o['size']/o['distance_clip'],1).astype(np.float32)
    return np.stack([occupancy,distance])

def goal_features(local_goal_robot, cfg):
    dx,dy=np.asarray(local_goal_robot,dtype=float)
    d=float(np.hypot(dx,dy))
    if d<1e-8: return np.zeros(4,dtype=np.float32)
    return np.array([dx/d,dy/d,min(d/cfg['observation']['goal_distance_scale'],1),math.atan2(dy,dx)/math.pi],dtype=np.float32)

def make_observation(ranges, local_goal_robot, velocity, previous_executed, cfg):
    r=cfg['robot']; scale=np.array([r['v_max'],r['w_max']])
    return np.concatenate([scan_to_bev(ranges,cfg).ravel(),goal_features(local_goal_robot,cfg),np.asarray(velocity)/scale,np.asarray(previous_executed)/scale]).astype(np.float32)
