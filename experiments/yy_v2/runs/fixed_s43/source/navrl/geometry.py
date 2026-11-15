"""Axis-aligned rectangles, circular footprint, exact ray/rectangle intersections."""
import heapq
import math
import numpy as np
from numba import njit

@njit(cache=True)
def raycast(pose, rectangles, beams, rmin, rmax, sx, sy, syaw):
    result = np.full(beams, np.inf, dtype=np.float32)
    c, s = math.cos(pose[2]), math.sin(pose[2])
    ox, oy = pose[0]+c*sx-s*sy, pose[1]+s*sx+c*sy
    for i in range(beams):
        angle = pose[2]+syaw-math.pi+i*2*math.pi/beams
        dx, dy = math.cos(angle), math.sin(angle)
        best = rmax
        for j in range(len(rectangles)):
            lo, hi = -1e20, 1e20
            valid = True
            for k in range(2):
                origin = ox if k == 0 else oy
                direction = dx if k == 0 else dy
                low = rectangles[j, k]
                high = rectangles[j, k+2]
                if abs(direction) < 1e-10:
                    if origin < low or origin > high:
                        valid = False
                        break
                else:
                    a, b = (low-origin)/direction, (high-origin)/direction
                    lo, hi = max(lo, min(a,b)), min(hi, max(a,b))
            if valid and hi >= max(lo,0.0):
                t = max(lo, 0.0)
                if t < best:
                    best = t
        if best < rmax:
            result[i] = best if best >= rmin else np.nan
    return result

def clearance_at(position, rectangles, radius):
    x, y = position[:2]
    dx = np.maximum(np.maximum(rectangles[:,0]-x, 0), x-rectangles[:,2])
    dy = np.maximum(np.maximum(rectangles[:,1]-y, 0), y-rectangles[:,3])
    return float(np.min(np.hypot(dx,dy))-radius)

def integrate(pose, v, w, dt):
    """Exact constant-command unicycle integration, no Euler turning bias."""
    x,y,a = pose
    if abs(w) < 1e-8:
        x += v*math.cos(a)*dt
        y += v*math.sin(a)*dt
    else:
        x += v/w*(math.sin(a+w*dt)-math.sin(a))
        y += v/w*(-math.cos(a+w*dt)+math.cos(a))
    return np.array([x,y,(a+w*dt+math.pi)%(2*math.pi)-math.pi])

def segment_clear(a, b, rectangles, radius, spacing=0.035):
    points = np.linspace(a,b,max(2, int(np.linalg.norm(b-a)/spacing)+2))
    return all(clearance_at(p,rectangles,radius)>0 for p in points)

def plan_path(start, goal, rectangles, cfg):
    """A* in footprint-inflated grid; reject corner cuts, then line-of-sight pruning."""
    e, robot = cfg['environment'], cfg['robot']
    h, res = e['room_half_size'], e['planner_resolution']
    size = int(2*h/res)+1
    xy = np.linspace(-h,h,size)
    X,Y = np.meshgrid(xy,xy)
    free = np.ones((size,size),dtype=bool)
    radius = robot['radius']+e['planner_margin']
    for rect in rectangles:
        dx=np.maximum(np.maximum(rect[0]-X,0),X-rect[2])
        dy=np.maximum(np.maximum(rect[1]-Y,0),Y-rect[3])
        free &= np.hypot(dx,dy)>radius
    def cell(p):
        return tuple(np.clip(np.rint((np.asarray(p)+h)/(2*h)*(size-1)),0,size-1).astype(int)[::-1])
    src,dst=cell(start),cell(goal)
    if not free[src] or not free[dst]:
        return None
    queue=[(0.0,src)]
    cost={src:0.0}
    parent={}
    while queue:
        _,u=heapq.heappop(queue)
        if u==dst: break
        for dr,dc in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            v=(u[0]+dr,u[1]+dc)
            if not (0<=v[0]<size and 0<=v[1]<size) or not free[v]: continue
            if dr and dc and (not free[u[0]+dr,u[1]] or not free[u[0],u[1]+dc]): continue
            g=cost[u]+math.hypot(dr,dc)
            if g < cost.get(v,math.inf):
                cost[v]=g
                parent[v]=u
                heapq.heappush(queue,(g+math.hypot(v[0]-dst[0],v[1]-dst[1]),v))
    if dst not in cost: return None
    nodes=[dst]
    while nodes[-1]!=src: nodes.append(parent[nodes[-1]])
    raw=[np.asarray(start,dtype=float)]+[np.array([xy[c],xy[r]]) for r,c in nodes[::-1]]+[np.asarray(goal,dtype=float)]
    # Validate every raw connector, including continuous start/goal to grid.
    if not all(segment_clear(a,b,rectangles,radius) for a,b in zip(raw[:-1],raw[1:])): return None
    selected=[raw[0]]
    i=0
    while i<len(raw)-1:
        j=len(raw)-1
        while j>i+1 and not segment_clear(raw[i],raw[j],rectangles,radius): j-=1
        selected.append(raw[j])
        i=j
    # Dense path supports bounded progress tracking and arc-length lookahead.
    dense=[]
    for a,b in zip(selected[:-1],selected[1:]):
        n=max(2,int(np.linalg.norm(b-a)/0.04)+1)
        dense.extend(np.linspace(a,b,n,endpoint=False))
    dense.append(selected[-1])
    return np.array(dense)
