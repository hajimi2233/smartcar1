"""Bounded startup search, then selected-wall tracking. No ground-truth input.

Experimental: search uses only the initial guess, scan and full-map surfaces.
It is not a global unknown-pose localizer, nor a real-time performance guarantee.
"""
import math
import numpy as np
from wall_features import match


class SurfaceField(object):
    def __init__(self,samples,resolution=.025,cap=.35):
        xy=np.asarray(samples)[:,:2]
        self.resolution=resolution;self.cap=cap
        self.origin=np.floor((xy.min(axis=0)-cap)/resolution)*resolution
        shape=np.ceil((xy.max(axis=0)+cap-self.origin)/resolution).astype(int)+1
        self.grid=np.full((shape[1],shape[0]),cap)
        indices=np.rint((xy-self.origin)/resolution).astype(int)
        occupied=np.zeros(self.grid.shape,dtype=bool);occupied[indices[:,1],indices[:,0]]=True
        yy,xx=np.nonzero(occupied);radius=int(math.ceil(cap/resolution))
        for dy in range(-radius,radius+1):
            for dx in range(-radius,radius+1):
                distance=math.hypot(dx,dy)*resolution
                if distance>cap:continue
                x=xx+dx;y=yy+dy
                valid=(x>=0)&(x<shape[0])&(y>=0)&(y<shape[1])
                x=x[valid];y=y[valid]
                self.grid[y,x]=np.minimum(self.grid[y,x],distance)

    def score(self,points,poses):
        poses=np.asarray(poses);c=np.cos(poses[:,2,None]);s=np.sin(poses[:,2,None])
        x=poses[:,0,None]+c*points[None,:,0]-s*points[None,:,1]
        y=poses[:,1,None]+s*points[None,:,0]+c*points[None,:,1]
        ix=np.rint((x-self.origin[0])/self.resolution).astype(int)
        iy=np.rint((y-self.origin[1])/self.resolution).astype(int)
        valid=(ix>=0)&(ix<self.grid.shape[1])&(iy>=0)&(iy<self.grid.shape[0])
        values=np.full(ix.shape,self.cap)
        values[valid]=self.grid[iy[valid],ix[valid]]
        return np.mean(values**2,axis=1)


def startup_search(points,normals,initial,all_walls,field=None):
    """Search +/-0.6m XY, +/-0.30rad yaw, then refine the best hypotheses.

    Always use this at startup, including apparently successful local fits; a
    locally full-rank fit does not prove that it is the correct map association.
    """
    if len(points)<20:return None,dict(state='SEARCH_NO_SCAN')
    if field is None:field=SurfaceField(np.vstack([w['samples'] for w in all_walls]))
    poses=np.array([(initial[0]+dx,initial[1]+dy,initial[2]+da)
        for da in np.linspace(-.30,.30,13)
        for dx in np.linspace(-.6,.6,13)
        for dy in np.linspace(-.6,.6,13)])
    costs=field.score(points,poses);best=None;best_cost=float('inf');checked=[]
    for index in np.argsort(costs):
        seed=poses[index]
        # Refine distinct basins rather than eight adjacent samples of one basin.
        if any(np.linalg.norm(seed[:2]-p[:2])<.12 and abs(seed[2]-p[2])<.08 for p in checked):continue
        checked.append(seed)
        fitted,info=match(points,normals,seed,all_walls)
        if fitted is not None and info.get('map_rank')==3:
            cost=float(field.score(points,np.asarray([fitted]))[0])
            if cost<best_cost:best=fitted;best_cost=cost
        if len(checked)>=8:break
    if best is None or best_cost>.06**2:
        return None,dict(state='SEARCH_UNCONFIRMED',cost=best_cost)
    return best,dict(state='SEARCH_CONFIRMED',cost=best_cost,seeds=len(checked))
