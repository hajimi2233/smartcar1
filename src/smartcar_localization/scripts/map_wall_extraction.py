"""Select ALL map wall cells covered by a dilated line segment (Python 2/3).

A capsule mask includes both endpoint disks. It does not choose a unique wall,
fill gaps, or turn the manually drawn line into a localization target.
"""
import math
import numpy as np
from wall_features import GROUPS, validate_groups


def map_geometry(msg):
    res=float(msg.info.resolution)
    if not np.isfinite(res) or res<=0: raise ValueError('Invalid map resolution')
    grid=np.asarray(msg.data).reshape(msg.info.height,msg.info.width)
    occupied=grid>=65;free=(grid>=0)&(grid<=25)
    adjacent=np.zeros(grid.shape,dtype=bool)
    adjacent[1:,:] |= free[:-1,:];adjacent[:-1,:] |= free[1:,:]
    adjacent[:,1:] |= free[:,:-1];adjacent[:,:-1] |= free[:,1:]
    o=msg.info.origin;q=o.orientation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    c,s=math.cos(yaw),math.sin(yaw);rotation=np.array([[c,s],[-s,c]])
    def coordinates(mask):
        ys,xs=np.nonzero(mask)
        return np.dot(np.column_stack(((xs+.5)*res,(ys+.5)*res)),rotation)+[o.position.x,o.position.y]
    boundary_mask=occupied&adjacent
    boundary=coordinates(boundary_mask)
    ys,xs=np.nonzero(boundary_mask)
    facing=[]
    for y,x in zip(ys,xs):
        directions=[]
        for dx,dy in ((0,1),(0,-1),(1,0),(-1,0)):
            yy,xx=y+dy,x+dx
            if 0<=yy<grid.shape[0] and 0<=xx<grid.shape[1] and free[yy,xx]: directions.append((dx,dy))
        direction=np.sum(directions,axis=0).astype(float)
        if np.linalg.norm(direction)<1e-6: direction=np.array(directions[0],dtype=float)
        direction/=np.linalg.norm(direction);facing.append(np.dot(direction,rotation))
    facing=np.asarray(facing).reshape((-1,2))
    # Local map normals smooth raster stair steps, without fitting one line across
    # separate walls. Only supported locally linear boundary patches enter matching.
    samples=[]
    for i,point in enumerate(boundary):
        delta=boundary-point
        # Do not mix opposite faces of a thick wall when estimating its tangent.
        near=delta[(np.sum(delta*delta,axis=1)<=(res*2.6)**2)&(np.dot(facing,facing[i])>.3)]
        if len(near)<3: continue
        centered=near-np.mean(near,axis=0)
        values,vectors=np.linalg.eigh(np.dot(centered.T,centered))
        if values[-1]<=1e-12 or values[0]>values[-1]*.25: continue
        normal=vectors[:,0]
        samples.append([point[0],point[1],normal[0],normal[1]])
    return coordinates(occupied),np.asarray(samples).reshape((-1,4)),res


def capsule_mask(points,selector,radius):
    a,b=np.asarray(selector['start']),np.asarray(selector['end']);delta=b-a
    t=np.clip(np.dot(points-a,delta)/np.dot(delta,delta),0.,1.)
    offsets=points-(a+t[:,None]*delta)
    return np.sum(offsets*offsets,axis=1)<=radius*radius+1e-12


def extract_wall(geometry,selector,radius=.30):
    cells,samples,res=geometry
    radius=float(selector.get('radius',radius))
    if not np.isfinite(radius) or radius<=0: raise ValueError('Selection radius must be positive')
    selected=cells[capsule_mask(cells,selector,radius)]
    if not len(selected): raise ValueError('Selection covers no occupied map cells')
    surfaces=samples[capsule_mask(samples[:,:2],selector,radius)]
    if len(surfaces)<3: raise ValueError('Selected cells have no usable known-free wall surface')
    return dict(id=selector['id'],start=selector['start'],end=selector['end'],radius=radius,
                resolution=res,cells=selected.tolist(),samples=surfaces.tolist())


def extract_groups(msg,selectors,radius=.30):
    selectors=validate_groups(selectors);geometry=map_geometry(msg)
    groups={group:[] for group in GROUPS};errors=[]
    for group in GROUPS:
        for wall in selectors[group]:
            try: groups[group].append(extract_wall(geometry,wall,radius))
            except ValueError as exc: errors.append('%s #%d: %s'%(group,wall['id'],exc))
    if errors: raise ValueError('; '.join(errors))
    return groups
