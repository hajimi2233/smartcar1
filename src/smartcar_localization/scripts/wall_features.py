"""Map-bound wall annotations and feature-only SE(2) matching (Python 2/3)."""
import hashlib
import json
import math
import os
import numpy as np

GROUPS = ('inside', 'outside')
DEFAULTS = dict(association_distance=.30, endpoint_margin=.05,
                normal_angle_deg=25., surface_gap=.06, min_points=8, min_span=.20,
                max_rms=.06, max_translation=.25, max_rotation=.20,
                iterations=10, eigen_ratio=.005)


def fingerprint(msg):
    o = msg.info.origin
    meta = [msg.header.frame_id.lstrip('/'), msg.info.width, msg.info.height,
            msg.info.resolution, o.position.x, o.position.y, o.position.z,
            o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w]
    h = hashlib.sha256(json.dumps(meta).encode('ascii'))
    h.update(bytearray((v+256)%256 for v in msg.data))
    return h.hexdigest()


def validate_groups(groups):
    if not isinstance(groups, dict) or set(groups) != set(GROUPS):
        raise ValueError('Expected inside and outside wall groups')
    result = {}
    for group in GROUPS:
        result[group] = []
        ids = set()
        for wall in groups[group]:
            ident = wall['id']
            if not isinstance(ident, int) or isinstance(ident, bool) or ident < 1 or ident in ids:
                raise ValueError('Wall IDs must be unique positive integers in each group')
            points = np.asarray([wall['start'], wall['end']], dtype=float)
            if points.shape != (2,2) or not np.all(np.isfinite(points)):
                raise ValueError('Wall endpoints must be finite XY pairs')
            if np.linalg.norm(points[1]-points[0]) < .10:
                raise ValueError('Wall must be at least 0.10 m long')
            ids.add(ident)
            item=dict(id=ident,start=points[0].tolist(),end=points[1].tolist())
            if 'radius' in wall:
                radius=float(wall['radius'])
                if not np.isfinite(radius) or radius<=0: raise ValueError('Radius must be positive')
                item['radius']=radius
            result[group].append(item)
    return result


def validate_targets(groups):
    selectors=validate_groups(groups)
    result={group:[] for group in GROUPS}
    for group in GROUPS:
        for selector,wall in zip(selectors[group],groups[group]):
            cells=np.asarray(wall['cells'],dtype=float);samples=np.asarray(wall['samples'],dtype=float)
            if cells.ndim!=2 or cells.shape[1]!=2 or len(cells)==0 or not np.all(np.isfinite(cells)):
                raise ValueError('Invalid selected occupied cells')
            if samples.ndim!=2 or samples.shape[1]!=4 or len(samples)<3 or not np.all(np.isfinite(samples)):
                raise ValueError('Invalid map surface samples')
            if np.any(np.abs(np.linalg.norm(samples[:,2:],axis=1)-1.)>.01):
                raise ValueError('Invalid surface normals')
            res=float(wall['resolution'])
            if not np.isfinite(res) or res<=0: raise ValueError('Invalid map resolution')
            selector.update(cells=cells.tolist(),samples=samples.tolist(),resolution=res)
            result[group].append(selector)
    return result


def save_file(path, groups, map_id, frame, selectors=None):
    data = dict(schema_version=1, frame_id=frame, map_sha256=map_id,
                groups=validate_groups(groups))
    if selectors is not None:
        data.update(schema_version=3, selectors=validate_groups(selectors), groups=validate_targets(groups))
    directory = os.path.dirname(os.path.abspath(path))
    if not os.path.isdir(directory): os.makedirs(directory)
    with open(path+'.tmp', 'w') as stream:
        json.dump(data, stream, indent=2)
        stream.flush(); os.fsync(stream.fileno())
    os.rename(path+'.tmp', path)


def load_file(path, map_id, frame):
    with open(path) as stream: data = json.load(stream)
    if data.get('schema_version') not in (1,2,3) or data.get('map_sha256') != map_id or data.get('frame_id') != frame:
        raise ValueError('Wall file belongs to another map/frame or schema')
    return validate_targets(data['groups']) if data['schema_version']==3 else validate_groups(data['groups'])


def compose(a, b):
    c,s = math.cos(a[2]), math.sin(a[2])
    return (a[0]+c*b[0]-s*b[1], a[1]+s*b[0]+c*b[1],
            math.atan2(math.sin(a[2]+b[2]), math.cos(a[2]+b[2])))


def inverse(a):
    c,s = math.cos(a[2]), math.sin(a[2])
    return (-c*a[0]-s*a[1], s*a[0]-c*a[1], -a[2])


def scan_points(scan, laser_pose, max_range=12.):
    """Keep contiguous planar surfaces; invalid beams break neighborhoods."""
    r = np.asarray(scan.ranges, dtype=float)
    angles = scan.angle_min+np.arange(len(r))*scan.angle_increment
    valid = np.isfinite(r) & (r > scan.range_min) & (r < min(scan.range_max,max_range))
    r = np.where(valid,r,0.)
    points = np.column_stack((r*np.cos(angles),r*np.sin(angles)))
    if len(r) < 3: return np.empty((0,2)), np.empty((0,2))
    before, after = points[1:-1]-points[:-2], points[2:]-points[1:-1]
    lb,la = np.linalg.norm(before,axis=1),np.linalg.norm(after,axis=1)
    continuous = valid[:-2]&valid[1:-1]&valid[2:]&(lb>.002)&(la>.002)&(lb<.25)&(la<.25)
    dot = np.sum(before*after,axis=1)/np.maximum(lb*la,1e-12)
    continuous &= dot > math.cos(math.radians(35))
    tangent = points[2:]-points[:-2]
    normals = np.column_stack((-tangent[:,1],tangent[:,0]))
    normals /= np.maximum(np.linalg.norm(normals,axis=1)[:,None],1e-12)
    c,s = math.cos(laser_pose[2]),math.sin(laser_pose[2])
    rot = np.array([[c,-s],[s,c]])
    return (np.dot(points[1:-1][continuous],rot.T)+laser_pose[:2],
            np.dot(normals[continuous],rot.T))


def match(points, normals, predicted, walls, options=None, motion_prior=None):
    """Only selected wall residuals enter the solve. Nullspace stays at odometry.

    Local association needs a reasonable initial pose; this is not global
    localization and cannot distinguish repeated identical parallel walls.
    """
    cfg = dict(DEFAULTS); cfg.update(options or {})
    if not walls: return None, dict(reason='NO_WALLS')
    if len(points) < cfg['min_points']: return None, dict(reason='NO_FEATURE_POINTS')
    points, normals = np.asarray(points),np.asarray(normals)
    if not np.all(np.isfinite(points)) or not np.all(np.isfinite(normals)):
        return None, dict(reason='INVALID_SCAN')
    pose = np.asarray(predicted, dtype=float).copy()
    last = None
    def associate_lines(pose):
        c,s=math.cos(pose[2]),math.sin(pose[2]);rot=np.array([[c,-s],[s,c]])
        relative=np.dot(points,rot.T);world=relative+pose[:2];directions=np.dot(normals,rot.T)
        distances=[]; projections=[]; wall_normals=[]
        for wall in walls:
            a,b=np.asarray(wall['start']),np.asarray(wall['end'])
            tangent=(b-a)/np.linalg.norm(b-a);n=np.array([-tangent[1],tangent[0]])
            residual=np.dot(world-a,n);along=np.dot(world-a,tangent)
            eligible=(np.abs(residual)<=cfg['association_distance'])&(along>=-cfg['endpoint_margin'])&(along<=np.linalg.norm(b-a)+cfg['endpoint_margin'])
            eligible &= np.abs(np.dot(directions,n))>=math.cos(math.radians(cfg['normal_angle_deg']))
            distances.append(np.where(eligible,np.abs(residual),np.inf));projections.append((residual,along));wall_normals.append(n)
        distances=np.asarray(distances);choice=np.argmin(distances,axis=0)
        js=[];rs=[];ws=[];used=[]
        for i,wall in enumerate(walls):
            take=(choice==i)&np.isfinite(distances[i]);residual,along=projections[i]
            indices=np.flatnonzero(take)
            if len(indices)<cfg['min_points']: continue
            # Never average two distinct parallel return surfaces onto one annotation.
            # Choose the supported surface nearest the predicted feature, not the one
            # with the most laser points. Initial-pose association remains necessary.
            indices=indices[np.argsort(residual[indices])]
            clusters=np.split(indices,np.flatnonzero(np.diff(residual[indices])>cfg['surface_gap'])+1)
            clusters=[c for c in clusters if len(c)>=cfg['min_points'] and np.ptp(along[c])>=cfg['min_span']]
            if not clusters: continue
            chosen=min(clusters,key=lambda c: abs(float(np.median(residual[c]))))
            take=np.zeros(len(points),dtype=bool);take[chosen]=True
            n=wall_normals[i];rel=relative[take];r=residual[take]
            j=np.column_stack((np.full(len(r),n[0]),np.full(len(r),n[1]),-n[0]*rel[:,1]+n[1]*rel[:,0]))
            # Each manually selected wall has equal total weight, regardless of beam count.
            weight=np.minimum(1.,.03/np.maximum(np.abs(r),1e-12))/len(r)
            js.append(j);rs.append(r);ws.append(weight);used.append(wall['id'])
        if not js: return None
        return np.concatenate(js),np.concatenate(rs),np.concatenate(ws),used
    def associate_mask(pose):
        c,s=math.cos(pose[2]),math.sin(pose[2]);rot=np.array([[c,-s],[s,c]])
        relative=np.dot(points,rot.T);world=relative+pose[:2];directions=np.dot(normals,rot.T)
        best=np.full(len(points),np.inf);chosen=np.full(len(points),-1,dtype=int)
        target=np.zeros((len(points),4))
        for i,wall in enumerate(walls):
            samples=np.asarray(wall['samples'])
            # Chunked nearest compatible surface association. Distances are to actual
            # selected map cells; the selection line never enters the residual.
            for offset in range(0,len(samples),256):
                batch=samples[offset:offset+256]
                delta=world[:,None,:]-batch[None,:,:2]
                distance=np.sum(delta*delta,axis=2)
                compatible=np.abs(np.dot(directions,batch[:,2:].T))>=math.cos(math.radians(cfg['normal_angle_deg']))
                distance[~compatible]=np.inf
                index=np.argmin(distance,axis=1);minimum=distance[np.arange(len(points)),index]
                take=(minimum<best)&(minimum<=cfg['association_distance']**2)
                best[take]=minimum[take];chosen[take]=i;target[take]=batch[index[take]]
        js=[];rs=[];ws=[];used=[]
        for i,wall in enumerate(walls):
            take=chosen==i
            if np.sum(take)<cfg['min_points']: continue
            observed=world[take]
            if np.linalg.norm(np.ptp(observed,axis=0))<cfg['min_span']: continue
            n=target[take,2:];r=np.sum((observed-target[take,:2])*n,axis=1);rel=relative[take]
            j=np.column_stack((n[:,0],n[:,1],-n[:,0]*rel[:,1]+n[:,1]*rel[:,0]))
            weight=np.minimum(1.,.03/np.maximum(np.abs(r),1e-12))/len(r)
            js.append(j);rs.append(r);ws.append(weight);used.append(wall['id'])
        if not js: return None
        return np.concatenate(js),np.concatenate(rs),np.concatenate(ws),used
    associate=associate_mask if all('samples' in wall for wall in walls) else associate_lines
    for iteration in range(int(cfg['iterations'])):
        last=associate(pose)
        if last is None: return None,dict(reason='NO_WALL_SUPPORT')
        j,r,w,used=last;h=np.dot(j.T*w,j);g=np.dot(j.T,w*r)
        rank_threshold=max(np.linalg.eigvalsh(h)[-1]*cfg['eigen_ratio'],1e-9)
        if motion_prior is not None:
            from wheel_distance import prior_residual_jacobian
            residual,jacobian=prior_residual_jacobian(pose,motion_prior)
            jacobian=np.asarray(jacobian)
            # Map residual scale is 5 cm. Encoder is a longitudinal increment only;
            # it supplies no absolute position or independent yaw observation.
            strength=.0025/motion_prior['sigma']**2
            h+=strength*np.outer(jacobian,jacobian);g+=strength*jacobian*residual
        values,vectors=np.linalg.eigh(h);keep=values>rank_threshold
        rank=int(np.sum(keep))
        if rank < 2: return None,dict(reason='DEGENERATE',rank=rank)
        step=-np.dot(vectors[:,keep],np.dot(vectors[:,keep].T,g)/values[keep])
        pose+=step
        delta=pose-np.asarray(predicted)
        if np.linalg.norm(delta[:2])>cfg['max_translation'] or abs(delta[2])>cfg['max_rotation']:
            return None,dict(reason='CORRECTION_TOO_LARGE')
        if np.linalg.norm(step)<1e-5: break
    last=associate(pose)
    if last is None: return None,dict(reason='LOST_SUPPORT')
    j,r,w,used=last;rms=math.sqrt(float(np.sum(w*r*r)/np.sum(w)))
    if rms>cfg['max_rms']: return None,dict(reason='RESIDUAL_TOO_HIGH',rms=rms)
    # Report weak directions explicitly, never claim a parallel wall fixes along-wall position.
    h=np.dot(j.T*w,j)
    rank_threshold=max(np.linalg.eigvalsh(h)[-1]*cfg['eigen_ratio'],1e-9)
    map_rank=int(np.sum(np.linalg.eigvalsh(h)>rank_threshold))
    if motion_prior is not None:
        residual,jacobian=prior_residual_jacobian(pose,motion_prior)
        h+=(.0025/motion_prior['sigma']**2)*np.outer(jacobian,jacobian)
    values,vectors=np.linalg.eigh(h);keep=values>rank_threshold
    covariance=np.dot(vectors,np.dot(np.diag(np.where(keep,.0025/np.maximum(values,1e-9),1.)),vectors.T))
    return tuple(pose),dict(reason='MATCHED' if map_rank==3 else 'PARTIAL',rank=int(np.sum(keep)),map_rank=map_rank,
                           rms=rms,points=len(r),wall_ids=used,covariance=covariance.tolist())
