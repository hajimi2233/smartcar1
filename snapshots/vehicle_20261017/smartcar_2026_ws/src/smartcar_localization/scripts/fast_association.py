"""Exact nearest normal-compatible 1 cm surfaces, with bounded KD queries."""
import numpy as np
try:
    from scipy.spatial import cKDTree
except ImportError:
    cKDTree=None

def nearest_compatible(world,directions,samples,tree,radius,cosine):
    count=len(samples);n=len(world)
    best=np.full(n,np.inf);selected=np.full(n,-1,dtype=int)
    if not count or not n:return best,selected
    pending=np.arange(n);k=min(8,count)
    while len(pending):
        distances,indices=tree.query(world[pending],k=k,eps=0.,distance_upper_bound=radius+1e-12)
        distances=np.asarray(distances).reshape(len(pending),k)
        indices=np.asarray(indices).reshape(len(pending),k)
        valid=indices<count;safe=np.minimum(indices,count-1)
        target=samples[safe]
        delta=world[pending,None,:]-target[:,:,:2]
        squared=np.sum(delta*delta,axis=2)
        normal=np.abs(np.sum(directions[pending,None,:]*target[:,:,2:],axis=2))
        squared[~valid|(normal<cosine)|(squared>radius*radius)]=np.inf
        values=np.min(squared,axis=1)
        # Preserve exhaustive-association tie ordering by original sample index.
        ids=np.min(np.where((squared==values[:,None])&np.isfinite(squared),indices,count),axis=1)
        certified=np.isinf(distances[:,-1])|(values<distances[:,-1]**2-1e-12)
        if k==count:certified[:]=True
        resolved=pending[certified];best[resolved]=values[certified]
        selected[resolved]=np.where(ids[certified]<count,ids[certified],-1)
        pending=pending[~certified];k=min(count,k*2)
    return best,selected
