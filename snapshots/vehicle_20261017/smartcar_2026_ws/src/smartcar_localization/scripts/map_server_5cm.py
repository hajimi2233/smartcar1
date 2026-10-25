#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Conservatively resample a trinary map before launching the standard map_server."""
from __future__ import print_function,division
import sys,os,json,hashlib,math
import numpy as np
from PIL import Image
import yaml

def convert(source,resolution=.05):
    source=os.path.realpath(os.path.expanduser(source))
    with open(source,'rb') as f:raw_yaml=f.read()
    meta=yaml.safe_load(raw_yaml)
    old=float(meta['resolution']);target=float(resolution)
    if not all(x>0 and not math.isnan(x) and not math.isinf(x) for x in (old,target)):
        raise ValueError('invalid map resolution')
    if meta.get('mode','trinary')!='trinary':raise ValueError('only trinary occupancy maps are supported')
    if old>target+1e-7:raise ValueError('cannot invent a finer map from a coarser input')
    if abs(old-target)<1e-7:return source
    image_path=meta['image']
    if not os.path.isabs(image_path):image_path=os.path.join(os.path.dirname(source),image_path)
    with open(image_path,'rb') as f:raw_image=f.read()
    digest=hashlib.sha256(raw_yaml+raw_image+str(target).encode('ascii')).hexdigest()
    folder=os.path.join(os.path.dirname(source),'navigation_%dcm'%int(round(target*100)))
    output=os.path.join(folder,'map.yaml');manifest=os.path.join(folder,'conversion.json')
    if os.path.isfile(output) and os.path.isfile(manifest):
        with open(manifest) as f:cached=json.load(f)
        if cached.get('source_hash')==digest and os.path.isfile(os.path.join(folder,'map.pgm')):return output
    pixels=np.asarray(Image.open(image_path).convert('L'),dtype=np.float64)[::-1,:]
    occupancy=pixels/255. if int(meta.get('negate',0)) else 1.-pixels/255.
    occupied=float(meta.get('occupied_thresh',.65));free=float(meta.get('free_thresh',.196))
    if not 0<free<occupied<1:raise ValueError('invalid occupancy thresholds')
    hit=occupancy>occupied;unknown=~(hit|(occupancy<free))
    # Integral rectangles preserve even a one-pixel obstacle; no image interpolation.
    def integral(mask):return np.pad(mask.astype(np.int32).cumsum(0).cumsum(1),((1,0),(1,0)),mode='constant')
    hi=integral(hit);ui=integral(unknown)
    height,width=pixels.shape;ratio=target/old
    w=int(math.ceil(width/ratio-1e-7));h=int(math.ceil(height/ratio-1e-7))
    xs=np.floor(np.arange(w)*ratio+1e-7).astype(int)
    xe=np.minimum(width,np.ceil((np.arange(w)+1)*ratio-1e-7).astype(int))
    unknown_value=int(round(255*(1.-.5*(free+occupied))))
    if not free<=1.-unknown_value/255.<=occupied:raise ValueError('thresholds cannot represent an unknown grayscale pixel')
    result=np.empty((h,w),dtype=np.uint8);result.fill(unknown_value)
    for y in range(h):
        ys=int(math.floor(y*ratio+1e-7));ye=min(height,int(math.ceil((y+1)*ratio-1e-7)))
        counts=hi[ye,xe]-hi[ys,xe]-hi[ye,xs]+hi[ys,xs]
        unclear=ui[ye,xe]-ui[ys,xe]-ui[ye,xs]+ui[ys,xs]
        padding=((np.arange(w)+1)*ratio>width+1e-6) | ((y+1)*ratio>height+1e-6)
        result[y,(counts==0)&(unclear==0)&~padding]=255
        result[y,counts>0]=0
    if not os.path.isdir(folder):os.makedirs(folder)
    # Stage every file before publishing the YAML that points to it.
    img_tmp=os.path.join(folder,'map.pgm.tmp')
    Image.fromarray(result[::-1,:],mode='L').save(img_tmp,format='PPM')
    os.rename(img_tmp,os.path.join(folder,'map.pgm'))
    out=dict(meta);out.update(image='map.pgm',resolution=target,negate=0)
    yaml_tmp=output+'.tmp'
    with open(yaml_tmp,'w') as f:yaml.safe_dump(out,f,default_flow_style=False)
    os.rename(yaml_tmp,output)
    with open(manifest,'w') as f:json.dump(dict(source=source,source_hash=digest,resolution=target,source_size=[width,height],output_size=[w,h],origin=meta['origin'],method='occupied-first, unknown-second conservative area pooling'),f,indent=2)
    return output

if __name__=='__main__':
    conversion_only=len(sys.argv)>1 and sys.argv[1]=='--convert'
    args=sys.argv[2:] if conversion_only else sys.argv[1:]
    if not args:raise SystemExit('map_server_5cm.py [--convert] map.yaml [resolution] [ROS remappings]')
    source=args[0];resolution=float(args[1]) if len(args)>1 and ':=' not in args[1] else .05
    remappings=args[2:] if len(args)>1 and ':=' not in args[1] else args[1:]
    result=convert(source,resolution)
    print('Map for localization/navigation: '+result);sys.stdout.flush()
    if not conversion_only:
        import roslib.packages
        executable=roslib.packages.find_node('map_server','map_server')[0]
        os.execv(executable,[executable,result]+remappings)
