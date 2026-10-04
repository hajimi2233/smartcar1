#!/usr/bin/env python3
"""Generate paired SDF/occupancy map; channel geometry remains unchanged."""
import pathlib,xml.etree.ElementTree as ET,json,math,hashlib,struct
ROOT=pathlib.Path(__file__).resolve().parents[1]
OUT=ROOT/'data/maps/sim_field_narrow';OUT.mkdir(parents=True,exist_ok=True)
tree=ET.parse(str(ROOT/'src/smartcar_sim/worlds/inspection.world'))
for model in tree.findall('./world/model'):
 name=model.get('name')
 if name in ('outer_y_1','outer_y_-1'):
  p=model.find('pose').text.split();p[1]='3.2' if name=='outer_y_1' else '-3.0';model.find('pose').text=' '.join(p)
 if name in ('outer_x_1','outer_x_-1'):
  p=model.find('pose').text.split();p[1]='0.1';model.find('pose').text=' '.join(p)
  for size in model.findall('./link/*/geometry/box/size'):size.text='0.08 6.2 1'
tree.write(str(OUT/'inspection.world'),encoding='utf-8',xml_declaration=True)
# Rasterize the actual laser-height collision boxes, not a separately hand-drawn map.
boxes=[]
for model in tree.findall('./world/model'):
 p=list(map(float,model.find('pose').text.split()))
 for col in model.findall('./link/collision'):
  size=col.find('geometry/box/size')
  if size is None:continue
  sx,sy,sz=map(float,size.text.split())
  if not p[2]-sz/2 <= .5 <= p[2]+sz/2:continue
  assert abs(p[5])<1e-9
  boxes.append((p[0]-sx/2,p[0]+sx/2,p[1]-sy/2,p[1]+sy/2))
w,h,res=248,168,.05;ox,oy=-6.2,-4.2;data=[]
for j in range(h):
 for i in range(w):
  x,y=ox+i*res,oy+j*res
  outside=x+res<=-6.04 or x>=6.04 or y+res<=-3.04 or y>=3.24
  occupied=any(x<b and x+res>a and y<d and y+res>c for a,b,c,d in boxes)
  data.append(100 if occupied or outside else 0)
raw=bytes(0 if data[j*w+i] else 254 for j in range(h-1,-1,-1) for i in range(w))
(OUT/'map.pgm').write_bytes(('P5\n%d %d\n255\n'%(w,h)).encode()+raw)
(OUT/'map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [-6.2, -4.2, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
meta=['map',w,h,struct.unpack('f',struct.pack('f',res))[0],ox,oy,0.,0.,0.,0.,1.];digest=hashlib.sha256(json.dumps(meta,separators=(',',':')).encode('ascii'));digest.update(bytearray(data))
# Provisional lines centred in the remaining turn areas; saved separately.
lines=[[[-3.29,2.48],[3.29,2.48]],[[-3.14,-2.38],[3.29,-2.38]]]
(OUT/'low_cost_lines.json').write_text(json.dumps({'schema_version':1,'maps':{digest.hexdigest():lines}},indent=2))
(OUT/'geometry.json').write_text(json.dumps(dict(width=w,height=h,resolution=res,origin=[ox,oy,0.],data=data,boxes=boxes,lines=lines),indent=2))
(OUT/'map_info.txt').write_text('frame_id=sim_world\nupper_wall_center_y=3.2\nlower_wall_center_y=-3.0\nupper_clear_turn_area=1.36\nlower_clear_turn_area=1.16\nmap_source=laser-height SDF collision boxes\n')
print(OUT)
