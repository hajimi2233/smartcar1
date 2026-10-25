from __future__ import print_function
import json,math,sys
rows=[json.loads(s) for s in open(sys.argv[1]) if s.strip()];limit=int(sys.argv[2]);count=0
for row in rows:
    if row['target_id']=='inspect_10':raise ValueError('Excluded target reached before stop')
    print('%s x=%.4f y=%.4f heading=%.2f deg'%(row['target_id'],row['x'],row['y'],math.degrees(row['yaw_rad'])))
    if row.get('inspect'):count+=1
    if count==limit:break
assert count==limit
print('Preview only. No ROS node or drive goal sent. Excluded: inspect_10; no exit goal.')
