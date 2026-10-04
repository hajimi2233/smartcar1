#!/usr/bin/env python
"""Replay recorded policy-suite sensors to evaluate startup search + role split.

Ground truth is copied only into output error evaluation, never passed to search
or the policy engine. This is a sensor-time replay, not a latency benchmark.
"""
from __future__ import print_function
import os,sys,json,math,time,glob,bisect
import numpy as np
import rosbag
ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,ROOT+'/src/smartcar_localization/scripts')
from map_wall_extraction import map_geometry,extract_groups
from wall_features import scan_points
from wheel_distance import PulseBuffer,wrap,wheel_prediction
from localization_roles import Policies
from localization_search import SurfaceField,startup_search


def interpolate_odom(stamp,times,poses):
    i=bisect.bisect_left(times,stamp)
    if i<len(times) and abs(times[i]-stamp)<1e-6:return poses[i]
    if i==0 or i==len(times):return None
    a,b=poses[i-1],poses[i];f=(stamp-times[i-1])/(times[i]-times[i-1])
    return (a[0]+f*(b[0]-a[0]),a[1]+f*(b[1]-a[1]),a[2]+f*wrap(b[2]-a[2]))


def run_trial(directory,trial,outdir):
    case=trial['case'];name=case['name']+'_r'+str(trial['repeat'])
    rows=[json.loads(x) for x in open(directory+'/'+name+'.jsonl')]
    odom_rows=[r for r in rows if 'scan_odom' in r];times=[r['stamp'] for r in odom_rows];odoms=[r['scan_odom'] for r in odom_rows]
    scans={};pulses=[];anchors=[];mapmsg=None;truths=[]
    with rosbag.Bag(directory+'/'+name+'.bag') as bag:
        for topic,m,t in bag.read_messages(topics=['/map','/experiment/scan','/wheel/pulses','/amcl_pose','/sim/ground_truth/odom']):
            if topic=='/map':mapmsg=m
            elif topic=='/experiment/scan':scans[round(m.header.stamp.to_sec(),6)]=m
            elif topic=='/wheel/pulses':pulses.append(m)
            elif topic=='/sim/ground_truth/odom':
                p=m.pose.pose;q=p.orientation
                truths.append((m.header.stamp.to_sec(),(p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))))
            else:
                p=m.pose.pose;q=p.orientation;stamp=m.header.stamp.to_sec()
                odom=interpolate_odom(stamp,times,odoms)
                if odom is not None:anchors.append(((p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))),odom,stamp,max(m.pose.covariance[0],m.pose.covariance[7],m.pose.covariance[35])))
    anchors.sort(key=lambda a:a[2]);anchor_times=[a[2] for a in anchors]
    # Score every input scan, including callbacks dropped during a TF outage.
    # The baseline output at a missing timestamp is its last available pose.
    # Truth below belongs to the evaluator only, not the estimator.
    original={round(r['stamp'],6):r for r in rows};complete=[];previous=None
    truth_times=[t for t,p in truths];truth_poses=[p for t,p in truths]
    for key in sorted(scans)[-trial['input_scans']:]:
        if key in original:row=original[key]
        else:
            if previous is None:raise RuntimeError('Missing first estimate')
            truth=interpolate_odom(key,truth_times,truth_poses);nearest=False
            if truth is None:
                j=min(range(len(truth_times)),key=lambda i:abs(truth_times[i]-key))
                if abs(truth_times[j]-key)>.025:raise RuntimeError('Truth unavailable for scan')
                truth=truth_poses[j];nearest=True
            row=dict(stamp=key,phase=previous['phase'],truth=truth,poses=previous['poses'],
                details={m:dict(valid=False,state='HELD_MISSING_SCAN') for m in previous['poses']},errors={},
                filled_missing_scan=True,nearest_truth=nearest)
            for m,p in row['poses'].items():row['errors'][m]=dict(position=math.hypot(p[0]-truth[0],p[1]-truth[1]),yaw=abs(wrap(p[2]-truth[2])),valid=False)
        complete.append(row);previous=row
    rows=complete
    with open(outdir+'/'+name+'_complete.jsonl','w') as f:
        for r in rows:f.write(json.dumps(r)+'\n')
    cells,samples,res=map_geometry(mapmsg)
    full=[dict(id=0,start=[-6,-4],end=[6,4],radius=20.,cells=cells.tolist(),samples=samples.tolist(),resolution=res)]
    selectors=dict(inside=[],outside=[dict(id=1,start=[-5.8,4],end=[5.8,4]),dict(id=2,start=[6,-3.8],end=[6,3.8]),dict(id=3,start=[-5.8,-4],end=[5.8,-4]),dict(id=4,start=[-6,-3.8],end=[-6,3.8])])
    if case['partial']:selectors['outside']=selectors['outside'][:1]
    if case.get('straight_only'):selectors['outside']=[dict(id=1,start=[-4.5,4],end=[4.5,4])]
    walls=extract_groups(mapmsg,selectors)['outside'];field=SurfaceField(samples)
    initial=(3.86+(.35 if case['offset'] else .02),2.8+(-.2 if case['offset'] else -.01),math.pi+(.15 if case['offset'] else .005))
    engine=None;buffer=PulseBuffer(window=120.);last=None;ip=0;initialized=False;searches=[];result=[]
    good_pose=None;good_stamp=None;heading_history=[];yaw_rate=0.
    for row in rows:
        stamp=row['stamp']
        # Allow the same interpolation at the scan time used by the live suite.
        while ip<len(pulses) and pulses[ip].header.stamp.to_sec()<=stamp+.03:
            m=pulses[ip];ip+=1
            try:buffer.add(m.header.stamp.to_sec(),m.position[0],m.header.frame_id)
            except ValueError:pass
        start=time.time();search_info=None
        if 'scan_odom' not in row:
            pose=engine.poses['role_split'] if engine else initial
            detail=dict(valid=False,state='FRONTEND_UNAVAILABLE')
        else:
            odom=row['scan_odom'];scan=scans[round(stamp,6)];points,normals=scan_points(scan,(0.,0.,0.))
            if engine is None:engine=Policies(initial,odom,stamp,modes=('role_split',))
            if not initialized:
                pose,search_info=startup_search(points,normals,initial,full,field)
                searches.append(dict(stamp=stamp,seconds=time.time()-start,info=search_info))
                if pose is not None:
                    engine=Policies(pose,odom,stamp,modes=('role_split',));initialized=True
            delta=None
            if last is not None and row.get('wheel_available') and not (search_info and initialized):
                try:delta=buffer.delta(last,stamp)
                except ValueError:pass
            j=bisect.bisect_right(anchor_times,stamp)-1;anchor=anchors[j] if j>=0 else None
            poses,details=engine.update(stamp,odom,points,normals,walls,full,delta,anchor)
            pose=poses['role_split'];detail=details['role_split'];last=stamp
        coast_pose=pose;coast_detail=detail
        if 'scan_odom' in row and detail['state'] not in ('PREDICT_ONLY','FRONTEND_UNAVAILABLE'):
            heading_history.append((stamp,row['scan_odom'][2]))
            heading_history=[h for h in heading_history if h[0]>=stamp-.4]
            if len(heading_history)>=2:
                dt=stamp-heading_history[0][0]
                yaw_rate=max(-.6,min(.6,wrap(row['scan_odom'][2]-heading_history[0][1])/dt))
            good_pose=pose;good_stamp=stamp
        elif good_pose is not None and stamp-good_stamp<=1.2:
            try:
                ds=buffer.delta(good_stamp,stamp)
                coast_pose=wheel_prediction(good_pose,yaw_rate*(stamp-good_stamp),ds,.31)
                coast_detail=dict(valid=False,state='COAST_UNOBSERVED_YAW',age=stamp-good_stamp,yaw_rate=yaw_rate)
            except ValueError:pass
        truth=row['truth']
        error=dict(position=math.hypot(pose[0]-truth[0],pose[1]-truth[1]),yaw=abs(wrap(pose[2]-truth[2])),valid=detail['valid'])
        coast_error=dict(position=math.hypot(coast_pose[0]-truth[0],coast_pose[1]-truth[1]),yaw=abs(wrap(coast_pose[2]-truth[2])),valid=coast_detail['valid'])
        result.append(dict(stamp=stamp,phase=row['phase'],case=case['name'],repeat=trial['repeat'],truth=truth,
            poses={'role_search':pose,'role_search_coast':coast_pose},details={'role_search':detail,'role_search_coast':coast_detail},
            errors={'role_search':error,'role_search_coast':coast_error},compute_s=time.time()-start,search=search_info))
    with open(outdir+'/'+name+'.jsonl','w') as f:
        for r in result:f.write(json.dumps(r)+'\n')
    summary=dict(case=case,repeat=trial['repeat'],searches=searches,samples=len(result),
        maximum_m=max(r['errors']['role_search']['position'] for r in result),
        maximum_after_init_m=max(r['errors']['role_search']['position'] for r in result if r['phase']!='initialization'))
    print(name,'search',searches,'max_cm',100*summary['maximum_after_init_m']);sys.stdout.flush()
    return summary


def main():
    repeat=os.environ.get('REPLAY_REPEAT','0');root='/experiments';outdir=root+'/policy-search-r'+repeat
    if not os.path.isdir(outdir):os.makedirs(outdir)
    results=[]
    for family in os.environ.get('REPLAY_FAMILIES','policy-suite,policy-straight,policy-turnchange').split(','):
        directory=root+'/'+family+'-r'+repeat
        for trial in json.load(open(directory+'/results.json')):
            results.append(run_trial(directory,trial,outdir))
            with open(outdir+'/results.json','w') as f:json.dump(results,f,indent=2)

if __name__=='__main__':main()
