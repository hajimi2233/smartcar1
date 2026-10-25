#!/usr/bin/env python
"""Explicit simulated-label camera viewer; imports the installed detector unchanged."""
from __future__ import print_function
import argparse,importlib,json,math,os,random,sys,threading,time
from patrol_preview import SimulatedLabels


class DemoOverlay(object):
    def __init__(self,cv2,draw,engine,publish=lambda message:None,clock=None):
        self.cv2=cv2;self.original_draw=draw;self.engine=engine;self.publish=publish
        self.clock=clock or getattr(time,'monotonic',time.time)
        self.lock=threading.RLock();self.current=None;self.last_published=None
    def context(self,data):
        with self.lock:
            try:self.engine.update(data,self.clock())
            except (ValueError,TypeError,KeyError,AttributeError):self.engine.invalidate()
    def observe(self,result):
        with self.lock:
            sample=self.engine.sample(self.clock())
            self.current=sample
            # A predicted class is a UI sample, never a real inspection event.
            if sample is not None and result.get('target') is not None and result.get('status')=='LOCKED':
                key=(sample['run_id'],sample['target_id'],sample['sample_time'])
                if key!=self.last_published:
                    event=dict(sample,camera_box_present=True)
                    self.publish(event);self.last_published=key
    def draw(self,frame,result,show_rejected_glare=True):
        cv2=self.cv2
        with self.lock:
            # Recheck TTL even when drawing a previously observed frame.
            sample=self.engine.sample(self.clock())
        if sample is None:
            vis=self.original_draw(frame,result,show_rejected_glare)
            title='SIMULATED MODE | '+self.engine.reason
        else:
            shown=dict(result)
            target=result.get('target')
            if target is not None:
                shown['target']=dict(target);shown['target'].pop('fire_info',None)
            vis=self.original_draw(frame,shown,show_rejected_glare)
            title='SIMULATED | P%d | %s'%(sample['channel_number'],sample['phase'])
            if target is not None and result.get('status')!='FAR':
                color=(0,0,230) if sample['is_abnormal'] else (0,180,0)
                x,y,bw,bh=target['bbox'];h,w=vis.shape[:2]
                # Remove the original locate-only caption before adding the
                # synthetic caption, without modifying the classifier result.
                bar_h=min(32,max(16,int(h*.12)));label_h=16 if w<220 else 24
                old_bottom=min(h-1,max(bar_h+label_h,int(y)))
                old_top=max(bar_h,old_bottom-label_h)
                old_left=max(0,min(int(x),max(0,w-60)))
                vis[old_top:old_bottom+1,old_left:w]=frame[old_top:old_bottom+1,old_left:w]
                poly=target.get('poly')
                if poly is not None:cv2.polylines(vis,[poly.reshape(-1,1,2)],True,color,3)
                text='SIM P%d %s'%(sample['channel_number'],sample['label'])
                (tw,th),baseline=cv2.getTextSize(text,cv2.FONT_HERSHEY_SIMPLEX,.46,1)
                left=max(0,min(int(x),max(0,w-tw-8)));bottom=max(60,min(int(y),h-22))
                cv2.rectangle(vis,(left,bottom-th-baseline-5),(min(w-1,left+tw+7),bottom+2),color,-1)
                cv2.putText(vis,text,(left+3,bottom-baseline),cv2.FONT_HERSHEY_SIMPLEX,.46,(255,255,255),1)
        h,w=vis.shape[:2]
        cv2.rectangle(vis,(0,0),(w-1,min(31,h-1)),(45,45,45),-1)
        cv2.putText(vis,title,(6,min(23,h-2)),cv2.FONT_HERSHEY_SIMPLEX,.42 if w<480 else .5,(0,220,255),1)
        if h>=90:
            cv2.rectangle(vis,(0,h-20),(w-1,h-1),(45,45,45),-1)
            cv2.putText(vis,('SIMULATED LABELS' if w<480 else 'SIMULATED LABELS | camera box geometry only'),(5,h-6),cv2.FONT_HERSHEY_SIMPLEX,.42,(0,220,255),1)
        return vis


def main():
    parser=argparse.ArgumentParser(description=__doc__,add_help=False)
    parser.add_argument('--demo-error-percent',type=float,default=20.)
    parser.add_argument('--demo-update-seconds',type=float,default=1.)
    parser.add_argument('--demo-seed',type=int,default=None)
    options,remaining=parser.parse_known_args()
    engine=SimulatedLabels(options.demo_error_percent/100.,options.demo_update_seconds,rng=random.Random(options.demo_seed))
    import rospy
    from std_msgs.msg import String
    rospy.init_node('patrol_vision_demo',anonymous=True,disable_signals=True)
    # The existing classifier and its CLI are used in place, without replacement.
    directory=os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0,directory)
    detector_module=importlib.import_module('overhead_pic_detector_v5')
    publisher=rospy.Publisher('/inspection/demo_preview',String,queue_size=10)
    def publish(event):
        publisher.publish(String(data=json.dumps(event,sort_keys=True)))
        print('[SIMULATED] P%d %s noise=%s'%(event['channel_number'],event['label'],event['injected_error']))
    overlay=DemoOverlay(detector_module.cv2,detector_module.OverheadPicDetector.draw_result,engine,publish)
    def receive(msg):
        try:data=json.loads(msg.data)
        except (ValueError,TypeError):data=None
        overlay.context(data)
    subscriber=rospy.Subscriber('/inspection/patrol_context',String,receive,queue_size=10)
    original_detect=detector_module.OverheadPicDetector.detect_frame
    def detect(instance,frame):
        result,masks=original_detect(instance,frame);overlay.observe(result);return result,masks
    detector_module.OverheadPicDetector.detect_frame=detect
    detector_module.OverheadPicDetector.draw_result=staticmethod(overlay.draw)
    original_imshow=detector_module.cv2.imshow
    detector_module.cv2.imshow=lambda title,image:original_imshow('SIMULATED PATROL VISION',image)
    sys.argv=[sys.argv[0]]+remaining
    print('[SIMULATED MODE] Patrol labels with %.1f%% synthetic errors; real classifier output is unchanged.'%options.demo_error_percent)
    try:detector_module.main()
    finally:
        subscriber.unregister();rospy.signal_shutdown('demo viewer closed')


if __name__=='__main__':main()
