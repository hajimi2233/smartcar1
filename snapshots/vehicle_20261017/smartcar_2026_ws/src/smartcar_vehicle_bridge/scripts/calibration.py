from __future__ import division
import math

def finite(x):
    return not (math.isnan(x) or math.isinf(x))

def interpolate(x, pairs):
    if x <= pairs[0][0]: return pairs[0][1]
    if x >= pairs[-1][0]: return pairs[-1][1]
    for (a,b),(c,d) in zip(pairs,pairs[1:]):
        if a <= x <= c: return b+(d-b)*(x-a)/(c-a)
    raise ValueError('invalid interpolation')

def validate(table):
    if len(table)<2 or any(len(p)!=2 or not all(finite(float(x)) for x in p) for p in table):
        raise ValueError('invalid calibration table')
    if any(b[0]<=a[0] or b[1]<=a[1] for a,b in zip(table,table[1:])):
        raise ValueError('calibration table must increase in both columns')

def command(v,w,speeds,steering,middle,scale,allow_reverse=False):
    if not all(finite(x) for x in (v,w)):raise ValueError('nonfinite command')
    if v<0 and not allow_reverse:raise ValueError('reverse not calibrated')
    if abs(v)>speeds[-1][0]+1e-8:raise ValueError('speed exceeds calibrated range')
    if abs(v)<1e-6:
        if abs(w)>1e-6:raise ValueError('cannot rotate Ackermann chassis in place')
        k=0.
    else:k=w/v
    k=max(steering[0][1],min(steering[-1][1],k))
    raw=interpolate(k,[(p[1],p[0]) for p in steering])
    raw=max(middle-scale,min(middle+scale,raw))
    k=interpolate(raw,steering)
    throttle=math.copysign(interpolate(abs(v),speeds),v) if v else 0.
    legacy=(middle-raw)/scale
    if abs(legacy)>1.+1e-6:raise ValueError('steering target exceeds driver limit')
    return throttle,max(-1.,min(1.,legacy)),k,raw

def feedback_curvature(raw,steering,raw_min=None,raw_max=None):
    """Measured feedback: extend endpoint fit only inside verified hardware bounds.

    Command interpolation still clamps to the measured command table. Callers
    without explicit hardware bounds retain the old 25-count acceptance range.
    """
    if not finite(raw):raise ValueError('nonfinite steering feedback')
    if (raw_min is None)!=(raw_max is None):raise ValueError('incomplete feedback bounds')
    if raw_min is None:
        if raw<steering[0][0]-25 or raw>steering[-1][0]+25:raise ValueError('steering feedback out of range')
        return interpolate(raw,steering)
    if not all(finite(x) for x in (raw_min,raw_max)) or not raw_min<=steering[0][0]<steering[-1][0]<=raw_max:
        raise ValueError('feedback bounds do not contain calibrated table')
    if not raw_min<=raw<=raw_max:raise ValueError('steering feedback outside physical bounds')
    if raw<steering[0][0]:a,b=steering[:2]
    elif raw>steering[-1][0]:a,b=steering[-2:]
    else:return interpolate(raw,steering)
    return a[1]+(b[1]-a[1])*(raw-a[0])/(b[0]-a[0])


def joints(raw,v,steering,L,T,Tr,radius,raw_min=None,raw_max=None):
    if not finite(v) or min(L,T,Tr,radius)<=0:raise ValueError('invalid feedback geometry/speed')
    k=feedback_curvature(raw,steering,raw_min,raw_max)
    if min(1-T*k/2.,1+T*k/2.)<=0:raise ValueError('invalid Ackermann geometry')
    angles=[math.atan(L*k/(1-T*k/2.)),math.atan(L*k/(1+T*k/2.))]
    wheels=[v*(1-Tr*k/2.)/radius,v*(1+Tr*k/2.)/radius]
    return angles,wheels,k
