"""Navigation-only range smoothing and bounded fresh-frame recovery (Python 2/3)."""
import math


def smooth_ranges(values, minimum, maximum):
    out = list(values)
    valid = lambda x: not math.isnan(x) and not math.isinf(x) and minimum <= x < maximum
    for i in range(1, len(values)-1):
        window = values[i-1:i+2]
        # Smooth only coherent surfaces; retain isolated near returns and edges
        # so a narrow real obstacle is not discarded as an outlier.
        if all(valid(v) for v in window) and max(window)-min(window) <= .06:
            out[i] = sorted(window)[1]
    return out


class Recovery(object):
    def __init__(self, now):
        self.started = now
        self.last_stamp = None
        self.clear_since = None
        self.clear_frames = 0

    def update(self, now, stamp, clear, stopped):
        if stamp != self.last_stamp:
            self.last_stamp = stamp
            if clear and stopped:
                if self.clear_since is None: self.clear_since = now
                self.clear_frames += 1
            else:
                self.clear_since = None
                self.clear_frames = 0
        if stopped and self.clear_frames >= 5 and now-(self.clear_since or now) >= .5:
            return 'resume'
        if now-self.started >= 5. and stopped:
            return 'replan'
        if now-self.started >= 10.:
            return 'fail'
        return 'wait'


class ConsecutiveFailures(object):
    """Count recovery incidents, not control-loop ticks; ignore coordinates."""
    def __init__(self):
        self.last = None
        self.count = 0

    def record(self, key):
        self.count = self.count+1 if key == self.last else 1
        self.last = key
        return self.count < 2


def obstacle_key(reason):
    import re
    groups = re.findall(r'sources=\[([^]]*)\]', reason)
    sources = sorted(set(re.findall(r'\b(REGION|STATIC_MAP|LIVE_SCAN|MAP_UNKNOWN|MAP_BOUNDS|INVALID_POSE)\b',
                                    ' '.join(groups))))
    if not sources and 'TRACKING_ENVELOPE' in reason:
        return 'LOCAL:TRACKING_ENVELOPE'
    return 'OBSTACLE:' + ('+'.join(sources) if sources else 'LOCAL_TRAJECTORY')
