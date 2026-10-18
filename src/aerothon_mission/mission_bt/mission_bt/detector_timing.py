"""Qualify delivered observations using distinct camera capture timestamps."""
from collections import deque
import math


class DetectorTiming:
    def __init__(self):
        self.samples = deque(maxlen=40)

    def add(self, detail, received):
        try:
            capture=float(detail['capture_stamp'])
            processed=float(detail['processed_stamp'])
        except (KeyError,ValueError,TypeError):
            return False
        if not all(math.isfinite(t) for t in (capture,processed,received)):
            return False
        if capture<=0 or capture>processed or processed>received+0.1:
            return False
        if self.samples and capture<=self.samples[-1][0]:
            return False
        self.samples.append((capture,received,max(0,received-capture)))
        return True

    def qualified(self, now, max_age=1.0):
        recent=[s for s in self.samples if now-s[1]<=8.0]
        if len(recent)<4 or not 0<=now-recent[-1][0]<=max_age:
            return None
        first,last=recent[0],recent[-1]
        spans=(last[0]-first[0],last[1]-first[1])
        if min(spans)<=0:
            return None
        latency=sorted(s[2] for s in recent)
        return dict(hz=min((len(recent)-1)/span for span in spans),
                    latency_s=latency[min(len(latency)-1,int(.95*len(latency)))],
                    capture_stamp=last[0],samples=len(recent))


def search_speed(lookahead, clearance, rate, latency, ceiling, decel=.5):
    """Four-frame confirmation, delivery latency and braking inside the view."""
    room=max(0,lookahead-clearance)
    if rate<=0 or decel<=0:
        return 0.0
    delay=4/rate+latency+.6
    return min(ceiling,max(0,-decel*delay+math.sqrt((decel*delay)**2+2*decel*room)))
