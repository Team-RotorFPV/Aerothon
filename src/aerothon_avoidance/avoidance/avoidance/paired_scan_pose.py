"""Pair lidar captures with onboard pose history without extrapolation."""
import bisect
import math
from types import MethodType


def capture_pose(history, stamp, max_gap_s=.2):
    rows=sorted(history,key=lambda row:row[0])
    if not rows or not math.isfinite(stamp):return None
    index=bisect.bisect_left([row[0] for row in rows],stamp)
    if index<len(rows) and abs(rows[index][0]-stamp)<1e-8:
        row=rows[index]
        return row[1:4],row[4]
    if not 0<index<len(rows):return None
    a,b=rows[index-1:index+1];gap=b[0]-a[0]
    if not 0<gap<=max_gap_s:return None
    fraction=(stamp-a[0])/gap
    qa,qb=list(a[4]),list(b[4])
    na,nb=math.sqrt(sum(x*x for x in qa)),math.sqrt(sum(x*x for x in qb))
    if min(na,nb)<1e-9:return None
    qa=[x/na for x in qa];qb=[x/nb for x in qb]
    dot=sum(x*y for x,y in zip(qa,qb))
    if dot<0:qb=[-x for x in qb];dot=-dot
    dot=min(1.,max(-1.,dot))
    if dot>.9995:
        q=[x+fraction*(y-x) for x,y in zip(qa,qb)]
    else:
        angle=math.acos(dot);scale=math.sin(angle)
        q=[(math.sin((1-fraction)*angle)*x+math.sin(fraction*angle)*y)/scale
           for x,y in zip(qa,qb)]
    length=math.sqrt(sum(x*x for x in q));q=tuple(x/length for x in q)
    x,y,z,w=q
    yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    pose=(a[1]+fraction*(b[1]-a[1]),a[2]+fraction*(b[2]-a[2]),yaw)
    if not all(map(math.isfinite,(*pose,*q))):return None
    return pose,q


def install(node):
    original=node._registered_scan
    def registered(_node,*args,**kwargs):
        paired=capture_pose(_node._pose_history,_node._scan_t)
        if paired is None:
            _node._scan_pose=_node._scan_orientation=None
        else:
            _node._scan_pose,_node._scan_orientation=paired
        return original(*args,**kwargs)
    node._registered_scan=MethodType(registered,node)
