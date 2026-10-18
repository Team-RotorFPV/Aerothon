"""Independent recording metrics, never imported by flight software."""
import json
import math
from pathlib import Path

import numpy as np


def delivered_metrics(rows):
    unique={}
    rejected=0
    for row in rows:
        detail=row.get('detail',row)
        stamp=detail.get('capture_stamp')
        received=row.get('received_stamp')
        if not isinstance(stamp,(int,float)) or not math.isfinite(stamp) or stamp<=0:
            rejected+=1
            continue
        if stamp not in unique:
            unique[stamp]=(received,detail,row.get('received_wall_s'))
    stamps=sorted(unique)
    arrivals=[unique[t][0] for t in stamps]
    wall_arrivals=[unique[t][2] for t in stamps
                   if isinstance(unique[t][2],(int,float)) and math.isfinite(unique[t][2])]
    def rate(values):
        return (len(values)-1)/(values[-1]-values[0]) if len(values)>1 and values[-1]>values[0] else None
    latency=[a-t for t,a in zip(stamps,arrivals) if a is not None and a>=t]
    wall=[d['processing_wall_s'] for _,d,_ in unique.values() if 'processing_wall_s' in d]
    return dict(distinct_captures=len(stamps),invalid_capture_events=rejected,
        capture_hz=rate(stamps),delivered_hz=rate(arrivals),
        delivered_wall_hz=rate(sorted(set(wall_arrivals))),
        time_basis='capture_hz and delivered_hz use simulated time; delivered_wall_hz uses host monotonic time',
        maximum_capture_gap_s=max(np.diff(stamps)) if len(stamps)>1 else None,
        latency_sim_p95_s=float(np.percentile(latency,95)) if latency else None,
        processing_wall_p95_s=float(np.percentile(wall,95)) if wall else None)


def recording_metrics(out):
    out=Path(out)
    events=[json.loads(line) for line in (out/'detector_events.jsonl').open()]
    camera=[json.loads(line) for line in (out/'camera_capture_events.jsonl').open()]
    search=[r for r in events if r.get('mission')=='SEARCH_QR']
    result=dict(camera=delivered_metrics(camera),search={
        name:delivered_metrics([r for r in search if r['detector']==name])
        for name in ('qr_timing','red_timing')})
    result['search']['usable_red_ground']=delivered_metrics([r for r in search
        if r['detector']=='red_timing' and r['detail'].get('status') in ('CLEAR','RED')
        and 0<=r['received_stamp']-r['detail'].get('capture_stamp',0)<=1.0])
    return result
