#!/usr/bin/env python3
"""Offline geometric audit; its output is never a drone input."""
import argparse
import hashlib
import json
import math
from pathlib import Path

from world_spec import lane_bottleneck, polygon_gap, rect_corners, load


def audit(path, clearance=.70, lane_key='return_corridor'):
    data=load(path)
    lane=dict(data[lane_key],obstacles=data[lane_key].get('obstacles',[]))
    length,width=lane['length'],lane['width']
    polygons=[rect_corners(o['u'],o['v'],o['w'],o['d'],math.radians(o['yaw_deg']))
              for o in lane['obstacles']]
    polygons += [rect_corners(length/2,width/2+.05,length,.1),
                 rect_corners(length/2,-width/2-.05,length,.1)]
    names=[f'obstacle {i+1}' for i in range(len(polygons)-2)]+['upper wall','lower wall']
    # Exact polygon distance <= 2r is equivalent to intersection of their
    # Euclidean Minkowski sums with a radius-r disk, including rotated boxes.
    edges={i:[] for i in range(len(polygons))}
    gaps={}
    for i,a in enumerate(polygons):
        for j in range(i+1,len(polygons)):
            d=polygon_gap(a,polygons[j])
            if d<=2*clearance:
                edges[i].append(j); edges[j].append(i); gaps[i,j]=d
    upper,lower=len(polygons)-2,len(polygons)-1
    parents={upper:None}; queue=[upper]
    for i in queue:
        for j in edges[i]:
            if j not in parents: parents[j]=i;queue.append(j)
    barrier=[]
    if lower in parents:
        at=lower
        while at is not None: barrier.append(at);at=parents[at]
        barrier.reverse()
    return dict(world=str(path),lane=lane_key,sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        clearance_m=clearance, expansion='Euclidean Minkowski sum with 0.70 m disk',
        barrier=[names[i] for i in barrier],
        barrier_gaps_m=[gaps[tuple(sorted((a,b)))] for a,b in zip(barrier,barrier[1:])],
        classification='geometrically blocked' if barrier else 'no cross-lane barrier found',
        grid_bottlenecks_m={str(r):lane_bottleneck(lane,r) for r in (.05,.025)})


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('world',type=Path)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    result=audit(args.world)
    args.out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
