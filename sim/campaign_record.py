"""Independent campaign evidence and grading. Never imported by flight nodes."""
import hashlib
import json
import math
import os
from functools import lru_cache
from pathlib import Path


@lru_cache(maxsize=1)
def source_manifest(root):
    result={}
    for directory in ('src','scripts','sim'):
        for path,dirs,files in os.walk(Path(root)/directory):
            dirs[:]=[d for d in dirs if d not in ('node_modules','__pycache__','.git','build','dist')]
            for name in files:
                p=Path(path)/name
                if p.suffix in ('.py','.sh','.cpp','.hpp','.h','.xml','.yaml','.yml','.json','.sdf','.xacro','.toml','.ts','.tsx','.rs','.parm','.cfg','.ini','.txt','.urdf','.glb','.dae','.stl','.ply','.obj','.mtl','.png','.jpg','.jpeg','.webp','.svg'):
                    result[str(p.relative_to(root))]=hashlib.sha256(p.read_bytes()).hexdigest()
    return result


def navigation_grade(track,layout,target,independent_contacts=None):
    from check_track import grade,lanes,to_lane,rect_gap,red_polygons
    result=grade(track,layout,target)
    flying=[r for r in track if r['armed'] and r['z']>.3]
    red=min((rect_gap(poly,r['x'],r['y']) for r in flying
             for _,poly in red_polygons(layout)),default=math.inf)
    gap=[]
    for r in flying:
        for lane in lanes(layout):
            u,v=to_lane(r,layout,lane)
            if 0<=u<=lane[3] and abs(v)<=lane[4]/2 and r['z']<lane[5]:
                gap.append(lane[4]/2-abs(v))
        for obstacle in layout.get('obstacles',[]):
            if r['z']<obstacle['h']+.3:
                hx,hy=layout.get('home_world',[0,0])
                gap.append(rect_gap([(x-hx,y-hy) for x,y in obstacle['poly']],r['x'],r['y']))
    # Delivery success remains visible in the full grade, but this object-level
    # fixture has no real motor signal and cannot qualify that confirmation.
    nav_checks={k:v for k,v in result['checks'].items()
                if 'payload' not in k and not k.startswith('drop within')}
    nav_checks['restricted-zone clearance at least 1.50 m']=red>=1.5
    nav_checks['corridor clearance at least 0.70 m']=bool(gap) and min(gap)>=.70
    # The layout proxy covers slalom obstacles, but not every native fixture
    # collision (notably the overhead board). Require the independent world
    # record as well; a missing record is not evidence of zero contacts.
    contacts_valid=isinstance(independent_contacts,list) and all(
        isinstance(event,(list,tuple)) and len(event)==5 and
        isinstance(event[1],str) and bool(event[1]) and
        all(isinstance(event[i],(int,float)) and math.isfinite(event[i])
            for i in (0,2,3,4)) and event[0]>=0
        for event in independent_contacts)
    no_contacts=contacts_valid and not independent_contacts
    nav_checks['no obstacle contact']=bool(nav_checks.get('no obstacle contact',True) and no_contacts)
    result['checks']['no obstacle contact']=nav_checks['no obstacle contact']
    result['pass']=all(result['checks'].values())
    return dict(checks=nav_checks,pass_navigation=all(nav_checks.values()),
                measured_red_clearance_m=red if math.isfinite(red) else None,
                measured_corridor_clearance_m=min(gap,default=None),full_grade=result,
                rendered_perception_qualified=False,motor_delivery_qualified=False,
                independent_contact_evidence_valid=contacts_valid,
                independent_contacts=independent_contacts if contacts_valid else None)
