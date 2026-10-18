"""Scratch camera-only boundary evidence; never a confirmed restricted zone.

Use solid, physically wide red clipped by the image boundary. Narrow red QR
features and perforated finder patterns do not qualify. Include only the
connected white backing actually visible beside that red component.
"""
import math
import cv2
import numpy as np

CONFIG = dict(min_major_m=.50, min_solid_fraction=.85, min_pixels=36,
              white_s_max=55, white_v_min=190, edge_px=2)

def partial_boundaries(frame, altitude_m, hfov_rad, project, *, bands=None):
    from perception_banner.colours import mask
    if not math.isfinite(altitude_m) or altitude_m <= 1.5:
        return []
    h,w = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    red = mask(hsv, bands or [])
    red = cv2.morphologyEx(red,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    white = cv2.inRange(hsv,(0,0,CONFIG['white_v_min']),(179,CONFIG['white_s_max'],255))
    _,backing = cv2.connectedComponents(red | white, connectivity=8)
    contours,_ = cv2.findContours(red,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    px_per_m = w/(2*math.tan(hfov_rad/2)*altitude_m)
    candidates=[]
    for c in contours:
        x,y,cw,ch = cv2.boundingRect(c)
        if min(x,y,w-x-cw,h-y-ch)>CONFIG['edge_px']:
            continue
        major=max(cv2.minAreaRect(c)[1])/px_per_m
        if major<CONFIG['min_major_m']:
            continue
        hull=cv2.convexHull(c)
        patch=np.zeros((h,w),np.uint8)
        cv2.fillConvexPoly(patch,hull,255)
        pixels=int(cv2.countNonZero(red & patch))
        solid=pixels/max(1,cv2.countNonZero(patch))
        if pixels<CONFIG['min_pixels'] or solid<CONFIG['min_solid_fraction']:
            continue
        labels=np.unique(backing[(red & patch)>0]); labels=labels[labels>0]
        support=np.isin(backing,labels).astype(np.uint8)*255
        outlines,_=cv2.findContours(support,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for outline in outlines:
            polygon=project(outline,(w,h))
            if len(polygon)>=3:
                candidates.append(dict(polygon=polygon,major_m=major,solid_fraction=solid,
                    red_pixels=pixels,confirmed=False,basis='clipped solid red and connected visible backing'))
    return candidates
