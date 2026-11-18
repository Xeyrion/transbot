# -*- coding: utf-8 -*-
"""Brown-Conrady camera rays, shared by extraction, projection and calibration."""
import numpy as np


def coefficients(values, model='plumb_bob'):
    values=list(values or [])
    if not np.isfinite(values).all(): raise ValueError('Nonfinite lens distortion')
    if any(abs(v)>1e-12 for v in values) and model not in ('plumb_bob','rational_polynomial'):
        raise ValueError('Unsupported distortion model: '+model)
    if len(values) not in (0,4,5,8): raise ValueError('Expected 0, 4, 5 or 8 distortion coefficients')
    return values+[0.]*(8-len(values))


def distort(x,y,values):
    k1,k2,p1,p2,k3,k4,k5,k6=coefficients(values)
    r2=x*x+y*y
    radial=(1+k1*r2+k2*r2*r2+k3*r2*r2*r2)/(1+k4*r2+k5*r2*r2+k6*r2*r2*r2)
    return x*radial+2*p1*x*y+p2*(r2+2*x*x),y*radial+p1*(r2+2*y*y)+2*p2*x*y


def undistort(xd,yd,values):
    x,y=np.asarray(xd,dtype=float).copy(),np.asarray(yd,dtype=float).copy()
    for _ in range(30):
        px,py=distort(x,y,values)
        dx,dy=xd-px,yd-py
        x,y=x+dx,y+dy
        if np.all(np.abs(dx)<1e-10) and np.all(np.abs(dy)<1e-10): break
    px,py=distort(x,y,values)
    valid=np.isfinite(x)&np.isfinite(y)&(np.abs(px-xd)<1e-6)&(np.abs(py-yd)<1e-6)
    return np.where(valid,x,np.nan),np.where(valid,y,np.nan)
