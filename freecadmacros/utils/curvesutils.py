

from FreeCAD import Vector
from barmesh.basicgeo import P3, P2, Along

def isdiscretizableobject(s):
    if s.isDerivedFrom("Sketcher::SketchObject"):
        print("we could do joining up of pieces by coincidence here")
        assert hasattr(s, "Shape") and hasattr(s.Shape, "Edges")
        return True
    elif hasattr(s, "Shape") and hasattr(s.Shape, "Edges"):
	    return True
    return False

def discretizeobject(s, deflection=0.02):
    pts = [ ]
    for edge in s.Shape.Edges:
        epts = edge.discretize(Deflection=deflection)
        gap = (pts[-1] - epts[0]).Length if pts else 0
        if gap > 0.1:
            print("Gap between edges", gap)
        pts.extend(epts[1 if gap < 0.1 else 0:])
    return pts


def thinptstotolerance(pts, tol):
    i0 = 0
    istack = [ len(pts)-1 ]
    ithinned = [ i0 ]
    while istack:
        i1 = istack[-1]
        p0, p1 = pts[i0], pts[i1]
        v = p1 - p0
        vsq = v.Lensq()
        imid = -1
        dmid = 0.0
        for i in range(i0+1, i1):
            p = pts[i]
            lam = P3.Dot(v, (p - p0))/vsq
            lam = max(0.0, min(1.0, lam))
            pm = p0 + v*lam
            d = (pm - p).Len()
            if imid == -1 or d > dmid:
                imid = i
                dmid = d
        if dmid <= tol:
            i0 = istack.pop()
            ithinned.append(i0)
        else:
            assert imid != -1
            istack.append(imid)
    return [ pts[i]  for i in ithinned ]

def cumlengthlist(pts):
    ptcls = [ 0.0 ]
    for i in range(1, len(pts)):
        ptcls.append(ptcls[-1] + (pts[i] - pts[i-1]).Len())
    return ptcls

def seglampos(d, ptcls):
    i0, i1 = 0, len(ptcls)-1
    while i1 > i0 + 1:
        im = (i0 + i1)//2
        #print(f'assert i0 < im < i1 ({i0} < {im} < {i1})')
        assert i0 < im < i1
        if ptcls[im] < d:
            i0 = im
        else:
            i1 = im
        #print(f'assert ptcls[i0] <= d <= ptcls[i1] ({ptcls[i0]} <= {d} <= {ptcls[i1]})')
        assert ptcls[i0] <= d <= ptcls[i1] 
    d01 = (ptcls[i1] - ptcls[i0])
    lam = (d - ptcls[i0]) / d01 if d01 != 0 else 0.5
    return i0, lam
