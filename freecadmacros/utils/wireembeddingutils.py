
from barmesh.tribarmes import TriangleBarMesh, TriangleBar, MakeTriangleBoxing
from barmesh.basicgeo import I1, Partition1, P3, P2, Along
import Draft, Part, Mesh, MeshPart

def TriangleCrossCutPlane(bar, lam, bGoRight, driveperpvec, driveperpvecDot):
    nodeAhead = bar.GetNodeFore(bGoRight)
    nodeBehind = bar.GetNodeFore(not bGoRight)
    barAhead = bar.GetForeRightBL(bGoRight)
    barAheadGoRight = (barAhead.nodeback == nodeAhead)
    nodeOpposite = barAhead.GetNodeFore(barAheadGoRight)
    barBehind = barAhead.GetForeRightBL(barAheadGoRight)
    DbarBehindGoRight = (barBehind.nodeback == nodeOpposite)
    assert nodeBehind == barBehind.GetNodeFore(DbarBehindGoRight)
    dpvdAhead = P3.Dot(driveperpvec, nodeAhead.p)
    dpvdBehind = P3.Dot(driveperpvec, nodeBehind.p)
    dpvdOpposite = P3.Dot(driveperpvec, nodeOpposite.p)
    assert dpvdAhead > driveperpvecDot - 0.001 and dpvdBehind < driveperpvecDot + 0.001
    bAheadSeg = (dpvdOpposite < driveperpvecDot)
    barCrossing = (barAhead if bAheadSeg else barBehind)
    dpvdAB = (dpvdAhead if bAheadSeg else dpvdBehind)
    barCrossingLamO = -(dpvdOpposite - driveperpvecDot)/(dpvdAB - dpvdOpposite)
    assert barCrossingLamO >= 0.0
    barCrossingLam = barCrossingLamO if (barCrossing.nodeback == nodeOpposite) else 1-barCrossingLamO
    barCrossingGoRight = (barCrossing.nodeback == nodeOpposite) == bAheadSeg
    return barCrossing, barCrossingLam, barCrossingGoRight

def planecutembeddedcurve(startbar, startlam, driveperpvec):
    startpt = Along(startlam, startbar.nodeback.p, startbar.nodefore.p)
    driveperpvecDot = P3.Dot(driveperpvec, startpt)
    drivebars = [ (startbar, startlam) ]
    bGoRight = (P3.Dot(driveperpvec, startbar.nodefore.p - startbar.nodeback.p) > 0)
    bar, lam = startbar, startlam
    while True:
        bar, lam, bGoRight = TriangleCrossCutPlane(bar, lam, bGoRight, driveperpvec, driveperpvecDot)
        drivebars.append((bar, lam))
        if bar == startbar:
            assert abs(startlam - lam) < 0.001
            drivebars[-1] = drivebars[0]#
            break
        if len(drivebars) > 500:
            print("failed, too many drivebars")
            break
    return drivebars

def planecutbars(tbarmesh, driveperpvec, driveperpvecDot):
    for bar in tbarmesh.bars:
        dp0 = P3.Dot(bar.nodeback.p, driveperpvec)
        dp1 = P3.Dot(bar.nodefore.p, driveperpvec)
        if (dp0 < driveperpvecDot) != (dp1 < driveperpvecDot):
            lam = (driveperpvecDot - dp0)/(dp1 - dp0)
            return bar, lam
    return None, 0.0
