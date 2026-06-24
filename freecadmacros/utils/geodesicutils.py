import math
import Part
from FreeCAD import Vector
from barmesh.basicgeo import P3, P2, Along
from utils.curvesutils import seglampos
from utils.trianglemeshutils import facetbetweenbars


def Square(X):
    return X*X


def TOL_ZERO(X, msg=""):
    if not abs(X) < 0.0001:
        print("TOL_ZERO fail", X, msg)


# Fold between the two triangles is alpha (0=flat)
# Incoming angle to edge: beta, outgoing angle to edge: gamma
# Incoming vector: (sin(beta), cos(beta), 0)
# Outgoing vector: (cos(alpha)sin(gamma), cos(gamma), sin(alpha)sin(gamma))
# Resultant force on edge: (cos(alpha)sin(gamma) - sin(beta), cos(gamma) - cos(beta), sin(alpha)sin(gamma))
# Along edge force: E = cos(gamma) - cos(beta)
# Into edge force: N = |(cos(alpha)sin(gamma) - sin(beta), sin(alpha)sin(gamma))|
# Friction coefficent r, so |E| <= rN


def GeoCrossAxisE(a, Vae, Vab, Isq, Isgn):
    # Solve: Isq*x.Lensq() - Square(P3.Dot(x, Vab)) = 0   for x = a + Vae*q
    # 0 = Isq*(a^2 + 2q a.Vae + q^2 Vae^2) - (a.Vab + Vae.Vab q)^2
    #   = Isq*(a^2 + 2q adf + q^2 Vae^2) - (adv + fdv q)^2

    fdv = P3.Dot(Vae, Vab)
    adv = P3.Dot(a, Vab)
    adf = P3.Dot(a, Vae)
    qA = Square(fdv) - Vae.Lensq()*Isq
    qB2 = adv*fdv - adf*Isq
    qC = Square(adv) - a.Lensq()*Isq
    if abs(qA) <= abs(qB2)*1e-7:
        if qB2 == 0:
            return -1.0
        q = -qC/(2*qB2)
    else:
        qdq = Square(qB2) - qA*qC
        if qdq < 0.0:
            return -1.0
        qs = math.sqrt(qdq) / qA
        qm = -qB2 / qA
        q = qm + qs*Isgn
    # q = qs +- qm,  x = a + Vae*q,  Dot(x, Vab) same sign as Dot(Vcd, Vab)
    if abs(q) < 100:
        TOL_ZERO(qA*Square(q) + qB2*2*q + qC)
    return q


#
# This is the basic function that crosses from one triangle to the next
#
def GeoCrossAxis(Ga, Gb, Gcfrom, lam, Geopposite, VabInterpolated):
    Gd = Along(lam, Ga, Gb)
    Vcd = Gd - Gcfrom
    if Vcd.Len() == 0:
        bEnd = True
        bAEcrossing = False
        q = 0
        Gx = Gcfrom
    else:
        bEnd = False
        cdDab = P3.Dot(Vcd, VabInterpolated)
        Isq = Square(cdDab) / Vcd.Lensq()
        Isgn = -1 if cdDab < 0 else 1
        qVae = GeoCrossAxisE(Ga - Gd, Geopposite - Ga, VabInterpolated, Isq, Isgn)
        qVbe = GeoCrossAxisE(Gb - Gd, Geopposite - Gb, -VabInterpolated, Isq, -Isgn)
        bAEcrossing = (abs(qVae - 0.5) < abs(qVbe - 0.5))
        q = qVae if bAEcrossing else qVbe
        Gx = (Ga + (Geopposite - Ga)*q) if bAEcrossing else (Gb + (Geopposite - Gb)*q)
        Dx = Gx - Gd
        TOL_ZERO(Isq - Square(P3.Dot(Dx, VabInterpolated)/Dx.Len()))
        TOL_ZERO(P3.Dot(Vcd, VabInterpolated)/Vcd.Len() - P3.Dot(Dx, VabInterpolated)/Dx.Len())
    return bAEcrossing, q, Gx, bEnd


def TriangleNodeOpposite(bar, bGoRight):
    bartop = bar.GetForeRightBL(bGoRight)
    if bartop is not None:
        return bartop.GetNodeFore(bartop.nodeback == bar.GetNodeFore(bGoRight))
    return None


def GeoCrossBar(c, bar, lam, bGoRight, flatbartangents):
    Na, Nb = bar.nodeback, bar.nodefore
    d = Along(lam, Na.p, Nb.p)
    Ne = TriangleNodeOpposite(bar, bGoRight)
    if Ne is None:
        # GeoCrossBar fail
        return (None, None, 0.0, False)
    if flatbartangents is not None:
        vback, vfore = flatbartangents[bar.i]
        VabInterpolated = Along(lam, vback, vfore)
    else:
        VabInterpolated = Nb.p - Na.p
    bAEcrossing, q, Gx, bEnd = GeoCrossAxis(Na.p, Nb.p, c, lam, Ne.p,
                                            VabInterpolated)
    if bGoRight:
        if bAEcrossing:
            bar = bar.barforeright.GetForeRightBL(bar.barforeright.nodeback == Nb)
            lam = q if bar.nodeback == Na else 1-q
            bGoRight = bar.nodeback == Na
        else:
            bar = bar.barforeright
            lam = q if bar.nodeback == Nb else 1-q
            bGoRight = not bar.nodeback == Nb
    else:
        if bAEcrossing:
            bar = bar.barbackleft
            lam = q if bar.nodeback == Na else 1-q
            bGoRight = not bar.nodeback == Na
        else:
            bar = bar.barbackleft.GetForeRightBL(bar.barbackleft.nodeback == Na)
            lam = q if bar.nodeback == Nb else 1-q
            bGoRight = bar.nodeback == Nb

    c = bar.nodeback.p + (bar.nodefore.p - bar.nodeback.p)*lam
    TOL_ZERO((c - Gx).Len())
    return (d, bar, lam, bGoRight)


def barfacetnormal(bar, bGoRight, ptcommon=None):
    nodeAhead = bar.GetNodeFore(bGoRight)
    nodeBehind = bar.GetNodeFore(not bGoRight)
    barAhead = bar.GetForeRightBL(bGoRight)
    if barAhead:
        barAheadGoRight = barAhead.nodeback == nodeAhead
        nodeOpposite = barAhead.GetNodeFore(barAheadGoRight)
        ptc = nodeOpposite.p if ptcommon is None else ptcommon
        v2ahead = nodeAhead.p - ptc
        v2behind = nodeBehind.p - ptc
        return P3.ZNorm(P3.Cross(v2ahead, v2behind))
    else:
        return None


def InvAlong(v, a, b):
    return (v - a)/(b - a)


def TriangleExitCrossCutPlaneRight(tbar, perpvec, perpvecDot):
    node2 = tbar.barforeright.GetNodeFore(tbar.barforeright.nodeback == tbar.nodefore)
    dpvdnodefore = P3.Dot(perpvec, tbar.nodefore.p)
    dpvdnodeback = P3.Dot(perpvec, tbar.nodeback.p)
    dpvdnode2 = P3.Dot(perpvec, node2.p)
    assert tbar.nodeback.i < tbar.nodefore.i

    if dpvdnodefore <= perpvecDot <= dpvdnodeback:
        tbarlam = InvAlong(perpvecDot, dpvdnodeback, dpvdnodefore)
        return tbar, tbarlam, False

    if dpvdnodeback <= perpvecDot <= dpvdnode2:
        tbarlam = InvAlong(perpvecDot, dpvdnodeback, dpvdnode2)
        barbackright = tbar.barforeright.GetForeRightBL(tbar.barforeright.nodeback == tbar.nodefore)
        assert barbackright.nodeback.i < barbackright.nodefore.i
        return barbackright, tbarlam, True

    if dpvdnode2 <= perpvecDot <= dpvdnodefore:
        bforerightFore = node2.i < tbar.nodefore.i
        tbarlamF = InvAlong(perpvecDot, dpvdnode2, dpvdnodefore)
        tbarlam = tbarlamF if bforerightFore else 1.0 - tbarlamF
        return tbar.barforeright, tbarlam, bforerightFore

    return None, 0, False


def triclambarlam(tbar, barlam):
    bar, lam = barlam
    if bar == tbar:
        return lam
    if bar == tbar.barforeright:
        return 1.0 + (lam if tbar.barforeright.nodeback == tbar.nodefore
                      else 1.0 - lam)
    assert bar == tbar.barforeright.GetForeRightBL(tbar.barforeright.nodeback
                                                   == tbar.nodefore)
    return 2.0 + (1.0 - lam)


def trilamrightangleproj(clam):
    if clam <= 1.0:
        return P2(0.0, clam)
    if clam <= 2.0:
        return P2(clam - 1.0, 2.0 - clam)
    return P2(3.0 - clam, 0.0)


def trilinecrossing(tbar, barlamA0, barlamA1, barlamB0, barlamB1):
    clamA0 = triclambarlam(tbar, barlamA0)
    clamA1 = triclambarlam(tbar, barlamA1)
    clamB0 = triclambarlam(tbar, barlamB0)
    clamB1 = triclambarlam(tbar, barlamB1)
    cseq = [(clamA0, 0), (clamA1, 0), (clamB0, 1), (clamB1, 1)]
    cseq.sort()
    if cseq[0][1] == cseq[1][1] or cseq[1][1] == cseq[2][1]:
        return -1.0
    # project clam values into simple right angle triangle, which is a linear transform of the real triangle, so the parametrized intersection position remains the same
    tA0 = trilamrightangleproj(clamA0)
    tA1 = trilamrightangleproj(clamA1)
    tB0 = trilamrightangleproj(clamB0)
    tB1 = trilamrightangleproj(clamB1)
    vBperp = P2.CPerp(tB1 - tB0)
    vBperpdot = P2.Dot(vBperp, tB0)
    vBperpdotA0 = P2.Dot(vBperp, tA0)
    vBperpdotA1 = P2.Dot(vBperp, tA1)
    lamA = (vBperpdot - vBperpdotA0)/(vBperpdotA1 - vBperpdotA0)
    assert 0.0 <= lamA <= 1.0, lamA
    DptA0 = Along(barlamA0[1], barlamA0[0].nodeback.p, barlamA0[0].nodefore.p)
    DptA1 = Along(barlamA1[1], barlamA1[0].nodeback.p, barlamA1[0].nodefore.p)
    DptB0 = Along(barlamB0[1], barlamB0[0].nodeback.p, barlamB0[0].nodefore.p)
    DptB1 = Along(barlamB1[1], barlamB1[0].nodeback.p, barlamB1[0].nodefore.p)
    DptcrossA = Along(lamA, DptA0, DptA1)
    DvB = DptB1 - DptB0
    DlamB = P3.Dot(DptcrossA - DptB0, DvB)/DvB.Lensq()
    DptcrossB = Along(DlamB, DptB0, DptB1)
    assert (DptcrossB - DptcrossA).Len() < 0.001
    return lamA


def drivecurveintersectionfinder(drivebars, tridrivebarsmap, gb0, gb1,
                                 LRdirection=0):
    tbar = facetbetweenbars(gb0.bar, gb1.bar)
    if tbar.i not in tridrivebarsmap:
        return None
    dseg = tridrivebarsmap[tbar.i]
    Dtbar = facetbetweenbars(drivebars[dseg][0], drivebars[dseg+1][0])
    assert tbar == Dtbar
    dlam = trilinecrossing(tbar, drivebars[dseg], drivebars[dseg+1],
                           (gb0.bar, gb0.lam), (gb1.bar, gb1.lam))
    if dlam == -1.0:
        return None
    res = GBarT(drivebars, dseg, dlam)
    if LRdirection != 0:
        tperpdorapproach = P3.Dot(res.tperp, gb1.pt - gb0.pt)
        if (tperpdorapproach > 0.0) == (LRdirection == 1):
            # print("skipping reverse side crossing point")
            return None
    res.dcseg = dseg
    res.dclam = dlam
    TOL_ZERO(P3.Cross(gb1.tnorm_incoming, res.tnorm).Len())
    res.tnorm_incoming = gb1.tnorm_incoming
    res.gbBackbarC = gb0
    res.gbForebarC = gb1
    return res


def GBCrossBarRS(gb, ptpushfrom, sideslipturningfactor):
    v = gb.bar.nodefore.p - gb.bar.nodeback.p
    vn = P3.ZNorm(v)
    pullforceFrom = P3.ZNorm(gb.pt - ptpushfrom)

    sinalpha = P3.Dot(vn, pullforceFrom)
    cosalpha = math.sqrt(max(0.0, 1.0 - sinalpha**2))

    barforerightBL = gb.bar.GetForeRightBL(gb.bGoRight)
    if barforerightBL is None:
        return None
    tnodeopposite = barforerightBL.GetNodeFore(barforerightBL.nodeback ==
                                               gb.bar.GetNodeFore(gb.bGoRight))

    trisidenorm = P3.ZNorm(P3.Cross(tnodeopposite.p - gb.bar.nodeback.p, v))
    trisideperp = P3.Cross(vn, trisidenorm)
    assert P3.Dot(trisideperp, tnodeopposite.p - gb.pt) >= 0.0

    fromGoRight = gb.bGoRight

    if sideslipturningfactor != 0.0:
        barforerightBLBack = gb.bar.GetForeRightBL(not gb.bGoRight)
        tnodeoppositeBack = barforerightBLBack.GetNodeFore(barforerightBLBack.nodeback ==
                                                           gb.bar.GetNodeFore(not gb.bGoRight))
        trisidenormBack = P3.ZNorm(P3.Cross(tnodeoppositeBack.p -
                                            gb.bar.nodeback.p, v))
        costheta = -P3.Dot(trisidenorm, trisidenormBack)
        tfoldangle = math.acos(costheta)
        siderotangle = sideslipturningfactor*tfoldangle*(-1 if gb.bGoRight
                                                         else 1)
        sinra = math.sin(siderotangle)
        cosra = math.cos(siderotangle)
        sinbeta = sinalpha*cosra + cosalpha*sinra
        cosbeta = cosalpha*cosra - sinalpha*sinra
        TOL_ZERO(math.hypot(sinbeta, cosbeta) - 1.0)
        if cosbeta < 0.0:
            print("bouncing back from glancing edge")
            cosbeta = -cosbeta
            fromGoRight = not gb.bGoRight
            tnodeopposite = tnodeoppositeBack
            trisidenorm = trisidenormBack
            trisideperp = P3.Cross(vn, trisidenorm)
            assert P3.Dot(trisideperp, tnodeopposite.p - gb.pt) >= 0.0

    else:
        sinbeta = sinalpha
        cosbeta = cosalpha

    vecoppouttoPerp = vn*cosbeta - trisideperp*sinbeta
    vecoppouttoPerpD0 = P3.Dot(vecoppouttoPerp, gb.pt)
    vecoppouttoPerpSide = P3.Dot(vecoppouttoPerp, tnodeopposite.p)

    bForeTriSide = (vecoppouttoPerpSide <= vecoppouttoPerpD0)
    barcrossing = (barforerightBL if fromGoRight == bForeTriSide
                   else barforerightBL.GetForeRightBL(barforerightBL.nodefore
                                                      == tnodeopposite))

    if not (barcrossing.GetNodeFore(barcrossing.nodeback == tnodeopposite) ==
            gb.bar.GetNodeFore(bForeTriSide)):
        print("*** print debugs before assert, prob due to unhandled rebound")
        print("sideslipturningfactor", sideslipturningfactor)
        print("gb.bar", gb.bar, gb.bar.nodeback, gb.bar.nodefore, gb.bGoRight)
        print("barforerightBL", barforerightBL)
        print("barcrossing", barcrossing, barcrossing.nodeback,
              barcrossing.nodefore)

    assert barcrossing.GetNodeFore(barcrossing.nodeback ==
                                   tnodeopposite) == gb.bar.GetNodeFore(bForeTriSide)

    vecoppouttoPerpDI = P3.Dot(vecoppouttoPerp,
                               gb.bar.GetNodeFore(bForeTriSide).p)
    lamfromside = ((vecoppouttoPerpD0 - vecoppouttoPerpSide) /
                   (vecoppouttoPerpDI - vecoppouttoPerpSide))
    assert 0.0 <= lamfromside <= 1.0, lamfromside
    lambarcrossing = (lamfromside if barcrossing.nodeback == tnodeopposite
                      else 1.0 - lamfromside)

    Dptbarcrossing = Along(lambarcrossing, barcrossing.nodeback.p,
                           barcrossing.nodefore.p)
    Dsinbeta = P3.Dot(P3.ZNorm(Dptbarcrossing - gb.pt), vn)
    TOL_ZERO(Dsinbeta - sinbeta)
    lambarcrossingGoRight = ((barcrossing.nodeback == tnodeopposite)
                             == (bForeTriSide == fromGoRight))

    return GBarC(barcrossing, lambarcrossing, lambarcrossingGoRight)


class GBarC:
    """
    An object for a continuation of a geodesic crossing an edge containing:
    bar: the edge in the trianglebarmesh being crossed
    lam: the distance along the edge of the crossing point
    bGoRight: a boolean describing the direction of crossing the bar
    pt: the crossing point (as a P3 Vector)
    tnorm_incoming: the normal vector of the face 'before' this bar

    To initialise requires:
    bar: the bar being crossed
    lam: the distance along of the crossing point
    bGoRight: the direction to cross
    """

    def __init__(self, bar, lam, bGoRight):
        self.bar = bar
        self.lam = lam
        self.bGoRight = bGoRight
        self.pt = Along(self.lam, self.bar.nodeback.p, self.bar.nodefore.p)

        # Tries to use 'previous' facet of mesh, but in situations this doesn't
        # work (i.e. edge of part, literal edge case!) then uses 'next' facet
        bfn = barfacetnormal(self.bar, not self.bGoRight)
        if bfn:
            self.tnorm_incoming = bfn
        else:
            self.tnorm_incoming = barfacetnormal(self.bar, self.bGoRight)

    def GBCrossBar(self, ptpushfrom, flatbartangents):
        c, bar, lam, bGoRight = GeoCrossBar(ptpushfrom, self.bar, self.lam,
                                            self.bGoRight, flatbartangents)
        if not bar:
            return None
        res = GBarC(bar, lam, bGoRight)
        TOL_ZERO((c - self.pt).Len())
        return res


class GBarT:
    def __init__(self, drivebars, dseg, dlam):
        self.tbar = facetbetweenbars(drivebars[dseg][0], drivebars[dseg+1][0])
        dpt = Along(drivebars[dseg][1], drivebars[dseg][0].nodeback.p,
                    drivebars[dseg][0].nodefore.p)
        dpt1 = Along(drivebars[dseg+1][1], drivebars[dseg+1][0].nodeback.p,
                     drivebars[dseg+1][0].nodefore.p)
        self.pt = Along(dlam, dpt, dpt1)
        self.vsegN = P3.ZNorm(dpt1 - dpt)
        self.tnorm = barfacetnormal(self.tbar, True)
        self.tperp = P3.Cross(self.vsegN, self.tnorm)

    def drivepointstartfromangle(self, dangle, getbackbar=False):
        perpvec = (-self.vsegN*math.sin(math.radians(dangle)) -
                   self.tperp*math.cos(math.radians(dangle)))
        perpvecDot = P3.Dot(perpvec, self.pt)
        bar, lam, bGoRight = TriangleExitCrossCutPlaneRight(self.tbar, perpvec,
                                                            perpvecDot)
        res = GBarC(bar, lam, bGoRight)
        TOL_ZERO((self.tnorm - res.tnorm_incoming).Len(), "oo")
        if not getbackbar:
            return res
        backbar, backlam, backbGoRight = TriangleExitCrossCutPlaneRight(self.tbar, -perpvec, -perpvecDot)
        resbackbar = GBarC(backbar, backlam, not backbGoRight)
        return res, resbackbar

    def drivecurveanglefromvec(self, vec):
        ang = math.degrees(math.atan2(-P3.Dot(self.tperp, vec),
                                      P3.Dot(self.vsegN, vec)))
        return ang if ang > 0.0 else 360 + ang


def drivegeodesic(drivebars, tridrivebarsmap, dpts, dptcls, ds, dsangle, flatbartangents=None):
    dsseg, dslam = seglampos(ds, dptcls)
    gbStart = GBarT(drivebars, dsseg, dslam)
    gb = gbStart.drivepointstartfromangle(dsangle)
    gbs = [gbStart, gb]
    gbEnd = None
    Nconcavefolds = 0
    while gbEnd is None:
        gb = gbs[-1].GBCrossBar(gbs[-2].pt, flatbartangents)
        if not gb or len(gbs) > 450:
            return gbs, -1, -1
        gbEnd = drivecurveintersectionfinder(drivebars, tridrivebarsmap,
                                             gbs[-1], gb)
        gbs.append(gb if gbEnd is None else gbEnd)

        while len(gbs) >= 3:
            veccurr = gbs[-1].pt - gbs[-2].pt
            TOL_ZERO(P3.Dot(gbs[-1].tnorm_incoming, P3.ZNorm(veccurr)))
            fndot = P3.Dot(P3.ZNorm(veccurr), gbs[-2].tnorm_incoming)
            if fndot >= 0.0:
                break
            Nconcavefolds += 1
            del gbs[-2]
            Dprevtnorm_incoming = gbs[-1].tnorm_incoming
            if not gbEnd:
                gbs[-1].tnorm_incoming = barfacetnormal(gbs[-1].bar,
                                                        not gbs[-1].bGoRight,
                                                        gbs[-2].pt)
            else:
                ltn = P3.Cross(gbs[-1].vsegN, gbs[-2].pt - gbs[-1].pt)
                gbs[-1].tnorm_incoming = P3.ZNorm(ltn if
                                                  P3.Dot(ltn, gbs[-1].tnorm) > 0
                                                  else -ltn)
            assert P3.Dot(gbs[-1].tnorm_incoming, Dprevtnorm_incoming) > 0.8, P3.Dot(gbs[-1].tnorm_incoming, Dprevtnorm_incoming)

    # print("Nconcavefolds removed", Nconcavefolds, "leaving", len(gbs))
    angcross = gbEnd.drivecurveanglefromvec(gbs[-1].pt - gbs[-2].pt)
    dcross = Along(gbEnd.dclam, dptcls[gbEnd.dcseg], dptcls[gbEnd.dcseg+1])
    return gbs, dcross, angcross


def makebicolouredwire(gbs, name, colfront=(1.0, 0.0, 0.0),
                       colback=(0.0, 0.3, 0.0), leadcolornodes=-1):
    """
    Creates a wire in two colours for a geodesic
    Requires:
    gbs: a list of geodesic bar type objects
    name: Name to call the wire
    colfront: Tuple of RGB values to set the colour of the first section
    colback: Tuple of RGB values to set the colour of the second section
    leadcolornodes: number of nodes to colour in front colour or
    -1 to  set it to half of the total number of points (or a maximum of 40)
    """

    # Check if the last point in the list 'gbs' is None type and handle it
    if gbs[-1]:
        # If not None, create a polygon (wire) from the pts in 'gbs' and show
        wire = Part.show(Part.makePolygon([Vector(*gb.pt)
                                           for gb in gbs]), name)  # added[:-1]
    else:
        # If the last point is None, exclude it from the polygon creation
        wire = Part.show(Part.makePolygon([Vector(*gb.pt)
                                           for gb in gbs[:-1]]), name)

    # If 'leadcolornodes' is not specified, set it to half of the total number of points (or a maximum of 40)
    if leadcolornodes == -1:
        leadcolornodes = min(len(gbs)//2, 40)

    # Set the colours for the wire: first 'leadcolornodes' points will have 'colfront' colour,
    # and the remaining points will have 'colback' colour
    wire.ViewObject.LineColorArray = ([colfront]*leadcolornodes +
                                      [colback]*(len(gbs) - leadcolornodes))
    return wire


def geodesic_from_pt(ubtm, startpt, startdirn, wname, sideslipturningfactor=0,
                     maxlength=10000, MAX_SEGMENTS=1000, bothways=False):
    """
    Function to make a geodesic across a mesh from a starting point. Requires:
    ubtm: A UsefulBoxedTriangleMesh object
    startpt: The point to start the goedesic from, as P3 or FreeCAD.Vector
    startdirn: The direction to run the geodesic in, as P3 or FreeCAD.Vector
    wname: the name to give the wire as a string
    sideslipturningfactor: Allowable amount of sideslip
    maxlength: Maximum length for path in drawing units (e.g. mm)
    MAX_SEGMENTS: Maximum length for path in segments
    bothways: If False line only goes in startdirn,
    if True also extends in negative dirn

    returns:
    gbs: A list of the bars crossed by the geodesic of GBarC type
    wire: The wire object
    """
    startbar, startlam = ubtm.FindClosestEdge(startpt, 10)
    gbstart = GBarC(startbar, startlam, True)
    pushpt = gbstart.pt - startdirn  # Create a point before the first point

    # Now need to check which side of the bar the push pt is on to set bGoRight
    vbar = startbar.nodefore.p - startbar.nodeback.p
    vpushpt = pushpt-startbar.nodeback.p
    startnorm = P3.Cross(vbar, vpushpt)
    if P3.Dot(gbstart.tnorm_incoming, startnorm) < 0:
        gbstart.bGoRight = False

    gbs = [gbstart]
    gbFore = GBCrossBarRS(gbs[-1], pushpt, sideslipturningfactor)
    gbs.append(gbFore)
    if gbFore:
        dlength = (gbFore.pt - gbs[-1].pt).Len()
        while True:
            # finds next point along, 'extrapolating' from previous two points along mesh surface
            gbFore = GBCrossBarRS(gbs[-1], gbs[-2].pt, sideslipturningfactor)
            # stop path if gone off edge
            if not gbFore:
                gbs.append(None)
                break
            dlength += (gbFore.pt - gbs[-1].pt).Len()
            # stop path if exceeded max length
            if len(gbs) > MAX_SEGMENTS or (maxlength != -1 and
                                           dlength > maxlength):
                gbs.append(None)
                break
            gbs.append(gbFore)
        if bothways:
            gbs[0].bGoRight = not gbs[0].bGoRight
            while True:
                gbFore = GBCrossBarRS(gbs[0], gbs[1].pt, sideslipturningfactor)
                # stop path if gone off edge
                if not gbFore:
                    break
                dlength += (gbFore.pt - gbs[0].pt).Len()
                # stop path if exceeded max length
                if len(gbs) > MAX_SEGMENTS or (maxlength != -1 and
                                               dlength > maxlength):
                    break
                gbs.insert(0, gbFore)
    else:
        # This is the case for starting on an edge and heading straight off it
        return None, None

    wire = makebicolouredwire(gbs, wname, colfront=(1.0, 0.0, 0.0),
                              colback=(0.0, 0.3, 0.0), leadcolornodes=len(gbs))
    return gbs, wire