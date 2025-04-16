from barmesh.tribarmes import TriangleBarMesh, TriangleBar, MakeTriangleBoxing
from barmesh.basicgeo import I1, Partition1, P3, P2, Along
from utils.curvesutils import cumlengthlist, seglampos
from utils.trianglemeshutils import facetbetweenbars
import math

def Square(X):
    return X*X
def TOL_ZERO(X, msg=""):
    if not (abs(X) < 0.0001):
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
            #print("qdq", qdq)
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
    if bartop != None:
        return bartop.GetNodeFore(bartop.nodeback == bar.GetNodeFore(bGoRight))
    return None
    
def GeoCrossBar(c, bar, lam, bGoRight, flatbartangents):
    Na, Nb = bar.nodeback, bar.nodefore
    d = Along(lam, Na.p, Nb.p)
    Ne = TriangleNodeOpposite(bar, bGoRight)
    if Ne == None:
        #print("GeoCrossBar fail", Na.p, Nb.p, lam, bGoRight, c)
        #print(bar.barforeright, bar.barbackleft)
        return (None, None, 0.0, False)
    if flatbartangents is not None:
        vback, vfore = flatbartangents[bar.i]
        VabInterpolated = Along(lam, vback, vfore)
    else:
        VabInterpolated = Nb.p - Na.p
    bAEcrossing, q, Gx, bEnd = GeoCrossAxis(Na.p, Nb.p, c, lam, Ne.p, VabInterpolated)
    if bGoRight:
        if bAEcrossing:
            bar = bar.barforeright.GetForeRightBL(bar.barforeright.nodeback == Nb)
            lam = q if bar.nodeback == Na else 1-q
            bGoRight = (bar.nodeback == Na)   
        else:
            bar = bar.barforeright
            lam = q if bar.nodeback == Nb else 1-q
            bGoRight = not (bar.nodeback == Nb)
    else:
        if bAEcrossing:
            bar = bar.barbackleft
            lam = q if bar.nodeback == Na else 1-q
            bGoRight = not (bar.nodeback == Na)
        else:
            bar = bar.barbackleft.GetForeRightBL(bar.barbackleft.nodeback == Na)
            lam = q if bar.nodeback == Nb else 1-q
            bGoRight = (bar.nodeback == Nb)
            
    c = bar.nodeback.p + (bar.nodefore.p - bar.nodeback.p)*lam
    TOL_ZERO((c - Gx).Len())
    return (d, bar, lam, bGoRight)
    
    
def barfacetnormal(bar, bGoRight, ptcommon=None):
    nodeAhead = bar.GetNodeFore(bGoRight)
    nodeBehind = bar.GetNodeFore(not bGoRight)
    barAhead = bar.GetForeRightBL(bGoRight)
    barAheadGoRight = (barAhead.nodeback == nodeAhead)
    nodeOpposite = barAhead.GetNodeFore(barAheadGoRight)
    ptc = nodeOpposite.p if ptcommon is None else ptcommon
    v2ahead = nodeAhead.p - ptc
    v2behind = nodeBehind.p - ptc
    return P3.ZNorm(P3.Cross(v2ahead, v2behind))


def InvAlong(v, a, b):
    return (v - a)/(b - a)

def TriangleExitCrossCutPlaneRight(tbar, perpvec, perpvecDot):
    node2 = tbar.barforeright.GetNodeFore(tbar.barforeright.nodeback == tbar.nodefore)
    dpvdnodefore = P3.Dot(perpvec, tbar.nodefore.p)
    dpvdnodeback = P3.Dot(perpvec, tbar.nodeback.p)
    dpvdnode2 = P3.Dot(perpvec, node2.p)
    assert tbar.nodeback.i < tbar.nodefore.i
    
    if dpvdnodefore <= perpvecDot <= dpvdnodeback:
        tbarlam = InvAlong(perpvecDot,dpvdnodeback, dpvdnodefore)
        return tbar, tbarlam, False

    if dpvdnodeback <= perpvecDot <= dpvdnode2:
        tbarlam = InvAlong(perpvecDot, dpvdnodeback, dpvdnode2)
        barbackright = tbar.barforeright.GetForeRightBL(tbar.barforeright.nodeback == tbar.nodefore)
        assert barbackright.nodeback.i < barbackright.nodefore.i
        return barbackright, tbarlam, True

    if dpvdnode2 <= perpvecDot <= dpvdnodefore:
        bforerightFore = (node2.i < tbar.nodefore.i)
        tbarlamF = InvAlong(perpvecDot, dpvdnode2, dpvdnodefore)
        tbarlam = tbarlamF if bforerightFore else 1.0 - tbarlamF
        return tbar.barforeright, tbarlam, bforerightFore

    return None, 0, False         


def triclambarlam(tbar, barlam):
    bar, lam = barlam
    if bar == tbar:
        return lam
    if bar == tbar.barforeright:
        return 1.0 + (lam if tbar.barforeright.nodeback == tbar.nodefore else 1.0 - lam)
    assert bar == tbar.barforeright.GetForeRightBL(tbar.barforeright.nodeback == tbar.nodefore)
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
    cseq = [ (clamA0, 0), (clamA1, 0), (clamB0, 1), (clamB1, 1) ]
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
    

def drivecurveintersectionfinder(drivebars, tridrivebarsmap, gb0, gb1, LRdirection=0):
    tbar = facetbetweenbars(gb0.bar, gb1.bar)
    if tbar.i not in tridrivebarsmap:
        return None
    dseg = tridrivebarsmap[tbar.i]
    Dtbar = facetbetweenbars(drivebars[dseg][0], drivebars[dseg+1][0])
    assert tbar == Dtbar
    dlam = trilinecrossing(tbar, drivebars[dseg], drivebars[dseg+1], (gb0.bar, gb0.lam), (gb1.bar, gb1.lam))
    if dlam == -1.0:
        return None
    res = GBarT(drivebars, dseg, dlam)
    if LRdirection != 0:
        tperpdorapproach = P3.Dot(res.tperp, gb1.pt - gb0.pt)
        if (tperpdorapproach > 0.0) == (LRdirection == 1):
            #print("skipping reverse side crossing point")
            return None
    res.dcseg = dseg
    res.dclam = dlam
    TOL_ZERO(P3.Cross(gb1.tnorm_incoming, res.tnorm).Len())
    res.tnorm_incoming = gb1.tnorm_incoming
    res.gbBackbarC = gb0
    res.gbForebarC = gb1
    return res


    

class GBarC:
    def __init__(self, bar, lam, bGoRight):
        self.bar = bar
        self.lam = lam
        self.bGoRight = bGoRight
        self.pt = Along(self.lam, self.bar.nodeback.p, self.bar.nodefore.p)  # Cauing attribute errors
        self.tnorm_incoming = barfacetnormal(self.bar, not self.bGoRight)

    def GBCrossBar(self, ptpushfrom, flatbartangents):
        c, bar, lam, bGoRight = GeoCrossBar(ptpushfrom, self.bar, self.lam, self.bGoRight, flatbartangents)
        if not bar:
            return None
        res = GBarC(bar, lam, bGoRight)
        TOL_ZERO((c - self.pt).Len())
        return res




class GBarT:
    def __init__(self, drivebars, dseg, dlam):
        self.tbar = facetbetweenbars(drivebars[dseg][0], drivebars[dseg+1][0])
        dpt = Along(drivebars[dseg][1], drivebars[dseg][0].nodeback.p, drivebars[dseg][0].nodefore.p)
        dpt1 = Along(drivebars[dseg+1][1], drivebars[dseg+1][0].nodeback.p, drivebars[dseg+1][0].nodefore.p)
        self.pt = Along(dlam, dpt, dpt1)
        self.vsegN = P3.ZNorm(dpt1 - dpt)
        self.tnorm = barfacetnormal(self.tbar, True)
        self.tperp = P3.Cross(self.vsegN, self.tnorm)

    def drivepointstartfromangle(self, dangle, getbackbar=False):
        perpvec = -self.vsegN*math.sin(math.radians(dangle)) - self.tperp*math.cos(math.radians(dangle))
        perpvecDot = P3.Dot(perpvec, self.pt)
        bar, lam, bGoRight = TriangleExitCrossCutPlaneRight(self.tbar, perpvec, perpvecDot)
        res = GBarC(bar, lam, bGoRight)
        TOL_ZERO((self.tnorm - res.tnorm_incoming).Len(), "oo")
        if not getbackbar:
            return res
        backbar, backlam, backbGoRight = TriangleExitCrossCutPlaneRight(self.tbar, -perpvec, -perpvecDot)
        resbackbar = GBarC(backbar, backlam, not backbGoRight)
        return res, resbackbar

    def drivecurveanglefromvec(self, vec):
        ang = math.degrees(math.atan2(-P3.Dot(self.tperp, vec), P3.Dot(self.vsegN, vec)))
        return ang if ang > 0.0 else 360 + ang
        


def drivegeodesic(drivebars, tridrivebarsmap, dpts, dptcls, ds, dsangle, flatbartangents=None):
    dsseg, dslam = seglampos(ds, dptcls)
    gbStart = GBarT(drivebars, dsseg, dslam)
    gb = gbStart.drivepointstartfromangle(dsangle)
    gbs = [ gbStart, gb ]
    gbEnd = None
    Nconcavefolds = 0
    while gbEnd is None:
        gb = gbs[-1].GBCrossBar(gbs[-2].pt, flatbartangents)
        if not gb or len(gbs) > 450:
            return gbs, -1, -1
        gbEnd = drivecurveintersectionfinder(drivebars, tridrivebarsmap, gbs[-1], gb)
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
                gbs[-1].tnorm_incoming = barfacetnormal(gbs[-1].bar, not gbs[-1].bGoRight, gbs[-2].pt)
            else:
                ltn = P3.Cross(gbs[-1].vsegN, gbs[-2].pt - gbs[-1].pt)
                gbs[-1].tnorm_incoming = P3.ZNorm(ltn if P3.Dot(ltn, gbs[-1].tnorm) > 0 else -ltn)
            assert P3.Dot(gbs[-1].tnorm_incoming, Dprevtnorm_incoming) > 0.8, P3.Dot(gbs[-1].tnorm_incoming, Dprevtnorm_incoming)

    #print("Nconcavefolds removed", Nconcavefolds, "leaving", len(gbs))
    angcross = gbEnd.drivecurveanglefromvec(gbs[-1].pt - gbs[-2].pt)
    dcross = Along(gbEnd.dclam, dptcls[gbEnd.dcseg], dptcls[gbEnd.dcseg+1])
    #print("angcross", angcross, dcross)
    return gbs, dcross, angcross