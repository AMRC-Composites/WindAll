import math
import Part
from FreeCAD import Vector
from barmesh.basicgeo import P3, P2, Along
from utils.curvesutils import cumlengthlist, seglampos
from utils.trianglemeshutils import UsefulBoxedTriangleMesh, facetbetweenbars
from utils.wireembeddingutils import planecutembeddedcurve, planecutbars
from utils.geodesicutils import InvAlong, GBarT, GBarC, TOL_ZERO
from utils.geodesicutils import GBCrossBarRS, drivecurveintersectionfinder


def calcfriction(gb, pt0, pt1):
    vn = P3.ZNorm(gb.bar.nodefore.p - gb.bar.nodeback.p)
    pullforce = P3.ZNorm(gb.pt - pt0) + P3.ZNorm(gb.pt - pt1)
    return calcfriccoeff(pullforce, vn)


def calcfriccoeff(pullforce, vn):
    alongforce = P3.Dot(pullforce, vn)
    sideforcesq = pullforce.Lensq() - alongforce*alongforce
    assert sideforcesq >= -0.001
    sideforce = math.sqrt(max(0, sideforcesq))
    return alongforce/sideforce


def calcfriccoeffbarEnds(pullforceFrom, vn):
    alongforce = P3.Dot(pullforceFrom, vn)
    sideforcesq = pullforceFrom.Lensq() - alongforce*alongforce
    assert sideforcesq >= -0.001
    sideforce = math.sqrt(max(0, sideforcesq))
    return (alongforce + 1.0)/sideforce, (alongforce - 1.0)/sideforce


def drivesetBFstartfromangle(drivebars, dpts, dptcls, ds, dsangle):
    dsseg, dslam = seglampos(ds, dptcls)
    gbStart = GBarT(drivebars, dsseg, dslam)
    gb, gbbackbar = gbStart.drivepointstartfromangle(dsangle, getbackbar=True)
    gbStart.gbBackbarC = gbbackbar
    gbStart.gbForebarC = gb
    gbStart.dcseg = dsseg
    gbStart.dclam = dslam
    return gbStart


def drivegeodesicRI(gbStart, drivebars, tridrivebarsmap, LRdirection=1,
                    sideslipturningfactor=0.0,
                    MAX_SEGMENTS=2000, maxlength=-1):
    # initialises first two points of gbs located on drivecurve and distance dlength from drivecurve at angle 'dsangle'
    gbs = [gbStart, gbStart.gbForebarC]
    # calculates dlength for distance between first two points
    dlength = (gbs[1].pt - gbs[0].pt).Len()

    while True:
        # finds next point along, 'extrapolating' from previous two points along mesh surface
        gbFore = GBCrossBarRS(gbs[-1], gbs[-2].pt, sideslipturningfactor)
        # stop path if gone off edge
        if not gbFore:
            gbs.append(None)
            break
        dlength += (gbFore.pt - gbs[-1].pt).Len()
        # stop path if exceeded max length
        if len(gbs) > MAX_SEGMENTS or (maxlength != -1 and dlength > maxlength):
            gbs.append(None)
            break
        # check if reached end (back at drivecurve). drivecurveintersectionfinder in geodesicutils
        gbEnd = drivecurveintersectionfinder(drivebars, tridrivebarsmap,
                                             gbs[-1], gbFore,
                                             LRdirection=LRdirection)
        if gbEnd:
            gbs.append(gbEnd)
            break
        gbs.append(gbFore)
    # print('LENGTH:', dlength)
    return gbs


class DriveCurve:
    def __init__(self, drivebars):
        self.drivebars = drivebars
        self.tridrivebarsmap = dict((facetbetweenbars(drivebars[dseg][0], 
                                                      drivebars[dseg+1][0]).i,
                                     dseg)
                                    for dseg in range(len(drivebars)-1))
        self.dpts = [Along(lam, bar.nodeback.p, bar.nodefore.p)
                     for bar, lam in drivebars]
        self.dptcls = cumlengthlist(self.dpts)

    def startalongangle(self, alongwire, dsangle):
        ds = Along(alongwire, self.dptcls[0], self.dptcls[-1])
        gbStart = drivesetBFstartfromangle(self.drivebars, self.dpts,
                                           self.dptcls, ds, dsangle)
        Dds = Along(gbStart.dclam, self.dptcls[gbStart.dcseg],
                    self.dptcls[gbStart.dcseg + 1])
        Dalongwire = InvAlong(Dds, self.dptcls[0], self.dptcls[-1])
        TOL_ZERO(alongwire - Dalongwire)
        Ddsangle = 180 + gbStart.drivecurveanglefromvec(gbStart.gbBackbarC.pt -
                                                        gbStart.gbForebarC.pt)
        TOL_ZERO(((dsangle - Ddsangle + 180) % 360) - 180)
        return gbStart

    def endalongpositionA(self, gbEnd):
        dslanded = Along(gbEnd.dclam, self.dptcls[gbEnd.dcseg],
                         self.dptcls[gbEnd.dcseg + 1])
        alongwirelanded = InvAlong(dslanded, self.dptcls[0], self.dptcls[-1])
        angcross = gbEnd.drivecurveanglefromvec(gbEnd.gbForebarC.pt -
                                                gbEnd.gbBackbarC.pt)
        return alongwirelanded, angcross

    def endalongposition(self, gbEnd):
        return self.endalongpositionA(gbEnd)[0]


def makedrivecurve(sketchplane, utbm, mandrelradius):
    driveperpvec = sketchplane.Placement.Rotation.multVec(Vector(0, 0, 1))
    driveperpvecDot = driveperpvec.dot(sketchplane.Placement.Base)
    startbar, startlam = planecutbars(utbm.tbarmesh, driveperpvec,
                                      driveperpvecDot)
    drivebars = planecutembeddedcurve(startbar, startlam, driveperpvec)
    drivecurve = DriveCurve(drivebars)
    return drivecurve


def directedgeodesic(combofoldbackmode, sketchplane_or_drivecurve,
                     meshobject_or_utbm, alongwire, alongwireI, dsangle,
                     Maxsideslipturningfactor, mandrelradius,
                     sideslipturningfactorZ, maxlength,
                     outputfilament, showpaths):
    utbm = meshobject_or_utbm if isinstance(meshobject_or_utbm, UsefulBoxedTriangleMesh)else UsefulBoxedTriangleMesh(meshobject_or_utbm.Mesh)
    drivecurve = sketchplane_or_drivecurve if isinstance(sketchplane_or_drivecurve, DriveCurve) else makedrivecurve(sketchplane_or_drivecurve, utbm, mandrelradius)
    gbStart = drivecurve.startalongangle(alongwire, dsangle)
    fLRdirection = 1 if ((dsangle + 360) % 360) < 180.0 else -1
    if combofoldbackmode != 0:
        fLRdirection = -fLRdirection

    gbs = drivegeodesicRI(gbStart, drivecurve.drivebars,
                          drivecurve.tridrivebarsmap, LRdirection=fLRdirection,
                          sideslipturningfactor=sideslipturningfactorZ,
                          maxlength=maxlength)
    if gbs[-1] is None:
        if outputfilament:
            if showpaths == 'All paths':
                Part.show(Part.makePolygon([Vector(*gb.pt)
                                            for gb in gbs[:-1]]),
                          outputfilament)

        return gbs, 0, -1, None

    alongwirelanded = drivecurve.endalongposition(gbs[-1])
    if alongwireI is None:
        return gbs, 0, -1, alongwirelanded

    gbsS = [gbs[0].gbBackbarC] + gbs[1:-1] + [gbs[-1].gbForebarC]
    drivebarsB = [(gb.bar, gb.lam) for gb in gbsS]
    tridrivebarsmapB = dict((facetbetweenbars(drivebarsB[dseg][0],
                                              drivebarsB[dseg+1][0]).i, dseg)
                            for dseg in range(len(drivebarsB)-1))

    alongwireI1 = min([alongwireI, alongwireI+1], key=lambda X: abs(X - alongwirelanded))
    LRdirectionI = 1 if (alongwireI1 > alongwirelanded) else -1
    sideslipturningfactor = Maxsideslipturningfactor*LRdirectionI

    dsangleI = dsangle+180.0 if combofoldbackmode == 0 else 180.0-dsangle
    gbStartI = drivecurve.startalongangle(alongwireI, dsangleI)
    gbsI = drivegeodesicRI(gbStartI, drivebarsB, tridrivebarsmapB,
                           sideslipturningfactor=sideslipturningfactor,
                           LRdirection=LRdirectionI, MAX_SEGMENTS=len(gbs))

    if gbsI[-1] is None:
        if outputfilament:
            Part.show(Part.makePolygon([Vector(*gb.pt)
                                        for gb in gbs]), outputfilament)
            Part.show(Part.makePolygon([Vector(*gb.pt)
                                        for gb in gbsI[:-1]]), outputfilament)
        return

    for j in range(2):
        sideslipturningfactor *= 0.75
        gbsIN = drivegeodesicRI(gbStartI, drivebarsB, tridrivebarsmapB,
                                sideslipturningfactor=sideslipturningfactor,
                                LRdirection=LRdirectionI, MAX_SEGMENTS=len(gbs))
        if gbsIN[-1] is not None:
            gbsI = gbsIN
            print("smaller sideslipturningfactor", sideslipturningfactor, "worked")
        else:
            break

    # Making join bit
    dseg = tridrivebarsmapB[gbsI[-1].tbar.i]
    gbarT1 = gbsI[0]
    gbarT1.gbBackbarC, gbarT1.gbForebarC = gbarT1.gbForebarC, gbarT1.gbBackbarC
    gbsjoined = gbs[:dseg+1] + [GBarC(gb.bar, gb.lam, not gb.bGoRight)
                                for gb in gbsI[-2:0:-1]] + [gbarT1]
    return gbsjoined, fLRdirection, dseg, None


# Modified directedgeodesic to find angle out. Duplicated to avoid affect others files with dependencies to directedgeodesic
def directedgeodesic_find_AngCross(combofoldbackmode, sketchplane_or_drivecurve,
                                   meshobject_or_utbm, alongwire, alongwireI,
                                   dsangle, Maxsideslipturningfactor,
                                   mandrelradius, sideslipturningfactorZ,
                                   maxlength, outputfilament):

    utbm = meshobject_or_utbm if isinstance(meshobject_or_utbm, UsefulBoxedTriangleMesh) else UsefulBoxedTriangleMesh(meshobject_or_utbm.Mesh)
    drivecurve = sketchplane_or_drivecurve if isinstance(sketchplane_or_drivecurve, DriveCurve) else makedrivecurve(sketchplane_or_drivecurve, utbm, mandrelradius)
    gbStart = drivecurve.startalongangle(alongwire, dsangle)
    fLRdirection = 1 if ((dsangle + 360) % 360) < 180.0 else -1
    if combofoldbackmode != 0:
        fLRdirection = -fLRdirection

    gbs = drivegeodesicRI(gbStart, drivecurve.drivebars,
                          drivecurve.tridrivebarsmap, LRdirection=fLRdirection,
                          sideslipturningfactor=sideslipturningfactorZ,
                          maxlength=maxlength)
    if gbs[-1] is None:
        return gbs, 0, -1, None, None

    alongwirelanded = drivecurve.endalongposition(gbs[-1])
    alongwirelanded, angcross = drivecurve.endalongpositionA(gbs[-1])

    if alongwireI is None:
        return gbs, 0, -1, alongwirelanded, angcross

    gbsS = [gbs[0].gbBackbarC] + gbs[1:-1] + [gbs[-1].gbForebarC]
    drivebarsB = [(gb.bar, gb.lam) for gb in gbsS]
    tridrivebarsmapB = dict((facetbetweenbars(drivebarsB[dseg][0],
                                              drivebarsB[dseg+1][0]).i, dseg)
                            for dseg in range(len(drivebarsB)-1))

    alongwireI1 = min([alongwireI, alongwireI+1], key=lambda X: abs(X - alongwirelanded))
    LRdirectionI = 1 if (alongwireI1 > alongwirelanded) else -1
    sideslipturningfactor = Maxsideslipturningfactor*LRdirectionI
    dsangleI = dsangle+180.0 if combofoldbackmode == 0 else 180.0-dsangle
    gbStartI = drivecurve.startalongangle(alongwireI, dsangleI)
    gbsI = drivegeodesicRI(gbStartI, drivebarsB, tridrivebarsmapB,
                           sideslipturningfactor=sideslipturningfactor,
                           LRdirection=LRdirectionI, MAX_SEGMENTS=len(gbs))

    if gbsI[-1] is None:    # Worked with -2?
        print("Reversed path did not intersect")
        return

    for j in range(2):
        sideslipturningfactor *= 0.75
        gbsIN = drivegeodesicRI(gbStartI, drivebarsB, tridrivebarsmapB,
                                sideslipturningfactor=sideslipturningfactor,
                                LRdirection=LRdirectionI, MAX_SEGMENTS=len(gbs))
        if gbsIN[-1] is not None:
            gbsI = gbsIN
            print("smaller sideslipturningfactor", sideslipturningfactor,
                  "worked")
        else:
            break

    # Making join bit
    dseg = tridrivebarsmapB[gbsI[-1].tbar.i]
    gbarT1 = gbsI[0]
    gbarT1.gbBackbarC, gbarT1.gbForebarC = gbarT1.gbForebarC, gbarT1.gbBackbarC
    gbsjoined = gbs[:dseg+1] + [GBarC(gb.bar, gb.lam, not gb.bGoRight)
                                for gb in gbsI[-2:0:-1]] + [gbarT1]
    return gbsjoined, fLRdirection, dseg, None, None
    # Make copy of directed geodesic to output Ang cross of end of path


def directedgeodesicalongangcross(combofoldbackmode, sketchplane_or_drivecurve,
                                  meshobject_or_utbm, alongwire, alongwireI,
                                  dsangle, Maxsideslipturningfactor,
                                  mandrelradius, sideslipturningfactorZ,
                                  maxlength, outputfilament):

    utbm = meshobject_or_utbm if isinstance(meshobject_or_utbm, UsefulBoxedTriangleMesh) else UsefulBoxedTriangleMesh(meshobject_or_utbm.Mesh)
    drivecurve = sketchplane_or_drivecurve if isinstance(sketchplane_or_drivecurve, DriveCurve) else makedrivecurve(sketchplane_or_drivecurve, utbm, mandrelradius)
    gbStart = drivecurve.startalongangle(alongwire, dsangle)
    fLRdirection = 1 if ((dsangle + 360) % 360) < 180.0 else -1
    if combofoldbackmode != 0:
        fLRdirection = -fLRdirection

    gbs = drivegeodesicRI(gbStart, drivecurve.drivebars,
                          drivecurve.tridrivebarsmap, LRdirection=fLRdirection,
                          sideslipturningfactor=sideslipturningfactorZ,
                          maxlength=maxlength)
    if gbs[-1] is None:
        return gbs, 0, -1, None, None

    alongwirelanded = drivecurve.endalongposition(gbs[-1])
    alongwirelanded, angcross = drivecurve.endalongpositionA(gbs[-1])

    if alongwireI is None:
        return gbs, 0, -1, alongwirelanded, angcross

    gbsS = [gbs[0].gbBackbarC] + gbs[1:-1] + [gbs[-1].gbForebarC]
    drivebarsB = [(gb.bar, gb.lam) for gb in gbsS]
    tridrivebarsmapB = dict((facetbetweenbars(drivebarsB[dseg][0],
                                              drivebarsB[dseg+1][0]).i, dseg)
                            for dseg in range(len(drivebarsB)-1))

    alongwireI1 = min([alongwireI, alongwireI+1], key=lambda X: abs(X - alongwirelanded))
    LRdirectionI = 1 if (alongwireI1 > alongwirelanded) else -1
    sideslipturningfactor = Maxsideslipturningfactor*LRdirectionI
    dsangleI = dsangle+180.0 if combofoldbackmode == 0 else 180.0-dsangle
    gbStartI = drivecurve.startalongangle(alongwireI, dsangleI)
    gbsI = drivegeodesicRI(gbStartI, drivebarsB, tridrivebarsmapB,
                           sideslipturningfactor=sideslipturningfactor,
                           LRdirection=LRdirectionI, MAX_SEGMENTS=len(gbs))

    if gbsI[-1] is None:    # Worked with -2?
        print("Reversed path did not intersect")
        return

    for j in range(2):
        sideslipturningfactor *= 0.75
        gbsIN = drivegeodesicRI(gbStartI, drivebarsB, tridrivebarsmapB,
                                sideslipturningfactor=sideslipturningfactor,
                                LRdirection=LRdirectionI, MAX_SEGMENTS=len(gbs))
        if gbsIN[-1] is not None:
            gbsI = gbsIN
            print("smaller sideslipturningfactor", sideslipturningfactor, "worked")
        else:
            break

    # Making join bit
    dseg = tridrivebarsmapB[gbsI[-1].tbar.i]
    gbarT1 = gbsI[0]
    gbarT1.gbBackbarC, gbarT1.gbForebarC = gbarT1.gbForebarC, gbarT1.gbBackbarC
    gbsjoined = gbs[:dseg+1] + [GBarC(gb.bar, gb.lam, not gb.bGoRight)
                                for gb in gbsI[-2:0:-1]] + [gbarT1]
    return gbsjoined, fLRdirection, dseg, None, None
    # Make copy of directed geodesic to output Ang cross of end of path


def windingangle(gbs, rotplanevecX, rotplanevecY):
    prevFV = None
    sumA = 0.0
    for gb in gbs:
        FV = P2(P3.Dot(rotplanevecX, gb.pt), P3.Dot(rotplanevecY, gb.pt))
        if prevFV is not None:
            dvFA = P2(P2.Dot(prevFV, FV), P2.Dot(P2.APerp(prevFV), FV)).Arg()
            sumA += dvFA
        prevFV = FV
    return sumA
