
import math
import Part
from FreeCAD import Vector, Rotation
from barmesh.basicgeo import I1, Partition1, P3, P2, Along
from utils.curvesutils import isdiscretizableobject, discretizeobject, cumlengthlist, seglampos 
from utils.trianglemeshutils import UsefulBoxedTriangleMesh, facetbetweenbars
from utils.wireembeddingutils import planecutembeddedcurve, planecutbars

from utils.geodesicutils import drivegeodesic, InvAlong, GBarT, GBarC, drivecurveintersectionfinder, trilinecrossing, TOL_ZERO


def calcfriccoeff(pullforce, vn):
    alongforce = P3.Dot(pullforce, vn)
    sideforcesq = pullforce.Lensq() - alongforce*alongforce
    assert sideforcesq >= -0.001
    sideforce = math.sqrt(max(0, sideforcesq))
    return alongforce/sideforce

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



def GBCrossBarRS(gb, ptpushfrom, sideslipturningfactor):
    v = gb.bar.nodefore.p - gb.bar.nodeback.p
    vn = P3.ZNorm(v)
    pullforceFrom = P3.ZNorm(gb.pt - ptpushfrom)

    sinalpha = P3.Dot(vn, pullforceFrom)
    cosalpha = math.sqrt(max(0.0, 1.0 - sinalpha**2))

    barforerightBL = gb.bar.GetForeRightBL(gb.bGoRight)
    if barforerightBL is None:
        return None
    tnodeopposite = barforerightBL.GetNodeFore(barforerightBL.nodeback == gb.bar.GetNodeFore(gb.bGoRight))

    trisidenorm = P3.ZNorm(P3.Cross(tnodeopposite.p - gb.bar.nodeback.p, v))
    trisideperp = P3.Cross(vn, trisidenorm)
    assert P3.Dot(trisideperp, tnodeopposite.p - gb.pt) >= 0.0
    
    fromGoRight = gb.bGoRight
    
    if sideslipturningfactor != 0.0:
        barforerightBLBack = gb.bar.GetForeRightBL(not gb.bGoRight)
        tnodeoppositeBack = barforerightBLBack.GetNodeFore(barforerightBLBack.nodeback == gb.bar.GetNodeFore(not gb.bGoRight))
        trisidenormBack = P3.ZNorm(P3.Cross(tnodeoppositeBack.p - gb.bar.nodeback.p, v))
        costheta = -P3.Dot(trisidenorm, trisidenormBack)
        tfoldangle = math.acos(costheta)
        siderotangle = sideslipturningfactor*tfoldangle*(-1 if gb.bGoRight else 1)
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

    vecoppoutto = vn*sinbeta + trisideperp*cosbeta
    vecoppouttoPerp = vn*cosbeta - trisideperp*sinbeta
    vecoppouttoPerpD0 = P3.Dot(vecoppouttoPerp, gb.pt)
    vecoppouttoPerpSide = P3.Dot(vecoppouttoPerp, tnodeopposite.p)

    bForeTriSide = (vecoppouttoPerpSide <= vecoppouttoPerpD0)
    barcrossing = barforerightBL if fromGoRight == bForeTriSide else barforerightBL.GetForeRightBL(barforerightBL.nodefore == tnodeopposite)

    if not (barcrossing.GetNodeFore(barcrossing.nodeback == tnodeopposite) == gb.bar.GetNodeFore(bForeTriSide)):
        print("*** print debugs before assert, prob due to unhandled bounce back")
        print("sideslipturningfactor", sideslipturningfactor)
        print("gb.bar", gb.bar, gb.bar.nodeback, gb.bar.nodefore, gb.bGoRight)
        print("barforerightBL", barforerightBL)
        print("barcrossing", barcrossing, barcrossing.nodeback, barcrossing.nodefore)

    assert barcrossing.GetNodeFore(barcrossing.nodeback == tnodeopposite) == gb.bar.GetNodeFore(bForeTriSide)

    vecoppouttoPerpDI = P3.Dot(vecoppouttoPerp, gb.bar.GetNodeFore(bForeTriSide).p)
    lamfromside = (vecoppouttoPerpD0 - vecoppouttoPerpSide) / (vecoppouttoPerpDI - vecoppouttoPerpSide)
    assert 0.0 <= lamfromside <= 1.0, lamfromside
    lambarcrossing = lamfromside if barcrossing.nodeback == tnodeopposite else 1.0 - lamfromside
    
    Dptbarcrossing = Along(lambarcrossing, barcrossing.nodeback.p, barcrossing.nodefore.p)
    Dsinbeta = P3.Dot(P3.ZNorm(Dptbarcrossing - gb.pt), vn)
    TOL_ZERO(Dsinbeta - sinbeta)
    lambarcrossingGoRight = (barcrossing.nodeback == tnodeopposite) == (bForeTriSide == fromGoRight)

    return GBarC(barcrossing, lambarcrossing, lambarcrossingGoRight)


def drivesetBFstartfromangle(drivebars, dpts, dptcls, ds, dsangle):
    dsseg, dslam = seglampos(ds, dptcls)
    gbStart = GBarT(drivebars, dsseg, dslam)
    gb, gbbackbar = gbStart.drivepointstartfromangle(dsangle, getbackbar=True)
    gbStart.gbBackbarC = gbbackbar
    gbStart.gbForebarC = gb
    gbStart.dcseg = dsseg
    gbStart.dclam = dslam
    return gbStart


def drivegeodesicRI(gbStart, drivebars, tridrivebarsmap, LRdirection=1, sideslipturningfactor=0.0, MAX_SEGMENTS=2000, maxlength=-1):
    #initialises first two points of gbs located on drivecurve and distance dlength from drivecurve at angle 'dsangle'
    gbs = [ gbStart, gbStart.gbForebarC ]
    #calculates dlength for distance between first two points
    dlength = (gbs[1].pt - gbs[0].pt).Len()
    Nconcavefolds = 0
    while True:
        #finds next point along, 'extrapolating' from previous two points along mesh surface
        gbFore = GBCrossBarRS(gbs[-1], gbs[-2].pt, sideslipturningfactor)
        #stop path if gone off edge
        if not gbFore:
            #print("gone off edge")
            gbs.append(None)
            break
        dlength += (gbFore.pt - gbs[-1].pt).Len()
        #stop path if exceeded max length
        if len(gbs) > MAX_SEGMENTS or (maxlength != -1 and dlength > maxlength):
            gbs.append(None)
            break
        #check if reached end (back at drivecurve). drivecurveintersectionfinder in geodesicutils
        gbEnd = drivecurveintersectionfinder(drivebars, tridrivebarsmap, gbs[-1], gbFore, LRdirection=LRdirection)
        if gbEnd:
            gbs.append(gbEnd)
            break
        gbs.append(gbFore)
    # print('LENGTH:', dlength)
    return gbs

def makebicolouredwire(gbs, name, colfront=(1.0,0.0,0.0), colback=(0.0,0.3,0.0), leadcolornodes=-1):

    # Check if the last point in the list 'gbs' is None type and handle it
    if gbs[-1]:
        # If not None, create a polygon (wire) from the points in 'gbs' and show it
        wire = Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in gbs]), name)  ## added [:-1]
    else:
        # If the last point is None, exclude it from the polygon creation
        wire = Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in gbs[:-1]]), name)

    # If 'leadcolornodes' is not specified, set it to half of the total number of points (or a maximum of 40)
    if leadcolornodes == -1:
        leadcolornodes = min(len(gbs)//2, 40)

    # Set the colours for the wire: first 'leadcolornodes' points will have 'colfront' colour, 
    # and the remaining points will have 'colback' colour
    wire.ViewObject.LineColorArray= [colfront]*leadcolornodes + [colback]*(len(gbs) - leadcolornodes)
    return wire


class DriveCurve:
    def __init__(self, drivebars):
        self.drivebars = drivebars
        self.tridrivebarsmap = dict((facetbetweenbars(drivebars[dseg][0], drivebars[dseg+1][0]).i, dseg)  for dseg in range(len(drivebars)-1))
        self.dpts = [ Along(lam, bar.nodeback.p, bar.nodefore.p)  for bar, lam in drivebars ]
        self.dptcls = cumlengthlist(self.dpts)

    def startalongangle(self, alongwire, dsangle):
        ds = Along(alongwire, self.dptcls[0], self.dptcls[-1])
        gbStart = drivesetBFstartfromangle(self.drivebars, self.dpts, self.dptcls, ds, dsangle)
        Dds = Along(gbStart.dclam, self.dptcls[gbStart.dcseg], self.dptcls[gbStart.dcseg + 1])
        Dalongwire = InvAlong(Dds, self.dptcls[0], self.dptcls[-1])
        TOL_ZERO(alongwire - Dalongwire)
        Ddsangle = 180 + gbStart.drivecurveanglefromvec(gbStart.gbBackbarC.pt - gbStart.gbForebarC.pt)
        TOL_ZERO(((dsangle - Ddsangle + 180)%360) - 180)
        return gbStart

    def endalongpositionA(self, gbEnd):
        dslanded = Along(gbEnd.dclam, self.dptcls[gbEnd.dcseg], self.dptcls[gbEnd.dcseg + 1])
        alongwirelanded = InvAlong(dslanded, self.dptcls[0], self.dptcls[-1])
        angcross = gbEnd.drivecurveanglefromvec(gbEnd.gbForebarC.pt - gbEnd.gbBackbarC.pt)
        #print("angcross", angcross)
        return alongwirelanded, angcross

    def endalongposition(self, gbEnd):
        return self.endalongpositionA(gbEnd)[0]


def makedrivecurve(sketchplane, utbm, mandrelradius):
    mandrelgirth = 2*math.pi*mandrelradius
    driveperpvec = sketchplane.Placement.Rotation.multVec(Vector(0,0,1))
    driveperpvecDot = driveperpvec.dot(sketchplane.Placement.Base)
    rotplanevecX = sketchplane.Placement.Rotation.multVec(Vector(1,0,0))
    rotplanevecY = sketchplane.Placement.Rotation.multVec(Vector(0,1,0))
    startbar, startlam = planecutbars(utbm.tbarmesh, driveperpvec, driveperpvecDot)
    drivebars = planecutembeddedcurve(startbar, startlam, driveperpvec)
    drivecurve = DriveCurve(drivebars)
    # print("girth comparison", 'Nominal:',mandrelgirth, 'Drivecurve length:',drivecurve.dptcls[-1])
    return drivecurve

def directedgeodesic(combofoldbackmode, sketchplane_or_drivecurve, meshobject_or_utbm, 
                     alongwire, alongwireI, dsangle, Maxsideslipturningfactor,
                     mandrelradius, sideslipturningfactorZ, maxlength, outputfilament, showpaths):
    utbm = meshobject_or_utbm if isinstance(meshobject_or_utbm, UsefulBoxedTriangleMesh) else UsefulBoxedTriangleMesh(meshobject_or_utbm.Mesh)
    drivecurve = sketchplane_or_drivecurve if isinstance(sketchplane_or_drivecurve, DriveCurve) else makedrivecurve(sketchplane_or_drivecurve, utbm, mandrelradius)
    gbStart = drivecurve.startalongangle(alongwire, dsangle)
    fLRdirection = 1 if ((dsangle + 360)%360) < 180.0 else -1
    if combofoldbackmode != 0:
        fLRdirection = -fLRdirection

    #print("doing alongwire %.2f foldback=%d  alongwireI %.2f" % (alongwire, fLRdirection, alongwireI or 0))
    gbs = drivegeodesicRI(gbStart, drivecurve.drivebars, drivecurve.tridrivebarsmap, LRdirection=fLRdirection, sideslipturningfactor=sideslipturningfactorZ, maxlength=maxlength)
    if gbs[-1] == None:
        if outputfilament:
            if showpaths == 'All paths':
                Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in gbs[:-1]]), outputfilament)

        return gbs, 0, -1, None
        
    alongwirelanded = drivecurve.endalongposition(gbs[-1])
    if alongwireI is None:
        wirelength = sum((gb2.pt - gb1.pt).Len()  for gb1, gb2 in zip(gbs, gbs[1:]))
        #makebicolouredwire(gbs, outputfilament, colfront=(1.0,0.0,0.0), colback =(0.0,0.6,0.0) if abs(dsangle) < 90 else (0.0,0.0,0.9))
        #print("alongwirelanded %4f  leng=%.2f   ** setting AlngWre advance to the difference" % (alongwirelanded, wirelength))
        return gbs, 0, -1, alongwirelanded
    
    gbsS = [ gbs[0].gbBackbarC ] + gbs[1:-1] + [ gbs[-1].gbForebarC ]
    drivebarsB = [ (gb.bar, gb.lam)  for gb in gbsS ]
    tridrivebarsmapB = dict((facetbetweenbars(drivebarsB[dseg][0], drivebarsB[dseg+1][0]).i, dseg)  for dseg in range(len(drivebarsB)-1))

    alongwireI1 = min([alongwireI, alongwireI+1], key=lambda X: abs(X - alongwirelanded))
    LRdirectionI = 1 if (alongwireI1 > alongwirelanded) else -1
    sideslipturningfactor = Maxsideslipturningfactor*LRdirectionI
    
    dsangleI = dsangle+180.0 if combofoldbackmode == 0 else 180.0-dsangle
    #print("dsangleI", dsangleI, dsangle, combofoldbackmode)
    gbStartI = drivecurve.startalongangle(alongwireI, dsangleI)
    gbsI = drivegeodesicRI(gbStartI, drivebarsB, tridrivebarsmapB, sideslipturningfactor=sideslipturningfactor, LRdirection=LRdirectionI, MAX_SEGMENTS=len(gbs))
    
    if gbsI[-1] == None:
        if outputfilament:
            Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in gbs]), outputfilament)
            Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in gbsI[:-1]]), outputfilament)
        return

    for j in range(2):
        sideslipturningfactor *= 0.75
        gbsIN = drivegeodesicRI(gbStartI, drivebarsB, tridrivebarsmapB, sideslipturningfactor=sideslipturningfactor, LRdirection=LRdirectionI, MAX_SEGMENTS=len(gbs))
        if gbsIN[-1] != None:
            gbsI = gbsIN
            print("smaller sideslipturningfactor", sideslipturningfactor, "worked")
        else:
            break
            
    #print("making join bit")
    dseg = tridrivebarsmapB[gbsI[-1].tbar.i]
    gbarT1 = gbsI[0]
    gbarT1.gbBackbarC, gbarT1.gbForebarC = gbarT1.gbForebarC, gbarT1.gbBackbarC
    gbsjoined = gbs[:dseg+1] + [ GBarC(gb.bar, gb.lam, not gb.bGoRight)  for gb in gbsI[-2:0:-1] ] + [ gbarT1 ]
    #makebicolouredwire(gbsjoined, outputfilament, colfront=(1.0,0.0,0.0) if fLRdirection == -1 else (0.0,0.0,1.0), colback=(0.7,0.7,0.0), leadcolornodes=dseg+1)
    return gbsjoined, fLRdirection, dseg, None


# Modified directedgeodesic to find angle out. Duplicated to avoid affect others files with dependencies to directedgeodesic  
def directedgeodesic_find_AngCross(combofoldbackmode, sketchplane_or_drivecurve, meshobject_or_utbm, 
                     alongwire, alongwireI, dsangle, Maxsideslipturningfactor,
                     mandrelradius, sideslipturningfactorZ, maxlength, outputfilament):
    
    utbm = meshobject_or_utbm if isinstance(meshobject_or_utbm, UsefulBoxedTriangleMesh) else UsefulBoxedTriangleMesh(meshobject_or_utbm.Mesh)
    drivecurve = sketchplane_or_drivecurve if isinstance(sketchplane_or_drivecurve, DriveCurve) else makedrivecurve(sketchplane_or_drivecurve, utbm, mandrelradius)
    gbStart = drivecurve.startalongangle(alongwire, dsangle)
    fLRdirection = 1 if ((dsangle + 360)%360) < 180.0 else -1
    if combofoldbackmode != 0:
        fLRdirection = -fLRdirection
        
    gbs = drivegeodesicRI(gbStart, drivecurve.drivebars, drivecurve.tridrivebarsmap, LRdirection=fLRdirection, sideslipturningfactor=sideslipturningfactorZ, maxlength=maxlength)
    if gbs[-1] == None:
        return gbs, 0, -1, None, None
        
    alongwirelanded = drivecurve.endalongposition(gbs[-1])

    alongwirelanded, angcross = drivecurve.endalongpositionA(gbs[-1])

    if alongwireI is None:
        wirelength = sum((gb2.pt - gb1.pt).Len()  for gb1, gb2 in zip(gbs, gbs[1:]))
        return gbs, 0, -1, alongwirelanded, angcross
    
    gbsS = [ gbs[0].gbBackbarC ] + gbs[1:-1] + [ gbs[-1].gbForebarC ]
    drivebarsB = [ (gb.bar, gb.lam)  for gb in gbsS ]
    tridrivebarsmapB = dict((facetbetweenbars(drivebarsB[dseg][0], drivebarsB[dseg+1][0]).i, dseg)  for dseg in range(len(drivebarsB)-1))

    alongwireI1 = min([alongwireI, alongwireI+1], key=lambda X: abs(X - alongwirelanded))
    LRdirectionI = 1 if (alongwireI1 > alongwirelanded) else -1
    sideslipturningfactor = Maxsideslipturningfactor*LRdirectionI
    
    dsangleI = dsangle+180.0 if combofoldbackmode == 0 else 180.0-dsangle
    #print("dsangleI", dsangleI, dsangle, combofoldbackmode)
    gbStartI = drivecurve.startalongangle(alongwireI, dsangleI)
    gbsI = drivegeodesicRI(gbStartI, drivebarsB, tridrivebarsmapB, sideslipturningfactor=sideslipturningfactor, LRdirection=LRdirectionI, MAX_SEGMENTS=len(gbs))
    
    if gbsI[-1] == None:    # Worked with -2?
        print("Reversed path did not intersect")
        return

    for j in range(2):
        sideslipturningfactor *= 0.75
        gbsIN = drivegeodesicRI(gbStartI, drivebarsB, tridrivebarsmapB, sideslipturningfactor=sideslipturningfactor, LRdirection=LRdirectionI, MAX_SEGMENTS=len(gbs))
        if gbsIN[-1] != None:
            gbsI = gbsIN
            print("smaller sideslipturningfactor", sideslipturningfactor, "worked")
        else:
            break
            
    #print("making join bit")
    dseg = tridrivebarsmapB[gbsI[-1].tbar.i]
    gbarT1 = gbsI[0]
    gbarT1.gbBackbarC, gbarT1.gbForebarC = gbarT1.gbForebarC, gbarT1.gbBackbarC
    gbsjoined = gbs[:dseg+1] + [ GBarC(gb.bar, gb.lam, not gb.bGoRight)  for gb in gbsI[-2:0:-1] ] + [ gbarT1 ]
    #makebicolouredwire(gbsjoined, outputfilament, colfront=(1.0,0.0,0.0) if fLRdirection == -1 else (0.0,0.0,1.0), colback=(0.7,0.7,0.0), leadcolornodes=dseg+1)
    return gbsjoined, fLRdirection, dseg, None, None
    # Make copy of directed geodesic to output Ang cross of end of path


def directedgeodesicalongangcross(combofoldbackmode, sketchplane_or_drivecurve, meshobject_or_utbm, 
                     alongwire, alongwireI, dsangle, Maxsideslipturningfactor,
                     mandrelradius, sideslipturningfactorZ, maxlength, outputfilament):
    
    utbm = meshobject_or_utbm if isinstance(meshobject_or_utbm, UsefulBoxedTriangleMesh) else UsefulBoxedTriangleMesh(meshobject_or_utbm.Mesh)
    drivecurve = sketchplane_or_drivecurve if isinstance(sketchplane_or_drivecurve, DriveCurve) else makedrivecurve(sketchplane_or_drivecurve, utbm, mandrelradius)
    gbStart = drivecurve.startalongangle(alongwire, dsangle)
    fLRdirection = 1 if ((dsangle + 360)%360) < 180.0 else -1
    if combofoldbackmode != 0:
        fLRdirection = -fLRdirection
        
    gbs = drivegeodesicRI(gbStart, drivecurve.drivebars, drivecurve.tridrivebarsmap, LRdirection=fLRdirection, sideslipturningfactor=sideslipturningfactorZ, maxlength=maxlength)
    if gbs[-1] == None:
        return gbs, 0, -1, None, None
        
    alongwirelanded = drivecurve.endalongposition(gbs[-1])

    alongwirelanded, angcross = drivecurve.endalongpositionA(gbs[-1])

    if alongwireI is None:
        wirelength = sum((gb2.pt - gb1.pt).Len()  for gb1, gb2 in zip(gbs, gbs[1:]))
        return gbs, 0, -1, alongwirelanded, angcross
    
    gbsS = [ gbs[0].gbBackbarC ] + gbs[1:-1] + [ gbs[-1].gbForebarC ]
    drivebarsB = [ (gb.bar, gb.lam)  for gb in gbsS ]
    tridrivebarsmapB = dict((facetbetweenbars(drivebarsB[dseg][0], drivebarsB[dseg+1][0]).i, dseg)  for dseg in range(len(drivebarsB)-1))

    alongwireI1 = min([alongwireI, alongwireI+1], key=lambda X: abs(X - alongwirelanded))
    LRdirectionI = 1 if (alongwireI1 > alongwirelanded) else -1
    sideslipturningfactor = Maxsideslipturningfactor*LRdirectionI
    
    dsangleI = dsangle+180.0 if combofoldbackmode == 0 else 180.0-dsangle
    #print("dsangleI", dsangleI, dsangle, combofoldbackmode)
    gbStartI = drivecurve.startalongangle(alongwireI, dsangleI)
    gbsI = drivegeodesicRI(gbStartI, drivebarsB, tridrivebarsmapB, sideslipturningfactor=sideslipturningfactor, LRdirection=LRdirectionI, MAX_SEGMENTS=len(gbs))
    
    if gbsI[-1] == None:    # Worked with -2?
        print("Reversed path did not intersect")
        return

    for j in range(2):
        sideslipturningfactor *= 0.75
        gbsIN = drivegeodesicRI(gbStartI, drivebarsB, tridrivebarsmapB, sideslipturningfactor=sideslipturningfactor, LRdirection=LRdirectionI, MAX_SEGMENTS=len(gbs))
        if gbsIN[-1] != None:
            gbsI = gbsIN
            print("smaller sideslipturningfactor", sideslipturningfactor, "worked")
        else:
            break
            
    #print("making join bit")
    dseg = tridrivebarsmapB[gbsI[-1].tbar.i]
    gbarT1 = gbsI[0]
    gbarT1.gbBackbarC, gbarT1.gbForebarC = gbarT1.gbForebarC, gbarT1.gbBackbarC
    gbsjoined = gbs[:dseg+1] + [ GBarC(gb.bar, gb.lam, not gb.bGoRight)  for gb in gbsI[-2:0:-1] ] + [ gbarT1 ]
    #makebicolouredwire(gbsjoined, outputfilament, colfront=(1.0,0.0,0.0) if fLRdirection == -1 else (0.0,0.0,1.0), colback=(0.7,0.7,0.0), leadcolornodes=dseg+1)
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
