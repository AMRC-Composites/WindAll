import numpy as np
import Fem
import utils.freecadutils as freecadutils
from barmesh.basicgeo import I1, P3
from barmesh.tribarmes.triangleboxing import TriangleBoxing

def TOL_ZERO(X, msg=""):
    if not (abs(X) < 0.0001):
        print("TOL_ZERO fail", X, msg)
        assert False

class BallPathCloseRegions:
    def __init__(self, cp, rad):
        self.cp = cp
        self.rad = rad
        self.radsq = rad*rad
        self.ranges = [ ]

    def addrange(self, rg):
        self.ranges.append(rg)

    def DistPoint(self, p, i):
        if (p - self.cp).Lensq() <= self.radsq:
            self.addrange(I1(i, i))
    
    def DistEdge(self, p0, p1, i):
        v = p1 - p0
        vsq = v.Lensq()
        p0mcp = p0 - self.cp
        lam = -P3.Dot(p0mcp, v)/vsq
        dsq = (p0mcp + v*lam).Lensq()
        if dsq <= self.radsq:
            lamd = np.sqrt((self.radsq - dsq)/vsq)
            TOL_ZERO((p0 + v*(lam + lamd) - self.cp).Len() - self.rad)
            lrg = I1(lam - lamd, lam + lamd)
            if lrg.hi >= 0.0 and lrg.lo <= 1.0:
                self.addrange(I1(i + max(0.0, lrg.lo), i + min(1.0, lrg.hi)))

    def mergeranges(self):
        self.ranges.sort(key=lambda X: X.lo)
        i = 0
        while i < len(self.ranges) - 1:
            if self.ranges[i+1].lo <= self.ranges[i].hi:
                self.ranges[i] = I1(self.ranges[i].lo, max(self.ranges[i].hi, self.ranges[i+1].hi))
                del self.ranges[i+1]
            else:
                i += 1

    def mergegaps(self, d, mandpaths):
        self.ranges.sort(key=lambda X: X.lo)
        i = 0
        while i < len(self.ranges) - 1:
            if self.ranges[i+1].lo > self.ranges[i].hi:
                ld = mandpaths.getgaplength(self.ranges[i].hi, self.ranges[i+1].lo)
                if ld is None or ld > d:
                    i += 1
                    continue
                
            self.ranges[i] = I1(self.ranges[i].lo, max(self.ranges[i].hi, self.ranges[i+1].hi))
            del self.ranges[i+1]

class MandrelPaths:
    def __init__(self, mandrelptpaths, towrad=None):
        self.mandrelptpaths = mandrelptpaths
        self.Nm = max(map(len, mandrelptpaths), default=1) + 1
        xrgs = [ ]
        yrgs = [ ]
        for mandrelwindpts in mandrelptpaths:
            xrgs.append(I1.AbsorbList(p.x  for p in mandrelwindpts))
            yrgs.append(I1.AbsorbList(p.y  for p in mandrelwindpts))
        self.xrg = I1.AbsorbList(iter(sum(xrgs, ())))  if xrgs  else I1(0,1)
        self.yrg = I1.AbsorbList(iter(sum(yrgs, ())))  if yrgs  else I1(0,1)

        self.hitreg = [0]*(self.Nm * len(self.mandrelptpaths))
        self.nhitreg = 0
        
        if towrad is not None:
            xrg = self.xrg.Inflate(towrad*2)
            yrg = self.yrg.Inflate(towrad*2)
            boxwidth = max(towrad, xrg.Leng()/30, yrg.Leng()/30)
            self.tbs = TriangleBoxing(None, xrg.lo, xrg.hi, yrg.lo, yrg.hi, boxwidth)
            print("Creating box set boxwidth=", boxwidth, self.Nm)
            self.addpathstotgbs(self.tbs)

    def BallCloseCount(self, pt, towrad):
        bpcr = BallPathCloseRegions(pt, towrad)
        self.nhitreg += 1
        for ix, iy in self.tbs.CloseBoxeGenerator(pt.x, pt.x, pt.y, pt.y, towrad):
            tbox = self.tbs.boxes[ix][iy]
            for i in tbox.pointis:
                bpcr.DistPoint(self.getpt(i), i)
            for i in tbox.edgeis:
                if self.hitreg[i] != self.nhitreg:
                    bpcr.DistEdge(self.getpt(i), self.getpt(i+1), i)
                    self.hitreg[i] = self.nhitreg
        bpcr.mergeranges()
        return len(bpcr.ranges)

    def encodei(self, j, k):
        return j*self.Nm + k

    def addpathstotgbs(self, tbs):
        for j, mandrelpath in enumerate(self.mandrelptpaths):
            pp = None
            for k, p in enumerate(mandrelpath):
                tbs.AddPoint(p.x, p.y, self.encodei(j, k))
                if pp is not None:
                    i = self.encodei(j, k-1)
                    tbs.AddEdgeR(pp.x, pp.y, p.x, p.y, i)
                    self.getpt(i+1)
                pp = p

    def getpt(self, i):
        mandrelwindpts = self.mandrelptpaths[i // self.Nm]
        k = (i % self.Nm)
        assert k < len(mandrelwindpts), (i, (i // self.Nm), self.Nm, k, len(mandrelwindpts))
        return mandrelwindpts[i % self.Nm]
        
    def getgaplength(self, llam0, llam1):
        si0 = int(llam0)
        si1 = int(llam1)
        ld = 0.0
        if (si0//self.Nm) != (si1//self.Nm):
            return None
        for k in range(si0, si1+1):
            prop = 1.0
            if k == si0 and k == si1:
                prop = llam1 - llam0 
            elif k == si0:
                prop = si0 + 1.0 - llam0 
            elif k == si1:
                prop = llam1 - si1 
            assert 0.0 <= prop <= 1.0
            if prop != 0.0:
                vlen = (self.getpt(k+1) - self.getpt(k)).Len()
                ld += prop*vlen
        return ld
        
def MakeFEAcoloredmesh(mesh, nodecolours):
    tria3 = Fem.FemMesh()
    for p in mesh.Mesh.Points:
        tria3.addNode(p.x, p.y, p.z, p.Index+1)
    for f in mesh.Mesh.Facets:
        tria3.addFace([i+1  for i in f.PointIndices])
    obj = freecadutils.doc.addObject("Fem::FemMeshObject", mesh.Label+"_TH")
    obj.FemMesh = tria3
    obj.Placement = mesh.Placement
    obj.ViewObject.DisplayMode = "Faces, Wireframe & Nodes"
    obj.ViewObject.BackfaceCulling = False
    obj.ViewObject.PointSize = 1
    obj.ViewObject.NodeColor = dict((i+1, col)  for i, col in enumerate(nodecolours))