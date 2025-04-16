import math
from .basicgeo import P3, P2, Along
from .import barmesh
from .tribarmes import MakeTriangleBoxing

# this module handles cuts in 2D of a 2D plane against a closed contour
# Unfinished as we hit problems with PointZone vector being looked at by the barmeshslicer and assuming it was P3s

class DistPFZ:
    def __init__(self, p, r):
        self.p = p
        self.r = r
        self.uplusraycuts = 0
        self.v = None
        
    def DistPFpointPZ(self, p0):
        lv = p0 - self.p
        lvlen = lv.Len()
        if lvlen < self.r:
            self.r = lvlen
            self.v = lv
    
    def DistPFedgePZ(self, p0, p1):
        lv = self.p - p0
        v = p1 - p0
        if (p0.v < self.p.v) != (p1.v < self.p.v):
            mu = lv.v/v.v
            if Along(mu, p0.u, p1.u) > self.p.u:
                self.uplusraycuts += 1
        vsq = v.Lensq()
        lam = P2.Dot(v, lv) / vsq
        if 0.0 < lam < 1.0:
            vd = lv - v * lam
            assert abs(P2.Dot(vd, v)) < 0.0001
            vdlen = vd.Len()
            if vdlen < self.r:
                self.r = vdlen
                self.v = -vd


class DistLamPZ:
    def __init__(self, p, vp, r):
        self.p = p
        self.vp = vp
        self.vpsq = vp.Lensq()
        self.r = r
        self.rsq = r*r
        self.lam = 2.0
        
    def DistLamPpointPZ(self, p0):
        lv = p0 - self.p
        if lv.z < min(self.vp.z, 0) - self.r or lv.z > max(self.vp.z, 0) + self.r:
            return
        # |lv - vp * lam| = r
        # qa = vpsq
        qb2 = -P3.Dot(self.vp, lv)
        qc = lv.Lensq() - self.rsq
        
        qdq = qb2*qb2 - self.vpsq*qc
        if qdq < 0.0:
            return
        qs = math.sqrt(qdq) / self.vpsq
        qm = -qb2 / self.vpsq
        assert abs(qc + (qm + qs)*(2*qb2 + (qm + qs)*self.vpsq)) < 0.002
        if qm + qs <= 0.0:
            return
        laml = qm - qs
        if laml < 0.0:
            self.lam = 0.0  # shouldn't happen
        elif laml < self.lam:
            self.lam = laml

    def DistLamPedgePZ(self, p0, p1):
        v = p1 - p0
        vsq = v.Lensq()
        
        lv = self.p - p0
        # solve |lv + vp * lam - v * mu| == r, where (lv + vp * lam - v * mu) . vp == 0
        mu0 = P3.Dot(lv, v)/vsq
        lvf = lv - v*mu0
        vpdv = P3.Dot(self.vp, v)
        muvp = vpdv/vsq
        
        vpf = self.vp - v*muvp
        vpfsq = vpf.Lensq()
        if vpfsq == 0.0:
            return
        assert abs(vpfsq - (self.vpsq - muvp * vpdv)) < 0.001
        
        lvfdvpf = P3.Dot(lvf, vpf)
        lamc = -lvfdvpf / vpfsq
        cp = lvf + vpf * lamc
        cpsq = cp.Lensq()
        lvfsq = lvf.Lensq()
        assert abs(cpsq - (lvfsq + 2 * lvfdvpf * lamc + vpfsq * lamc * lamc)) < 0.001
        assert abs(P3.Dot(cp, vpf)) < 0.001
        llamdsq = self.rsq - cp.Lensq()
        if llamdsq < 0.0:
            return
        lamd = math.sqrt(llamdsq / vpfsq)
        if lamc + lamd < 0.0:
            return
        lam = lamc - lamd
        if lam < 0.0:
            return  # check closer stuff
        if lam > self.lam:
            return
        mu = mu0 + muvp * lam
        if mu < 0 or mu > 1:
            return
        dv = lv + self.vp * lam - v * mu
        assert abs(dv.Len() - self.r) < 0.001
        assert abs(P3.Dot(dv, v)) < 0.001
        self.lam = lam
        
    
class ImplicitAreaFlatContour:
    def __init__(self, tbarmesh):
        self.tbarmesh = tbarmesh
        self.tboxing = MakeTriangleBoxing(tbarmesh)
        self.hitreg = [0]*len(tbarmesh.bars)
        self.nhitreg = 0

    def Isb2dcontournormals(self):
        return False
    
    def DistPN(self, pz, n):    # pz=PointZone
        return self.DistPF(pz, n.sp)
        
    def DistPF(self, pz, p):
        dpz = DistPFZ(p, pz.r) 
        for ix, iy in self.tboxing.CloseBoxeGenerator(p.u, p.u, p.v, p.v, pz.r):
            tbox = self.tboxing.boxes[ix][iy]
            for i in tbox.pointis:
                dpz.DistPFpointPZ(P2.ConvertLZ(self.tboxing.GetNodePoint(i)))
                
        self.nhitreg += 1
        for ix, iy in self.tboxing.CloseBoxeGenerator(p.u, self.tboxing.xpart.hi, p.v, p.v, pz.r):
            tbox = self.tboxing.boxes[ix][iy]
            for i in tbox.edgeis:
                if self.hitreg[i] != self.nhitreg:
                    p0, p1 = self.tboxing.GetBarPoints(i)
                    dpz.DistPFedgePZ(P2.ConvertLZ(p0), P2.ConvertLZ(p1))
                    self.hitreg[i] = self.nhitreg
                    
        pz.r = dpz.r if ((dpz.uplusraycuts % 2) == 0) else -dpz.r
        pz.v = dpz.v
        

    def CutposN(self, nodefrom, nodeto, cp, r):  # point, vector, cp=known close point to narrow down the cutoff search
        assert False
        p = nodefrom.p
        vp = nodeto.p - nodefrom.p
        dlpz = DistLamPZ(p, vp, r)  
        if cp is not None:
            dlpz.DistLamPpointPZ(cp)
            assert dlpz.lam != 2.0
        
        # solve |p0 + vp * lam - p| == r where Dist(p0) >= r >= Dist(p0 + vp)
        rexp = r + 0.01
        for ix, iy in self.tboxing.CloseBoxeGenerator(min(p.x, p.x+vp.x), max(p.x, p.x+vp.x), min(p.y, p.y+vp.y), max(p.y, p.y+vp.y), rexp):
            tbox = self.tboxing.boxes[ix][iy]
            for i in self.tboxing.SlicePointisZ(tbox.pointis, min(p.z,p.z+vp.z)-rexp, max(p.z,p.z+vp.z)+rexp):
            #for i in tbox.pointis:
                dlpz.DistLamPpointPZ(self.tboxing.GetNodePoint(i))
                
        self.nhitreg += 1
        for ix, iy in self.tboxing.CloseBoxeGenerator(min(p.x, p.x+vp.x*dlpz.lam), max(p.x, p.x+vp.x*dlpz.lam), min(p.y, p.y+vp.y*dlpz.lam), max(p.y, p.y+vp.y*dlpz.lam), r + 0.01):
            tbox = self.tboxing.boxes[ix][iy]
            for i in tbox.edgeis:
                if self.hitreg[i] != self.nhitreg:
                    dlpz.DistLamPedgePZ(*self.tboxing.GetBarPoints(i))
                    self.hitreg[i] = self.nhitreg
        
        self.nhitreg += 1
        for ix, iy in self.tboxing.CloseBoxeGenerator(min(p.x, p.x+vp.x*dlpz.lam), max(p.x, p.x+vp.x*dlpz.lam), min(p.y, p.y+vp.y*dlpz.lam), max(p.y, p.y+vp.y*dlpz.lam), r + 0.01):
            tbox = self.tboxing.boxes[ix][iy]
            for i in tbox.triangleis:
                if self.hitreg[i] != self.nhitreg:
                    dlpz.DistLamPtrianglePZ(*self.tboxing.GetTriPoints(i))
                    self.hitreg[i] = self.nhitreg
        
        if __debug__:
            if not (dlpz.lam == 0.0 or dlpz.lam == 2.0):
                pz = barmesh.PointZone(0, r + 1.1, None)
                self.DistP(pz, dlpz.p + dlpz.vp*dlpz.lam)
                assert abs(pz.r - r) < 0.002, ("cutposbad", pz.r, dlpz.lam, dlpz.p, dlpz.vp)
            
        return dlpz.lam


