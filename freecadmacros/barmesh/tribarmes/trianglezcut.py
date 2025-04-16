import sys
from ..basicgeo import P2, P3, Partition1, Along
from .. import barmesh, implicitareaballoffset, implicitareacyloffset
from .. import mainfunctions
from ..barmeshslicer import BarMeshSlicer

from . import TriangleBarMesh, MakeTriangleBoxing

class TriZCut:
    def __init__(self, tbarmesh):
        self.tbarmesh = tbarmesh
        self.tboxing = MakeTriangleBoxing(tbarmesh, -1)
        self.hitreg = [0]*len(tbarmesh.bars)
        self.nhitreg = 1
        self.hitstack = [ ]
        
    def BarLowerSide(self, hitstack, bar, x):
        if self.hitreg[bar.i] < self.nhitreg:
            assert bar.nodeback.p.x <= x < bar.nodefore.p.x
            lam = (x - bar.nodeback.p.x)/(bar.nodefore.p.x - bar.nodeback.p.x)
            cy = Along(lam, bar.nodeback.p.y, bar.nodefore.p.y)
            self.hitreg[bar.i] = len(hitstack) + self.nhitreg
            hitstack.append((lam, cy))
            assert (lam, cy) == hitstack[self.hitreg[bar.i] - self.nhitreg]
        else:
            lam, cy = hitstack[self.hitreg[bar.i] - self.nhitreg]
        return lam, cy

    def SingleTriCut(self, ztcuts, hitstack, tbar, x, y):
        assert tbar.nodeback.p.x <= tbar.nodefore.p.x 
        if x < tbar.nodeback.p.x:
            return
        tbar1 = tbar.barforeright
        assert tbar1.nodeback.p.x <= tbar1.nodefore.p.x 
        if tbar1.nodeback == tbar.nodefore:
            assert tbar.nodeback.p.x <= tbar1.nodefore.p.x
            if x < tbar.nodefore.p.x:
                tbarA = tbar
            elif x < tbar1.nodefore.p.x:
                tbarA = tbar1
            else:
                return
            tbarB = tbar1.barforeright
            assert tbarB.nodeback == tbar.nodeback
        else:
            if x < tbar.nodefore.p.x:
                tbarA = tbar
            else:
                return
            assert tbar1.nodeback.p.x >= tbar.nodeback.p.x
            if x < tbar1.nodeback.p.x:
                tbarB = tbar1.barbackleft
                assert tbarB.nodeback == tbar.nodeback
            else:
                tbarB = tbar1
        lamA, cyA = self.BarLowerSide(hitstack, tbarA, x)
        lamB, cyB = self.BarLowerSide(hitstack, tbarB, x)
        if (y < cyA) != (y < cyB):
            czA = Along(lamA, tbarA.nodeback.p.z, tbarA.nodefore.p.z)
            czB = Along(lamB, tbarB.nodeback.p.z, tbarB.nodefore.p.z)
            lamY = (y - cyA)/(cyB - cyA)
            ztcut = Along(lamY, czA, czB)
            ztcuts.append(ztcut)

    # this could be made to coordinate an entire const X column of Y-values at once
    # also need to handle the orientation of the triangle for backface or front face unambiguously
    def TriSurfCut(self, x, y):
        ztcuts = [ ]
        assert len(self.hitstack) == 0
        assert max(self.hitreg) < self.nhitreg
        for ix, iy in self.tboxing.CloseBoxeGenerator(x, x, y, y, 0): # should only be a single box
            tbox = self.tboxing.boxes[ix][iy]
            for i in tbox.triangleis:
                tbar = self.tbarmesh.bars[i]    
                self.SingleTriCut(ztcuts, self.hitstack, tbar, x, y)
        self.nhitreg += len(self.hitstack)
        del self.hitstack[:]  #self.hitstack.clear()  
        ztcuts.sort()  # we should really check orientation and number of intersections
        return ztcuts
