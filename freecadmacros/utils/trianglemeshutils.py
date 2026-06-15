from barmesh.tribarmes import TriangleBarMesh, MakeTriangleBoxing
from barmesh.basicgeo import P3
import Mesh
import Fem


def clamp(num, min_value, max_value):
    return max(min(num, max_value), min_value)


class UsefulBoxedTriangleMesh:
    """Class to hold a triangle bar mesh object with boxing"""

    def __init__(self, mesh, btriangleboxing=True):
        """
        Initialises a UBTM object, requires:
        mesh: Either  FreeCAD Mesh or 1st order triangular FemMesh
        btriangleboxing: boolean to determine whether or not to create boxing
        """
        global tbarmesh, tboxing, hitreg, nhitreg
        if isinstance(mesh, Mesh.Mesh):
            trpts = [sum(f.Points, ()) for f in mesh.Facets]
        elif isinstance(mesh, Fem.FemMesh):
            assert mesh.TriangleCount > 0, "Mesh must contain triangles"
            trpts = []
            for fid in mesh.Faces:
                fapts = []
                nids = mesh.getElementNodes(fid)
                assert len(nids) == 3, "Mesh must be 1st order triangular"
                nodes = [mesh.getNodeById(nid) for nid in nids]
                for n in nodes:
                    fapts.append(n.x)
                    fapts.append(n.y)
                    fapts.append(n.z)
                trpts.append(fapts)
        print(trpts)
        # print("building tbarmesh ", len(trpts))
        self.tbarmesh = TriangleBarMesh()
        self.tbarmesh.BuildTriangleBarmesh(trpts)
        if isinstance(mesh, Mesh.Mesh):
            assert len(self.tbarmesh.nodes) == mesh.CountPoints
            assert len(self.tbarmesh.bars) == mesh.CountEdges
        elif isinstance(mesh, Fem.FemMesh):
            assert len(self.tbarmesh.nodes) == mesh.NodeCount
        if btriangleboxing:
            self.tboxing = MakeTriangleBoxing(self.tbarmesh)
            self.hitreg = [0]*len(self.tbarmesh.bars)
            self.nhitreg = 0

    def FindClosestEdge(self, p, r=3):
        self.nhitreg += 1
        barclosest = None
        barclosestlam = 0.0
        for ix, iy in self.tboxing.CloseBoxeGenerator(p.x, p.x, p.y, p.y, r):
            tbox = self.tboxing.boxes[ix][iy]
            for i in tbox.edgeis:
                if self.hitreg[i] != self.nhitreg:
                    bar = self.tbarmesh.bars[i]
                    p0, p1 = bar.nodeback.p, bar.nodefore.p
                    lv = p - p0
                    v = p1 - p0
                    vsq = v.Lensq()
                    lam = P3.Dot(v, lv) / vsq
                    lam = clamp(lam, 0.0, 1.0)
                    vd = lv - v * lam
                    assert lam == 0.0 or lam == 1.0 or abs(P3.Dot(vd, v)) < 0.0001
                    vdlen = vd.Len()
                    if vdlen < r:
                        r = vdlen
                        barclosest = bar
                        barclosestlam = lam
                    self.hitreg[i] = self.nhitreg
        return barclosest, barclosestlam


def facetbetweenbars(bar, barN):
    assert bar != barN
    nodeR = bar.barforeright.GetNodeFore(bar.barforeright.nodeback == bar.nodefore)
    if barN.nodeback == nodeR or barN.nodefore == nodeR:
        if nodeR.i > bar.nodeback.i:
            tbar = bar
        else:
            tbar = bar.barforeright.GetForeRightBL(bar.barforeright.nodeback == bar.nodefore)
            assert tbar.nodefore == bar.nodeback
    else:
        nodeL = bar.barbackleft.GetNodeFore(bar.barbackleft.nodeback == bar.nodeback)
        if not (barN.nodeback == nodeL or barN.nodefore == nodeL):
            print("  b:", bar.nodeback, bar.nodefore)
            print("  N:", barN.nodeback, barN.nodefore)
        assert barN.nodeback == nodeL or barN.nodefore == nodeL
        if bar.nodeback.i > nodeL.i:
            tbar = bar.barbackleft.GetForeRightBL(bar.barbackleft.nodeback == bar.nodeback)
        else:
            tbar = bar.barbackleft

    DtbarR = tbar.barforeright.GetNodeFore(tbar.barforeright.nodeback == tbar.nodefore)
    assert DtbarR.i > tbar.nodeback.i
    Dtbars = [tbar, tbar.barforeright,
              tbar.barforeright.GetForeRightBL(tbar.barforeright.nodeback == 
                                               tbar.nodefore)]
    assert bar in Dtbars and barN in Dtbars
    return tbar


def GetBarForeLeftBRN(tbar, bforeleft):
    tbarR = tbar
    Dcounter = 0
    nodeC = tbar.GetNodeFore(bforeleft)
    while True:
        assert tbarR.nodeback == nodeC or tbarR.nodefore == nodeC
        tbarRnext = tbarR.GetForeRightBL(tbarR.nodefore == nodeC)
        if tbarRnext is None or tbarRnext == tbar:
            break
        tbarR = tbarRnext
        Dcounter += 1
        assert Dcounter < 1000
    return tbarR