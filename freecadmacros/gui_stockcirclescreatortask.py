# -*- coding: utf-8 -*-

# stock viewer task, a standard interactive panel on the left in FreeCAD.  
# To edit the UI file, nix-shell -p qtcreator, start up qtcreator
# select default current project and start editing stockviewertask.ui


import Draft, Part, Mesh, MeshPart, Fem
from FreeCAD import Vector, Rotation 
from PySide import QtGui, QtCore


import os, sys, math, time, numpy
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])


from barmesh.basicgeo import I1, Partition1, P3, P2, Along, lI1
from barmesh.tribarmes.triangleboxing import TriangleBoxing
from utils.pathutils import BallPathCloseRegions, MandrelPaths, MakeFEAcoloredmesh
import utils.freecadutils as freecadutils
freecadutils.init(App)
from utils.curvesutils import cumlengthlist, seglampos

#import imp
import utils.directedgeodesic
#imp.reload(utils.directedgeodesic)
from utils.directedgeodesic import directedgeodesic, makebicolouredwire, makedrivecurve, drivegeodesicRI, DriveCurve
from utils.trianglemeshutils import UsefulBoxedTriangleMesh
from utils.geodesicutils import drivegeodesic, InvAlong, GBarT, GBarC, drivecurveintersectionfinder, trilinecrossing, TOL_ZERO

import FreeCADGui as Gui
import PySide.QtGui as QtGui
import PySide.QtCore as QtCore


def sgetlabelofselectedmesh(sel):
    for s in sel:
        if hasattr(s, "Mesh") and isinstance(s.Mesh, Mesh.Mesh):
            return s.Label
        if s.TypeId == "Mesh::Curvature":
            if "ValueAtIndex" in s.PropertiesList:
                return s.Label
            else:
                print("***  Wrong version of FreeCAD, \n\nMeshCurvature object missing ValueAtIndex hack.")
                return "**WrongFCversion"
    return ""

def sgetlabelofselectedsketch(sel):
    for s in sel:
        if hasattr(s, "Module") and s.Module == 'Sketcher':
            return s.Label
    return ""
    
def sfindobjectbylabel(doc, lab):
    objs = [ obj  for obj in doc.findObjects(Label=lab)  if obj.Label == lab ]
    return objs[0] if objs else None

def setpropertyval(obj, atype, name, value):
    if name not in obj.PropertiesList:
        obj.addProperty(atype, name, "filwind")
    setattr(obj, name, value)


mandrelradius = 110  # fc6 file
dsangle = 90.0
LRdirection = 1
appaturepapproachpoint = P3(0,-150,0)

def spherecutlam(p0, p1, sphpt, sphrad):
    v = p1 - p0
    vs = p0 - sphpt
    qa = v.Lensq()
    qb2 = P3.Dot(vs, v)
    qc = vs.Lensq() - sphrad*sphrad
    qdq = qb2*qb2 - qa*qc
    qs = math.sqrt(qdq) / qa
    qm = -qb2 / qa
    q = qm + qs
    TOL_ZERO(qa*q*q + qb2*2*q + qc)
    Dpt = vs + v*q
    assert (-0.01 <= q <= 1.01), q
    return q


def TriangleCrossSphereRight(tbar, bGoRight, sphpt, sphrad, bContinuation):
    nodeAhead = tbar.GetNodeFore(bGoRight)
    nodeBehind = tbar.GetNodeFore(not bGoRight)
    barAhead = tbar.GetForeRightBL(bGoRight)
    barAheadGoRight = (barAhead.nodeback == nodeAhead)
    nodeOpposite = barAhead.GetNodeFore(barAheadGoRight)
    barBehind = barAhead.GetForeRightBL(barAheadGoRight)
    DbarBehindGoRight = (barBehind.nodeback == nodeOpposite)
    assert nodeBehind == barBehind.GetNodeFore(DbarBehindGoRight)

    nds = [ nodeBehind, nodeAhead, nodeOpposite ]
    brs = [ tbar, barAhead, barBehind ]
    dnds = [ (b.p - sphpt).Len()  for b in nds ]
    for i in range(3):
        i1 = i + 1 if i != 2 else 0
        if (dnds[i] < sphrad) and (dnds[i1] >= sphrad):
            break
    jlam = spherecutlam(nds[i].p, nds[i1].p, sphpt, sphrad)
    barCrossing = brs[i]
    barCrossingGoRight = not (barCrossing.nodeback == nds[i])
    barCrossingLam = 1 - jlam if barCrossingGoRight else jlam
    barCrossingpt = Along(barCrossingLam, barCrossing.nodeback.p, barCrossing.nodefore.p)
    TOL_ZERO((barCrossingpt - sphpt).Len() - sphrad)
    return barCrossing, barCrossingLam, barCrossingGoRight


def makestockcircle(gbt, sphpt):
    sphrad = (gbt.pt - sphpt).Len()
    bar, lam, bGoRight = TriangleCrossSphereRight(gbt.tbar, True, sphpt, sphrad, False)
    bar0 = bar
    sphbars = [ GBarC(bar, lam, bGoRight) ]
    for i in range(600):
        bar, lam, bGoRight = TriangleCrossSphereRight(bar, bGoRight, sphpt, sphrad, True)
        sphbars.append(GBarC(bar, lam, bGoRight))
        if bar == bar0:
            break
    return sphbars, sphrad



def chaseupcolumngbtsfromdrivecurveSingle(drivecurve, alongwire, colsamplestep):
    colgbStart = drivecurve.startalongangle(alongwire, dsangle)
    colgbs = drivegeodesicRI(colgbStart, drivecurve.drivebars, drivecurve.tridrivebarsmap, LRdirection=LRdirection, maxlength=400.0)
    if colgbs[-1] == None:
        colgbs.pop()

    colgbsT = [ None ] + [ (gb.bar, gb.lam)  for gb in colgbs[1:] ]
      # first element is a GBarT, which could be extended back to a GBarC but it's not an important area
      
    colcls = cumlengthlist([ gb.pt  for gb in colgbs ])

    gbts = [ ]
    ds = 0.0
    while ds < colcls[-1] and len(gbts) < 1000:
        dsseg, dslam = seglampos(ds, colcls)
        if dsseg != 0:
            gbt = GBarT(colgbsT, dsseg, dslam)
            gbts.append(gbt)
            #pt = Along(dslam, colgbs[dsseg].pt, colgbs[dsseg+1].pt)
            #norm = colgbs[dsseg+1].tnorm if hasattr(colgbs[dsseg+1], "tnorm") else colgbs[dsseg+1].tnorm_incoming
        ds += colsamplestep
    return gbts


class StockCirclesTaskPanel(QtGui.QWidget):
    def __init__(self):
        x = os.path.join(os.path.split(__file__)[0], "stockcirclescreatortask.ui")
        self.form = FreeCADGui.PySideUic.loadUi(x)
        self.form.setMinimumSize(0, 300)
        QtCore.QObject.connect(self.form.pushButton, QtCore.SIGNAL("pressed()"), self.pushbutton)  
        self.update()

    def update(self):
        self.doc = App.ActiveDocument
        self.sel = App.Gui.Selection.getSelection()
        self.form.qmeshobject.setText(sgetlabelofselectedmesh(self.sel))
        self.form.qsketchplane.setText(sgetlabelofselectedsketch(self.sel))

    def pushbutton(self):
        print("Pushbutton pressed")

    def apply(self):
        print("apply!!")
        sketchplane = sfindobjectbylabel(self.doc, self.form.qsketchplane.text())
        meshobject = sfindobjectbylabel(self.doc, self.form.qmeshobject.text())
        stockcirclesname = self.form.qstockcirclesname.text()
        alongwire = float(self.form.qalongwire.text())
        colsamplestep = float(self.form.qcolsamplestep.text())
        assert self.form.qappatureapproachpoint.text() == "P3(0,-150,0)"

        utbm = UsefulBoxedTriangleMesh(meshobject.Mesh)
        drivecurve = makedrivecurve(sketchplane, utbm, mandrelradius)

        gbts = chaseupcolumngbtsfromdrivecurveSingle(drivecurve, alongwire, colsamplestep)
        stockcolumn = Part.show(Part.makePolygon([Vector(gbt.pt - gbt.tnorm*0.0)  for gbt in gbts]), "stockcolumn")

        stockcirclesgroup = freecadutils.getemptyfolder(self.doc, stockcirclesname)
        setpropertyval(stockcirclesgroup, "App::PropertyFloat", "alongwire", alongwire)
        setpropertyval(stockcirclesgroup, "App::PropertyString", "sketchplane", sketchplane.Label)
        setpropertyval(stockcirclesgroup, "App::PropertyString", "meshobject", meshobject.Label)
        setpropertyval(stockcirclesgroup, "App::PropertyBool", "stockcirclestype", True)
        for gbt in gbts:
            cgbs, sphrad = makestockcircle(gbt, appaturepapproachpoint)
            name = 'sc%.2f' % sphrad
            cply = Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in cgbs]), name)
            setpropertyval(cply, "App::PropertyFloat", "sphrad", sphrad)
            setpropertyval(cply, "App::PropertyVector", "pt", Vector(gbt.pt))
            setpropertyval(cply, "App::PropertyVector", "norm", Vector(-gbt.tnorm))
            setpropertyval(cply, "App::PropertyFloat", "onionthickness", 0.0)
            stockcirclesgroup.addObject(cply)


    def getStandardButtons(self):
        return int(QtGui.QDialogButtonBox.Cancel
                   | QtGui.QDialogButtonBox.Ok
                   | QtGui.QDialogButtonBox.Apply)

    def clicked(self, bt):
        if bt == QtGui.QDialogButtonBox.Apply:
            self.apply()

    def accept(self):
        print("Accept")
        self.apply()
        self.finish()

    def reject(self):
        print("Reject")
        self.finish()

    def finish(self):
        print("Finish")
        Gui.Control.closeDialog()

Gui.Control.showDialog(StockCirclesTaskPanel())
