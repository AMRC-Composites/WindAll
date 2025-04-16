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

import imp
import utils.directedgeodesic
imp.reload(utils.directedgeodesic)
from utils.directedgeodesic import directedgeodesic, makebicolouredwire, makedrivecurve, drivegeodesicRI, DriveCurve
from utils.trianglemeshutils import UsefulBoxedTriangleMesh


def sgetlabelstowwires(sel):
    labels = [ ]
    lsel = sel.copy()
    while lsel:
        s = lsel.pop()
        if hasattr(s, "Shape") and isinstance(s.Shape, Part.Wire):
            if "towwidth" in s.PropertiesList and not (s.Label in labels):
                labels.append(s.Label)
            else:
                if not (hasattr(s, "Module") and s.Module == 'Sketcher'):
                    print("Warning: %s has no towwidth property" % s.Label)
        elif hasattr(s, "OutList"):
            lsel.extend(s.OutList)
    return ",".join(labels)
    
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

def makemesh(opts, facetis, measstockmeshname):
    facets = [ (Vector(*opts[i1]), Vector(*opts[i2]), Vector(*opts[i3]))  for i1, i2, i3 in facetis ]
    mmesh = freecadutils.doc.addObject("Mesh::Feature", measstockmeshname)
    mmesh.Mesh = Mesh.Mesh(facets)
    mmesh.ViewObject.Lighting = "Two side"
    mmesh.ViewObject.DisplayMode = "Flat Lines"
    return mmesh
    
def makemandpaths(mandrelpaths, towrad):
    mandrelptpaths = [ [ P3(p.X, p.Y, p.Z)  for p in mandrelpath.Shape.Vertexes ]  for mandrelpath in mandrelpaths ]
    mandpaths = MandrelPaths(mandrelptpaths)
    xrg = mandpaths.xrg.Inflate(towrad*2)
    yrg = mandpaths.yrg.Inflate(towrad*2)
    boxwidth = max(towrad, xrg.Leng()/30, yrg.Leng()/30)
    tbs = TriangleBoxing(None, xrg.lo, xrg.hi, yrg.lo, yrg.hi, boxwidth)
    print("Creating box set boxwidth=", boxwidth, mandpaths.Nm)
    mandpaths.addpathstotgbs(tbs)
    return mandpaths, tbs
    
def towcountonpoint(mp, towrad, mandpaths, tbs):
    bpcr = BallPathCloseRegions(mp, towrad)
    mandpaths.nhitreg += 1
    for ix, iy in tbs.CloseBoxeGenerator(mp.x, mp.x, mp.y, mp.y, towrad):
        tbox = tbs.boxes[ix][iy]
        for i in tbox.pointis:
            bpcr.DistPoint(mandpaths.getpt(i), i)
        for i in tbox.edgeis:
            if mandpaths.hitreg[i] != mandpaths.nhitreg:
                bpcr.DistEdge(mandpaths.getpt(i), mandpaths.getpt(i+1), i)
                mandpaths.hitreg[i] = mandpaths.nhitreg
    bpcr.mergeranges()
    #ss = len(bpcr.ranges)
    #bpcr.mergegaps(0.1, mandpaths)
    #if ss != len(bpcr.ranges):
    #    print("Gap actually merged")
    return len(bpcr.ranges)


import FreeCADGui as Gui
import PySide.QtGui as QtGui
import PySide.QtCore as QtCore

# next bit is to offset by the thickness of material

class StockViewerTaskPanel(QtGui.QWidget):
    def __init__(self):
        print("loading stockviewertask.ui")
        x = os.path.join(os.path.split(__file__)[0], "stockviewertask.ui")
        self.form = FreeCADGui.PySideUic.loadUi(x)
        self.form.setMinimumSize(0, 300)
        QtCore.QObject.connect(self.form.pushButton, QtCore.SIGNAL("pressed()"), self.pushbutton)  
        self.update()

    def update(self):
        self.doc = App.ActiveDocument
        self.sel = App.Gui.Selection.getSelection()
        self.form.qbasestockmeshname.setText(sgetlabelofselectedmesh(self.sel))
        self.form.qmandrelpaths.setText(sgetlabelstowwires(self.sel))

    def pushbutton(self):
        print("Pushbutton pressed")

    def apply(self):
        print("apply!!")
        basestockmeshobject = sfindobjectbylabel(self.doc, self.form.qbasestockmeshname.text())
        pts = [ P3(p.x, p.y, p.z)  for p in basestockmeshobject.Mesh.Points ]
        facetis = [ f.PointIndices  for f in basestockmeshobject.Mesh.Facets ]
        norms = [ P3(*p)  for p in basestockmeshobject.Normals ]
        measstockmeshname = self.form.qmeasstockmesh.text()
        stockmultiplier = float(self.form.qstockmultiplier.text())
        
        mandrelpaths = [ freecadutils.findobjectbylabel(mandrelpathname)  for mandrelpathname in self.form.qmandrelpaths.text().split(",") ]
        towrad = mandrelpaths[0].towwidth/2.0   # we assume single width and thickness
        towthick = mandrelpaths[0].towthickness
        mandpaths, tbs = makemandpaths(mandrelpaths, towrad)
        thickcounts = [ towcountonpoint(p, towrad, mandpaths, tbs)  for p in pts ]
        opts = [ p + n*(th*towthick*stockmultiplier)  for p, n, th in zip(pts, norms, thickcounts) ]
        makemesh(opts, facetis, measstockmeshname)


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

Gui.Control.showDialog(StockViewerTaskPanel())
