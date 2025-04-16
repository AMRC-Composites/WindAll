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

def sgetlabelofselectedgroupwithproperties(sel, properties):
    for s in sel:
        if isinstance(s, App.DocumentObjectGroup) and set(properties).issubset(s.PropertiesList):
            return s.Label
    return ""

mandrelradius = 110  # fc6 file
dsangle = 90.0
LRdirection = 1

def chaseupcolumnptnormalsfromdrivecurve(drivecurve, alongwireLo, alongwireHi, columns, colsamplestep):
    columnptnormals = [ ]
    for alongwire in numpy.linspace(alongwireLo, alongwireHi, columns+1):
        colgbStart = drivecurve.startalongangle(alongwire, dsangle)
        colgbs = drivegeodesicRI(colgbStart, drivecurve.drivebars, drivecurve.tridrivebarsmap, LRdirection=LRdirection, maxlength=400.0)
        if colgbs[-1] == None:
            colgbs.pop()

        colcls = cumlengthlist([gb.pt  for gb in colgbs])
        ds = 0.0
        ptnormals = [ ]
        while ds < colcls[-1] and len(ptnormals) < 1000:
            dsseg, dslam = seglampos(ds, colcls)
            pt = Along(dslam, colgbs[dsseg].pt, colgbs[dsseg+1].pt)
            norm = colgbs[dsseg+1].tnorm if hasattr(colgbs[dsseg+1], "tnorm") else colgbs[dsseg+1].tnorm_incoming
            ptnormals.append((pt, -norm))
            ds += colsamplestep
        columnptnormals.append(ptnormals)
    return columnptnormals

def makedrivemeshfromcolumnpts(columnptnormals, stockmeshname):
    facets = [ ]
    for j in range(len(columnptnormals)-1):
        c0, c1 = columnptnormals[j], columnptnormals[j+1]
        for i in range(min(len(c0), len(c1))-1):
            p0, p1 = c0[i][0], c0[i+1][0]
            pe0, pe1 = c1[i][0], c1[i+1][0]
            facets.append([p0, pe0, p1])
            facets.append([p1, pe0, pe1])
            
    # there is no way to associate normals to these mesh vertices 
    # so we have to look them up afterwards, using an n^2 algorithm
    # The correct way would be to set the components of the mesh directly
    mesh = Mesh.Mesh(facets)
    assert mesh.CountPoints == len(columnptnormals)*len(columnptnormals[0])
    ptnormalsdict = sum(columnptnormals, [ ])
    normalslist = [ ]
    for m in mesh.Points:
        s = min(ptnormalsdict, key=lambda X: (X[0] - P3(m.x, m.y, m.z)).Lensq())
        normalslist.append(s[1])
    
    drivemesh = freecadutils.doc.addObject("Mesh::Feature", stockmeshname)
    drivemesh.Mesh = mesh
    drivemesh.addProperty("App::PropertyVectorList", "Normals")
    drivemesh.Normals = normalslist
    
    #drivemesh.addProperty("App::PropertyFloatList", "VertexThicknesses")
    #measuremesh.VertexThicknesses = [ c*towthick  for c in thickcount ]
    drivemesh.ViewObject.Lighting = "Two side"
    drivemesh.ViewObject.DisplayMode = "Flat Lines"


class StockViewerTaskPanel(QtGui.QWidget):
    def __init__(self):
        x = os.path.join(os.path.split(__file__)[0], "stockmeshcreatortask.ui")
        self.form = FreeCADGui.PySideUic.loadUi(x)
        self.form.setMinimumSize(0, 400)
        self.update()

    def update(self):
        self.doc = App.ActiveDocument
        self.sel = App.Gui.Selection.getSelection()
        self.supersel = set(self.sel).union(s.getParent()  for s in self.sel)
        stockcircles = sgetlabelofselectedgroupwithproperties(self.supersel, ["stockcirclestype"])
        if stockcircles:
            self.form.qstockcircles.setText(stockcircles)
        stockcirclesgroup = sfindobjectbylabel(self.doc, self.form.qstockcircles.text())
        if stockcirclesgroup:
            sci0, sci1 = 0, 0
            sci1 = len(stockcirclesgroup.OutList)
            self.form.qsketchplane.setText(stockcirclesgroup.sketchplane)
            self.form.qmeshobject.setText(stockcirclesgroup.meshobject)
            selstockcircleindexes = [ stockcirclesgroup.OutList.index(sc)  for sc in self.sel  if sc in stockcirclesgroup.OutList ]
            if selstockcircleindexes:
                sci0, sci1 = min(selstockcircleindexes), max(selstockcircleindexes)
            self.form.qstockcircleindexes.setText("%d,%d" % (sci0, sci1))
        self.sketchplane = sfindobjectbylabel(self.doc, self.form.qsketchplane.text())
        self.meshobject = sfindobjectbylabel(self.doc, self.form.qmeshobject.text())



    def apply(self):
        print("apply!!")
        stockcirclesgroup = sfindobjectbylabel(self.doc, self.form.qstockcircles.text())
        stockmeshname = self.form.qstockmeshname.text()
        sci0, sci1 = list(map(int, self.form.qstockcircleindexes.text().split(",")))
        selstockcolumn = [ (P3(*stockcirclesgroup.OutList[i].pt), P3(*stockcirclesgroup.OutList[i].norm))  for i in range(sci0, sci1+1) ]
        colsamplestep = float(self.form.qcolsamplestep.text())
        colcls = cumlengthlist([P3(*x[0])  for x in selstockcolumn])
        ds = 0.0
        stockcolumn = [ ]
        while ds < colcls[-1] and len(stockcolumn) < 1000:
            dsseg, dslam = seglampos(ds, colcls)
            ptn0, ptn1 = selstockcolumn[dsseg], selstockcolumn[min(len(selstockcolumn)-1, dsseg+1)]
            stockcolumn.append((Along(dslam, ptn0[0], ptn1[0]), P3.ZNorm(Along(dslam, ptn0[1], ptn1[1]))))
            ds += colsamplestep

        anglo, anghi, angstep = list(map(float, (x.strip() for x in self.form.qanglerangestep.text().split(","))))
        ang = anglo
        columnptnormals = [ ]
        while ang < anghi:
            ptnormals = [ ]
            cosang = math.cos(math.radians(ang))
            sinang = math.sin(math.radians(ang))
            for pt, norm in stockcolumn:
                lpt = P3(pt.x*cosang + pt.z*sinang, pt.y, pt.z*cosang - pt.x*sinang)
                lnorm = P3(norm.x*cosang + norm.z*sinang, norm.y, norm.z*cosang - norm.x*sinang)
                ptnormals.append((lpt, lnorm))
            ang += angstep
            columnptnormals.append(ptnormals)
        makedrivemeshfromcolumnpts(columnptnormals, stockmeshname)



# We are making a radial path that can be a secondary drive curve
# that we will need to rotate around the axis
# And use this to define the underlying stock mesh
# which will lie in a sector and have its own normal vectors embedded
# and we can use this to project outward for the stock meshes.
# call this build stock basis mesh


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
