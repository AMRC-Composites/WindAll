# -*- coding: utf-8 -*-

# directional geodesics from embedded curve controlling endpoint

# Embed a curve into a mesh so we can head off in different directions and tell when it is crossed

import Draft, Part, Mesh, MeshPart
from FreeCAD import Vector, Rotation 
from PySide import QtGui, QtCore

import os, sys, math, time
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])

import utils.curvesutils;  import sys;  sys.modules.pop("utils.curvesutils")
import utils.trianglemeshutils;  import sys;  sys.modules.pop("utils.trianglemeshutils")
import utils.geodesicutils;  import sys;  sys.modules.pop("utils.geodesicutils")
import utils.freecadutils as freecadutils;  import sys;  sys.modules.pop("utils.freecadutils")

from barmesh.basicgeo import I1, Partition1, P3, P2, Along
from utils.curvesutils import isdiscretizableobject, discretizeobject, thinptstotolerance
from utils.curvesutils import cumlengthlist, seglampos
from utils.trianglemeshutils import UsefulBoxedTriangleMesh, facetbetweenbars
from utils.wireembeddingutils import planecutembeddedcurve, planecutbars
from utils.geodesicutils import drivegeodesic, InvAlong, GBarT, GBarC, drivecurveintersectionfinder, trilinecrossing, TOL_ZERO

freecadutils.init(App)


def okaypressed():
    print("Okay Pressed") 
    singlewindobjects = [ freecadutils.findobjectbylabel(singlewindname)  for singlewindname in qsinglewindpath.text().split(",") ]
    mandrelwindings = int(qmandrelwindings.text())
    thintol = float(qthintol.text())
    
    Ssinglewindpts = [ ]
    for singlewindobject in singlewindobjects:
        singlewindpts = [ P3(p.X, p.Y, p.Z)  for p in singlewindobject.Shape.Vertexes ]
        tsinglewindpts = thinptstotolerance(singlewindpts, tol=thintol)
        print("Thinned", len(singlewindpts), "points to", len(tsinglewindpts), "at tol", thintol)
        if len(Ssinglewindpts) != 0:
            sgapleng = (Ssinglewindpts[-1][-1] - tsinglewindpts[0]).Len()
            print("sgapleng", sgapleng)
            assert sgapleng < 0.01, "Wire sections do not connect together"
            ptjoin = (Ssinglewindpts[-1][-1] + tsinglewindpts[0])*0.5
            Ssinglewindpts[-1][-1] = ptjoin
            tsinglewindpts[0] = ptjoin
        Ssinglewindpts.append(tsinglewindpts)
        
    ptfront, ptback = Ssinglewindpts[0][0], Ssinglewindpts[-1][-1]
    fvec0 = P2(ptfront.x, ptfront.z)
    fvec1 = P2(ptback.x, ptback.z)
    print("drive curve y-vals", ptfront.y, ptback.y, "rads", fvec0.Len(), fvec1.Len())
    angadvance = P2(P2.Dot(fvec0, fvec1), P2.Dot(fvec0, P2.APerp(fvec1))).Arg()

    tpt0 = Ssinglewindpts[0][:]
    for singlewindpts in Ssinglewindpts[1:]:
        tpt0.extend(singlewindpts[1:])
        
    tpt = tpt0[:]
    for i in range(1, mandrelwindings):
        rotcos = math.cos(math.radians(i*angadvance))
        rotsin = math.sin(math.radians(i*angadvance))
        for pt in tpt0[1:]:
            tpt.append(P3(pt.x*rotcos + pt.z*rotsin, pt.y, pt.z*rotcos - pt.x*rotsin))
    wire = Part.show(Part.makePolygon([Vector(pt)  for pt in tpt]), qoutputfilament.text())
    wire.addProperty("App::PropertyString", "filename", group="WindAll")
    wire.filename = App.ActiveDocument.Name
    wire.addProperty("App::PropertyFloatList", "angles", group="WindAll")
    wire.angles = [x.angle for x in singlewindobjects]
    wire.addProperty("App::PropertyFloatList", "alongwires", group="WindAll")
    wire.alongwires = [x.alongwire for x in singlewindobjects]
    wire.addProperty("App::PropertyFloatList", "alongwireAdvs", group="WindAll")
    wire.alongwireAdvs = [x.alongwireAdv for x in singlewindobjects]
    wire.addProperty("App::PropertyFloat", "maxlength", group="WindAll")
    maxlength = 0
    for x in singlewindobjects:
        maxlength += x.maxlength
    wire.maxlength = maxlength
    wire.addProperty("App::PropertyBool", "mode", group="WindAll")
    mode = singlewindobjects[0].mode
    for x in singlewindobjects:
        mode = mode and x.mode
    wire.mode = mode
    
    qw.hide()

qw = QtGui.QWidget()
qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
qw.setGeometry(700, 500, 570, 350)
qw.setWindowTitle('Repeat winding toolpath')
qw.setStyleSheet("background-color: darkgray;")  # Set background colour to dark gray for better visibility
qsinglewindpath = freecadutils.qrow(qw, "Single wind: ", 15+35*1)

qmandrelwindings = freecadutils.qrow(qw, "Repetitions: ", 15+35*2, "%d" % 10)
qoutputfilament = freecadutils.qrow(qw, "Output name: ", 15+35*3, "t1")
qthintol = freecadutils.qrow(qw, "Thinning tol: ", 15+35*4, "0.2")

okButton = QtGui.QPushButton("Repeat", qw)
okButton.move(180, 15+35*7)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)  

qsinglewindpath.setText(freecadutils.getlabelofselectedwire(multiples=True))

qw.show()


