# -*- coding: utf-8 -*-
# Use the MeshPart library to project a selected wire onto a selected mesh

import Draft, Part, Mesh, MeshPart
from FreeCAD import Vector, Rotation 
from PySide import QtGui, QtCore

import os, sys, math, time
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])

#import sys;  sys.modules.pop("freecadutils")
import freecadutils

freecadutils.init(App)

screenrot = Gui.ActiveDocument.ActiveView.getCameraOrientation()
projectionDir = screenrot.multVec(Vector(0,0,-1))


def okaypressed():
    print("Okay Pressed") 
    wireobject = freecadutils.findobjectbylabel(qwireshape.text())
    meshobject = freecadutils.findobjectbylabel(qmeshobject.text())
    freecadutils.removeobjectbylabel(qprojectedobject.text())
    if wireobject and meshobject:
        polylines = MeshPart.projectShapeOnMesh(wireobject.Shape, meshobject.Mesh, projectionDir)
        for i in polylines:
            Part.show(Part.makePolygon(i), qprojectedobject.text())
    else:
        print("Need to select a Wire and a Mesh object in the UI to make this work")
    print("qoptioncheck", qoptioncheck.isChecked())
    qw.hide()


qw = QtGui.QWidget()
qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
qw.setGeometry(700, 400, 300, 200)
qw.setWindowTitle('Project path')
qwireshape = freecadutils.qrow(qw, "Wireshape: ", 15)
qmeshobject = freecadutils.qrow(qw, "Meshobject: ", 50)
qprojectedobject = freecadutils.qrow(qw, "Projected: ", 85)
qoptioncheck = QtGui.QCheckBox("Center on XY", qw)
qoptioncheck.setChecked(True)
qoptioncheck.move(80, 115)
okButton = QtGui.QPushButton("Project Wire", qw)
okButton.move(160, 150)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)  
qwireshape.setText(freecadutils.getlabelofselectedwire())
qmeshobject.setText(freecadutils.getlabelofselectedmesh())
qw.show()


    


