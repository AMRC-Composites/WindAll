# -*- coding: utf-8 -*-
# Use the MeshPart library to project a selected wire onto a selected mesh

import Draft, Part, Mesh, MeshPart
from barmesh.basicgeo import I1, Partition1, P3, P2, Along
from FreeCAD import Vector, Rotation 
from PySide import QtGui, QtCore

import os, sys, math, time
sys.path.append(os.path.join(os.path.split(__file__)[0]))

#import sys;  sys.modules.pop("trianglemeshutils")
from wireembeddingutils import planecutembeddedcurve, planecutbars
from trianglemeshutils import UsefulBoxedTriangleMesh, facetbetweenbars, GetBarForeLeftBRN
#import sys;  sys.modules.pop("freecadutils")

import freecadutils
freecadutils.init(App)

def okaypressed():
    print("Okay Pressed") 
    sketchplane = freecadutils.findobjectbylabel(qsketchplane.text())
    meshobject = freecadutils.findobjectbylabel(qmeshobject.text())
    if sketchplane and meshobject:
        driveperpvec = sketchplane.Placement.Rotation.multVec(Vector(0,0,1))
        driveperpvecDot = driveperpvec.dot(sketchplane.Placement.Base)
        print("driveperpvec", driveperpvec, driveperpvecDot)
        utbm = UsefulBoxedTriangleMesh(meshobject.Mesh)
        startbar, startlam = planecutbars(utbm.tbarmesh, driveperpvec, driveperpvecDot)
        drivebars = planecutembeddedcurve(startbar, startlam, driveperpvec)
        if qcutwire.text():
            freecadutils.showdrivebarscurve(drivebars, qcutwire.text())
        if qcutwiresnake.text():
            freecadutils.showdrivebarsmesh(drivebars, qcutwiresnake.text())
    else:
        print("Need to select a Sketch and a Mesh object in the UI to make this work")
    qw.hide()



def makemeshboundaries():
    print("makemeshboundaries Pressed") 
    meshobject = freecadutils.findobjectbylabel(qmeshobject.text())
    seenboundbars = set()
    if meshobject:
        utbm = UsefulBoxedTriangleMesh(meshobject.Mesh, False)
        for bar in utbm.tbarmesh.bars:
            if (bar.barforeright == None or bar.barbackleft == None) and bar not in seenboundbars:
                bbar = bar
                bforeleft = (bbar.barbackleft == None)
                bnode = bbar.GetNodeFore(bforeleft)
                bnodeseq = [ bnode ]
                while True:
                    seenboundbars.add(bbar)
                    bbar = GetBarForeLeftBRN(bbar, bforeleft)
                    if bbar == bar:
                        break
                    bforeleft = (bbar.nodeback == bnode)
                    bnode = bbar.GetNodeFore(bforeleft)
                    assert bbar.GetForeRightBL(not bforeleft) == None
                    bnodeseq.append(bnode)
                    assert len(bnodeseq) <= len(utbm.tbarmesh.nodes) + 1
                print(len(bnodeseq))
                epts = [ node.p  for node in bnodeseq ]
                wire = Part.show(Part.makePolygon(epts))
                #freecadutils.showdrivebarscurve(drivebars, qcutwire.text())
    else:
        print("Need to select a Mesh object in the function to make this work")
    qw.hide()


qw = QtGui.QWidget()
qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
qw.setGeometry(700, 400, 300, 200)
qw.setWindowTitle('Cut sketch plane')
qsketchplane = freecadutils.qrow(qw, "Sketchplane: ", 15)
qmeshobject = freecadutils.qrow(qw, "Meshobject: ", 50)
qcutwire = freecadutils.qrow(qw, "Cut wire: ", 85)
qcutwiresnake = freecadutils.qrow(qw, "Cut wire snake: ", 115)
okButton = QtGui.QPushButton("Cut plane", qw)
okButton.move(180, 150)
qmakemeshboundaries = QtGui.QPushButton("Make boundaries", qw)
qmakemeshboundaries.move(20, 150)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)  
QtCore.QObject.connect(qmakemeshboundaries, QtCore.SIGNAL("pressed()"), makemeshboundaries)  

qsketchplane.setText(freecadutils.getlabelofselectedsketch())
qmeshobject.setText(freecadutils.getlabelofselectedmesh())
qcutwire.setText("planecutwire")
qcutwiresnake.setText("planecutwiresnake")

qw.show()


    


