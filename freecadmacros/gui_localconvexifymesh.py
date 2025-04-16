# -*- coding: utf-8 -*-
# Use the MeshPart library to project a selected wire onto a selected mesh

import Draft, Part, Mesh, MeshPart
from FreeCAD import Vector, Rotation 

from PySide import QtGui, QtCore

import os, sys, math, time
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])

#import utils.trianglemeshutils;  import sys;  sys.modules.pop("trianglemeshutils")
import utils.freecadutils as freecadutils 
from utils.trianglemeshutils import UsefulBoxedTriangleMesh
from barmesh.basicgeo import I1, Partition1, P3, P2, Along

freecadutils.init(App)

def calcbarfoldtanangle(bar):
    node2 = bar.barforeright.GetNodeFore(bar.nodefore == bar.barforeright.nodeback)
    v2fore = bar.nodefore.p - node2.p
    v2back = bar.nodeback.p - node2.p
    vsegB = bar.nodefore.p - bar.nodeback.p
    vsegB2 = node2.p - bar.nodeback.p
    tnorm = P3.ZNorm(P3.Cross(vsegB2, vsegB))
    tperp = P3.ZNorm(P3.Cross(vsegB, tnorm))
    n2rhsdist = P3.Dot(tperp, vsegB2)
    assert n2rhsdist >= 0.0, n2rhsdist
    node2lhs = bar.barbackleft.GetNodeFore(bar.nodeback == bar.barbackleft.nodeback)
    vsegB2lhs = node2lhs.p - bar.nodeback.p
    n2lhsdist = P3.Dot(tperp, vsegB2lhs)
    n2lhsz = P3.Dot(tnorm, node2.p - node2lhs.p)
    assert n2lhsdist <= 0.0, n2lhsdist
    tanfoldlhs = n2lhsz / n2lhsdist
    return tanfoldlhs
    
def flipbar(bar):
    node2rhs = bar.barforeright.GetNodeFore(bar.nodefore == bar.barforeright.nodeback)
    barbackright = bar.barforeright.GetForeRightBL(bar.nodefore == bar.barforeright.nodeback)
    assert barbackright.GetNodeFore(node2rhs == barbackright.nodeback) == bar.nodeback
    node2lhs = bar.barbackleft.GetNodeFore(bar.nodeback == bar.barbackleft.nodeback)
    barforeleft = bar.barbackleft.GetForeRightBL(bar.nodeback == bar.barbackleft.nodeback)
    assert barforeleft.GetNodeFore(node2lhs == barforeleft.nodeback) == bar.nodefore

    assert bar.barforeright.GetForeRightBL(bar.barforeright.nodeback == bar.nodefore) == barbackright
    bar.barforeright.SetForeRightBL(bar.barforeright.nodeback == bar.nodefore, bar)
    assert barbackright.GetForeRightBL(barbackright.nodefore == bar.nodeback) == bar
    barbackright.SetForeRightBL(barbackright.nodefore == bar.nodeback, bar.barbackleft)
    assert bar.barbackleft.GetForeRightBL(bar.barbackleft.nodeback == bar.nodeback) == barforeleft
    bar.barbackleft.SetForeRightBL(bar.barbackleft.nodeback == bar.nodeback, bar)
    assert barforeleft.GetForeRightBL(barforeleft.nodefore == bar.nodefore) == bar
    barforeleft.SetForeRightBL(barforeleft.nodefore == bar.nodefore, bar.barforeright)

    barlefttoright = (node2lhs.i < node2rhs.i)
    bar.nodefore = node2rhs if barlefttoright else node2lhs
    bar.nodeback = node2lhs if barlefttoright else node2rhs
    bar.SetForeRightBL(barlefttoright, barbackright)
    bar.SetForeRightBL(not barlefttoright, barforeleft)
    

def flipconcavebars(tbarmesh, flatangletoleranceTan):
    barfoldangles = { }
    def foldangleaddbars(bars):
        for bar in bars:
            if bar.barforeright is not None and bar.barbackleft is not None:
                tanfoldlhs = calcbarfoldtanangle(bar)
                if tanfoldlhs < -flatangletoleranceTan:
                    barfoldangles[bar] = tanfoldlhs
    foldangleaddbars(tbarmesh.bars)
    nflips = 0
    while len(barfoldangles) != 0:
        bar = min(barfoldangles.items(), key=lambda X:X[1])[0]
        barfoldangles.pop(bar)
        flipbar(bar)
        nflips += 1
        barbackright = bar.barforeright.GetForeRightBL(bar.nodefore == bar.barforeright.nodeback)
        barforeleft = bar.barbackleft.GetForeRightBL(bar.nodeback == bar.barbackleft.nodeback)
        foldangleaddbars([ bar.barforeright, barbackright, bar.barbackleft, barforeleft ])
    print("nflips", nflips)


def okaypressed():
    print("Okay Pressed") 
    meshobject = freecadutils.findobjectbylabel(qmeshobject.text())
    flatangletolerance = float(qflatangletolerance.text())
    flatangletoleranceTan = math.tan(math.radians(flatangletolerance))
    print("flatangletoleranceTan", flatangletoleranceTan)
    if meshobject:
        utbm = UsefulBoxedTriangleMesh(meshobject.Mesh, btriangleboxing=False)
        tbarmesh = utbm.tbarmesh
        flipconcavebars(tbarmesh, flatangletoleranceTan)    
        facets = [ [ Vector(*p) for p in t]  for t in tbarmesh.GetBarMeshTriangles() ]
        mesh = freecadutils.doc.addObject("Mesh::Feature", "convexedmesh")
        mesh.Mesh = Mesh.Mesh(facets)

    else:
        print("Need to select a Mesh object in the UI to make this work")
    qw.hide()


qw = QtGui.QWidget()
qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
qw.setGeometry(700, 400, 300, 200)
qw.setWindowTitle('Project path')
qmeshobject = freecadutils.qrow(qw, "Meshobject: ", 50)
qflatangletolerance = freecadutils.qrow(qw, "flat ang tol: ", 85, "1.0")
okButton = QtGui.QPushButton("Convexify mesh", qw)
okButton.move(160, 150)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)  
qmeshobject.setText(freecadutils.getlabelofselectedmesh())
qw.show()


    


