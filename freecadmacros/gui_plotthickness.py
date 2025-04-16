# -*- coding: utf-8 -*-

# directional geodesics from embedded curve controlling endpoint

# Embed a curve into a mesh so we can head off in different directions and tell when it is crossed

import Draft, Part, Mesh, MeshPart, Fem
from FreeCAD import Vector, Rotation 
from PySide import QtGui, QtCore


import os, sys, math, time
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])


from barmesh.basicgeo import I1, Partition1, P3, P2, Along, lI1
from barmesh.tribarmes.triangleboxing import TriangleBoxing
from utils.pathutils import BallPathCloseRegions, MandrelPaths, MakeFEAcoloredmesh

import utils.freecadutils as freecadutils
freecadutils.init(App)

def okaypressed():
    print("Okay Pressed") 
    meshobject = freecadutils.findobjectbylabel(qmeshpointstomeasure.text())
    if "VertexColors" not in meshobject.PropertiesList:
        meshobject.addProperty("App::PropertyColorList", "VertexColors")
    bnormcolours = qoptionnormcolours.isChecked()
    print(meshobject.PropertiesList)

    col0 = P3(*[float(x.strip())  for x in qcol0.text().split(",") ])
    col1 = P3(*[float(x.strip())  for x in qcol1.text().split(",") ])
    col2 = P3(*[float(x.strip())  for x in qcol2.text().split(",") ])
    colv0, colv2 = [float(x.strip())  for x in qcolrange.text().split(",") ]
    colv1 = (colv0 + colv2)/2
    print(col0, col1, col2)
    def convcl(v, v0, v1, cl0, cl1):
        lam = max(0, min(1, (v - v0)/(v1 - v0)))
        c = cl0*(255*(1-lam)) + cl1*(255*lam)
        return (int(c[0])<<24) + (int(c[1])<<16) + (int(c[2])<<8) + 255
    def convcol(v):
        return convcl(v, colv0, colv1, col0, col1) if v < colv1 else convcl(v, colv1, colv2, col1, col2)

    if bnormcolours:
        meshobject.VertexColors = [ convcol(v[2])  for v in meshobject.VertexNormals ]
        #meshobject.VertexColors = [ convcol(v.y*0.1)  for v in meshobject.Mesh.Points ]
    else:
        meshobject.VertexColors = [ convcol(v)  for v in meshobject.VertexThicknesses ]
    meshobject.ViewObject.Coloring = True
    qw.hide()

qw = QtGui.QWidget()
qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
qw.setGeometry(700, 500, 300, 350)
qw.setWindowTitle('Color thickness')
qmeshpointstomeasure = freecadutils.qrow(qw, "Mesh: ", 15+35*1)
qcolrange = freecadutils.qrow(qw, "Colrange: ", 15+35*3, "0,5")
qcol0 = freecadutils.qrow(qw, "Colour0: ", 15+35*4, "1,0,0")
qcol1 = freecadutils.qrow(qw, "Colour1: ", 15+35*5, "0.9,0.9,0.9")
qcol2 = freecadutils.qrow(qw, "Colour2: ", 15+35*6, "0,0,1")

qoptionnormcolours = QtGui.QCheckBox("NormColours", qw)
qoptionnormcolours.setChecked(False)
qoptionnormcolours.move(50, 15+35*7)

okButton = QtGui.QPushButton("Colour", qw)
okButton.move(180, 15+35*8)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)  

qmeshpointstomeasure.setText(freecadutils.getlabelofselectedmesh())
meshobject = freecadutils.findobjectbylabel(qmeshpointstomeasure.text())
if meshobject:
    if "VertexThicknesses" in meshobject.PropertiesList:
        assert len(meshobject.VertexThicknesses) == meshobject.Mesh.CountPoints
        minthick = min(meshobject.VertexThicknesses)
        maxthick = max(meshobject.VertexThicknesses) + 0.01
        qcolrange.setText("%.2f,%.2f" % (minthick, maxthick))
    else:
        print("Warning: no VertexThicknesses in this mesh")
        qmeshpointstomeasure.setText("missing VertexThicknesses")

    
qw.show()

#obj.ViewObject.HighlightedNodes = [1, 2, 3]
#The individual elements of a mesh can be modified by passing a dictionary with the appropriate key:value pairs.
#Set volume 1 to red
#obj.ViewObject.ElementColor = {1:(1,0,0)}
#Set nodes 1, 2 and 3 to a certain color; the faces between the nodes acquire an interpolated color.
#obj.ViewObject.NodeColor = {1:(1,0,0), 2:(0,1,0), 3:(0,0,1)}
#Displace the nodes 1 and 2 by the magnitude and direction defined by a vector.
#obj.ViewObject.NodeDisplacement = {1:FreeCAD.Vector(0,1,0), 2:FreeCAD.Vector(1,0,0)}
