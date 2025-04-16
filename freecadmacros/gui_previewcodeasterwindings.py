# -*- coding: utf-8 -*-

# for developing the code that would be used in Femsolver/codeaster/writer in a macro 
# where it is more easily iterated

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
    assert len(meshobject.TriangleFibreDirectionsIndex) == meshobject.Mesh.CountFacets + 1
    assert meshobject.TriangleFibreDirectionsIndex[-1] == len(meshobject.IndexedTriangleFibreDirectionVectors)
    bangletox = qoptionangletox.isChecked()
    btrustemptydefaultsetting = True # so we don't set for every uncovered element (doesn't save anything is fully wrapped up)
    boptionwindall = qoptionwindall.isChecked()
    lamwindall = freecadutils.findobjectbylabel(qlamtowindall.text())

    # parallel arrays where elements is the index into the facets, numbered from +1
    elements = [ ]
    thicknesses = [ ]
    angles = [ ]
    basethickness = 0.18
    baseangle = 0.0

    for j in range(meshobject.Mesh.CountFacets):
        iv0 = meshobject.TriangleFibreDirectionsIndex[j]
        ivn = meshobject.TriangleFibreDirectionsIndex[j+1] - iv0
        if ivn == 0 and btrustemptydefaultsetting:
            continue # a triangle missing from elements gets the basethickness setting 
        ths = [ basethickness ]
        angs = [ baseangle ]
        for i in range(ivn):
            rvec = meshobject.IndexedTriangleFibreDirectionVectors[iv0 + i]
            fj = meshobject.Mesh.Facets[j]
            fnorm = P3(*fj.Normal)
            p0 = P3(*fj.Points[0])
            p1 = P3(*fj.Points[1])
            
            if bangletox:
                # solve (P3(1,0,0) + fnorm*a) . fnorm = 0
                a = -fnorm[0]
                fvec = P3.ZNorm(P3(1,0,0) + fnorm * a) # x projected into plane of triangle
            else:
                fvec = P3.ZNorm(p1 - p0) # first side of triangle

            fvecP = P3.Cross(fnorm, fvec)
            assert (abs(fvecP.Len() - 1) < 0.01)
            dang = P2(P3.Dot(rvec, fvec), P3.Dot(rvec, fvecP)).Arg()  # This is in degrees!
            ths.append(basethickness)
            # For Code Aster input, angles must be in range 90 to -90 degrees (in a laminate, an angle of 100 degrees == -80 as fibres go both directios)
            if dang > 90:
                dang -= 180
            elif dang < -90:
                dang += 180
            angs.append(dang)

        elements.append(j+1)
        thicknesses.append(ths)
        angles.append(angs)

    Windall = {'elements':elements, 'orientationlists':angles, 'thicknesslists':thicknesses}
    print(Windall)
    if boptionwindall:
        lamwindall.Windall = Windall

    qw.hide()

qw = QtGui.QWidget()
qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
qw.setGeometry(700, 500, 300, 350)
qw.setWindowTitle('Code Aster Output development')
qw.setStyleSheet("background-color: darkgray;")  # Set background colour to dark gray for better visibility

qmeshpointstomeasure = freecadutils.qrow(qw, "Mesh: ", 15+35*1)
qlamtowindall = freecadutils.qrow(qw, "Laminate: ", 15+35*2)

qoptionwindall = QtGui.QCheckBox("Add WindAll to laminate", qw)
qoptionwindall.setChecked(True)
qoptionwindall.move(50, 15+35*6)

qoptionangletox = QtGui.QCheckBox("AngleToX", qw)
qoptionangletox.setChecked(True)
qoptionangletox.move(50, 15+35*7)

okButton = QtGui.QPushButton("WindAll", qw)
okButton.move(180, 15+35*8)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)  

qmeshpointstomeasure.setText(freecadutils.getlabelofselectedmesh())
meshobject = freecadutils.findobjectbylabel(qmeshpointstomeasure.text())
if meshobject:
    assert "TriangleFibreDirectionsIndex" in meshobject.PropertiesList
    assert "IndexedTriangleFibreDirectionVectors" in meshobject.PropertiesList
    
qlamtowindall.setText(freecadutils.getlabelofselectedlam())

qw.show()

#obj.ViewObject.HighlightedNodes = [1, 2, 3]
#The individual elements of a mesh can be modified by passing a dictionary with the appropriate key:value pairs.
#Set volume 1 to red
#obj.ViewObject.ElementColor = {1:(1,0,0)}
#Set nodes 1, 2 and 3 to a certain color; the faces between the nodes acquire an interpolated color.
#obj.ViewObject.NodeColor = {1:(1,0,0), 2:(0,1,0), 3:(0,0,1)}
#Displace the nodes 1 and 2 by the magnitude and direction defined by a vector.
#obj.ViewObject.NodeDisplacement = {1:FreeCAD.Vector(0,1,0), 2:FreeCAD.Vector(1,0,0)}
