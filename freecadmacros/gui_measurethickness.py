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
from utils.trianglemeshutils import UsefulBoxedTriangleMesh

#import importlib
#import utils.trianglemeshutils
#print("should reload utils.trianglemeshutils")
#importlib.reload(utils.trianglemeshutils)
#UsefulBoxedTriangleMesh = utils.trianglemeshutils.UsefulBoxedTriangleMesh

import utils.freecadutils as freecadutils
freecadutils.init(App)


    # may also need to derive the normals, and make this perp to the normal
def setvertexorientationvecs(meshobject):
    print("making vertex orientation vecs")
    if "VertexOrientations" not in meshobject.PropertiesList:
        meshobject.addProperty("App::PropertyVectorList", "VertexOrientations")
    if "VertexNormals" not in meshobject.PropertiesList:
        meshobject.addProperty("App::PropertyVectorList", "VertexNormals")
    if len(meshobject.VertexOrientations) == meshobject.Mesh.CountPoints and len(meshobject.VertexNormals) == meshobject.Mesh.CountPoints:
        print("vertex orientations already set, skipping")
        #return
    utbm = UsefulBoxedTriangleMesh(meshobject.Mesh, btriangleboxing=False, boriginalindexes=True)
    tbarmesh = utbm.tbarmesh
    assert (len(tbarmesh.nodes) == meshobject.Mesh.CountPoints), (len(tbarmesh.nodes), meshobject.Mesh.CountPoints)
    for i in range(meshobject.Mesh.CountPoints):
        assert (tbarmesh.nodes[i].p - meshobject.Mesh.Points[i]).Len() < 1e-6, ("kk", i, tbarmesh.nodes[i].p, meshobject.Mesh.Points[i])
    vorients = [ None ]*len(tbarmesh.nodes)
    vnormals = [ None ]*len(tbarmesh.nodes)
    for bar in tbarmesh.bars:
        iback = bar.nodeback.i
        ifore = bar.nodefore.i
        if not (vorients[iback] is None or vorients[ifore] is None):
            continue
        barvec = P3.ZNorm(bar.nodefore.p - bar.nodeback.p)
        nodeR = bar.barforeright.GetNodeFore(bar.barforeright.nodeback == bar.nodefore) if bar.barforeright is not None else None
        if nodeR is not None:
            barnorm = -P3.ZNorm(P3.Cross(barvec, nodeR.p - bar.nodeback.p))
            DnodeL = bar.barbackleft.GetNodeFore(bar.barbackleft.nodeback == bar.nodeback) if bar.barbackleft is not None else None
            #if DnodeL is not None:
            #    Dbarnorm = P3.ZNorm(P3.Cross(barvec, DnodeL.p - bar.nodeback.p))
            #    print(P3.Dot(barnorm, Dbarnorm))
        else:
            nodeL = bar.barbackleft.GetNodeFore(bar.barbackleft.nodeback == bar.nodeback)
            barnorm = P3.ZNorm(P3.Cross(barvec, nodeL.p - bar.nodeback.p))
        
        if vorients[iback] is None:
            vorients[iback] = barvec
            vnormals[iback] = barnorm
            #print("gkk", bar.nodeback.p - meshobject.Mesh.Points[iback])
        if vorients[ifore] is None:
            vorients[ifore] = -barvec
            vnormals[ifore] = barnorm
    meshobject.VertexOrientations = vorients
    meshobject.VertexNormals = vnormals


def okaypressed():
    print("Okay Pressed") 
    mandrelpaths = [ freecadutils.findobjectbylabel(mandrelpathname)  for mandrelpathname in qmandrelpaths.text().split(",") ]
    towwidth = float(qtowwidth.text())/2
    towthick = float(qtowthick.text())
    btoworientations = qoptionorientations.isChecked()
    boptioncentretriangles = qoptioncentretriangles.isChecked()
    
    measuremesh = freecadutils.findobjectbylabel(qmeshpointstomeasure.text())
    
    if measuremesh.TypeId == "Mesh::Curvature":
        meshcurvature = measuremesh
        measuremesh = meshcurvature.Source
    else:
        meshcurvature = None
    
    for prop in ["TriangleFibreDirectionsIndex", "VertexFibreDirectionsIndex", 
                 "IndexedTriangleFibreDirections", "IndexedTriangleFibreDirectionVectors",
                 "IndexedVertexFibreDirections", "IndexedVertexFibreDirectionVectors",
                 "VertexThicknesses", "TriangleThicknesses", 
                 "VertexNormals", "VertexOrientations"]:
        measuremesh.removeProperty(prop)

    if btoworientations:
        meshobject = measuremesh
        if not boptioncentretriangles:
            setvertexorientationvecs(measuremesh)
        FibreDirectionsIndex = [ ]
        IndexedFibreDirections = [ ]
        IndexedFibreDirectionVectors = [ ]
        
    mandrelptpaths = [ ]
    for mandrelpath in mandrelpaths:
        mandrelwindpts = [ P3(p.X, p.Y, p.Z)  for p in mandrelpath.Shape.Vertexes ]
        mandrelptpaths.append(mandrelwindpts)
    mandpaths = MandrelPaths(mandrelptpaths)
    xrg = mandpaths.xrg.Inflate(towwidth*2)
    yrg = mandpaths.yrg.Inflate(towwidth*2)
    boxwidth = max(towwidth, xrg.Leng()/30, yrg.Leng()/30)
    tbs = TriangleBoxing(None, xrg.lo, xrg.hi, yrg.lo, yrg.hi, boxwidth)  # used for the paths, not the triangles
    print("Creating mandrel path box set boxwidth=", boxwidth, "segments=", mandpaths.Nm)
    mandpaths.addpathstotgbs(tbs)

    thickcount = [ ]
    maxthickcount = 0
    thickpoint = None

    pointsamplestomeasure = [ ]
    if boptioncentretriangles:
        if btoworientations:
            pointsampleorientations = [ ]
            pointsamplenormals = [ ]
        for j in range(measuremesh.Mesh.CountFacets):
            fj = measuremesh.Mesh.Facets[j]
            mp = P3(*fj.InCircle[0])
            pointsamplestomeasure.append(mp)
            if btoworientations:
                fvec = P3(*fj.Points[1]) - P3(*fj.Points[0])
                pointsampleorientations.append(P3.ZNorm(fvec))
                fnorm = P3(*fj.Normal)
                pointsamplenormals.append(fnorm)

    else:
        for j in range(measuremesh.Mesh.CountPoints):
            mp = measuremesh.Mesh.Points[j]
            pointsamplestomeasure.append(P3(mp.x, mp.y, mp.z))
        if btoworientations:
            pointsampleorientations = meshobject.VertexOrientations
            pointsamplenormals = meshobject.VertexNormals

    print("sampling on", len(pointsamplestomeasure), "points on", "trianglecentres" if boptioncentretriangles else "vertices", "with orientations" if boptioncentretriangles else "only thicknesses")
    for j in range(len(pointsamplestomeasure)):
        mmp = pointsamplestomeasure[j]
        #print("jj ", j, mmp)
        bpcr = BallPathCloseRegions(mmp, towwidth)
        mandpaths.nhitreg += 1
        for ix, iy in tbs.CloseBoxeGenerator(mmp.x, mmp.x, mmp.y, mmp.y, towwidth):
            tbox = tbs.boxes[ix][iy]
            for i in tbox.pointis:
                bpcr.DistPoint(mandpaths.getpt(i), i)
            for i in tbox.edgeis:
                if mandpaths.hitreg[i] != mandpaths.nhitreg:
                    bpcr.DistEdge(mandpaths.getpt(i), mandpaths.getpt(i+1), i)
                    mandpaths.hitreg[i] = mandpaths.nhitreg
        #print("jj ", j, Ddd, mpp)
        bpcr.mergeranges()
        #ss = len(bpcr.ranges)
        #bpcr.mergegaps(0.1, mandpaths)
        #if ss != len(bpcr.ranges):
        #    print("Gap actually merged")
        thickcount.append(len(bpcr.ranges))
        if thickcount[-1] > maxthickcount:
            maxthickcount = thickcount[-1]
            thickpoint = mp
            print("Setting thickpoint at ", j)
        
        if btoworientations:
            FibreDirectionsIndex.append(len(IndexedFibreDirections))
            for rg in bpcr.ranges:  # the fibre interval that's closest
                ir = rg.Along(0.5)  # midpoint of segment
                i = int(ir)
                rvec = mandpaths.getpt(i+1) - mandpaths.getpt(i) # direction of segment
                Dpt = mandpaths.getpt(i) + rvec*(ir - i)
                Dlen = (Dpt - mmp).Len()
                if Dlen > towwidth:
                    print("Point outside towwidth", Dlen, Dpt)
                IndexedFibreDirectionVectors.append(rvec) # record vecs
                plpU = pointsampleorientations[j]
                plpV = P3.Cross(pointsamplenormals[j], plpU)
                dang = P2(P3.Dot(rvec, plpU), P3.Dot(rvec, plpV)).Arg()
                IndexedFibreDirections.append(dang) # calculate angles (obsolete, push out to next phase)
                assert len(IndexedFibreDirections) == len(IndexedFibreDirectionVectors)
                #print("llp", P3.Dot(meshobject.VertexNormals[j], meshobject.VertexOrientations[j]), dang)

    if btoworientations:
        FibreDirectionsIndex.append(len(IndexedFibreDirections)) # final entry
        assert len(IndexedFibreDirections) == len(IndexedFibreDirectionVectors)
        assert len(FibreDirectionsIndex) == len(pointsamplestomeasure) + 1
        
    if btoworientations:
        if boptioncentretriangles:
            meshobject.addProperty("App::PropertyIntegerList", "TriangleFibreDirectionsIndex")
            meshobject.addProperty("App::PropertyFloatList", "IndexedTriangleFibreDirections")
            meshobject.addProperty("App::PropertyVectorList", "IndexedTriangleFibreDirectionVectors")
            meshobject.TriangleFibreDirectionsIndex = FibreDirectionsIndex
            meshobject.IndexedTriangleFibreDirections = IndexedFibreDirections  # to be calculated in the next phase, which makes it more flexible and closer to where it needs to be known
            meshobject.IndexedTriangleFibreDirectionVectors = IndexedFibreDirectionVectors
        else:
            meshobject.addProperty("App::PropertyIntegerList", "VertexFibreDirectionsIndex")
            meshobject.addProperty("App::PropertyFloatList", "IndexedVertexFibreDirections")
            meshobject.addProperty("App::PropertyVectorList", "IndexedVertexFibreDirectionVectors")
            meshobject.VertexFibreDirectionsIndex = FibreDirectionsIndex
            meshobject.IndexedVertexFibreDirections = IndexedFibreDirections
            meshobject.IndexedVertexFibreDirectionVectors = IndexedFibreDirectionVectors
        
    print("Max thick count", maxthickcount, "thickness", maxthickcount*towthick, "at point", thickpoint)
    print(max(thickcount), "thickcount")
    if meshcurvature != None:
        for i, c in enumerate(thickcount):
            meshcurvature.ValueAtIndex = (i, c*towthick, c)
            meshcurvature.recompute()
        print(" Setting of Min/Max curvatures to filament crossings")
    else: 
        if boptioncentretriangles:
            measuremesh.addProperty("App::PropertyFloatList", "TriangleThicknesses")
            measuremesh.TriangleThicknesses = [ c*towthick  for c in thickcount ]
        else:
            measuremesh.addProperty("App::PropertyFloatList", "VertexThicknesses")
            measuremesh.VertexThicknesses = [ c*towthick  for c in thickcount ]
    qw.hide()

# ------------------------- GUI ----------------------------
# Create the main Qt widget (window)
qw = QtGui.QWidget()
qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
qw.setGeometry(700, 500, 300, 350)
qw.setWindowTitle('Measure thickness')
qw.setStyleSheet("background-color: darkgray;")  # Set background colour to dark gray for better visibility

#Set boxes
qmeshpointstomeasure = freecadutils.qrow(qw, "Mesh: ", 15+35*1)
qmandrelpaths = freecadutils.qrow(qw, "Winding paths ", 15+35*2, "")
qtowwidth = freecadutils.qrow(qw, "Tow width: ", 15+35*3, "6.35")
qtowthick = freecadutils.qrow(qw, "Tow thick: ", 15+35*4, "0.18")

qoptionorientations = QtGui.QCheckBox("Tow Orientations", qw)
qoptionorientations.setChecked(True)
qoptionorientations.move(50, 15+35*5)

qoptioncentretriangles = QtGui.QCheckBox("On triangles", qw) # or vertices
qoptioncentretriangles.setChecked(True)
qoptioncentretriangles.move(50, 15+35*6)

okButton = QtGui.QPushButton("Measure", qw)
okButton.move(180, 15+35*8)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)  

qmandrelpaths.setText(freecadutils.getlabelofselectedwire(multiples=True))
qmeshpointstomeasure.setText(freecadutils.getlabelofselectedmesh())

qw.show()
