# -*- coding: utf-8 -*-

# directional geodesics from embedded curve

# Embed a curve into a mesh so we can head off in different directions and tell when it is crossed

import Draft, Part, Mesh, MeshPart
from FreeCAD import Vector, Rotation 
from PySide import QtGui, QtCore
from barmesh.basicgeo import P2

import os, sys, math, time
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])


import curvesutils;  import sys;  sys.modules.pop("curvesutils")
import geodesicutils;  import sys;  sys.modules.pop("trianglemeshutils")

from barmesh.basicgeo import I1, Partition1, P3, P2, Along
from curvesutils import isdiscretizableobject, discretizeobject
from curvesutils import cumlengthlist, seglampos
from trianglemeshutils import UsefulBoxedTriangleMesh, facetbetweenbars
from wireembeddingutils import planecutembeddedcurve, planecutbars

import geodesicutils;  import sys;  sys.modules.pop("geodesicutils")
from geodesicutils import drivegeodesic, InvAlong

import freecadutils
freecadutils.init(App)

def GetBarForeLeftT(bar):
    if bar.barbackleft is None:  
        return None
    barleft, barleftnodeback = bar.barbackleft, bar.nodeback
    Dcounter = 0
    while True:
        bbarleftnodefore = (barleft.nodeback == barleftnodeback)
        barleftnodeback = barleft.GetNodeFore(bbarleftnodefore)
        if barleftnodeback == bar.nodefore:
            break
        barleft = barleft.GetForeRightBL(bbarleftnodefore)
        assert barleft
        Dcounter += 1
        assert Dcounter < 1000, "Infinite loop in GetBarForeLeft"
    assert barleft.GetForeRightBL(barleft.nodefore == bar.nodefore) == bar, (barleft == bar, Dcounter, bar.nodeback.p, bar.nodefore.p)
    return barleft
    
def GetBarBackRightT(bar):
    if bar.barforeright is None:  
        return None
    barright, barrightnodeback = bar.barforeright, bar.nodefore
    Dcounter = 0
    while True:
        bbarrightnodefore = (barright.nodeback == barrightnodeback)
        barrightnodeback = barright.GetNodeFore(bbarrightnodefore)
        if barrightnodeback == bar.nodeback:
            break
        barright = barright.GetForeRightBL(bbarrightnodefore)
        assert barright, barrightnodeback.p
        Dcounter += 1
        assert Dcounter < 1000, "Infinite loop in GetBarBackRight"
    assert barright.GetForeRightBL(barright.nodefore == bar.nodeback) == bar
    return barright

def GetBarForeLeftBRT(bar, bforeleft):
    return GetBarForeLeftT(bar)  if bforeleft  else GetBarBackRightT(bar)

def calctnodenormals(tbarmesh):
    assert tbarmesh.nodes[-1].i == max(n.i  for n in tbarmesh.nodes)
    tnodenormals = [ None ]*(tbarmesh.nodes[-1].i + 1)
    for bar in tbarmesh.bars:
        for bfore in [ False, True ]:
            node = bar.GetNodeFore(bfore)
            if tnodenormals[node.i] is not None:
                continue
            vbars = [ bar ]
            while True:
                nbar = vbars[-1].GetForeRightBL(vbars[-1].nodefore == node)
                if nbar is None:
                    break
                vbars.append(nbar)
                if nbar == bar:
                    break
                assert len(vbars) < len(tbarmesh.bars)
            if nbar is None:
                while True:
                    nbar = GetBarForeLeftBRT(vbars[0], (vbars[0].nodefore == node))
                    if nbar is None:
                        break
                    vbars.insert(0, nbar)
                    assert len(vbars) < len(tbarmesh.bars)

            prevE = None
            sumcrossnorms = P3(0,0,0)
            for vbar in vbars:
                nextE = vbar.GetNodeFore(vbar.nodeback == node)
                if prevE is not None:
                    sumcrossnorms += P3.Cross(prevE.p - node.p, nextE.p - node.p)
                prevE = nextE
            tnodenormals[node.i] = P3.ZNorm(sumcrossnorms)
    return tnodenormals
   

   
def showoffsetmesh(tbarmesh, tnodenormals, rad):
    tnps = [ Vector(*(tbarmesh.nodes[i].p + tnodenormals[i]*rad))  for i in range(len(tbarmesh.nodes)) ]
    facets = [ ]
    for bar in tbarmesh.bars:
        if bar.barforeright is not None:
            node2 = bar.barforeright.GetNodeFore(bar.nodefore == bar.barforeright.nodeback)
            if node2.i > bar.nodeback.i:
                facets.append([ tnps[bar.nodeback.i], tnps[bar.nodefore.i], tnps[node2.i] ])
    mesh = freecadutils.doc.addObject("Mesh::Feature")
    mesh.Mesh = Mesh.Mesh(facets)
    return mesh
    
def makebarflatendtangents(tbarmesh, tnodenormals):
    flatbartangents = [ ]
    for Di, bar in enumerate(tbarmesh.bars):
        assert Di == bar.i
        v = bar.nodefore.p - bar.nodeback.p
        nback = tnodenormals[bar.nodeback.i]
        nfore = tnodenormals[bar.nodefore.i]
        vback = v + nback*P3.Dot(nback, v)
        vfore = v + nfore*P3.Dot(nfore, v)
        flatbartangents.append((vback, vfore))
    return flatbartangents



def okaypressed():
    print("Okay Pressed") 
    sketchplane = freecadutils.findobjectbylabel(qsketchplane.text())
    meshobject = freecadutils.findobjectbylabel(qmeshobject.text())
    alongwire = float(qalongwire.text())
    dsangle = float(qanglefilament.text())
    if sketchplane and meshobject:
        driveperpvec = sketchplane.Placement.Rotation.multVec(Vector(0,0,1))
        driveperpvecDot = driveperpvec.dot(sketchplane.Placement.Base)
        rotplanevecX = sketchplane.Placement.Rotation.multVec(Vector(1,0,0))
        rotplanevecY = sketchplane.Placement.Rotation.multVec(Vector(0,1,0))
        utbm = UsefulBoxedTriangleMesh(meshobject.Mesh)

        if qoptioncheckinttang.isChecked():
            tnodenormals = calctnodenormals(utbm.tbarmesh)
            flatbartangents = makebarflatendtangents(utbm.tbarmesh, tnodenormals)
            #showoffsetmesh(utbm.tbarmesh, tnodenormals, 5.0)
        else:
            flatbartangents = None

        startbar, startlam = planecutbars(utbm.tbarmesh, driveperpvec, driveperpvecDot)
        drivebars = planecutembeddedcurve(startbar, startlam, driveperpvec)
        tridrivebarsmap = dict((facetbetweenbars(drivebars[dseg][0], drivebars[dseg+1][0]).i, dseg)  for dseg in range(len(drivebars)-1))

        dpts = [ Along(lam, bar.nodeback.p, bar.nodefore.p)  for bar, lam in drivebars ]
        dptcls = cumlengthlist(dpts)

        ds = Along(alongwire, dptcls[0], dptcls[-1])
        gbs1, ds1, dsangle1 = drivegeodesic(drivebars, tridrivebarsmap, dpts, dptcls, ds, dsangle, flatbartangents)
        print("pos ds1", ds1, dsangle1)
        gbs2 = [ gbs1[-1] ]
        if ds1 != -1:
            gbs2, ds2, dsangle2 = drivegeodesic(drivebars, tridrivebarsmap, dpts, dptcls, ds1, dsangle1+0, flatbartangents)
        gbs = gbs1+gbs2[1:]
        print(windingangle(gbs, rotplanevecX, rotplanevecY))

        Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in gbs]), qoutputfilament.text())
        if ds1 != -1 and ds2 != -1:
            qalongwire.setText("%f" % InvAlong(ds2, dptcls[0], dptcls[-1]))
            qanglefilament.setText("%f" % dsangle2)
            print("Cylinder position angle advance degrees", 360*(ds2 - ds)/dptcls[-1])
            print("Leaving angle", dsangle, "Continuing angle", dsangle2)
            try:
                qoutputfilament.setText("w%d" % (int(qoutputfilament.text()[1:]) + 1))
            except ValueError:
                pass
    else:
        print("Need to select a Sketch and a Mesh object in the UI to make this work")
        qw.hide()


def windingangle(gbs, rotplanevecX, rotplanevecY):
    prevFV = None
    sumA = 0.0
    for gb in gbs:
        FV = P2(P3.Dot(rotplanevecX, gb.pt), P3.Dot(rotplanevecY, gb.pt))
        if prevFV is not None:
            dvFA = P2(P2.Dot(prevFV, FV), P2.Dot(P2.APerp(prevFV), FV)).Arg()
            sumA += dvFA
        prevFV = FV
    return sumA

def spraylines():
    sketchplane = freecadutils.findobjectbylabel(qsketchplane.text())
    meshobject = freecadutils.findobjectbylabel(qmeshobject.text())
    alongwire = float(qalongwire.text())
    dsangle = float(qanglefilament.text())
    if sketchplane and meshobject:
        driveperpvec = sketchplane.Placement.Rotation.multVec(Vector(0,0,1))
        driveperpvecDot = driveperpvec.dot(sketchplane.Placement.Base)

        rotplanevecX = sketchplane.Placement.Rotation.multVec(Vector(1,0,0))
        rotplanevecY = sketchplane.Placement.Rotation.multVec(Vector(0,1,0))

        utbm = UsefulBoxedTriangleMesh(meshobject.Mesh)
        if qoptioncheckinttang.isChecked():
            tnodenormals = calctnodenormals(utbm.tbarmesh)
            flatbartangents = makebarflatendtangents(utbm.tbarmesh, tnodenormals)
        else:
            flatbartangents = None
        
        startbar, startlam = planecutbars(utbm.tbarmesh, driveperpvec, driveperpvecDot)
        drivebars = planecutembeddedcurve(startbar, startlam, driveperpvec)
        tridrivebarsmap = dict((facetbetweenbars(drivebars[dseg][0], drivebars[dseg+1][0]).i, dseg)  for dseg in range(len(drivebars)-1))
        dpts = [ Along(lam, bar.nodeback.p, bar.nodefore.p)  for bar, lam in drivebars ]
        dptcls = cumlengthlist(dpts)
        ds = Along(alongwire, dptcls[0], dptcls[-1])

        a0, a1, sa = 10, 82, 10
        #a0, a1, sa = 30, 31, 50
        #a0, a1, sa = 10, 25, 40
        #a0, a1, sa = 20, 21, 1000
        # error with 57.75 and defaults on fc4
        # error with 19.5 and defaults on fc6working

        fout = open(qoutputcsvfile.text(), "w") if qoutputcsvfile.text() else None
        if fout:  fout.write("angleout,pathlength,cylangleadvance,windingrot,anglein\n")
        for i in range((a1 - a0)*sa):
            ldsangle = a0 + i/sa
            try:
                gbs1, ds1, dsangle1 = drivegeodesic(drivebars, tridrivebarsmap, dpts, dptcls, ds, ldsangle, flatbartangents)
                gbs2 = [ gbs1[-1] ]
                gbs = gbs1
                if ds1 != -1:
                    gbs2, ds2, dsangle2 = drivegeodesic(drivebars, tridrivebarsmap, dpts, dptcls, ds1, dsangle1+0, flatbartangents)
                    gbs = gbs1+gbs2[1:]
                else:
                    ds2, dsangle2 = -1, -1
            except AssertionError as e:
                print("Bad on angle", ldsangle)
                continue
                
            pathlength = sum((a.pt - b.pt).Len()  for a, b in zip(gbs, gbs[1:]))
            
            windingrot = windingangle(gbs, rotplanevecX, rotplanevecY)
            
            angleadvance = (360*(ds2 - ds)/dptcls[-1] + 360)%360 if ds2 != -1 else -1
            if fout:  fout.write("%f, %f, %f, %f, %f\n" % (ldsangle, pathlength, angleadvance, windingrot, dsangle2))
            if qoutputfilament.text():
                Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in gbs1+gbs2[1:]]), qoutputfilament.text())
        if fout:  fout.close()
                
    else:
        print("Need to select a Sketch and a Mesh object in the UI to make this work")
    qw.hide()


qw = QtGui.QWidget()
qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
qw.setGeometry(700, 500, 300, 350)
qw.setWindowTitle('Drive geodesic')
qsketchplane = freecadutils.qrow(qw, "Sketchplane: ", 15+35*0)
qmeshobject = freecadutils.qrow(qw, "Meshobject: ", 15+35*1 )
qalongwire = freecadutils.qrow(qw, "Along wire: ", 15+35*2, "0.51")
qanglefilament = freecadutils.qrow(qw, "Angle filament: ", 15+35*3, "30.0")
qoutputfilament = freecadutils.qrow(qw, "Output filament: ", 15+35*4, "w1")
qoptioncheckinttang = QtGui.QCheckBox("Interpolate tangents", qw)
qoptioncheckinttang.move(80, 15+35*5)
qoptioncheckinttang.setChecked(True)
qoutputcsvfile = freecadutils.qrow(qw, "csv file: ", 15+35*6, "/home/julian/repositories/geodesic/geodesicspraylines2.csv")
okButton = QtGui.QPushButton("Drive", qw)
okButton.move(180, 15+35*7)
qspraylines = QtGui.QPushButton("Spray lines", qw)
qspraylines.move(20, 15+35*7)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)  
QtCore.QObject.connect(qspraylines, QtCore.SIGNAL("pressed()"), spraylines)  


qsketchplane.setText(freecadutils.getlabelofselectedsketch())
qmeshobject.setText(freecadutils.getlabelofselectedmesh())

qw.show()


