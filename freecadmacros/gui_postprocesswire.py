# -*- coding: utf-8 -*-

# directional geodesics from embedded curve controlling endpoint

# Embed a curve into a mesh so we can head off in different directions and tell when it is crossed

import Part, Mesh, Path
from FreeCAD import Vector, Rotation 
from PySide import QtGui, QtCore

import os, sys, math
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])

from barmesh.basicgeo import P3, P2, Along
from utils.curvesutils import thinptstotolerance
from utils.geodesicutils import TOL_ZERO
from utils.postprocutils import TCPplusfibre, projectToRvalcylinderRoundEnds, srcpt, srctcp, slerp, removebridges
import utils.freecadutils as freecadutils

freecadutils.init(App)

def okaypressed():
    print("Okay Pressed") 
    if qoptionsrcdebug.isChecked():
        SRCparameters.extend(["E1a", "fleng"])
    toolpaths = [ freecadutils.findobjectbylabel(toolpathname)  for toolpathname in qtoolpath.text().split(",") ]
    #toolpathobject = freecadutils.findobjectbylabel(qtoolpath.text())
    tcpconstXval = float(qxconst.text())
    tcpconstXarcYs = sorted([float(x.strip())  for x in qxconstarcys.text().split(",")])
    thintol = float(qthintol.text())

    tcpE3offset = float(qE3offset.text())
    Ymid = float(qyoffset.text())
    nswitcheroosplit = max(1, int(qswitchsplit.text()))

    cr = abs(tcpconstXval)
    crylohi = tcpconstXarcYs if tcpconstXarcYs else [ -1e5, 1e5 ]
    textlen = float(qtoolpathlength.text()) if len(qtoolpathlength.text()) != 0 and qtoolpathlength.text()[-1] != " " else None
    tapecurve = []
    for toolpath in toolpaths:
        print("toolpath", toolpath.Name)
        tapecurvesingle = [ P3(p.X, p.Y, p.Z)  for p in toolpath.Shape.Vertexes ]
        if tapecurve:
            gapv = tapecurvesingle[0] - tapecurve[-1]
            print("gap length to previous %.3f" % gapv.Len())
            print(" norms", P3.ZNorm(gapv), P3.ZNorm(tapecurvesingle[0]))
            tapecurvesingle = tapecurvesingle[1:]
        tapecurve += tapecurvesingle
        
    #Thin points by tolerance if one is set
    if thintol != 0:
        tapecurve = thinptstotolerance(tapecurve, thintol)
    
    #Remove points from bridging tows if option is set:
    if qoptionrmbridge.isChecked():
        tapecurve = removebridges(tapecurve)
        
    tcps = [ ]
    for i in range(len(tapecurve)):
        vecNout = P3.ZNorm(tapecurve[max(i,1)] - tapecurve[max(i,1)-1])
        ptR = tapecurve[i]
        tcpR = projectToRvalcylinderRoundEnds(ptR, vecNout, cr, crylohi[0], crylohi[1])
        tcp = TCPplusfibre(tcpR, ptR, tcpE3offset)
        tcps.append(tcp)
        if len(tcps) >= 2:
            prevtcp = tcps[-2]
            tcp.applyE3Winding(prevtcp.E3)
            tcp.applyE1Winding(prevtcp.E1)
            tgcpmov = (prevtcp.GetTCP(True) - tcp.GetTCP(True)).Len()
            if tgcpmov < 0.05:
                print("Skipping trivial linear aligned tcp motion", i, tgcpmov)
                tcps.pop()
        if i != 0 and textlen is not None:
            textlen -= (tapecurve[i] - tapecurve[i-1]).Len()
            if textlen <= 0:
                break

    tcpblocks = [ [ tcps[0] ] ]
    for itcp in range(1, len(tcps)):
        tcp = tcps[itcp]
        tcpprev = tcpblocks[-1][-1]
        Ydirectionchange = (len(tcpblocks[-1]) >= 2 and ((tcpblocks[-1][-2].Y < tcpblocks[-1][-1].Y) != (tcpblocks[-1][-1].Y < tcp.Y)))
        Yhardswitchback = P3.Dot(P3.ZNorm(tcpprev.GetVecR(True)), P3.ZNorm(tcp.GetVecR(True))) < -0.5
        if Ydirectionchange:
            if not Yhardswitchback:
                tcpblocks.append([ tcpprev ])
            else:
                tcpblocks.append([ ])
        else:
            assert not Yhardswitchback, "hardswitchback should be a Ydirectionchange"
        tcpblocks[-1].append(tcp)

    tcpblockYdirection = [ ]
    tcpblockE3direction = [ ]
    tcpblockstartwithswitchback = [ ]
    for i in range(len(tcpblocks)):
        tcpblock = tcpblocks[i]
        tcpblockYdirection.append(1 if tcpblock[0].Y < tcpblock[-1].Y else -1)
        tcpblockE3direction.append(1 if tcpblock[0].E3 < tcpblock[-1].E3 else -1)
        Yhardswitchback = 0
        if i != 0:
            tcpprev = tcpblocks[i-1][-1]
            tcpcurr = tcpblock[0]
            backvecdot = P3.Dot(P3.ZNorm(tcpprev.GetVecR(True)), P3.ZNorm(tcpcurr.GetVecR(True)))
            #print("backvecdot", backvecdot, tcpprev.Y, tcpcurr.Y)
            if backvecdot < -0.5:
                print("Yhardswitchback block", i, "to Y direction", tcpblockYdirection[i], "spin", tcpblockE3direction[i])
                Yhardswitchback = 1 if tcpblockE3direction[i]==1 else -1
        tcpblockstartwithswitchback.append(Yhardswitchback)
    
    tcpblockslinked = [ ]
    tcpblockslinkedstarthalt = [ ]
    for i in range(len(tcpblocks)):
        tcpblock = tcpblocks[i]
        if tcpblockstartwithswitchback[i] != 0:
            tcp0, tcp1 = tcpblocks[i-1][-1], tcpblock[0]
            tcp0p, tcp1p = tcp0.GetTCP(True), tcp1.GetTCP(True)
            vecr0, vecr1 = tcp0.GetVecR(True), tcp1.GetVecR(True)
            fp0, fp1 = tcp0p + vecr0, tcp1p + vecr1
            
            print("switchback from to", fp0, fp1, tcp0.freefibrelength, tcp1.freefibrelength)
            ptrmid = (fp0 + fp1)*0.5
            fflengmid = (tcp0.freefibrelength + tcp1.freefibrelength)*0.5
            vecmid = P3.ZNorm(P3(ptrmid.x, 0, ptrmid.z))*fflengmid
            tcpmid = TCPplusfibre(ptrmid + vecmid, ptrmid, tcpE3offset)

            tcplink = [ tcp0 ]
            for vec, pt in slerp(-P3.ZNorm(vecr0), P3.ZNorm(vecmid), nswitcheroosplit, fp0, ptrmid, tcp0.freefibrelength, fflengmid):
                tcp = TCPplusfibre(pt + vec, pt, tcpE3offset)
                tcplink.append(tcp)
            tcplink.append(tcpmid)
            for vec, pt in slerp(P3.ZNorm(vecmid), -P3.ZNorm(vecr1), nswitcheroosplit, ptrmid, fp1, fflengmid, tcp1.freefibrelength):
                tcp = TCPplusfibre(pt + vec, pt, tcpE3offset)
                tcplink.append(tcp)
            tcplink.append(tcp1)
            for j in range(1, len(tcplink)-1):
                tcplink[j].applyE3Winding(tcp0.E3)
                tcplink[j].applyE1Winding(tcp0.E1)

            tcpblockslinked.append(tcplink)
            tcpblockslinkedstarthalt.append(tcpblockstartwithswitchback[i])
        tcpblockslinked.append(tcpblock)
        tcpblockslinkedstarthalt.append(0)
    
    foutputsrc = qoutputsrcfile.text()
    headersrc = os.path.join(os.path.split(__file__)[0], "header.src")
    print("outputting src toolpath ", os.path.abspath(foutputsrc))

    sweepmesh = qoutputsweepmesh.text() if len(qoutputsweepmesh.text()) != 0 and qoutputsweepmesh.text()[-1] != "*" else None
    sweeppath = qoutputsweeppath.text() if len(qoutputsweeppath.text()) != 0 and qoutputsweeppath.text()[-1] != "*" else None
    if sweepmesh:
        facets = [ ]
        for i in range(len(tcpblockslinked)):
            tcpblock = tcpblockslinked[i]
            #if not tcpblockslinkedstarthalt[i]:  continue
            for j in range(len(tcpblock)-1):
                tcp0, tcp1 = tcpblock[j].GetTCP(True), tcpblock[j+1].GetTCP(True)
                vecr0, vecr1 = tcpblock[j].GetVecR(True), tcpblock[j+1].GetVecR(True)
                fp0, fp1 = tcp0 + vecr0, tcp1 + vecr1
                facets.append([Vector(*tcp0), Vector(*fp0), Vector(*tcp1)])
                facets.append([Vector(*tcp1), Vector(*fp0), Vector(*fp1)])
        mesh = freecadutils.doc.addObject("Mesh::Feature", qoutputsweepmesh.text())
        mesh.ViewObject.Lighting = "Two side"
        mesh.ViewObject.DisplayMode = "Flat Lines"
        mesh.Mesh = Mesh.Mesh(facets)
        
    if sweeppath:
        pp = Path.Path()
        for i in range(len(tcpblockslinked)):
            tcpblock = tcpblockslinked[i]
            pp.addCommands(Path.Command(["K01", "K00", "K99"][tcpblockslinkedstarthalt[i]+1]))
            for tcp in tcpblock:
                # the ABC settings cause it to be drawn with splines going everywhere they don't belong because 
                # the plotting of the orientation is not done properly
                #c = Path.Command("G1", {"X":tcp.X, "Y":tcp.Y, "Z":tcp.Z, "E3":tcp.E3*1000/360, "B":tcp.E1, "C":tcp.E1a, "L":tcp.freefibrelength})
                c = Path.Command("G1", {"X":tcp.X, "Y":tcp.Y, "Z":tcp.Z, "E3":tcp.E3*1000/360})  
                pp.addCommands(c)
            Part.show(Part.makePolygon([Vector(*tcp.GetTCP(True))  for tcp in tcpblock]), sweeppath)
        o = freecadutils.doc.addObject("Path::Feature","mypath")
        o.Path = pp
        o.ViewObject.StartPosition = Vector(tcpblockslinked[0][0].X, tcpblockslinked[0][0].Y, tcpblockslinked[0][0].Z)

    print("blocks ", list(map(len, tcpblockslinked)))

    headersrc = os.path.join(os.path.split(__file__)[0], "header.src")
    print("making toolpath: ", os.path.abspath(foutputsrc))
    fout = open(foutputsrc, "w")
    fout.write(open(headersrc).read())
    fout.write("SLIN %s\n" % srcpt({"X":-200, "Y":Ymid, "Z":0, "A":0, "E1":0, "E3":0, "E4":0}))
    for i in range(len(tcpblockslinked)):
        tcpblock = tcpblockslinked[i]
        if i == 0:
            fout.write("\nSLIN %s\n" % srctcp(tcpblock[0],Ymid))
            fout.write("HALT\n")
        elif tcpblockslinkedstarthalt[i]:
            fout.write("HALT  ; switchback %s\n" % ("up" if tcpblockslinkedstarthalt[i] == 1 else "down"))
        fout.write("SPLINE\n")
        for tcp in tcpblock[1:]:
            fout.write("SPL %s\n" % srctcp(tcp,Ymid))
        fout.write("ENDSPLINE\n\n")
    fout.write("SLIN %s\n" % srcpt({"X":-200, "Y":Ymid, "E1":0}))
    fout.write("HALT\nEND\n")
    fout.close()
    qw.hide()
    
qw = QtGui.QWidget()
qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
qw.setGeometry(700, 500, 570, 350)
qw.setWindowTitle('Post process toolpath')
qw.setStyleSheet("background-color: darkgray;")  # Set background colour to dark gray for better visibility
qtoolpath = freecadutils.qrow(qw, "Toolpath: ", 15+35*1)
qyoffset = freecadutils.qrow(qw, "Yoffset: ", 15+35*5, "307.5")
qoutputsrcfile = freecadutils.qrow(qw, "Output file: ", 15+35*6, os.path.abspath("filwin10.src"))
qthintol = freecadutils.qrow(qw, "Thinning tol: ", 15+35*7, "0.2")
qoutputsweepmesh = freecadutils.qrow(qw, "sweepmesh: ", 15+35*6, "m1*", 260)
qoutputsweeppath = freecadutils.qrow(qw, "sweeppath: ", 15+35*7, "h1*", 260)

okButton = QtGui.QPushButton("Post", qw)
okButton.move(180, 15+35*8)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)  

qtoolpath.setText(freecadutils.getlabelofselectedwire(multiples=True))
qxconst = freecadutils.qrow(qw, "xconst: ", 15+35*2, "-115")
qxconstarcys = freecadutils.qrow(qw, "xconst-arcys: ", 15+35*3, "-140,140")

qE3offset = freecadutils.qrow(qw, "E3offset ang: ", 15+35*4, "-45")   # in the XZ from the horizontal plane
qtoolpathlength = freecadutils.qrow(qw, "(Length): ", 15+35*1, "0 ", 260)
qswitchsplit = freecadutils.qrow(qw, "switchsplit: ", 15+35*2, "3", 260)

qoptionsrcdebug = QtGui.QCheckBox("Dbg SRC params", qw)
qoptionsrcdebug.move(80+260, 15+35*3)
qoptionsrcdebug.setChecked(False)

qoptionrmbridge = QtGui.QCheckBox("Remove bridges", qw)
qoptionrmbridge.move(80+260, 15+35*4)
qoptionrmbridge.setChecked(True)

toolpaths = [ freecadutils.findobjectbylabel(toolpathname)  for toolpathname in qtoolpath.text().split(",") ]
for toolpathobject in toolpaths:
    qtoolpathlength.setText("%.0f " % toolpathobject.Shape.Length)
    print("xmax", toolpathobject.Shape.BoundBox.XMax, "zmax", toolpathobject.Shape.BoundBox.ZMax)
    boxdiagrad = toolpathobject.Shape.BoundBox.XMax*math.sqrt(2)  # 45 degree diagonal puts us above the mandrel
    qxconst.setText("%.1f" % (-(boxdiagrad + 5.0)))
    qxconstarcys.setText("%.1f,%.1f" % (toolpathobject.Shape.BoundBox.YMin-2, toolpathobject.Shape.BoundBox.YMax+2))

qw.show()


