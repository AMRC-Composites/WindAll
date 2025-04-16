# -*- coding: utf-8 -*-

# directional geodesics from embedded curve controlling endpoint

# Embed a curve into a mesh so we can head off in different directions and tell when it is crossed

import Draft, Part, Mesh, MeshPart, Path
from FreeCAD import Vector, Rotation 
from PySide import QtGui, QtCore

import os, sys, math, time
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])


from barmesh.basicgeo import I1, Partition1, P3, P2, Along
from utils.curvesutils import isdiscretizableobject, discretizeobject, thinptstotolerance, cumlengthlist, seglampos
from utils.trianglemeshutils import UsefulBoxedTriangleMesh, facetbetweenbars
from utils.wireembeddingutils import planecutembeddedcurve, planecutbars

from utils.geodesicutils import drivegeodesic, InvAlong, GBarT, GBarC, drivecurveintersectionfinder, trilinecrossing, TOL_ZERO

import utils.freecadutils as freecadutils
freecadutils.init(App)
    
# X - moves towards/away from the tool (perpendicular to the carriage).
# Y - moves along the carriage axis
# Z - Vertical positioning (rarely used)
# A - Yaw of the robot (around the Z axis)
# E1 - Rotation of the payout eye
# E1a - Angle we roll out from the payout eye (not a real control)
# E3 - Rotation both chucks (headstock and tailstock)
# E4 - Rotation of tailstock relative to headstock (rarely used)

def projectToRvalcylinderRoundEnds(pt, vec, cr, crylo, cryhi):
    qa = P2(vec.x, vec.z).Lensq()
    qb2 = pt.x*vec.x + pt.z*vec.z
    qc = P2(pt.x, pt.z).Lensq() - cr*cr
    qdq = qb2*qb2 - qa*qc
    qs = math.sqrt(qdq) / qa
    qm = -qb2 / qa
    q = qm + qs
    TOL_ZERO(qa*q*q + qb2*2*q + qc)
    res = pt + vec*q
    TOL_ZERO(P2(res.x, res.z).Len() - cr)
    if not crylo < res.y < cryhi:
        dc = -1 if res.y <= crylo else +1
        assert vec.y < 0 if dc == -1 else vec.y > 0
        cry = crylo if dc == -1 else cryhi
        ha = qa + vec.y*vec.y
        hb2 = qb2 + (pt.y - cry)*vec.y
        hc = qc + (pt.y - cry)*(pt.y - cry)
        hdh = hb2*hb2 - ha*hc
        hs = math.sqrt(hdh) / ha
        hm = -hb2 / ha
        h = hm + hs
        TOL_ZERO(ha*h*h + hb2*2*h + hc)
        assert (h <= q)
        res = pt + vec*h
        assert res.y*dc >= cry*dc, (res.y, cry, dc, (h, q))
        TOL_ZERO((res - P3(0, cry, 0)).Len() - cr)
    return res

def sRotByE3(sE3, pt):
    rvX = P3(math.cos(math.radians(sE3)), 0, math.sin(math.radians(sE3)))
    rvZ = P3(-rvX.z, 0, rvX.x)
    return rvX*pt.x + P3(0, pt.y, 0) + rvZ*pt.z

class TCPplusfibre:
    def __init__(self, tcpR, ptR, tcpE3offset):
        vecR = ptR - tcpR
        self.freefibrelength = vecR.Len()
        self.DvecR = vecR
        
        if tcpE3offset == 0:
            self.E3 = P2(-tcpR.x, -tcpR.z).Arg()
            
            tcpRd = P2(tcpR.x, tcpR.z).Len()
            rvX = P3(-tcpR.x/tcpRd, 0, -tcpR.z/tcpRd)
            rvY = P3(0, 1, 0)
            rvZ = P3(-rvX.z, 0, rvX.x)
            
            self.X = -tcpRd
            self.Y = tcpR.y
            self.Z = 0
            
            DtcpR = rvX*self.X + rvY*self.Y + rvZ*self.Z
            TOL_ZERO((DtcpR - tcpR).Len())
            vec = P3(P3.Dot(rvX, vecR), P3.Dot(rvY, vecR), P3.Dot(rvZ, vecR))
        else:
            self.E3 = P2(-tcpR.x, -tcpR.z).Arg() + tcpE3offset
            vec = sRotByE3(-self.E3, vecR)
            tcp = sRotByE3(-self.E3, tcpR)
            self.X = tcp.x
            self.Y = tcp.y
            self.Z = tcp.z
            
        TOL_ZERO(self.freefibrelength - vec.Len(), "freefibrewrong")
        TOL_ZERO((self.RotByE3(vec) - vecR).Len(), ("rotByE3vec failed"))

        self.E1 = P2(vec.z, vec.y).Arg()
        cE1a = vec.x/self.freefibrelength
        assert abs(cE1a) < 1.0001, cE1a
        self.E1a = math.degrees(math.acos(min(1.0, max(-1.0, cE1a))))
        
        if vec.z < 0.0:
            self.E1 = 180 + self.E1
            self.E1a = -self.E1a
            
        TOL_ZERO((tcpR - self.GetTCP(True)).Len(), "tcpmismatch")
        TOL_ZERO((vecR - self.GetVecR(True)).Len(), (vecR, self.GetVecR(True)))

    def RotByE3(self, pt):
        rvX = P3(math.cos(math.radians(self.E3)), 0, math.sin(math.radians(self.E3)))
        rvZ = P3(-rvX.z, 0, rvX.x)
        return rvX*pt.x + P3(0, pt.y, 0) + rvZ*pt.z
    
    def GetTCP(self, bRotated):
        res = P3(self.X, self.Y, self.Z)
        return self.RotByE3(res) if bRotated else res
        
    def GetVecR(self, bRotated):
        cosE1a = math.cos(math.radians(self.E1a))
        sinE1a = math.sin(math.radians(self.E1a))
        cosE1 = math.cos(math.radians(self.E1))
        sinE1 = math.sin(math.radians(self.E1))
        res = P3(cosE1a, sinE1a*sinE1, sinE1a*cosE1)*self.freefibrelength
        return self.RotByE3(res) if bRotated else res

    def applyE3Winding(self, prevE3):
        lE3 = self.E3
        self.E3 = lE3 + 360*int((abs(prevE3 - lE3)+180)/360)*(1 if prevE3 > lE3 else -1)

    def applyE1Winding(self, prevE1):
        lE1 = self.E1
        self.E1 = lE1 + 360*int((abs(prevE1 - lE1)+180)/360)*(1 if prevE1 > lE1 else -1)

SRCparameters = ["X", "Y", "Z", "A", "E1", "E3", "E4"]
def srcpt(ps):
    return "{%s}" % ", ".join("%s %+9.3f" % (c, ps[c])  for c in SRCparameters  if c in ps)


def slerp(vec1, vec2, nsplit, pt1, pt2, fleng1, fleng2):
    res = [ ]
    for i in range(1, nsplit):
        lam = i*1.0/nsplit
        fleng = Along(lam, fleng1, fleng2)
        vec = P3.ZNorm(Along(lam, vec1, vec2))*fleng
        pt = Along(lam, pt1, pt2)
        res.append((vec, pt))
    return res

def okaypressed():
    print("Okay Pressed") 
    if qoptionsrcdebug.isChecked():
        SRCparameters.extend(["E1a", "fleng"])
    
    toolpathobject = freecadutils.findobjectbylabel(qtoolpath.text())
    tcpconstXval = float(qxconst.text())
    tcpconstXarcYs = sorted([float(x.strip())  for x in qxconstarcys.text().split(",")])

    tcpE3offset = float(qE3offset.text())
    Ymid = float(qyoffset.text())
    nswitcheroosplit = max(1, int(qswitchsplit.text()))

    cr = abs(tcpconstXval)
    crylohi = tcpconstXarcYs if tcpconstXarcYs else [ -1e5, 1e5 ]
    textlen = float(qtoolpathlength.text()) if len(qtoolpathlength.text()) != 0 and qtoolpathlength.text()[-1] != " " else None
    tapecurve = [ P3(p.X, p.Y, p.Z)  for p in toolpathobject.Shape.Vertexes ]
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

    def srctcp(tcp):
        return srcpt({"X":tcp.X, "Y":tcp.Y + Ymid, "Z":tcp.Z, "E1":tcp.E1, "E1a":tcp.E1a, "E3":tcp.E3*1000/360, "fleng":tcp.freefibrelength})

    headersrc = os.path.join(os.path.split(__file__)[0], "header.src")
    print("making toolpath: ", os.path.abspath(foutputsrc))
    fout = open(foutputsrc, "w")
    fout.write(open(headersrc).read())
    fout.write("SLIN %s\n" % srcpt({"X":-200, "Y":Ymid, "Z":0, "A":0, "E1":0, "E3":0, "E4":0}))
    for i in range(len(tcpblockslinked)):
        tcpblock = tcpblockslinked[i]
        if i == 0:
            fout.write("\nSLIN %s\n" % srctcp(tcpblock[0]))
            fout.write("HALT\n")
        elif tcpblockslinkedstarthalt[i]:
            fout.write("HALT  ; switchback %s\n" % ("up" if tcpblockslinkedstarthalt[i] == 1 else "down"))
        fout.write("SPLINE\n")
        for tcp in tcpblock[1:]:
            fout.write("SPL %s\n" % srctcp(tcp))
        fout.write("ENDSPLINE\n\n")
    fout.write("SLIN %s\n" % srcpt({"X":-200, "Y":Ymid, "E1":0}))
    fout.write("HALT\nEND\n")
    fout.close()
    qw.hide()
    
qw = QtGui.QWidget()
qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
qw.setGeometry(700, 500, 570, 350)
qw.setWindowTitle('Post process toolpath')
qtoolpath = freecadutils.qrow(qw, "Toolpath: ", 15+35*1)
qyoffset = freecadutils.qrow(qw, "Yoffset: ", 15+35*5, "307.5")
qoutputsrcfile = freecadutils.qrow(qw, "Output file: ", 15+35*6, os.path.abspath("filwin10.src"))
qthintol = freecadutils.qrow(qw, "Thinning tol: ", 15+35*7, "0.2")
qoutputsweepmesh = freecadutils.qrow(qw, "sweepmesh: ", 15+35*6, "m1*", 260)
qoutputsweeppath = freecadutils.qrow(qw, "sweeppath: ", 15+35*7, "h1*", 260)

okButton = QtGui.QPushButton("Post", qw)
okButton.move(180, 15+35*8)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)  

qtoolpath.setText(freecadutils.getlabelofselectedwire())
qxconst = freecadutils.qrow(qw, "xconst: ", 15+35*2, "-115")
qxconstarcys = freecadutils.qrow(qw, "xconst-arcys: ", 15+35*3, "-140,140")

qE3offset = freecadutils.qrow(qw, "E3offset ang: ", 15+35*4, "-45")   # in the XZ from the horizontal plane
qtoolpathlength = freecadutils.qrow(qw, "(Length): ", 15+35*1, "0 ", 260)
qswitchsplit = freecadutils.qrow(qw, "switchsplit: ", 15+35*2, "3", 260)

qoptionsrcdebug = QtGui.QCheckBox("Dbg SRC params", qw)
qoptionsrcdebug.move(80+260, 15+35*3)
qoptionsrcdebug.setChecked(False)


toolpathobject = freecadutils.findobjectbylabel(qtoolpath.text())
if toolpathobject:
    qtoolpathlength.setText("%.0f " % toolpathobject.Shape.Length)
    print("xmax", toolpathobject.Shape.BoundBox.XMax, "zmax", toolpathobject.Shape.BoundBox.ZMax)
    boxdiagrad = toolpathobject.Shape.BoundBox.XMax*math.sqrt(2)  # 45 degree diagonal puts us above the mandrel
    qxconst.setText("%.1f" % (-(boxdiagrad + 5.0)))
    qxconstarcys.setText("%.1f,%.1f" % (toolpathobject.Shape.BoundBox.YMin-2, toolpathobject.Shape.BoundBox.YMax+2))

qw.show()


