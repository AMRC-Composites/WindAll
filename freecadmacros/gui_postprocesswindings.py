# -*- coding: utf-8 -*-

# stock viewer task, a standard interactive panel on the left in FreeCAD.  
# To edit the UI file, nix-shell -p qtcreator, start up qtcreator
# select default current project and start editing stockviewertask.ui


import Draft, Part, Mesh, MeshPart, Fem, Path
from FreeCAD import Vector, Rotation 
from PySide import QtGui, QtCore


import os, sys, math, time, numpy, datetime
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])


from barmesh.basicgeo import I1, Partition1, P3, P2, Along, lI1
from barmesh.tribarmes.triangleboxing import TriangleBoxing

import imp
import utils.directedgeodesic
imp.reload(utils.directedgeodesic)
import utils.pathutils
imp.reload(utils.pathutils)

from utils.pathutils import BallPathCloseRegions, MandrelPaths, MakeFEAcoloredmesh
import utils.freecadutils as freecadutils
freecadutils.init(App)
from utils.curvesutils import cumlengthlist, seglampos


from utils.directedgeodesic import directedgeodesic, makebicolouredwire, makedrivecurve, drivegeodesicRI, DriveCurve
from utils.trianglemeshutils import UsefulBoxedTriangleMesh
from utils.geodesicutils import drivegeodesic, InvAlong, GBarT, GBarC, drivecurveintersectionfinder, trilinecrossing, TOL_ZERO
from utils.curvesutils import thinptstotolerance

import FreeCADGui as Gui
import PySide.QtGui as QtGui
import PySide.QtCore as QtCore
import numpy

def sgetlabelofselectedmesh(sel):
    for s in sel:
        if hasattr(s, "Mesh") and isinstance(s.Mesh, Mesh.Mesh):
            return s.Label
        if s.TypeId == "Mesh::Curvature":
            if "ValueAtIndex" in s.PropertiesList:
                return s.Label
            else:
                print("***  Wrong version of FreeCAD, \n\nMeshCurvature object missing ValueAtIndex hack.")
                return "**WrongFCversion"
    return ""

def sgetlabelofselectedsketch(sel):
    for s in sel:
        if hasattr(s, "Module") and s.Module == 'Sketcher':
            return s.Label
    return ""

def sgetlabelofselectedgroupwithproperties(sel, properties):
    for s in sel:
        if isinstance(s, App.DocumentObjectGroup) and set(properties).issubset(s.PropertiesList):
            return s.Label
    return ""

def sfindobjectbylabel(doc, lab):
    objs = [ obj  for obj in doc.findObjects(Label=lab)  if obj.Label == lab ]
    return objs[0] if objs else None

def setpropertyval(obj, atype, name, value):
    if name not in obj.PropertiesList:
        obj.addProperty(atype, name, "filwind")
    setattr(obj, name, value)



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






def adjustlandingrepeatstobecoprime(alongwire, alongwirelanded, totalrepeats):
    alongwireadvance = alongwirelanded - alongwire
    if alongwireadvance <= 0.0:
        alongwireadvance += 1.0
    assert alongwireadvance > 0
    totalturns = round(alongwireadvance*totalrepeats)
    
    for ttdiff in range(10):
        if math.gcd(totalturns, totalrepeats + ttdiff) == 1:
            newtotalrepeats = totalrepeats + ttdiff
            break
        elif math.gcd(totalturns, totalrepeats - ttdiff) == 1:
            newtotalrepeats = totalrepeats - ttdiff
            break
    else:
        print("didn't find non coprime total turns ", totalturns, totalrepeats)

    newalongwireadvance =  totalturns / newtotalrepeats
    newalongwirelanded = (alongwire + newalongwireadvance) % 1
    return newalongwirelanded, newtotalrepeats

def repeatwindingpath(rpts, repeats, thintol):
    ptfront, ptback = rpts[0], rpts[-1]
    fvec0 = P2(ptfront.x, ptfront.z)
    fvec1 = P2(ptback.x, ptback.z)
    angadvance = P2(P2.Dot(fvec0, fvec1), P2.Dot(fvec0, P2.APerp(fvec1))).Arg()
    print("repeatwindingpath angadvance prop", angadvance/360)
    rpts = thinptstotolerance(rpts, tol=thintol*2)
    ptsout = rpts[:]
    for i in range(1, repeats):
        rotcos = math.cos(math.radians(i*angadvance))
        rotsin = math.sin(math.radians(i*angadvance))
        for pt in rpts:
            ptsout.append(P3(pt.x*rotcos + pt.z*rotsin, pt.y, pt.z*rotcos - pt.x*rotsin))
    ptsout = thinptstotolerance(ptsout, tol=thintol)
    return ptsout



class PostProcessWindingsTaskPanel(QtGui.QWidget):
    def __init__(self):
        x = os.path.join(os.path.split(__file__)[0], "postprocesswindingstask.ui")
        self.form = FreeCADGui.PySideUic.loadUi(x)
        self.form.setMinimumSize(0, 490)
        QtCore.QObject.connect(self.form.qbuttseesweepmesh, QtCore.SIGNAL("pressed()"), self.seesweepmesh)
        self.update()
        self.tcpblockslinked = None

    def update(self):
        self.doc = App.ActiveDocument
        self.sel = App.Gui.Selection.getSelection()
        outputwindings = sgetlabelofselectedgroupwithproperties(self.sel, ["outputwindingsgrouptype"])
        if outputwindings:
            self.form.qoutputwindings.setText(outputwindings)
        outputwindingsgroup = sfindobjectbylabel(self.doc, self.form.qoutputwindings.text())
        owbb = None
        owlngth = 0.0
        for outputwinding in outputwindingsgroup.OutList:
            owbb = owbb.united(outputwinding.Shape.BoundBox) if owbb else outputwinding.Shape.BoundBox
            owlngth += outputwinding.Shape.Length
        if owbb == None:
            print("No output winding toolpath selected")
            return
        self.form.qtoolpathlength.setValue(owlngth)
        print("xmax", owbb.XMax, "zmax", owbb.ZMax)
        boxdiagrad = owbb.XMax*math.sqrt(2)  # 45 degree diagonal puts us above the mandrel
        self.form.qxconst.setValue(-(boxdiagrad + 5.0))
        self.form.qxconstarcys.setText("%.1f,%.1f" % (owbb.YMin-2, owbb.YMax+2))
        print(self.form.qoptionsrcdebug)

    def apply(self):
        print("apply!!")
        if self.form.qoptionsrcdebug.isChecked():
            SRCparameters.extend(["E1a", "fleng"])
        outputwindingsgroup = sfindobjectbylabel(self.doc, self.form.qoutputwindings.text())
        toolpaths = outputwindingsgroup.OutList
        #toolpathobject = freecadutils.findobjectbylabel(qtoolpath.text())
        tcpconstXval = float(self.form.qxconst.text())
        tcpconstXarcYs = sorted([float(x.strip())  for x in self.form.qxconstarcys.text().split(",")])

        tcpE3offset = float(self.form.qE3offset.text())
        Ymid = float(self.form.qyoffset.text())
        nswitcheroosplit = max(1, int(self.form.qswitchsplit.text()))

        cr = abs(tcpconstXval)
        crylohi = tcpconstXarcYs if tcpconstXarcYs else [ -1e5, 1e5 ]
        textlen = float(self.form.qtoolpathlength.text()) if len(self.form.qtoolpathlength.text()) != 0 and self.form.qtoolpathlength.text()[-1] != " " else None
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
        
        #Dn = len(toolpaths[0].Shape.Vertexes)
        #tapecurve = tapecurve[Dn-5:Dn+5]
            
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
            else:
                tcp.applyE1Winding(0.0)
                print("centring E1 on zero", tcp.E1)

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
    
        self.tcpblockslinked = tcpblockslinked
        self.form.qbuttseesweepmesh.setEnabled(True)
        print("tcpblockslinked made:", len(tcpblockslinked))
    
        foutputsrc = self.form.qoutputsrcfile.text()
        headersrc = os.path.join(os.path.split(__file__)[0], "header.src")
        print("outputting src toolpath ", os.path.abspath(foutputsrc))
        #print("blocks ", list(map(len, tcpblockslinked)))
        print("nblocks ", len(tcpblockslinked))

        def srctcp(tcp):
            return srcpt({"X":tcp.X, "Y":tcp.Y + Ymid, "Z":tcp.Z, "E1":tcp.E1, "E1a":tcp.E1a, "E3":tcp.E3*1000/360, "fleng":tcp.freefibrelength})

        headersrc = os.path.join(os.path.split(__file__)[0], "header.src")
        print("making toolpath: ", os.path.abspath(foutputsrc))
        fout = open(foutputsrc, "w")
        fout.write(open(headersrc).read())
        
        fout.write(";;;;;;;;; postprocessing parameters (date %s)\n" % datetime.datetime.now().isoformat())
        for qels in ["qxconst", "qxconstarcys", "qE3offset", "qyoffset", "qswitchsplit", "qthinningtol"]:
            fout.write("; %s: %s\n" % (qels, getattr(self.form, qels).text()))
        fout.write(";;;;;;;;;\n\n")
            
        tcpconstXval = float(self.form.qxconst.text())
        tcpconstXarcYs = sorted([float(x.strip())  for x in self.form.qxconstarcys.text().split(",")])

        
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

    def seesweepmesh(self):
        i = self.form.qsweepmeshblockindex.value()
        sweepmesh = sfindobjectbylabel(self.doc, self.form.qoutputsweepmesh.text())
        if not sweepmesh:
            sweepmesh = freecadutils.doc.addObject("Mesh::Feature", self.form.qoutputsweepmesh.text())
            sweepmesh.ViewObject.Lighting = "Two side"
            sweepmesh.ViewObject.DisplayMode = "Flat Lines"
        sweeppath = sfindobjectbylabel(self.doc, self.form.qoutputsweeppath.text())
        if not sweeppath:
            sweeppath = freecadutils.doc.addObject("Path::Feature", self.form.qoutputsweeppath.text())

        facets = [ ]
        tcpblock = self.tcpblockslinked[i]
        #if not tcpblockslinkedstarthalt[i]:  continue
        for j in range(len(tcpblock)-1):
            tcp0, tcp1 = tcpblock[j].GetTCP(True), tcpblock[j+1].GetTCP(True)
            vecr0, vecr1 = tcpblock[j].GetVecR(True), tcpblock[j+1].GetVecR(True)
            fp0, fp1 = tcp0 + vecr0, tcp1 + vecr1
            facets.append([Vector(*tcp0), Vector(*fp0), Vector(*tcp1)])
            facets.append([Vector(*tcp1), Vector(*fp0), Vector(*fp1)])
        sweepmesh.Mesh = Mesh.Mesh(facets)

        pp = Path.Path()
        pp.addCommands(Path.Command("K00"))
        for tcp in tcpblock:
            # the ABC settings cause it to be drawn with splines going everywhere they don't belong because 
            # the plotting of the orientation is not done properly
            #c = Path.Command("G1", {"X":tcp.X, "Y":tcp.Y, "Z":tcp.Z, "E3":tcp.E3*1000/360, "B":tcp.E1, "C":tcp.E1a, "L":tcp.freefibrelength})
            c = Path.Command("G1", {"X":tcp.X, "Y":tcp.Y, "Z":tcp.Z, "E3":tcp.E3*1000/360})  
            pp.addCommands(c)
        #Part.show(Part.makePolygon([Vector(*tcp.GetTCP(True))  for tcp in tcpblock]), sweeppath)
        sweeppath.Path = pp
        sweeppath.ViewObject.StartPosition = Vector(tcpblock[0].X, tcpblock[0].Y, tcpblock[0].Z)


    def getStandardButtons(self):
        return int(QtGui.QDialogButtonBox.Cancel
                   | QtGui.QDialogButtonBox.Ok
                   | QtGui.QDialogButtonBox.Apply)

    def clicked(self, bt):
        if bt == QtGui.QDialogButtonBox.Apply:
            self.apply()

    def accept(self):
        print("Accept")
        self.apply()
        self.finish()

    def reject(self):
        print("Reject")
        self.finish()

    def finish(self):
        print("Finish")
        Gui.Control.closeDialog()

Gui.Control.showDialog(PostProcessWindingsTaskPanel())
