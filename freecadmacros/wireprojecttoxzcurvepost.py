# -*- coding: utf-8 -*-
# Use the MeshPart library to project a selected wire onto a selected mesh

import Draft, Part, Mesh, MeshPart
from FreeCAD import Vector, Rotation 

from curvesutils import isdiscretizableobject, discretizeobject, thinptstotolerance
from barmesh.basicgeo import I1, Partition1, P3, P2, Along
import math, numpy, copy

doc = App.ActiveDocument
gui = App.Gui
sel = Gui.Selection.getSelection()

# X - moves towards/away from the tool (perpendicular to the carriage).
# Y - moves along the carriage axis
# Z - Vertical positioning (rarely used)
# A - Yaw of the robot (around the Z axis)
# E1 - Rotation of the payout eye
# (E2 - Deflection angle from the payout eye)
# E3 - Rotation both chucks (headstock and tailstock)
# E4 - Rotation of tailstock relative to headstock (rarely used)

def TOL_ZERO(X, msg=""):
    if not (abs(X) < 0.0001):
        print("TOL_ZERO fail", X, msg)
        assert False

tcpcurve = None
tapecurve = None
for s in sel:
    if isdiscretizableobject(s):
        pts = [ P3(*p)  for p in discretizeobject(s, deflection=0.02) ]
        rg = I1.AbsorbList(pt[2]  for pt in pts)
        if rg.lo == rg.hi == 0:
            tcpcurve = pts
        else:
            assert tapecurve is None, "two tape curves found"
            tapecurve = thinptstotolerance(pts, tol=0.2)
            tapecurve = pts
            
if not tapecurve:
    print("Need a linearizable tape curve object selected")
if not tcpcurve:
    print("Need a linearizable tool centre point (tcp) curve object selected")

import scipy.interpolate
tcpinterpolator = scipy.interpolate.CubicSpline([p.y for p in tcpcurve], [p.x for p in tcpcurve], axis=0, bc_type='natural')
tcpsplineys = numpy.linspace(tcpcurve[0].y, tcpcurve[-1].y, 200)
tcpsplinexs = tcpinterpolator(tcpsplineys)
#Part.show(Part.makePolygon([Vector(x, y, 0)  for x, y in zip(tcpsplinexs, tcpsplineys)]))
#Part.show(Part.makePolygon([Vector(*p)  for p in tcpcurve]))
tcpconstXval = min(p.x for p in tcpcurve)

def projectToXvalplane(pt, vec, cx):
    qa = P2(vec.x, vec.z).Lensq()
    qb2 = pt.x*vec.x + pt.z*vec.z
    qc = P2(pt.x, pt.z).Lensq() - cx*cx
    qdq = qb2*qb2 - qa*qc
    qs = math.sqrt(qdq) / qa
    qm = -qb2 / qa
    q = qm + qs
    TOL_ZERO(qa*q*q + qb2*2*q + qc)
    res = pt + vec*q
    TOL_ZERO(P2(res.x, res.z).Len() - cx)
    return res

print("tcpconstXval", tcpconstXval, len(tapecurve))


class TCPplusfibre:
    def __init__(self, tcpR, ptR, prevtcp):
        vecR = ptR - tcpR
        self.freefibrelength = vecR.Len()

        lE3 = P2(-tcpR.x, -tcpR.z).Arg()
        prevE3 = prevtcp.E3 if prevtcp is not None else lE3
        self.E3 = lE3 + 360*int((abs(prevE3 - lE3)+180)/360)*(1 if prevE3 > lE3 else -1)
        
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
        TOL_ZERO(self.freefibrelength - vec.Len())
        self.E1 = P2(vec.z, vec.y).Arg()
        self.E2 = math.degrees(math.acos(vec.z/self.freefibrelength))
        TOL_ZERO((tcpR - self.GetTCP(True)).Len())
#        TOL_ZERO((vecR - self.GetVecR(True)).Len())


    def RotByE3(self, pt):
        rvX = P3(math.cos(math.radians(self.E3)), 0, math.sin(math.radians(self.E3)))
        rvZ = P3(-rvX.z, 0, rvX.x)
        return rvX*pt.x + P3(0, pt.y, 0) + rvZ*pt.z
    
    def GetTCP(self, bRotated):
        res = P3(self.X, self.Y, self.Z)
        return self.RotByE3(res) if bRotated else res
        
    def GetVecR(self, bRotated):
        cosE2 = math.cos(math.radians(self.E2))
        sinE2 = math.sin(math.radians(self.E2))
        cosE1 = math.cos(math.radians(self.E1))
        sinE1 = math.sin(math.radians(self.E1))
        res = P3(cosE2, sinE2*sinE1, sinE2*cosE1)*self.freefibrelength
        return self.RotByE3(res) if bRotated else res

tcps = [ ]
for i in range(len(tapecurve)):
    vecNout = P3.ZNorm(tapecurve[max(i,1)] - tapecurve[max(i,1)-1])
    ptR = tapecurve[i]
    tcpR = projectToXvalplane(ptR, vecNout, abs(tcpconstXval))
    tcp = TCPplusfibre(tcpR, ptR, tcps[-1] if len(tcps) != 0 else None)
    tcps.append(tcp)

"""
facets = [ ]
    if tapetcps:
        facets.append([Vector(*tapetcps[-1]), Vector(*tapecurve[i-1]), Vector(*tcp)])
        facets.append([Vector(*tcp), Vector(*tapecurve[i-1]), Vector(*tapecurve[i])])
    tapevecs.append(vec)
    tapetcps.append(tcp)
mesh = doc.addObject("Mesh::Feature", "tcpmesh")
mesh.Mesh = Mesh.Mesh(facets)
"""

dE3 = tcps[-1].E3 - tcps[0].E3
tcpsC = tcps[2:]
def tadd(tcp, dE3):
    res = copy.copy(tcp)
    res.E3 += dE3
    return res
tcps.extend(tadd(tcp, dE3)  for tcp in tcpsC[1:])
tcps.extend(tadd(tcp, dE3*2)  for tcp in tcpsC[1:])

tcpYfilterdistance = 1.0
Ftcps = [ ]
for tcp in tcps:
    if not Ftcps or abs(Ftcps[-1].Y - tcp.Y) > tcpYfilterdistance:
        Ftcps.append(tcp)
print("Filtered by y distance", tcpYfilterdistance, "reducing from", len(tcps), "to", len(Ftcps), "points")
tcps = Ftcps

tcpblocks = [ ]
for tcp in tcps[1:]:
    if len(tcpblocks) == 0:
        tcpblocks.append([ ])
    elif len(tcpblocks[-1]) >= 2 and ((tcpblocks[-1][-2].Y < tcpblocks[-1][-1].Y) != (tcpblocks[-1][-1].Y < tcp.Y)):
        tcpblocks.append([ tcpblocks[-1][-1] ])
    tcpblocks[-1].append(tcp)

print("blocks ", list(map(len, tcpblocks)))

def srcpt(ps):
	return "{%s}" % ", ".join("%s %+9.3f" % (c, ps[c])  for c in ["X", "Y", "Z", "A", "E1", "E3", "E4"]  if c in ps)

Ymid = 1000
def srctcp(tcp):
    return srcpt({"X":tcp.X, "Y":tcp.Y + Ymid, "E1":tcp.E1*0, "E3":tcp.E3*1000/360*0})
fname = "filwin9gNoE1E3.src"
fname = "filwin9f.src"
fname = "filwin9fNoE1E3filter1.src"

headersrc = os.path.join(os.path.split(__file__)[0], "header.src")
print("making ", os.path.abspath(fname), " from ", __file__)
fout = open(fname, "w")
fout.write(open(headersrc).read())
fout.write("SLIN %s\n" % srcpt({"X":-200, "Y":Ymid, "Z":0, "A":0, "E1":0, "E3":0, "E4":0}))
for i in range(len(tcpblocks)):
    tcpblock = tcpblocks[i]
    if i == 0:
        fout.write("\nSLIN %s\n" % srctcp(tcpblock[0]))
        fout.write("HALT\n")
        tcpblock = tcpblock[1:]
    fout.write("SPLINE\n")
    for tcp in tcpblock:
        fout.write("SPL %s\n" % srctcp(tcp))
    fout.write("ENDSPLINE\n\n")
fout.write("SLIN %s\n" % srcpt({"X":-200, "Y":Ymid, "E1":0}))
fout.write("HALT\nEND\n")
fout.close()
