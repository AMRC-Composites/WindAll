import math
import FreeCAD
from barmesh.basicgeo import P3, P2, Along
from utils.geodesicutils import TOL_ZERO

# X - moves towards/away from the tool (perpendicular to the carriage).
# Y - moves along the carriage axis
# Z - Vertical positioning (rarely used)
# A - Yaw of the robot (around the Z axis)
# E1 - Rotation of the payout eye
# E1a - Angle we roll out from the payout eye (not a real control)
# E3 - Rotation both chucks (headstock and tailstock)
# E4 - Rotation of tailstock relative to headstock (rarely used)

SRCparameters = ["X", "Y", "Z", "A", "E1", "E3", "E4"]

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
            vec = self.RotByE3(vecR, rev = True)
            tcp = self.RotByE3(tcpR, rev = True)
            self.X = tcp.x
            self.Y = tcp.y
            self.Z = tcp.z
            
        TOL_ZERO(self.freefibrelength - vec.Len(), "freefibrewrong")
        TOL_ZERO((self.RotByE3(vec) - vecR).Len(), ("rotByE3vec failed"))

        self.E1 = P2(vec.z, vec.y).Arg()
        cE1a = vec.x/self.freefibrelength
        assert abs(cE1a) < 1.0001, cE1a
        self.E1a = math.degrees(math.acos(min(1.0, max(-1.0, cE1a))))
        
        # Not sure what this was trying to achieve, but it screws things up if you try to wind over the end.
        #print("vec.z:",vec.z)
        #if vec.z < 0.0:
        #    self.E1 = 180 + self.E1
        #    self.E1a = -self.E1a

        TOL_ZERO((tcpR - self.GetTCP(True)).Len(), "tcpmismatch")
        TOL_ZERO((vecR - self.GetVecR(True)).Len(), (vecR, self.GetVecR(True)))

    def RotByE3(self, pt, rev = False):
        if rev:
            E3 = -self.E3
        else:
            E3 = self.E3
        rvX = P3(math.cos(math.radians(E3)), 0, math.sin(math.radians(E3)))
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


def srcpt(ps):
    return "{%s}" % ", ".join("%s %+9.3f" % (c, ps[c])  for c in SRCparameters  if c in ps)
    
def srctcp(tcp,Ymid):
    return srcpt({"X":tcp.X, "Y":tcp.Y + Ymid, "Z":tcp.Z, "E1":tcp.E1, "E1a":tcp.E1a, "E3":tcp.E3*1000/360, "fleng":tcp.freefibrelength})


def slerp(vec1, vec2, nsplit, pt1, pt2, fleng1, fleng2):
    res = [ ]
    for i in range(1, nsplit):
        lam = i*1.0/nsplit
        fleng = Along(lam, fleng1, fleng2)
        vec = P3.ZNorm(Along(lam, vec1, vec2))*fleng
        pt = Along(lam, pt1, pt2)
        res.append((vec, pt))
    return res
    
def removebridges(pts):
    """Function to remove sections from a list of P3 points where the curvature changes from convex to concave"""
    newpts = [pts[0]]
    v1 = pts[1]-pts[0]
    for i in range(3,len(pts)):
        v2 = pts[i-1]-pts[i-2]
        v3 = pts[i]-pts[i-1]
        c1 = P3.Cross(v2,v1)
        c2 = P3.Cross(v3,v2)
        d = P3.Dot(c2,c1)
        if d >= 0:
            newpts.append(pts[i-2])
        else:
            FreeCAD.Console.PrintWarning("Bridging detected, dropping point: {}\n".format(pts[i-2]))
        v1 = v2
        v2 = v3
    newpts += [pts[-2],pts[-1]]
    return newpts
