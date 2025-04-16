import sys, imp
sys.path.append(r"/home/goatchurch/geom3d/barmesh")
from basicgeo import P2, P3, Partition1
import barmesh, triangleboxing, mainfunctions, barmeshcell, implicitareaballoffset

sendactivity("clearalltriangles")
#fname = "/home/goatchurch/geom3d/stlfiles/impellor1.stl"
#tbm = mainfunctions.TBMfromSTL(fname, (lambda t: (t[0], -t[2], t[1])))
#z = 6.5
fname = "/home/goatchurch/geom3d/stlfiles/barobox.stl"
tbm = mainfunctions.TBMfromSTL(fname)
z = tbm.zlo*0.7 + tbm.zhi*0.3

#sendactivity("triangles", codetriangles=tbm.GetBarMeshTriangles())
iaballoffset = implicitareaballoffset.ImplicitAreaBallOffset(tbm)

imp.reload(barmesh)
imp.reload(triangleboxing)
imp.reload(mainfunctions)
imp.reload(barmeshcell)

sendactivity("clearalltriangles")
sendactivity("clearallcontours")
sendactivity("clearallpoints")


# initial rectbarmesh
rd = 1.5
rex = rd + 2.5
xpart = Partition1(tbm.xlo-rex, tbm.xhi+rex, 45)
ypart = Partition1(tbm.ylo-rex, tbm.yhi+rex, 37)
bm = barmesh.BarMesh()
bm.BuildRectBarMesh(xpart, ypart, z)
rd2 = max(xpart.vs[1]-xpart.vs[0], ypart.vs[1]-ypart.vs[0], rd*1.5) + 0.1
bms = barmeshcell.barmeshslicer(bm, iaballoffset, rd=rd, rd2=rd2, contourdotdiff=0.95, contourdelta=0.05, lamendgap=0.001)

bms.initializecutsanddistances()
bms.plotslicer(sendactivity)
sendactivity("points", points=bms.getbarendclos(bm.bars), materialnumber=2)


currentcolour = -1



ncount = 0
while bms.barpolycuts:
    currentcolour += 1
    if currentcolour > bm.maxcellcolour:
        currentcolour = 0
        bms.plotslicer(sendactivity)
    bms.splitbarpolyscolour(currentcolour)
    ncount += 1
    if ncount == 10:
        print("breaking with", len(bms.barpolycuts))
        break


-----------------------


import sys, impimport sys, imp
sys.path.append(r"/home/goatchurch/geom3d/barmesh")
from basicgeo import P2, P3, Partition1
import barmesh, triangleboxing, mainfunctions, barmeshcell
import implicitareaballoffset, implicitareacyloffset

imp.reload(barmesh)
imp.reload(implicitareacyloffset)
imp.reload(mainfunctions)
imp.reload(barmeshcell)

sendactivity("clearalltriangles")
#fname = "/home/goatchurch/geom3d/stlfiles/impellor1.stl"
#tbm = mainfunctions.TBMfromSTL(fname, (lambda t: (t[0], -t[2], t[1])))
#z = 6.5
fname = "/home/goatchurch/geom3d/stlfiles/barobox.stl"
tbm = mainfunctions.TBMfromSTL(fname)
z = tbm.zlo*0.7 + tbm.zhi*0.3

#sendactivity("triangles", codetriangles=tbm.GetBarMeshTriangles())
iacyloffset = implicitareacyloffset.ImplicitAreaCylOffset(tbm)


sendactivity("clearalltriangles")
sendactivity("clearallcontours")
sendactivity("clearallpoints")


# initial rectbarmesh
rd = 3.3
rex = rd + 2.5
xpart = Partition1(tbm.xlo-rex, tbm.xhi+rex, 45)
ypart = Partition1(tbm.ylo-rex, tbm.yhi+rex, 37)
bm = barmesh.BarMesh()
bm.BuildRectBarMesh(xpart, ypart, z)
rd2 = max(xpart.vs[1]-xpart.vs[0], ypart.vs[1]-ypart.vs[0], rd*1.5) + 0.1
iacyloffset.SetCylZrg(z - 0.2, z + 5.2)
bms = barmeshcell.barmeshslicer(bm, iacyloffset, rd=rd, rd2=rd2, contourdotdiff=0.95, contourdelta=0.05, lamendgap=0.001)

bms.initializecutsanddistances()
bms.plotslicer(sendactivity)
sendactivity("points", points=bms.getbarendclos(bm.bars), materialnumber=2)
gggg

currentcolour = -1


ncount = 0
while bms.barpolycuts:
    currentcolour += 1
    if currentcolour > bm.maxcellcolour:
        currentcolour = 0
        bms.plotslicer(sendactivity)
    bms.splitbarpolyscolour(currentcolour)
    ncount += 1
    if ncount == 10:
        print("breaking with", len(bms.barpolycuts))
        break

sys.path.append(r"/home/goatchurch/geom3d/barmesh")
from basicgeo import P2, P3, Partition1
import barmesh, triangleboxing, implicitareaballoffset
import mainfunctions, barmeshcell

sendactivity("clearalltriangles")
#fname = "/home/goatchurch/geom3d/stlfiles/impellor1.stl"
#tbm = mainfunctions.TBMfromSTL(fname, (lambda t: (t[0], -t[2], t[1])))

fname = "/home/goatchurch/geom3d/stlfiles/barobox.stl"
tbm = mainfunctions.TBMfromSTL(fname, (lambda t: (t[0], t[1], t[2])))

#sendactivity("triangles", codetriangles=tbm.GetBarMeshTriangles())
iaballoffset = implicitareaballoffset.ImplicitAreaBallOffset(tbm)

imp.reload(barmesh)
imp.reload(triangleboxing)
imp.reload(mainfunctions)
imp.reload(barmeshcell)

sendactivity("clearalltriangles")
sendactivity("clearallcontours")
sendactivity("clearallpoints")

rd = 1.1

def MakeSlice(z):
    rex = rd + 2.5
    bm = barmesh.BarMesh()
    bm.BuildRectBarMesh(Partition1(tbm.xlo-rex, tbm.xhi+rex, 45), Partition1(tbm.ylo-rex, tbm.yhi+rex, 37), z)
    bms = barmeshcell.barmeshslicer(bm, iaballoffset, rd=rd, contourdotdiff=0.95, contourdelta=0.05, lamendgap=0.001)

    bms.initializecutsanddistances()

    currentcolour = -1

    ncount = 0
    while bms.barpolycuts:
        currentcolour += 1
        if currentcolour > bm.maxcellcolour:
            currentcolour = 0
        bms.splitbarpolyscolour(currentcolour)
        ncount += 1
        if ncount == 20:
            print("breaking with", len(bms.barpolycuts))
            break

    #print([bc.GetCellMark().cellcolour  for bc in bms.barpolycuts])
    print("totaltime:", bms.totalproctime, "triangletime:", bms.trianglehacktime)
    sendactivity("contours", contours=mainfunctions.BarMeshContoursF(bm, barmesh.PZ_BEYOND_R)[0])

for i in range(0, 12):
    lam = (i+0.5)/12
    z = (tbm.zlo-rd)*(1 - lam) + (tbm.zhi+rd)*lam
    MakeSlice(z)



-------------------------




import sys, imp
sys.path.append(r"/home/goatchurch/geom3d/barmesh")
from basicgeo import P2, P3, Partition1
import barmesh, triangleboxing, mainfunctions, barmeshcell
import implicitareaballoffset, implicitareacyloffset

imp.reload(barmesh)
imp.reload(implicitareacyloffset)
imp.reload(mainfunctions)
imp.reload(barmeshcell)

sendactivity("clearalltriangles")
#fname = "/home/goatchurch/geom3d/stlfiles/impellor1.stl"
#tbm = mainfunctions.TBMfromSTL(fname, (lambda t: (t[0], -t[2], t[1])))
#z = 6.5
fname = "/home/goatchurch/geom3d/stlfiles/barobox.stl"
tbm = mainfunctions.TBMfromSTL(fname)
z = tbm.zlo*0.7 + tbm.zhi*0.3

#sendactivity("triangles", codetriangles=tbm.GetBarMeshTriangles())
iacyloffset = implicitareacyloffset.ImplicitAreaCylOffset(tbm)


sendactivity("clearalltriangles")
sendactivity("clearallcontours")
sendactivity("clearallpoints")


# initial rectbarmesh
rd = 3.3
rex = rd + 2.5
xpart = Partition1(tbm.xlo-rex, tbm.xhi+rex, 45)
ypart = Partition1(tbm.ylo-rex, tbm.yhi+rex, 37)
bm = barmesh.BarMesh()
bm.BuildRectBarMesh(xpart, ypart, z)
rd2 = max(xpart.vs[1]-xpart.vs[0], ypart.vs[1]-ypart.vs[0], rd*1.5) + 0.1
iacyloffset.SetCylZrg(z - 0.2, z + 5.2)
bms = barmeshcell.barmeshslicer(bm, iacyloffset, rd=rd, rd2=rd2, contourdotdiff=0.95, contourdelta=0.05, lamendgap=0.001)

bms.initializecutsanddistances()
bms.plotslicer(sendactivity)
sendactivity("points", points=bms.getbarendclos(bm.bars), materialnumber=2)
gggg

currentcolour = -1


ncount = 0
while bms.barpolycuts:
    currentcolour += 1
    if currentcolour > bm.maxcellcolour:
        currentcolour = 0
        bms.plotslicer(sendactivity)
    bms.splitbarpolyscolour(currentcolour)
    ncount += 1
    if ncount == 10:
        print("breaking with", len(bms.barpolycuts))
        break
