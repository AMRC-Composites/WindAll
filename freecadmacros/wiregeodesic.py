# -*- coding: utf-8 -*-
# Use the Barmesh system to run a geodesic line continuing from the start of 
# the selected wire path

import Draft, Part, Mesh, MeshPart
from FreeCAD import Vector, Rotation 

import os, sys, math
sys.path.append(os.path.join(os.path.split(__file__)[0]))

from barmesh.tribarmes import TriangleBarMesh, TriangleBar, MakeTriangleBoxing
from barmesh.basicgeo import I1, Partition1, P3, P2, Along
from geodesicutils import GeoCrossAxis, GeoCrossBar
from trianglemeshutils import UsefulBoxedTriangleMesh, facetbetweenbars
from curvesutils import isdiscretizableobject, discretizeobject

doc = App.ActiveDocument
gui = App.Gui
sel = Gui.Selection.getSelection()

screenrot = Gui.ActiveDocument.ActiveView.getCameraOrientation()
projectionDir = screenrot.multVec(Vector(0,0,-1))

meshobject = None
drivepts = None
for s in sel:
    if hasattr(s, "Mesh") and isinstance(s.Mesh, Mesh.Mesh):
        meshobject = s.Mesh
    elif isdiscretizableobject(s):
        drivepts = discretizeobject(s, deflection=0.2)
if not meshobject:
    print("Need a mesh object selected")
if not drivepts:
    print("Need a linearizable object selected")

def TriangleNodeOpposite(bar, bGoRight):
    bartop = bar.GetForeRightBL(bGoRight)
    if bartop != None:
        return bartop.GetNodeFore(bartop.nodeback == bar.GetNodeFore(bGoRight))
    return None

utbm = UsefulBoxedTriangleMesh(meshobject)
p0 = P3(*drivepts[-1])
c = P3(*drivepts[-2])
startbar, startlam = utbm.FindClosestEdge(p0)
startpt = Along(startlam, startbar.nodeback.p, startbar.nodefore.p)
print("p0 ", p0, startpt, c)
bar, lam = startbar, startlam
Neright = TriangleNodeOpposite(bar, True)
Neleft = TriangleNodeOpposite(bar, False)
bGoRight = (P3.Dot(Neright.p - Neleft.p, p0 - c) > 0.0)
gpts = [ ]
for i in range(500):
    c, bar, lam, bGoRight = GeoCrossBar(c, bar, lam, bGoRight)
    if not c:
        print("jjjk ", c, bar, lam, bGoRight)
        break
    gpts.append(Vector(*c))
    if bar == None:
        break
Part.show(Part.makePolygon(gpts))
 
