# -*- coding: utf-8 -*-
# Use the Barmesh system to run a geodesic line continuing from the start of 
# the selected wire path

import Draft, Part, Mesh, MeshPart
from FreeCAD import Vector, Rotation 

import os, sys, math
sys.path.append(os.path.join(os.path.split(__file__)[0]))

from curvesutils import isdiscretizableobject, discretizeobject

doc = App.ActiveDocument
gui = App.Gui
sel = Gui.Selection.getSelection()

linearizablesel = [ s  for s in sel  if isdiscretizableobject(s) ] 
for s in linearizablesel:
    pts = discretizeobject(s, deflection=0.02)
    Part.show(Part.makePolygon(pts))

