from FreeCAD import Vector
import Draft, Part, Mesh, MeshPart
from PySide import QtGui ,QtCore
from barmesh.basicgeo import I1, Partition1, P3, P2, Along
from utils.trianglemeshutils import facetbetweenbars

doc = None
sel = [ ]

def init(App):
    global doc, sel
    doc = App.ActiveDocument
    sel = App.Gui.Selection.getSelection()
    return doc, sel

def qrow(qw, slab, yval, txt="", xposoffs=0):
    v = QtGui.QLineEdit(qw); 
    v.move(120+xposoffs, yval); 
    vlab = QtGui.QLabel(slab, qw)
    vlab.move(20+xposoffs, yval+5)
    v.setText(txt)
    return v

def getlabelofselectedwire(multiples=False):
    labels = [ ]
    for s in sel:
        if hasattr(s, "Shape") and isinstance(s.Shape, Part.Wire):
            labels.append(s.Label)
            if not multiples:
                break
    return ",".join(labels)
    
def getlabelofselectedmesh():
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
    
def getlabelofselectedlam():
    for s in sel:
        if hasattr(s, "Windall"):
            return s.Label
    return ""

def getlabelofselectedsketch():
    for s in sel:
        if hasattr(s, "Module") and s.Module == 'Sketcher':
            return s.Label
    return ""
    
def findobjectbylabel(lab):
    objs = [ obj  for obj in doc.findObjects(Label=lab)  if obj.Label == lab ]
    return objs[0] if objs else None

def removeobjectbylabel(lab):
    if lab:
        objs = doc.findObjects(Label=lab)
        for obj in objs:
            if obj.Label == lab:
                doc.removeObject(obj.Name)

def showdrivebarscurve(drivebars, lab):
    removeobjectbylabel(lab)
    epts = [ Along(lam, bar.nodeback.p, bar.nodefore.p)  for bar, lam in drivebars ]
    wire = Part.show(Part.makePolygon(epts), lab)
    return wire


def showdrivebarsmesh(drivebars, lab):
    removeobjectbylabel(lab)
    def facetnoderight(bar):
        return bar.barforeright.GetNodeFore(bar.barforeright.nodeback == bar.nodefore)
    tbarfacets = [ facetbetweenbars(drivebars[i][0], drivebars[i+1][0])  for i in range(len(drivebars)-1) ]
    facets = [ [ Vector(*tbar.nodeback.p), Vector(*tbar.nodefore.p), 
                 Vector(*facetnoderight(tbar).p) ]  for tbar in tbarfacets ]
    mesh = doc.addObject("Mesh::Feature", lab)
    mesh.Mesh = Mesh.Mesh(facets)
    return mesh

def removeObjectRecurse(doc, objname):
	for o in doc.findObjects(Name=objname)[0].OutList:
		removeObjectRecurse(doc, o.Name)
	doc.removeObject(objname)
    
def getemptyfolder(doc, foldername):
	objs = doc.findObjects(Label=foldername)
	folder = objs[0] if objs else doc.addObject("App::DocumentObjectGroup", foldername)
	for o in folder.OutList:
		removeObjectRecurse(doc, o.Name)
	return folder