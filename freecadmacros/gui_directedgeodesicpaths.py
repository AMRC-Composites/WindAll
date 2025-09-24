# -*- coding: utf-8 -*-

# directional geodesics from embedded curve controlling endpoint

# Embed a curve into a mesh so we can head off in different directions and tell when it is crossed

from PySide import QtGui, QtCore

import os, sys
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])

import utils.freecadutils as freecadutils
from utils.directedgeodesic import directedgeodesic, makebicolouredwire
freecadutils.init(App)

def okaypressed():
	sketchplane = freecadutils.findobjectbylabel(qsketchplane.text())
	print(f'sketchplane is {sketchplane}')
	meshobject = freecadutils.findobjectbylabel(qmeshobject.text())
	outputfilament = qoutputfilament.text()
	if not (sketchplane and meshobject):
		print("Need to select a Sketch and a Mesh object in the UI to make this work")
		qw.hide()
		return
	alongwire = float(qalongwire.text())
	dsangle = 90+float(qanglefilament.text())
	sideslipturningfactorZ = float(qsideslip.text())
	maxlength = float(qmaxlength.text())
	if len(qalongwireadvanceI.text()) != 0:
		alongwireadvanceI = float(qalongwireadvanceI.text())
		alongwireI = (alongwire + alongwireadvanceI) % 1.0
	else:
		alongwireI = None
	#print(f'sketchplane is {sketchplane}')
	gbs, fLRdirection, dseg, alongwirelanded = directedgeodesic(0, sketchplane, meshobject, alongwire, alongwireI, dsangle, Maxsideslipturningfactor, mandrelradius, sideslipturningfactorZ, maxlength, outputfilament, showpaths = None)
	wire = makebicolouredwire(gbs, outputfilament, colfront=(1.0,0.0,0.0) if fLRdirection == -1 else (0.0,0.0,1.0), colback=(0.7,0.7,0.0), leadcolornodes=dseg+1)
		
	if alongwirelanded:
	    alongwireI = alongwirelanded - alongwire + 1
	    qalongwireadvanceI.setText("%.4f" % ((alongwireI)%1))
		
	wire.addProperty("App::PropertyString", "filename", group="WindAll")
	wire.filename = App.ActiveDocument.Name
	wire.addProperty("App::PropertyFloat", "angle", group="WindAll")
	wire.angle = float(qanglefilament.text())
	wire.addProperty("App::PropertyFloat", "alongwire", group="WindAll")
	wire.alongwire = alongwire
	wire.addProperty("App::PropertyFloat", "alongwireAdv", group="WindAll")
	wire.alongwireAdv = alongwireI
	wire.addProperty("App::PropertyFloat", "maxlength", group="WindAll")
	wire.maxlength = maxlength
	wire.addProperty("App::PropertyBool", "mode", group="WindAll")
	wire.mode = False

Maxsideslipturningfactor = 0.26

mandrelradius = 110  

anglefilament = 20

maxlength = 6000

qw = QtGui.QWidget()
qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
qw.setGeometry(700, 500, 570, 350)
qw.setWindowTitle('Drive geodesic')
qw.setStyleSheet("background-color: darkgray;")  # Set background colour to dark gray for better visibility
qsketchplane = freecadutils.qrow(qw, "Drivecurve: ", 15+35*0)
qsketchplane.setToolTip("Enter the name of the sketch containing the drivecurve (autofilled if drivecurve is preselected)")
qmeshobject = freecadutils.qrow(qw, "Meshobject: ", 15+35*1 )
qmeshobject.setToolTip("Enter the name of the mesh to generate paths on (autofilled if mesh is preselected)")

qalongwire = freecadutils.qrow(qw, "Along wire: ", 15+35*2, "0.51")
qalongwire.setToolTip("Proportion of the way along the drive curve to start the filament path")
qanglefilament = freecadutils.qrow(qw, "Angle filament: ", 15+35*3, "%.1f" % anglefilament)
qanglefilament.setToolTip("Angle of the filament (relative to y axis)")

qmaxlength = freecadutils.qrow(qw, "maxlength: ", 15+35*1, "%.2f" % maxlength, 260)
qmaxlength.setToolTip("Maximum allowable length of path (path not generated if this is exceeded)")
qalongwireadvanceI = freecadutils.qrow(qw, "AlngWrAdv(+) ", 15+35*2, "", 260)
qalongwireadvanceI.setToolTip("Proportion of way around drive curve to path next crossing it. If left blank then this is calculated, if a value is entered the path is forced to that")

qsideslip = freecadutils.qrow(qw, "Side slip: ", 15+35*4, "0", 260)
qsideslip.setToolTip("Maximum allowable sideslip (zero to ignore)")

qoutputfilament = freecadutils.qrow(qw, "Output name: ", 15+35*4, "w1")
okButton = QtGui.QPushButton("Drive", qw)
okButton.move(180, 15+35*7)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)  

qsketchplane.setText(freecadutils.getlabelofselectedsketch())
qmeshobject.setText(freecadutils.getlabelofselectedmesh())

qw.show()

# When running on PV.Fcad
# ang=-30 pos=0.51  adv=0.57
# ang=-145 pos=0.08  adv=0.46 (actually -0.54)
# we have now advanced to 0.54, or 0.03* 2*pi*125 = 23.5mm


# Suppose we want to advance one tape width per switchback
# tapewidth = 10, tiltedtapewidth = 10/abs(sin(ang)) = 20
# advance = 20/girth = 0.0254
# equalize this advance between the two steps

# ang=-30 pos=0.51  adv=0.564 + 0.0254/2 = 0.5767
# return is 
#  ang=-(180-30) = -150
#  pos=0.51 + 0.5767 = 0.0867
#  adv=0.51 + 0.0254 - 0.0867 = 0.4487

#   this should be 0.017 instead of 0.0254 because of the tilt
# ang=-50 pos=0.51  adv=0.488 + 0.0254/2 = 0.5007
# return is 
#  ang=-(180-50) = -130
#  pos=0.51 + 0.5007 = 0.0107
#  adv=0.51 + 0.0254 - 0.0107 = 0.5247
