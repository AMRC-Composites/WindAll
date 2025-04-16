#gui_directedgeodesic_multisketchplanes

#-----------------------------------


# -*- coding: utf-8 -*-

# directional geodesics from embedded curve controlling endpoint

# Embed a curve into a mesh so we can head off in different directions and tell when it is crossed

from PySide import QtGui, QtCore

import os, sys
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])

import FreeCAD
import utils.freecadutils as freecadutils
from utils.directedgeodesic import directedgeodesic, makebicolouredwire
from utils.directedgeodesic_multidrivecurves import directedgeodesicmultidrivecurve
freecadutils.init(App)

def okaypressed():    
    meshobject = freecadutils.findobjectbylabel(qmeshobject.text())
    outputfilament = qoutputfilament.text()
    
    drivecurve1 = freecadutils.findobjectbylabel(qsketchplane1.text())
    drivecurve2 = freecadutils.findobjectbylabel(qsketchplane2.text())

    alongwire = float(qalongwire.text())
    dsangle = 90 + float(qanglefilament.text())
    sideslipturningfactorZ = float(qsideslip.text())
    maxlength = float(qmaxlength.text())
    
    if len(qalongwireadvanceI.text()) != 0:
        alongwireadvanceI = float(qalongwireadvanceI.text())
        alongwireI = (alongwire + alongwireadvanceI) % 1.0
    else:
        alongwireI = None

    #multi plane angle variables
    if len(qfoldback1.text()) != 0:
        foldback1 = float(qfoldback1.text())
    else:
        foldback1 = None

    if len(qfoldback2.text()) != 0:
        foldback2 = float(qfoldback2.text())
    else:
        foldback2 = None

    if len(qbetween1_2.text()) != 0:
        between1_2 = float(qbetween1_2.text())
    else:
        between1_2 = None


    print(f'foldback1 is {foldback1}, foldback2 is {foldback2}, between 1&2 is {between1_2}')
    



    # Run directed geodesic with multiple drivecurves
    #between 1 and 2
    #combofoldbackmode = 1 #foldback
    gbs, fLRdirection, dseg, alongwirelanded, angcross = directedgeodesicmultidrivecurve(0, 
        drivecurve1, drivecurve2, meshobject, alongwire, alongwireI, dsangle, Maxsideslipturningfactor,
        mandrelradius, sideslipturningfactorZ, maxlength, outputfilament)
    
    if alongwirelanded:
        print('path landed')
    else:
        print('path not landed')

    
    makebicolouredwire(gbs, outputfilament, colfront=(1.0, 0.0, 0.0) if fLRdirection == -1 else 
        (1.0, 0.0, 0.0), colback=(0.7, 0.7, 0.0), leadcolornodes=dseg+1)

Maxsideslipturningfactor = 0.26


mandrelradius = 110  # fc6 file
anglefilament = 20
maxlength = 6000

qw = QtGui.QWidget()
qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
qw.setGeometry(700, 500, 570, 380)
qw.setWindowTitle('Drive geodesic')
qw.setStyleSheet("background-color: darkgray;")  # Set background colour to dark gray for better visibility
qsketchplane1 = freecadutils.qrow(qw, "Sketchplane 1:", 15+35*0)
qsketchplane2 = freecadutils.qrow(qw, "Sketchplane 2:", 15+35*1)
qmeshobject = freecadutils.qrow(qw, "Meshobject: ", 15+35*2 )

slab = QtGui.QLabel("Starting values:", qw)
slab.move(20+260, 20+35*3)
qalongwire = freecadutils.qrow(qw, "Along wire: ", 15+35*4, "0.51", 260)
qanglefilament = freecadutils.qrow(qw, "Angle filamnt: ", 15+35*5, "%.1f" % anglefilament, 260)

qmaxlength = freecadutils.qrow(qw, "maxlength: ", 15+35*0, "%.2f" % maxlength, 260)
qalongwireadvanceI = freecadutils.qrow(qw, "AlngWrAdv(+) ", 15+35*1, "", 260)

qsideslip = freecadutils.qrow(qw, "Side slip: ", 15+35*2, "0", 260)

dlab = QtGui.QLabel("Target path angles. Blank if none:", qw)
dlab.move(20, 20+35*4)

### Arbitary variable assign
foldback1 = 0
foldback2 = 0
between1_2 = 0
qfoldback1 = freecadutils.qrow(qw, "Plane1 fback: ", 15+35*5, "")
qfoldback2 = freecadutils.qrow(qw, "Plane2 fback: ", 15+35*6, "")
qbetween1_2 = freecadutils.qrow(qw, "Between 1&2: ", 15+35*7, "")

qoutputfilament = freecadutils.qrow(qw, "Output name: ", 15+35*3, "w1")
okButton = QtGui.QPushButton("Drive", qw)
okButton.move(180, 15+35*9)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)  

selected_objects = FreeCAD.Gui.Selection.getSelection()

sketch_labels = []  # create list to store

# Iterate over selected objects and check if they are sketches
for obj in selected_objects:
    if obj.isDerivedFrom("Sketcher::SketchObject"):  # Check if it's a sketch
        sketch_labels.append(obj.Label)  # Get the label of the sketch

print(f'sketch labels are {sketch_labels}')
# Set the text for qsketchplane to display the labels
#qsketchplanestart.setText(", ".join(sketch_labels))  # Join the labels with a comma for display
qsketchplane1.setText(sketch_labels[0])  # Join the labels with a comma for display
qsketchplane2.setText(sketch_labels[1])  # Join the labels with a comma for display


#qsketchplane.setText(freecadutils.getlabelofselectedsketch())  #ERROR OCCURRING HERE
qmeshobject.setText(freecadutils.getlabelofselectedmesh())

qw.show()

