# -*- coding: utf-8 -*-

# This file builds on gui_directedgeodesicpaths.py to iterate geodesic curves to find working values for 'Along wire' and 'Angle filament'
# directional geodesics from embedded curve controlling endpoint
# Embed a curve into a mesh so we can head off in different directions and tell when it is crossed

# Import Qt GUI toolkits. These allow GUI's to be created within Python
from PySide import QtGui, QtCore
import FreeCAD

import os, sys
# Adding the current file's directory to the system path so it can import other scripts/modules
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])

# Import custom FreeCAD utilities for interacting with the FreeCAD environment
import utils.freecadutils as freecadutils
from utils.directedgeodesic import directedgeodesic, makebicolouredwire, directedgeodesic_find_AngCross

# Import numpy for math operations
import numpy as np

# Initialise the FreeCAD application
freecadutils.init(App)

# Function to be called when the "Drive" button is pressed
def okaypressed():

    print(f'\n DRIVE PRESSED \n')

    # Make input variables global
    global combofoldbackmode, sketchplane, meshobject, outputfilament, alongwire, dsangle, sideslipturningfactorZ, maxlength, passnumber, alongwireadvanceI, alongwireI, iterate_counter, showpaths, gbsall
    iterate_counter = 0
    gbsall = []
    print('Global variables defined')

    # Get the selected index of the combo box for foldback mode
    combofoldbackmode = qcombomode.currentIndex()
    
    # Retrieve objects by label from the FreeCAD environment (Sketch and Mesh)
    sketchplane = freecadutils.findobjectbylabel(qsketchplane.text())
    meshobject = freecadutils.findobjectbylabel(qmeshobject.text())
    
    # Get the output filament name from the corresponding input field
    outputfilament = qoutputfilament.text()
    
    # If either the Sketch or Mesh object isn't selected, display a message and exit
    if not (sketchplane and meshobject):
        print("Need to select a Sketch and a Mesh object in the UI to make this work")
        qw.hide()
        return
    


    # Retrieve values from text boxes, convert them to floats as needed
    alongwire = float(qalongwire.text())
    dsangle = 90 + float(qanglefilament.text())  # Convert angle input to filament angle
    sideslipturningfactorZ = float(qsideslip.text())  # Sideslip factor (used for geodesic calculations)
    maxlength = float(qmaxlength.text())  # Maximum length for the filament path
    passnumber = int(qpassnumber.text()) # Number of passes of filament around mandrel
    showpaths = str(qsetpath.currentText())  # Which paths to show


    # Handle optional advanced wire advancement input, if provided
    if len(qalongwireadvanceI.text()) != 0:
        alongwireadvanceI = float(qalongwireadvanceI.text())
        alongwireI = (alongwire + alongwireadvanceI) % 1.0  # Calculate new wire advancement
    else:
        alongwireI = None  # If not provided, set to None

    iterate_geodesics()
   

# Function to try different geodesic paths until finding one that works
def iterate_geodesics():
    
    # Tries different Anglefilament and Alongwire values systematically until working values found
    # Could have a function to find ratio of how changing filament angle vs along wire affects wind path. Attempts at this didn't work well so far
    # Current measure - vary along by 0.005 and angle by 0.3
    # Let i = angle, j = along

    # Try winding paths until one lands:

    for k in range (passnumber):
        print(f'\n PASS NUMBER {k + 1} \n')
        finished = False
        i = 0
        while not finished:
            for j in range(i + 1): 
                if check_landing(i - j, j):
                    finished = True
                    break  # Exit the loop if filament lands

                if (i - j) != 0 and check_landing(-i + j, j):
                    finished = True
                    break  

                if (j) != 0 and check_landing(i - j, -j):
                    finished = True
                    break 

                if (i - j) != 0 and (j) != 0 and check_landing(-i + j, -j):
                    finished = True
                    break  
            
            i += 1  # Increment i and continue searching in the next iteration

  
    # Plot combined path if selected in GUI
    if showpaths == 'Combined path only':
        makebicolouredwire(gbsall, outputfilament, colfront=(0.0,0.0,1.0), 
                        colback=(0.7,0.7,0.0), leadcolornodes=0)
    


#--------- Set along/angle increment values ---------

#def sinemod (angcross):
#    a = np.abs (np.sin(angcross * np.pi/180))
#    if a > 0.75:
#        a = 0.75

#    return (a)

# For high dsangle (~0, 180), high alo and low ang
# For low dsangle (~90, 270), low alo and high ang

#def set_inc_values (angcross):# Import FreeCAD for gui refresh
import FreeCAD
#    ang_inc = (sinemod(angcross)) * 0.3
#    alo_inc = (1 - sinemod(angcross)) * 0.05
#    return (ang_inc, alo_inc)

def set_inc_values (angcross):
    ang_inc = 0.3
    alo_inc = 0.005
    return (ang_inc, alo_inc)



def check_landing (i, j):
    global iterate_counter, dsangle, alongwire, showpaths
    iterate_counter += 1
    ang_inc, alo_inc = set_inc_values(dsangle)

    new_dsangle = dsangle + ang_inc * i
    new_alongwire = alongwire + alo_inc * j
    # Perform geodesic calculations using the directedgeodesic function

    # try used to handle assertion errors and attribute errors associated with long run times...

    try:
        gbs, fLRdirection, dseg, alongwirelanded = directedgeodesic(
            combofoldbackmode, sketchplane, meshobject, new_alongwire, alongwireI, new_dsangle,
            Maxsideslipturningfactor, mandrelradius, sideslipturningfactorZ, maxlength, outputfilament, showpaths
        )

        #print(f'gbs is {gbs}')
        print(f'DSangle is {dsangle}, along increment is {alo_inc}, angle increment is {ang_inc}')
        print(f'TESTING VALUES: i = {i}, j = {j}, anglefilament = {(dsangle-90):.4f} + {(i*ang_inc):.4f} = {(new_dsangle-90):.4f}, alongwire = {alongwire:.4f} + {(j*alo_inc):.4f} = {new_alongwire:.4f}')
        FreeCAD.Gui.updateGui()
        
        # Update the advanced wire input field if the wire has landed (completed its path)
        if alongwirelanded:
            qalongwireadvanceI.setText("%.4f" % ((alongwirelanded - alongwire + 1) % 1))
            print(f'FILAMENT LANDED, {iterate_counter + 1} ITERATIONS RUN')
            # Find angcross

            gbs, fLRdirection, dseg, alongwirelanded, angcross = directedgeodesic_find_AngCross(
            combofoldbackmode, sketchplane, meshobject, new_alongwire, alongwireI, new_dsangle,
            Maxsideslipturningfactor, mandrelradius, sideslipturningfactorZ, maxlength, outputfilament
        )

            print(f'Angcross is {angcross}, Alongwirelanded is {alongwirelanded}')
            #global dsangle, alongwire
            dsangle = angcross
            alongwire = alongwirelanded

            # Plot landed paths in different color
            if showpaths == 'Landed only' or showpaths == 'All paths':
                makebicolouredwire(gbs, outputfilament, colfront=(1.0,0.0,0.0) if fLRdirection == -1 else (0.0,0.0,1.0), 
                                colback=(0.7,0.7,0.0), leadcolornodes=dseg+1)


            # Create gbs for overall path (combines successful path from each pass)
            global gbsall 
            if len(gbsall) == 0:
                gbsall = gbs
                
            else:
                gbsall.extend(gbs) #add new gbs

            return(True) # Filament landed = True
        
        else:
            print('Filament not landed')
            return(False)
        
    except:
        print('\n EXCEPTION OCCURED \n')
        return(False)




# ------------------------- GUI ----------------------------

# Define constants used for geodesic calculations
Maxsideslipturningfactor = 0.26  # Maximum turning factor for the sideslip
mandrelradius = 110  # Mandrel radius for geodesic calculations (usually the radius of the structure the filament wraps around)
# mandrelradius = 125  # Alternative mandrel radius (commented out)
anglefilament = 20  # Angle of the filament (in degrees)
maxlength = 6000  # Maximum length of the filament path
passnumber = 1 # Set number of passes of filament around mandrel


# Create the main Qt widget (window)
qw = QtGui.QWidget()
qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)  # Keep the window on top
qw.setGeometry(700, 500, 570, 400)  # Set the window's geometry (position and size)
qw.setWindowTitle('Iterate drive geodesic')  # Set the window title
qw.setStyleSheet("background-color: darkgray;")  # Set background colour to dark gray for better visibility
# qw.width(300) # Set dialog boxes to minimum size ......

# Create input fields (QLineEdit) for various parameters and place them in the window
qsketchplane = freecadutils.qrow(qw, "Sketchplane: ", 15+35*0)
qmeshobject = freecadutils.qrow(qw, "Meshobject: ", 15+35*1)
qalongwire = freecadutils.qrow(qw, "Along wire: ", 15+35*2, "0.51")  # Default value "0.51"
qanglefilament = freecadutils.qrow(qw, "Angle filmnt: ", 15+35*3, "%.1f" % anglefilament)
qpassnumber = freecadutils.qrow(qw, "Pass number: ", 15+35*4, "%.0f" % passnumber)

qmaxlength = freecadutils.qrow(qw, "maxlength: ", 15+35*0, "%.2f" % maxlength, 260)
qalongwireadvanceI = freecadutils.qrow(qw, "AlgWre adv(+): ", 15+35*1, "", 260)

# Create a label to show instruction text
vlab = QtGui.QLabel("clear above to go one direction", qw)
vlab.move(20+260, 15+35*2 +5)

# Create a text box for the sideslip factor (Z direction)
qsideslip = freecadutils.qrow(qw, "Side slip: ", 15+35*3, "0", 260)

# Create a combo box to select the foldback mode for the geodesic (normal or foldback)
qcombomode = QtGui.QComboBox(qw)
qcombomode.move(120+260, 15+35*4)
qcombomode.addItem("Mode0 normal")  # Normal mode
qcombomode.addItem("Mode1 foldback")  # Foldback mode (other modes can be added if necessary)
# qcombomode.addItem("Mode2 reflect")  # Reflect mode (commented out)
qcombomode.setCurrentIndex(0)  # Set the default mode to "Mode0 normal"

# Create combo box for Show paths (show failed attempts or not)
showpathlabel = QtGui.QLabel("Show Paths", qw)
showpathlabel.move(20+260, 15+35*5 +5)

qsetpath = QtGui.QComboBox(qw)
qsetpath.setObjectName("setpaths")
qsetpath.move(120+260, 15+35*5)
qsetpath.addItem("Combined path only")  # Combines all successful parts, outputs one path
qsetpath.addItem("Landed only")  # Outputs all successful paths individually
qsetpath.addItem("All paths")  # Outputs all paths including failed attempts
qsetpath.setCurrentIndex(0)  # Set the default mode to Combined paths

# Create the text box for the output filament name
qoutputfilament = freecadutils.qrow(qw, "Output name: ", 15+35*5, "w1")

# Create the "Drive" button that triggers the okaypressed function when clicked
okButton = QtGui.QPushButton("Drive", qw)
okButton.move(180, 15+35*7)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)

# Pre-set the text fields with values from selected objects in FreeCAD (Sketch and Mesh)
qsketchplane.setText(freecadutils.getlabelofselectedsketch())
qmeshobject.setText(freecadutils.getlabelofselectedmesh())

# Show the main Qt widget (GUI)
qw.show()
