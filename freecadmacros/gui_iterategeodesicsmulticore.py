
import multiprocessing
from PySide import QtGui, QtCore
import FreeCAD

import os, sys
# Adding the current file's directory to the system path so it can import other scripts/modules
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])

# Disable input redirection for subprocesses (required for multiprocessing)
sys.stdin = open(os.devnull)

# Import custom FreeCAD utilities for interacting with the FreeCAD environment
import utils.freecadutils as freecadutils
from utils.directedgeodesic import directedgeodesic, makebicolouredwire, directedgeodesic_find_AngCross

# Initialise the FreeCAD application
freecadutils.init(App)

# Function to be called when the "Drive" button is pressed
def okaypressed():

    print(f'\n DRIVE PRESSED \n')

    # Make input variables global
    #global combofoldbackmode, sketchplane, meshobject, outputfilament, alongwire, dsangle, sideslipturningfactorZ, maxlength, passnumber, alongwireadvanceI, alongwireI, iterate_counter, showpaths, gbsall
    #iterate_counter = 0
    gbsall = []
    print('Global variables defined')

    # Get the selected index of the combo box for foldback mode
    combofoldbackmode = qcombomode.currentIndex()
    
    # Retrieve objects by label from the FreeCAD e    passnumber = 3nvironment (Sketch and Mesh)
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

    geodesic_vars_dict = [combofoldbackmode, sketchplane, meshobject, outputfilament,
                          alongwire, dsangle, sideslipturningfactorZ, maxlength, passnumber,
                          alongwireI, showpaths, gbsall
                          ]

    iterate_geodesics(geodesic_vars_dict)



def iterate_geodesics(geodesic_vars_dict):

# Tries different Anglefilament and Alongwire values systematically until working values found (path lands)
    # Let i = angle, j = along
    passnumber = geodesic_vars_dict[8]
    showpaths = geodesic_vars_dict[10]
    print(f'passnumber is {passnumber}')
    for k in range (passnumber):
        print(f'Generating pass {k + 1}')
        landed = False
        i = 0
        while not landed:
            FreeCAD.Gui.updateGui()

            processes= []
            output_queue = multiprocessing.Queue()  # Create a queue to collect results


            for j in range(i + 1): 
                p = multiprocessing.Process(target = check_landing, args=(geodesic_vars_dict, i-j, j, output_queue))
                processes.append(p)
                p.start()
            
                if (i - j) != 0:
                    p = multiprocessing.Process(target = check_landing, args=(geodesic_vars_dict, -i+j, j, output_queue))
                    processes.append(p)
                    p.start()

                if (j) != 0:
                    p = multiprocessing.Process(target = check_landing, args=(geodesic_vars_dict, i-j, -j, output_queue))
                    processes.append(p)
                    p.start()

                if (i - j) != 0 and (j) != 0:
                    p = multiprocessing.Process(target = check_landing, args=(geodesic_vars_dict, -i+j, -j, output_queue))
                    processes.append(p)
                    p.start()

            # Start all processes at once
            #for p in processes:
                #print(f'Starting process {p.name}')
            #    p.start()
            
            # Wait for all processes to finish
            for p in processes:
                p.join()
            

            # Collect results from the queue
            results = []
            #landed_results = []
            while not output_queue.empty(): #while data is available in output queue
                result = output_queue.get(block=False)  # Get the result from the queue
                results.append(result)

            # Process the results
            for result in results:
                if not landed:
                    if result[0]: #if directed geodesic landed
                        geodesic_vars_dict = process_landed_result(result, k, geodesic_vars_dict)
                        landed = True         
              
            
            i += 1  # Increment i and continue searching in the next iteration
    if showpaths == 'Combined path only':
        outputfilament = geodesic_vars_dict[3]
        gbsall = geodesic_vars_dict[11]
        makebicolouredwire(gbsall, outputfilament, colfront=(1.0,0.0,0.0), leadcolornodes=0)


def process_landed_result (landed_result, k, geodesic_vars_dict):

    landed, new_dsangle, new_alongwire = landed_result
    (combofoldbackmode, sketchplane, meshobject, outputfilament, alongwire, dsangle, sideslipturningfactorZ,
     maxlength, passnumber, alongwireI, showpaths, gbsall) = geodesic_vars_dict

    gbs, fLRdirection, dseg, alongwirelanded, angcross = directedgeodesic_find_AngCross(
            combofoldbackmode, sketchplane, meshobject, new_alongwire, alongwireI, new_dsangle,
            Maxsideslipturningfactor, mandrelradius, sideslipturningfactorZ, maxlength, outputfilament
    )
    geodesic_vars_dict[4] = alongwirelanded
    geodesic_vars_dict[5] = angcross

    if showpaths == 'Landed only':
        makebicolouredwire(gbs, outputfilament, colfront=(1.0,0.0,0.0) if fLRdirection == -1 else (0.0,0.0,1.0), 
                        colback=(0.7,0.7,0.0), leadcolornodes=dseg+1)
    elif showpaths == 'Combined path only':
        if len(gbsall) == 0:
                gbsall = gbs
                
        else:
            gbsall.extend(gbs) #add new gbs
    geodesic_vars_dict[11] = gbsall
    #print(f'Pass {k + 1} generated \n')
    return geodesic_vars_dict
                            

def set_inc_values ():
    ang_inc = 0.3
    alo_inc = 0.005
    return (ang_inc, alo_inc)

def check_landing(geodesic_vars_dict, i, j, output_queue):
    print('check landing called with multicore')

    (combofoldbackmode, sketchplane, meshobject, outputfilament, alongwire, dsangle, sideslipturningfactorZ,
     maxlength, passnumber, alongwireI, showpaths, gbsall) = geodesic_vars_dict

    ang_inc, alo_inc = set_inc_values()

    new_dsangle = dsangle + ang_inc * i
    new_alongwire = alongwire + alo_inc * j

    try:
        #See if parameters land
        gbs, fLRdirection, dseg, alongwirelanded, angcross = directedgeodesic_find_AngCross(
            combofoldbackmode, sketchplane, meshobject, new_alongwire, alongwireI, new_dsangle,
            Maxsideslipturningfactor, mandrelradius, sideslipturningfactorZ, maxlength, outputfilament
        )

        if alongwirelanded is not None:
            #Filament landed
            output_queue.put((True, new_dsangle, new_alongwire))

        else:
            output_queue.put((False, i, j))
    
    except Exception as e:
        print(f'Error occured in try block:')
        print(e)
        output_queue.put((False, i, j))

    




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
qw.setWindowTitle('Iterate drive geodesic multicore')  # Set the window title
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
#qsetpath.addItem("All paths")  # Outputs all paths including failed attempts
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
