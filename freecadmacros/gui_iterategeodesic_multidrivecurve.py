
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
from utils.directedgeodesic_multidrivecurves import directedgeodesicmultidrivecurve

# Initialise the FreeCAD application
freecadutils.init(App)

# Function to be called when the "Drive" button is pressed
def okaypressed():

    print(f'\n DRIVE PRESSED \n')

    # Make input variables global
    #iterate_counter = 0
    gbsall = []




    # Get the selected index of the combo box for foldback mode
    #combofoldbackmode = qcombomode.currentIndex()
    # DELETE LATER
    combofoldbackmode = 0
    
    # Retrieve objects by label from FreeCAD 
    #sketchplane = freecadutils.findobjectbylabel(qsketchplane.text())

    meshobject = freecadutils.findobjectbylabel(qmeshobject.text())
    outputfilament = qoutputfilament.text()

    drivecurve1 = freecadutils.findobjectbylabel(qsketchplane1.text())
    drivecurve2 = freecadutils.findobjectbylabel(qsketchplane2.text())
    
    
    # If either the Sketch or Mesh object isn't selected, display a message and exit
    if not (drivecurve1 and drivecurve2 and meshobject):
        print("Need to select a Sketch and a Mesh object in the UI to make this work")
        qw.hide()
        return
    


    # Retrieve values from text boxes, convert them to floats as needed
    alongwire = float(qalongwire.text())
    dsangle = 90 + float(qanglefilament.text())  # Convert angle input to filament angle
    sideslipturningfactorZ = float(qsideslip.text())  # Sideslip factor (used for geodesic calculations)
    maxlength = float(qmaxlength.text())  # Maximum length for the filament path
    passnumber = int(qpassnumber.text()) # Number of passes of filament around mandrel
    angthreshmin = float(qangthreshmin.text()) # Minimum angle range plusminus from target angle to iterate
    angthreshmax = float(qangthreshmax.text()) # Max angle range plusminus from target angle to iterate
    #showpaths = str(qsetpath.currentText())  # Which paths to show
    if len(qfoldback1.text()) != 0:
        foldback1 = float(qfoldback1.text()) #target foldback angle of first drivecurve
    else:
        foldback1 = None

    if len(qfoldback2.text()) != 0:
        foldback2 = float(qfoldback2.text()) #target foldback angle of first drivecurve
    else:
        foldback2 = None

    if len(qbetween1_2.text()) != 0:
        between1_2 = float(qbetween1_2.text()) #target foldback angle of first drivecurve
    else:
        between1_2 = None



    # Handle optional advanced wire advancement input, if provided
    if len(qalongwireadvanceI.text()) != 0:
        alongwireadvanceI = float(qalongwireadvanceI.text())
        alongwireI = (alongwire + alongwireadvanceI) % 1.0  # Calculate new wire advancement
    else:
        alongwireI = None  # If not provided, set to None

    anginc, aloinc = set_inc_values()
    geodesic_vars_dict = [combofoldbackmode, drivecurve1, drivecurve2, foldback1, foldback2, between1_2, 
                          meshobject, outputfilament, alongwire, dsangle, sideslipturningfactorZ, maxlength,
                          passnumber, alongwireI, gbsall, anginc, aloinc, angthreshmin, angthreshmax
                          ]

    find_paths(geodesic_vars_dict)


def find_paths(geodesic_vars_dict):
    
    (combofoldbackmode, drivecurve1, drivecurve2, foldback1, foldback2, between1_2, meshobject, outputfilament, alongwire,
     dsangle, sideslipturningfactorZ, maxlength, passnumber, alongwireI, gbsall, anginc, aloinc, angthreshmin, angthreshmax
     ) = geodesic_vars_dict
    
    if between1_2 is not None:
        between2_1 = between1_2
    else:
        between2_1 = None
    
    print(f'passnumber is {passnumber}')
    for k in range (passnumber):
        #foldback for drivecurve1
        print(f'Pass {k+1}, drivecurve1 foldback, dsangle {dsangle}, targetangle {foldback1}')
        alongwire, dsangle, gbsall = iterategeodesics_multidrivecurve(drivecurve1, drivecurve1, foldback1, meshobject, outputfilament,
                                         alongwire, dsangle, sideslipturningfactorZ, maxlength, passnumber, alongwireI, gbsall, 
                                         anginc, aloinc, angthreshmin, angthreshmax, combofoldbackmode = 1
                                         )
        #between drivecurve1 and drivecurve2
        #ADD NEW VARIABLES AT START
        print(f'Pass {k+1}, drivecurve1 to drivecurve2, dsangle {dsangle}, targetangle {between1_2}')
        alongwire, dsangle, gbsall = iterategeodesics_multidrivecurve(drivecurve1, drivecurve2, between1_2, meshobject, outputfilament,
                                         alongwire, dsangle, sideslipturningfactorZ, maxlength, passnumber, alongwireI, gbsall, 
                                         anginc, aloinc, angthreshmin, angthreshmax, combofoldbackmode = 0
                                         )

        #foldback for drivecurve2
        print(f'Pass {k+1}, drivecurve2 foldback, dsangle {dsangle}, targetangle {foldback2}')
        alongwire, dsangle, gbsall = iterategeodesics_multidrivecurve(drivecurve2, drivecurve2, foldback2, meshobject, outputfilament,
                                         alongwire, dsangle, sideslipturningfactorZ, maxlength, passnumber, alongwireI, gbsall, 
                                         anginc, aloinc, angthreshmin, angthreshmax, combofoldbackmode = 1
                                         )

        #betweeen drivecurve2 and drivecurve1
        print(f'Pass {k+1}, drivecurve1 to drivecurve2, dsangle {dsangle}, targetangle {between2_1}')
        alongwire, dsangle, gbsall = iterategeodesics_multidrivecurve(drivecurve2, drivecurve1, between2_1, meshobject, outputfilament,
                                         alongwire, dsangle, sideslipturningfactorZ, maxlength, passnumber, alongwireI, gbsall, 
                                         anginc, aloinc, angthreshmin, angthreshmax, combofoldbackmode = 0
                                         )
    
    makebicolouredwire(gbsall, outputfilament, colfront=(1.0,0.0,0.0), leadcolornodes=0)

def iterategeodesics_multidrivecurve(drivecurve1, drivecurve2, targetangle, meshobject, outputfilament, alongwire, dsangle,
                                         sideslipturningfactorZ, maxlength, passnumber, alongwireI, gbsall, anginc, aloinc,
                                         angthreshmin, angthreshmax, combofoldbackmode
                                         ):
    
    # Note input angle is +90 ie. 20 input is actually 110

    if targetangle is not None:
        if 0 < dsangle <= 90:
            targetangle = 90 - targetangle
        if 90 < dsangle <= 180:
            targetangle = targetangle + 90
        if 180 < dsangle <= 270:
            targetangle = 270 - targetangle
        if 270 < dsangle <= 360:
            targetangle = 270 + targetangle

    #print(f'Actual targetangle is {targetangle}')

    geodesic_vars_dict = (drivecurve1, drivecurve2, targetangle, meshobject, outputfilament, alongwire, dsangle,
                          sideslipturningfactorZ, maxlength, passnumber, alongwireI, gbsall, anginc, aloinc, combofoldbackmode)

    

    # ANGTHRESHMIN, ANGTHRESHMAX
    #angthreshmin = 3
    #angthreshmax = 20

    landed = False
    i = 0
    while not landed:
        FreeCAD.Gui.updateGui()
        processes= []
        output_queue = multiprocessing.Queue()  # Create a queue to collect results

        #angle, along
        

        for j in range(i + 1): 
            testangle = (i-j)*anginc + dsangle
            if  targetangle is None or (
                (testangle <= targetangle or abs(testangle-targetangle)< angthreshmin)
                and abs(testangle-targetangle) < angthreshmax
            ):
                p = multiprocessing.Process(target = checklanding_multidrivecurve, args=(geodesic_vars_dict, i-j, j, output_queue))
                processes.append(p)
                p.start()

                if (j) != 0:
                    p = multiprocessing.Process(target = checklanding_multidrivecurve, args=(geodesic_vars_dict, i-j, -j, output_queue))
                    processes.append(p)
                    p.start()
                
        
            testangle = (-i+j)*anginc + dsangle
            if  targetangle is None or (
                (testangle >= targetangle or abs(testangle-targetangle)< angthreshmin)
                and abs(testangle-targetangle) < angthreshmax
            ):
                if (i - j) != 0:
                    p = multiprocessing.Process(target = checklanding_multidrivecurve, args=(geodesic_vars_dict, -i+j, j, output_queue))
                    processes.append(p)
                    p.start()
            
                if (i - j) != 0 and (j) != 0:
                    p = multiprocessing.Process(target = checklanding_multidrivecurve, args=(geodesic_vars_dict, -i+j, -j, output_queue))
                    processes.append(p)
                    p.start()

        for p in processes:
                p.join()
        
        # Collect results from the queue
        results = []
        #landed_results = []
        while not output_queue.empty(): #while data is available in output queue
            result = output_queue.get(block=False)  # Get the result from the queue
            results.append(result)

        # Process the results
        #print(f'results are {results}')
        for result in results:
            if not landed:
                if result[0]: #if directed geodesic landed
                    #geodesic_vars_dict = process_landed_result(result, geodesic_vars_dict)
                    landed = True        
                    print('Path landed') 
                    alongwirelanded, angcross, gbsall = processresult_multidrivecurve(result, geodesic_vars_dict)
                    return(alongwirelanded, angcross, gbsall)
            
        
        i += 1  # Increment i and continue searching in the next iteration


def checklanding_multidrivecurve(geodesic_vars_dict, i, j, output_queue):
    # note - can't print statements from functions called as a multiprocessing.process

    (drivecurve1, drivecurve2, targetangle, meshobject, outputfilament, alongwire, dsangle,sideslipturningfactorZ,
     maxlength, passnumber, alongwireI, gbsall, anginc, aloinc, combofoldbackmode) = geodesic_vars_dict


    new_dsangle = dsangle + anginc * i
    new_alongwire = alongwire + aloinc * j

    try:
        #See if parameters land
        gbs, fLRdirection, dseg, alongwirelanded, angcross = directedgeodesicmultidrivecurve(combofoldbackmode, 
        drivecurve1, drivecurve2, meshobject, new_alongwire, alongwireI, new_dsangle, Maxsideslipturningfactor,
        mandrelradius, sideslipturningfactorZ, maxlength, outputfilament)


        if alongwirelanded is not None:
            #Filament landed
            output_queue.put((True, new_dsangle, new_alongwire))
            

        else:
            output_queue.put((False, i, j))
    
    except Exception as e:
        # print(f'Error occured in try block as: {e}')
        output_queue.put((False, i, j))



def processresult_multidrivecurve (landed_result, geodesic_vars_dict):
    #NOT CHECKED YET!

    landed, new_dsangle, new_alongwire = landed_result
    (drivecurve1, drivecurve2, targetangle, meshobject, outputfilament, alongwire, dsangle, sideslipturningfactorZ,
     maxlength, passnumber, alongwireI, gbsall, anginc, aloinc, combofoldbackmode) = geodesic_vars_dict

    gbs, fLRdirection, dseg, alongwirelanded, angcross = directedgeodesicmultidrivecurve(combofoldbackmode, 
        drivecurve1, drivecurve2, meshobject, new_alongwire, alongwireI, new_dsangle, Maxsideslipturningfactor,
        mandrelradius, sideslipturningfactorZ, maxlength, outputfilament
    )

    #delete later
    #makebicolouredwire(gbs, outputfilament, colfront=(1.0,0.0,0.0), leadcolornodes=0)


    if len(gbsall) == 0:
            gbsall = gbs  
    else:
        gbsall.extend(gbs) #add new path to gbsall

    #print(f'Pass {k + 1} generated \n')
    return alongwirelanded, angcross, gbsall
                            

def set_inc_values ():
    anginc = 0.3
    aloinc = 0.00520
    return (anginc, aloinc)

# ------------------------- GUI ----------------------------

Maxsideslipturningfactor = 0.26


mandrelradius = 110  # fc6 file
anglefilament = 30
maxlength = 6000
passnumber = 1
angthreshmin = 3
angthreshmax = 20

qw = QtGui.QWidget()
qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
qw.setGeometry(700, 500, 570, 380)
qw.setWindowTitle('Iterate geodesic multidrivecurve')
qw.setStyleSheet("background-color: darkgray;")  # Set background colour to dark gray for better visibility
qsketchplane1 = freecadutils.qrow(qw, "Sketchplane 1:", 15+35*0)
qsketchplane2 = freecadutils.qrow(qw, "Sketchplane 2:", 15+35*1)
qmeshobject = freecadutils.qrow(qw, "Meshobject: ", 15+35*2 )

qpassnumber = freecadutils.qrow(qw, "Pass number: ", 15+35*0, "%.0f" % passnumber, 260)

slab = QtGui.QLabel("Starting values:", qw)
slab.move(20+260, 20+35*4)
qalongwire = freecadutils.qrow(qw, "Along wire: ", 15+35*5, "0.51", 260)
qanglefilament = freecadutils.qrow(qw, "Angle filamnt: ", 15+35*6, "%.1f" % anglefilament, 260)

qmaxlength = freecadutils.qrow(qw, "maxlength: ", 15+35*1, "%.2f" % maxlength, 260)
qalongwireadvanceI = freecadutils.qrow(qw, "AlngWrAdv(+) ", 15+35*2, "", 260)

qsideslip = freecadutils.qrow(qw, "Side slip: ", 15+35*3, "0", 260)

qangthreshmin = freecadutils.qrow(qw, "Angthreshmin ", 15+35*7, "%.1f" % angthreshmin, 260)
qangthreshmax = freecadutils.qrow(qw, "Angthreshmax ", 15+35*8, "%.1f" % angthreshmax, 260)

dlab = QtGui.QLabel("Target path angles. Blank if none:", qw)
dlab.move(20, 20+35*4)

### Arbitary variable assign
foldback1 = None
foldback2 = None
between1_2 = 40
qfoldback1 = freecadutils.qrow(qw, "Plane1 fback: ", 15+35*5, "")
qfoldback2 = freecadutils.qrow(qw, "Plane2 fback: ", 15+35*6, "")
qbetween1_2 = freecadutils.qrow(qw, "Between 1&2: ", 15+35*7, "%.1f" % between1_2)

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
