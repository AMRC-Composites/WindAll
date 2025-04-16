import utils.freecadutils as freecadutils
import numpy as np
from PySide import QtGui, QtCore
import sys
import Draft

freecadutils.init(App)
sys.path.append(os.path.split(__file__)[0])

from barmesh.basicgeo import I1, Partition1, P3, P2, Along, lI1
from barmesh.tribarmes.triangleboxing import TriangleBoxing
from utils.pathutils import BallPathCloseRegions, MandrelPaths



def okaypressed():
    r = float(qr.text())
    yo = float(qy.text())
    towwidth = float(qtowwidth.text())
    towthick = float(qtowthick.text())
    mandrelpaths = [ freecadutils.findobjectbylabel(mandrelpathname)  for mandrelpathname in qmandrelpaths.text().split(",") ]
    pts=[]
    for a in np.linspace(0,2*np.pi,20):
    	pts.append(App.Vector(r*np.sin(a),yo,r*np.cos(a)))
    
    section = Draft.makeWire(pts, closed=False, face=False)
    section.Label = "ring_%d" % r
    ringsgroup.addObject(section)
    doc.recompute([ringsgroup])
    
    mandrelptpaths = [ ]
    for mandrelpath in mandrelpaths:
        mandrelwindpts = [ P3(p.X, p.Y, p.Z)  for p in mandrelpath.Shape.Vertexes ]
        mandrelptpaths.append(mandrelwindpts)
    mandpaths = MandrelPaths(mandrelptpaths)
    xrg = mandpaths.xrg.Inflate(towwidth)
    yrg = mandpaths.yrg.Inflate(towwidth)
    boxwidth = max(towwidth, xrg.Leng()/30, yrg.Leng()/30)
    tbs = TriangleBoxing(None, xrg.lo, xrg.hi, yrg.lo, yrg.hi, boxwidth)
    print("Creating box set boxwidth=", boxwidth, mandpaths.Nm)
    mandpaths.addpathstotgbs(tbs)

    thickcount = [ ]
    maxthickcount = 0
    thickpoint = None
    for mp in pts:
        mmp = P3(mp.x, mp.y, mp.z)
        bpcr = BallPathCloseRegions(mmp, towwidth)
        mandpaths.nhitreg += 1
        for ix, iy in tbs.CloseBoxeGenerator(mp.x, mp.x, mp.y, mp.y, towwidth):
            tbox = tbs.boxes[ix][iy]
            for i in tbox.pointis:
                bpcr.DistPoint(mandpaths.getpt(i), i)
            for i in tbox.edgeis:
                if mandpaths.hitreg[i] != mandpaths.nhitreg:
                    bpcr.DistEdge(mandpaths.getpt(i), mandpaths.getpt(i+1), i)
                    mandpaths.hitreg[i] = mandpaths.nhitreg
        bpcr.mergeranges()
        thickcount.append(len(bpcr.ranges))
        if thickcount[-1] > maxthickcount:
        	maxthickcount = thickcount[-1]
        	thickpoint = mp
    print('THICKNESSES AROUND RING:')
    print('MAX:',np.max(thickcount)*towthick,'MIN:',np.min(thickcount)*towthick,'MEAN:',np.mean(thickcount)*towthick)
    print('Raw number of layers around ring')
    print(thickcount)
    
doc = App.ActiveDocument    
ringsgroup = freecadutils.getemptyfolder(doc, "Rings")


w = freecadutils.findobjectbylabel(freecadutils.getlabelofselectedwire())
Ymax = 0
XZmin = np.Inf
furthestY = None
passY = None
for v in w.Shape.Vertexes:
	if v.Y > Ymax:
		Ymax = v.Y
		furthestY = v
	XZ = np.sqrt(v.X**2 + v.Z**2)
	if XZ < XZmin:
		XZmin = XZ
		passY = v
		
print('Furthest Y point at: ', furthestY.X,',', furthestY.Y,',',furthestY.Z)
print('Closest approach to Y axis of:',round(XZmin,2), 'at:', passY.X,',', passY.Y,',',passY.Z)

qw = QtGui.QWidget()
qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
qw.setGeometry(700, 500, 300, 350)
qw.setWindowTitle('Evaluate thickness')

qmandrelpaths = freecadutils.qrow(qw, "Winding paths ", 15+35*0, "")
qy = freecadutils.qrow(qw, "y coord of ring:", 15+35*1)
qr = freecadutils.qrow(qw, "Radius of ring:", 15+35*2 )
qtowwidth = freecadutils.qrow(qw, "Tow width: ", 15+35*3, "6.35")
qtowthick = freecadutils.qrow(qw, "Tow thickness: ", 15+35*4, "0.18")
okButton = QtGui.QPushButton("Evaluate", qw)
okButton.move(180, 15+35*7)
qmandrelpaths.setText(freecadutils.getlabelofselectedwire(multiples=True))
qy.setText("%.3f" % passY.Y)
qr.setText("%.3f" % XZmin)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)
qw.show()

