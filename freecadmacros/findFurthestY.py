import freecadutils
import numpy as np
freecadutils.init(App)

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
