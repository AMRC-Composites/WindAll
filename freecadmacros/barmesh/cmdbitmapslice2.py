import sys
sys.path.append("/home/goatchurch/geom3d/barmesh")
from basicgeo import P2, P3, Partition1, Along
from pngslicegen import PNGslice
from tribarmes import TriangleBarMesh, MakeTriangleBoxing, TriZCut
import time

fname = "/home/goatchurch/geom3d/barmesh/stlsamples/V29D_Fixed.stl"
bitmapx = 2500
bitmapy = 900
tstart = time.time()

def spinztox(t):
    return (t[2], t[1], -t[0])
tbarmesh = TriangleBarMesh(fname, spinztox)
#sendactivity("clearalltriangles")
#sendactivity(codetriangles=tbarmesh.GetBarMeshTriangles())
trizcut = TriZCut(tbarmesh)  # this is slow because it builds the boxing

ypixes = Partition1(tbarmesh.ylo-0.2, tbarmesh.yhi+0.2, bitmapy-1)
zpix0 = tbarmesh.zlo - 0.1
zfactopix = ypixes.nparts/(ypixes.vs[-1] - ypixes.vs[0])
#zfactopix *= 0.5

# this is in Z (note rotation by spinztox)
# (here we can make a loop for each slice)
for ix in range(0, 51):
    lx = ix/50.0
    xslice = Along(lx, tbarmesh.xlo, tbarmesh.xhi)  

    fname = "/home/goatchurch/geom3d/bitmapslices/whistle/slice%05d.png" % int(xslice*1000)
    pngslice = PNGslice(fname, bitmapx, bitmapy)
    for iy, y in enumerate(ypixes.vs):
        ztcuts = trizcut.TriSurfCut(xslice, y)
        ztcuts.sort()
        pngslice.AddRow([max(0,min(bitmapx, int((zt - zpix0)*zfactopix)))  for zt in ztcuts])
    pngslice.done()
    print(time.time() - tstart, fname)


    

