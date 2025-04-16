import sys
sys.path.append("/home/goatchurch/geom3d/barmesh")
from basicgeo import P2, P3, Partition1, Along
from PIL import Image, ImageDraw
from tribarmes import TriangleBarMesh, MakeTriangleBoxing, TriZCut

fname = "/home/goatchurch/geom3d/bitmapslices/circle-high-res-ascii-autocad.stl"
bitmapx = 1824
bitmapy = 1140

def spinztox(t):
    return (t[2], t[1], -t[0])
tbarmesh = TriangleBarMesh(fname, spinztox)
#sendactivity("clearalltriangles")
#sendactivity(codetriangles=tbarmesh.GetBarMeshTriangles())
trizcut = TriZCut(tbarmesh)  # this is slow because it builds the boxing

ypixes = Partition1(tbarmesh.ylo-0.2, tbarmesh.yhi+0.2, bitmapy-1)
zpix0 = tbarmesh.zlo - 0.1
zfactopix = ypixes.nparts/(ypixes.vs[-1] - ypixes.vs[0])

# this is in Z (note rotation by spinztox)
# (here we can make a loop for each slice)
xslice = Along(0.5, tbarmesh.xlo, tbarmesh.xhi)  

imgslice = Image.new("1", (bitmapx, bitmapy), 255)
imgslicedr = ImageDraw.Draw(imgslice)

for iy, y in enumerate(ypixes.vs):
    ztcuts = trizcut.TriSurfCut(xslice, y)
    ztcuts.sort()
    for i in range(1, len(ztcuts), 2):
        z0 = int((ztcuts[i-1] - zpix0)*zfactopix + 0.5)
        z1 = int((ztcuts[i] - zpix0)*zfactopix + 0.5)
        imgslicedr.line([z0,iy,z1,iy], 0)  # horizontal line
    #sendactivity(points=[(xslice, ypix, z)  for z in ztcuts])
    print(iy, len(ztcuts))
    
#imgslice.show()
imgslice.save(open("/home/goatchurch/slice%d.png" % int(xslice*1000), "wb"), "PNG")

