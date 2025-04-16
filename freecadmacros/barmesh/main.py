import re
from optparse import OptionParser
from basicgeo import P2, P3, Partition1
import barmesh, implicitareaballoffset, implicitareacyloffset
from tribarmes import TriangleBarMesh
import mainfunctions
from barmeshslicer import BarMeshSlicer
    
parser = OptionParser()
parser.add_option("-s", "--stl",        dest="stlfile",     help="Input STL file",      default="stlsamples/barobox.stl",    metavar="FILE")
parser.add_option("-o", "--output",     dest="outputfile",  help="Output slice file",   default=None, metavar="FILE")
parser.add_option("-t", "--transform",  dest="transform",   help="Transformation to apply to STL file", default="unit")
parser.add_option("-r", "--radius",     dest="rad",         help="Slice radius offset", default=2.5, type="float")
parser.add_option("-d", "--disk",       dest="diskheight",  help="Disk height (sphere if < 0)", default=-1, type="float")
parser.add_option("-v", "--verbose",    dest="verbose",     help="Verbose", action="store_true")
parser.add_option("-n", "--nslices",    dest="nslices",     help="Number of slices", type="int", default=12)
options, args = parser.parse_args()

transmaps = { "unit":lambda t: t, "swapyz": lambda t: (t[0], -t[2], t[1]) }
tbm = TriangleBarMesh(options.stlfile, transmaps[options.transform])

if options.diskheight >= 0.0:
    iaoffset = implicitareacyloffset.ImplicitAreaCylOffset(tbm)
else:
    iaoffset = implicitareaballoffset.ImplicitAreaBallOffset(tbm)

# substitute in the numpy version.  (with some accessor this could be used before construction of iaoffset)
from tribarmes import NTriangleBarMesh  # could be in prep for any OpenCL stuff
if NTriangleBarMesh:
    print("substituting NTriangleBarMesh!")
    iaoffset.tbarmesh = NTriangleBarMesh(iaoffset.tbarmesh)

rex = options.rad + 2.5
xpart = Partition1(tbm.xlo-rex, tbm.xhi+rex, 145)
ypart = Partition1(tbm.ylo-rex, tbm.yhi+rex, 137)
zlevels = [ ]
for i in range(0, options.nslices):
    lam = (i+0.5)/options.nslices
    zlevels.append((tbm.zlo-options.rad)*(1 - lam) + (tbm.zhi+options.rad)*lam)
    
fnameout = options.outputfile or (re.sub("\.stl$(?i)", "", options.stlfile) + ".tap")
ftapeout = open(fnameout, "w")
b2dcontournormals = False

sumtotalproctime = 0
for z in zlevels:
    if options.diskheight >= 0.0:
        iaoffset.SetCylZrg(z, z + options.diskheight)
    bm = barmesh.BarMesh()
    bm.BuildRectBarMesh(xpart, ypart, z)
    rd2 = max(xpart.vs[1]-xpart.vs[0], ypart.vs[1]-ypart.vs[0], options.rad*1.5) + 0.1
    bms = BarMeshSlicer(bm, iaoffset, rd=options.rad, rd2=rd2, contourdotdiff=0.95, contourdelta=0.05, lamendgap=0.001)
    bms.initializecutsanddistances()
    currentcolour = -1
    ncount = 0
    while bms.barpolycuts:
        currentcolour += 1
        if currentcolour > bm.maxcellcolour:
            currentcolour = 0
        bms.splitbarpolyscolour(currentcolour)
        ncount += 1
        if ncount == 80:
            if options.verbose:
                pass#print("breaking after", ncount, "iterations with", len(bms.barpolycuts), "barpolycuts remaining")
            break
        if options.verbose:
            print("ncount", ncount, currentcolour, len(bms.barpolycuts)) 

    if options.verbose:
        print("z:%.3f pztime:%.3f/%d cbtime:%.3f/%d remains: %3f" % (z, bms.pztime, bms.pzcalls, bms.cbtime+bms.cbztime, bms.cbcalls, (bms.totalproctime - bms.pztime - bms.cbtime)))
    sumtotalproctime += bms.totalproctime
    
    contours, contbars = mainfunctions.BarMeshContoursF(bm, barmesh.PZ_BEYOND_R)
    for cont in contours: 
        for i, p in enumerate(cont):
            if i == 0:
                ftapeout.write("G0X%.3fY%fZ%.3f\n" % (p.x, p.y, p.z))  # rapid move to new contour
            elif i == 1:
                ftapeout.write("G1X%.3fY%.3f\n" % (p.x, p.y))          # back to "cutting" move
            else:
                ftapeout.write("X%.3fY%.3f\n" % (p.x, p.y))            # no z required as contour is 2D

print("sumtotalproctime", sumtotalproctime)
# example function call (using pypy for its JIT capabilities)
# pypy -O main.py --stl=stlsamples/impellor1.stl -v -tswapyz -r0.2 -n52 -d0.18
# (slices take 6 to 8 seconds right now)


# code for reading and displaying the file into twistcodewiki
"""
import re
sendactivity("clearalltriangles")
sendactivity("clearallcontours")
conts = [ [(0,0,0), (50,0,0)], [(0,0,0), (0,30,0)], [(0,0,0), (0,0,10)], [(1,0,0), (1,0,10)] ]
sendactivity("contours", contours=conts)

fname = "/home/goatchurch/geom3d/barmesh/stlsamples/barobox.tap"
gv = { "G":0, "X":0.0, "Y":0.0, "Z":0.0 }
conts = [ [ ] ]
for line in open(fname):
    for g, v in re.findall("([XYZG])\s*([\d\.\-\+]+)", line):
        gv[g] = float(v)
    if gv["G"] == 0:
        if conts[-1]:
            conts.append([])
    conts[-1].append((gv["X"], gv["Y"], gv["Z"]))
sendactivity("contours", contours=conts)
"""    

