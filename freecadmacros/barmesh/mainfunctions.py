from .basicgeo import P3, AlongAcc
from . import barmesh

def BarMeshContourF(bm, bar, node, cizone, contournumber):
    barcont = [ ]
    cbar, cnode = bar, node
    bfullloop = False
    while True:
        assert not (cnode.pointzone.izone == cizone)
        assert cbar.GetNodeFore(cbar.nodeback == cnode).pointzone.izone == cizone
        assert cbar.midcontournumber < bm.startcontournumber
        cbar.midcontournumber = contournumber
        barcont.append(cbar)
        if cbar.GetForeRightBL(cbar.nodeback == cnode) is None:
            break            # tacking the contour terminated early (not bfullloop)
        Dcount = 0
        while True:
            cnode = cbar.GetNodeFore(cbar.nodeback == cnode)
            if not (cnode.pointzone.izone == cizone):
                break
            cbar = cbar.GetForeRightBL(cbar.nodefore == cnode)
            Dcount += 1
            assert Dcount < 1000, "polygon has too many sides"
        if cbar == bar:
            assert cnode == node
            bfullloop = True  # we scanned round to the start of the contour
            break
    
    # full contour loop
    if bfullloop:
        barcont.append(barcont[0])
        return barcont

    # extend the contour backwards to get the whole not closed piece (awkward)
    cbar, cnode = bar, node
    lbarcont = [ ]
    while True:
        if cbar.GetForeLeftBR(cbar.nodeback == cnode) is None:
            break
        Dcount = 0
        while True:
            cnode = cbar.GetNodeFore(cbar.nodeback == cnode)
            if not (cnode.pointzone.izone == cizone):
                break
            cbar = cbar.GetForeLeftBR(cbar.nodefore == cnode)
            Dcount += 1
            assert Dcount < 1000
        assert cbar != bar
        assert not (cnode.pointzone.izone == cizone)
        assert cbar.GetNodeFore(cbar.nodeback == cnode).pointzone.izone == cizone
        assert cbar.midcontournumber < bm.startcontournumber
        cbar.midcontournumber = contournumber
        lbarcont.append(cbar)
    lbarcont.reverse()
    return lbarcont + barcont
        

def BarMeshContoursF(bm, cizone):
    bm.startcontournumber = bm.endcontournumber
    conts, topbars = [ ], [ ]
    
    for bar in bm.bars:
        if bar.bbardeleted:
            continue
        if (bar.nodeback.pointzone.izone == cizone) == (bar.nodefore.pointzone.izone == cizone):
            continue
        if bar.midcontournumber >= bm.startcontournumber:
            continue
        node = bar.GetNodeFore(bar.nodeback.pointzone.izone == cizone)
        barcont = BarMeshContourF(bm, bar, node, cizone, bm.endcontournumber)
        bm.endcontournumber += 1
        conts.append([bar.nodemid.p  for bar in barcont])
        topbar = max(barcont, key=lambda X: max((X.nodeback.p.x, X.nodeback.p.y), (X.nodefore.p.x, X.nodefore.p.y)))
        topbars.append(topbar)
    return conts, topbars


def BarMeshContoursN(bm, cizone):
    bm.startcontournumber = bm.endcontournumber
    contsN, topbars = [ ], [ ]
    
    for bar in bm.bars:
        if bar.bbardeleted:
            continue
        if (bar.nodeback.pointzone.izone == cizone) == (bar.nodefore.pointzone.izone == cizone):
            continue
        if bar.midcontournumber >= bm.startcontournumber:
            continue
        node = bar.GetNodeFore(bar.nodeback.pointzone.izone == cizone)
        barcont = BarMeshContourF(bm, bar, node, cizone, bm.endcontournumber)
        bm.endcontournumber += 1
        contsN.append([bar.nodemid  for bar in barcont])
        topbar = max(barcont, key=lambda X: max(X.nodeback.pairusedfornesting(), X.nodefore.pairusedfornesting()))
        topbars.append(topbar)
    return contsN, topbars


    
def NodeStar(node, bar):   # used by NestContours
    barcycle = [ bar ]
    cbar = bar
    Dcount = 0
    while True:
        cbar = cbar.GetForeRightBL(cbar.nodefore == node)
        if cbar is None:
            break
        assert cbar.nodefore == node or cbar.nodeback == node
        if cbar == bar:
            return barcycle
        barcycle.append(cbar)
        Dcount += 1
        assert Dcount < 1000
        
    # go back case
    lbarcycle = [ ]
    cbar = bar
    while True:
        cbar = cbar.GetForeLeftBR(cbar.nodefore == node)
        if cbar is None:
            break
        assert cbar.nodefore == node or cbar.nodeback == node
        assert cbar != bar
        Dcount += 1
        assert Dcount < 1000
        lbarcycle.append(cbar)
    lbarcycle.reverse()
    return lbarcycle + barcycle

def FindOuterContourBar(topbar, topnode, cizone):  # used by NestContours
    Dbotnode = topbar.GetNodeFore(topbar.nodeback == topnode)
    assert topnode.pairusedfornesting() > Dbotnode.pairusedfornesting()
    while True:
        ntopnode, ntopbar = topnode, None
        for sbar in NodeStar(topnode, topbar):
            snode = sbar.GetNodeFore(sbar.nodeback == topnode)
            if snode.pairusedfornesting() > ntopnode.pairusedfornesting():
                ntopnode, ntopbar = snode, sbar
        if ntopbar is None:
            return None
        if (ntopbar.nodeback.pointzone.izone == cizone) != (ntopbar.nodefore.pointzone.izone == cizone):
            return ntopbar
        topnode, topbar = ntopnode, ntopbar

# this depends on sorting all contours and their representative points to the right and working backwards 
# while tracking through the maze of bars to the right, so that if an inner contour tracks outwards to 
# another inner contour, it can inherit its outer contour which will be far enough to the right not to hit another inner contour



def NestContours(topbars, cizone):
    topbarnodes = [ (topbar, max(topbar.nodeback, topbar.nodefore, key=lambda X: X.pairusedfornesting()))  for topbar in topbars ]
    topbarnodes.sort(key=lambda X: X[1].pairusedfornesting(), reverse=True)
    contnest = { }
    for topbar, topnode in topbarnodes:
        nextouterbar = FindOuterContourBar(topbar, topnode, cizone)
        assert topbar.midcontournumber not in contnest
        outxn = nextouterbar.midcontournumber if nextouterbar else -1
        while outxn != -1 and contnest[outxn][0] == topnode.pointzone.izone:
            outxn = contnest[outxn][1]
        contnest[topbar.midcontournumber] = (topnode.pointzone.izone, outxn, [ ])

    for cn, (izone, outxn, innlist) in contnest.items():
        if outxn != -1:
            contnest[outxn][2].append(cn)
            
    return contnest  # { cn: (izone == cizone for outer, outercontournumber, inner contour number) }

# to use NestContours    
# mconts = dict((topbar.midcontournumber, cont)  for cont, topbar in zip(conts, topbars))
# outerconts = [mconts[cn]  for cn, (izone, outxn, innlist) in contnest.items()  if izone == barmesh.PZ_BEYOND_R and outxn == -1]
# innerconts = [mconts[cn]  for cn, (izone, outxn, innlist) in contnest.items()  if not (izone == barmesh.PZ_BEYOND_R and outxn == -1)]
    

def PlotBarmesh(bm, sendactivity):
    sendactivity("clearallcontours")
    sendactivity("clearallpoints")
    sendactivity("contours", contours=[[bar.nodeback.p, bar.nodefore.p]  for bar in bm.bars  if  not bar.bbardeleted])
    sendactivity("points", points=[node.p  for node in bm.nodes  if not (node.pointzone.izone == barmesh.PZ_BEYOND_R)], materialnumber=1)
    sendactivity("contours", contours=BarMeshContoursF(bm, barmesh.PZ_BEYOND_R)[0], materialnumber=1)

def nodewithinpairs(bm, bwithin):
    nodepairs = [ ]
    for bar in bm.bars:
        if not bar.bbardeleted:
            if bar.nodeback.pointzone.izone == barmesh.PZ_WITHIN_R and bar.nodefore.pointzone.izone == barmesh.PZ_WITHIN_R:
                if bwithin:
                    nodepairs.append((bar.nodeback, bar.nodefore))
            elif bar.nodeback.pointzone.izone == barmesh.PZ_BEYOND_R and bar.nodefore.pointzone.izone == barmesh.PZ_BEYOND_R:
                if not bwithin:
                    nodepairs.append((bar.nodeback, bar.nodefore))
            elif bar.nodeback.pointzone.izone == barmesh.PZ_WITHIN_R and bar.nodefore.pointzone.izone == barmesh.PZ_BEYOND_R:
                if bwithin:
                    nodepairs.append((bar.nodeback, bar.nodemid))
                else:
                    nodepairs.append((bar.nodemid, bar.nodefore))
            elif bar.nodeback.pointzone.izone == barmesh.PZ_BEYOND_R and bar.nodefore.pointzone.izone == barmesh.PZ_WITHIN_R:
                if bwithin:
                    nodepairs.append((bar.nodemid, bar.nodefore))
                else:
                    nodepairs.append((bar.nodeback, bar.nodemid))
    return nodepairs

