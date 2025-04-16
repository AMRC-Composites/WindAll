from .trianglebarmesh import TriangleBarMesh, TriangleBar
from .triangleboxing import MakeTriangleBoxing
from .trianglezcut import TriZCut


bnumpyexists = True
try:  import numpy
except ImportError:  bnumpyexists = False

if bnumpyexists:
    from .ntrianglebarmesh import NTriangleBarMesh
else:
    NTriangleBarMesh = None
    
    
