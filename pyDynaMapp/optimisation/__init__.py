import sys 

from .PSO import *
from .cuPSO import *
from .CMAES import *
from .DEA import *

__all__ = [ ]
if 'cuda' in sys.argv:
   __all__.append('cuPSO')
   from cuPSO import *
else:
    cuPSO = None
