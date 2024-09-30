import sys 
from .PSO import *
from .cuPSO import *
from .CMAES import *
from .DEA import *
__all__ = [PSO, DEA]
if 'cuda' in sys.argv:
   __all__.append('cupso')
   from cuPSO import *
else:
    cuPSO = None
