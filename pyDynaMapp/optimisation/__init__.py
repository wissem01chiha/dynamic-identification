import sys 
from .pso import *
from .cupso import *
from .cmaes import *
from .dea import *
__all__ = [PSO, DEA]
if 'cuda' in sys.argv:
   __all__.append('cupso')
   from cupso import *
else:
    cuPSO = None
