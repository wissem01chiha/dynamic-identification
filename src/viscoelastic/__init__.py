import sys 

from .Dahl import *
from .cuLugre import *
from .Lugre import * 
from .Maxwell import *
from .MaxwellSlip import *
from .Backlash import *
from .Viscous import *

__all__ = ['Dahl', 'Lugre', 'Maxwell', 'MaxwellSlip', 'Backlash', 'Viscous']

if 'cuda' in sys.argv:
    __all__.append('cuLugre')
    import cuLugre
else:
    cuLugre = None
