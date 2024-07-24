import sys 

from .dahl import *
from .lugre import * 
from .maxwell import *
from .maxwell_slip import *
from .backlash import *
from .viscous import *

__all__ = ['Dahl', 'Lugre', 'maxwell', 'MaxwellSlip', 'Backlash', 'viscous']

if 'cuda' in sys.argv:
    __all__.append('cuLugre')
    from .cuLugre import *
else:
    cuLugre = None
