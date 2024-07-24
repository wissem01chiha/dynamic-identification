from abc import ABC, abstractmethod

class IDIM:
    """ base class for Inverse Dynamics identification models """

    @abstractmethod
    def set_upper_bounds(self):
        """ set the upper bounds for all x paramter values """
        pass
    
    @abstractmethod
    def set_lower_bound(self):
        pass