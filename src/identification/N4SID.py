import numpy as np 

class N4SID:
    """
     Numerical Subspace State Space System IDentification
     This method uses input-output data to construct a state-space model by minimizing 
     a prediction error criterion. It's robust and widely used for practical applications.

        

        - CVA (Canonical Variate Analysis): This technique maximizes the correlation between past and future data, leading to a state-space model that captures the system dynamics effectively.
    Van Overschee, Peter, and Bart De Moor. “N4SID: Subspace algorithms for the identification of combined deterministic-stochastic systems.” Automatica 30.1 (1994): 75-93
    """
    def __init__(self) -> None:
        pass