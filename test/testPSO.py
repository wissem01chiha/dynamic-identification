import unittest 
import numpy as np 
import sys 
import os 
src_folder = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.append(src_folder)

from optimisation import PSO


# --------------------------------------------------------------
search_space = Space(0, target_error, n_particles)
particles_vector = [Particle() for _ in range(search_space.n_particles)]
search_space.particles = particles_vector

iteration = 0
while iteration < n_iterations:
    search_space.set_pbest()
    search_space.set_gbest()
    
    if abs(search_space.gbest_value - search_space.target) <= search_space.target_error:
        break
    
    print("best fitness value : ", search_space.best_fitness_value)
    search_space.move_particles()
    iteration += 1

search_space.print_particles()
print("The best solution is: ", search_space.gbest_position.copy_to_host(), " in n_iterations: ", iteration)