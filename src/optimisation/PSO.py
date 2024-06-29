import random
import numpy as np
from numba import cuda
import warnings
from numba.cuda.dispatcher import NumbaPerformanceWarning

warnings.filterwarnings("ignore", category=NumbaPerformanceWarning)

W = 0.85
c1 = 0.72
c2 = 0.91

print(cuda.is_available())
n_iterations = int(input("Inform the number of iterations: "))
target_error = float(input("Inform the target error: "))
n_particles = int(input("Inform the number of particles: "))

@cuda.jit
def move_particles_kernel(positions, velocities, pbest_positions, gbest_position, W, c1, c2):
    idx = cuda.grid(1)
    if idx < positions.shape[0]:
        r1 = -0.2
        r2 = 1.1
        for d in range(positions.shape[1]):
            velocities[idx, d] = W * velocities[idx, d] + \
                                 c1 * r1 * (pbest_positions[idx, d] - positions[idx, d]) + \
                                 c2 * r2 * (gbest_position[d] - positions[idx, d])
            positions[idx, d] += velocities[idx, d]

class Particle():
    def __init__(self,dim=2):
        self.position = cuda.to_device(np.array([
            (-1) ** (bool(random.getrandbits(1))) * random.random() * 1,
            (-1) ** (bool(random.getrandbits(1))) * random.random() * 1
            ], dtype=np.float64))
        
        self.pbest_position = cuda.device_array(shape=(2,), dtype=np.float64)
        self.pbest_position.copy_to_device(self.position)   
        
        self.pbest_value = float('inf')
        
        self.velocity = cuda.device_array(shape=(2,), dtype=np.float64)
        self.velocity[0] = 0
        self.velocity[1] = 0

    def move(self):
        self.position += self.velocity

    def __str__(self):
        return f"I am at {\
            self.position.copy_to_host()}, my pbest is {self.pbest_position.copy_to_host()}"

class Space():
    def __init__(self, target, target_error, n_particles):
        self.target = target
        self.target_error = target_error
        self.n_particles = n_particles
        self.particles = []
        self.gbest_value = float('inf')
        self.gbest_position = cuda.to_device(np.array([random.random() * 1,\
            random.random() * 1], dtype=np.float64))
        self.best_fitness_value = float('inf')

    def print_particles(self):
        for particle in self.particles:
            print(particle)

    def fitness(self, particle):
        return particle.position[0] ** 2 + particle.position[1] ** 3 \
            + np.cos(particle.position[1]) - np.sin(particle.position[0])**4 + 1

    def set_pbest(self):
        for particle in self.particles:
            fitness_candidate = self.fitness(particle)
            if particle.pbest_value > fitness_candidate:
                particle.pbest_value = fitness_candidate
                particle.pbest_position= particle.position

    def set_gbest(self):
        for particle in self.particles:
            best_fitness_candidate = self.fitness(particle)
            if self.gbest_value > best_fitness_candidate:
                self.gbest_value = best_fitness_candidate
                self.gbest_position = particle.position
                self.best_fitness_value = best_fitness_candidate

    def move_particles(self):
        d_positions = cuda.to_device(np.array([particle.position for particle in self.particles], dtype=np.float64))
        d_velocities = cuda.to_device(np.array([particle.velocity for particle in self.particles], dtype=np.float64))
        d_pbest_positions = cuda.to_device(np.array([particle.pbest_position for particle in self.particles], dtype=np.float64))
        
        threads_per_block = 32
        blocks_per_grid = (self.n_particles + threads_per_block - 1) // threads_per_block
        
        move_particles_kernel[blocks_per_grid, threads_per_block](
            d_positions, d_velocities, d_pbest_positions, self.gbest_position, W, c1, c2)
        
        for i, particle in enumerate(self.particles):
            particle.position = d_positions[i]
            particle.velocity = d_velocities[i]



