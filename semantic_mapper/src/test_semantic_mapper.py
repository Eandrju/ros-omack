import numpy as np
from semantic_mapper import SemanticMap
import sys


print(sys.version)
sem = SemanticMap(3, 10)
sem.update_map(
    np.array([[1.7, 2.7, 0], [2.5, 0.9, 2], [1.2, 2.4, 5]]),
    1
)
print(sem._array)

assert sem._array[5, 8] != 0

print(sem.generate_point_cloud())
