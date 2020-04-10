#!/usr/bin/env python
from sympy import *

num_of_transforms = 7
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

# Create Modified DH parameters
DH = {'alpha': [0, -pi / 2, 0, -pi / 2, pi / 2, -pi / 2, 0],
      'a': [0, 0.35, 1.25, -0.054, 0, 0, 0],
      'd': [0.75, 0, 0, 1.5, 0, 0, 0],
      'q': [q1, q2 - pi / 2, q3, q4, q5, q6, q7]}

transforms = []
rotations = []
# Define Modified DH Transformation matrix
for i in range(num_of_transforms):
    alpha = DH['alpha'][i]
    a = DH['a'][i]
    d = DH['d'][i]
    q = DH['q'][i]

#    rotations.append(Matrix([[cos(q),             -sin(q),                         0]
#                            [sin(q)*cos(alpha),  cos(q)*cos(alpha),    -sin(alpha)]
#                            [sin(q)*sin(alpha),  cos(q)*sin(alpha,      cos(alpha)]])

    
    transforms.append(Matrix([[cos(q), -sin(q), 0, a],
                              [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                              [sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha), cos(alpha) * d],
                              [0, 0, 0, 1]]))
    print(transforms[i])
print(len(transforms))
'''
for i in range(1,num_of_transforms):
    transforms[i]=transforms[i-1]*transforms[i]
print(simplify(transforms[-1]))
'''
