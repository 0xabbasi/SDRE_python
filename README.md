# SDRE_python

# SDRE Solver in Python

This repository contains a Python implementation of a solver for the State-Dependent Riccati Equation (SDRE), which is useful for solving optimal control problems for nonlinear systems.

## Overview

The State-Dependent Riccati Equation (SDRE) is a nonlinear matrix equation that arises in the context of optimal control theory. It provides a way to approximate the solution to the Hamilton-Jacobi-Bellman equation for nonlinear systems, which is generally intractable to solve directly.

The SDRE solver in this repository takes a nonlinear system represented in the form:

dx/dt = A(x, u) + B(x, u)u

where `x` is the state vector, `u` is the input vector, `A(x, u)` is the state matrix, and `B(x, u)` is the input matrix. The solver computes the positive semi-definite solution `P(x, u)` to the SDRE and the corresponding state feedback gain matrix `K(x, u)`.

## Usage

The main function is `sdre_solver(A, B, Q, R)`, which takes the following arguments:

- `A`: A callable function that returns the state matrix `A(x, u)` for a given state `x` and input `u`.
- `B`: A callable function that returns the input matrix `B(x, u)` for a given state `x` and input `u`.
- `Q`: Either a callable function `Q(x, u)` that returns the state weight matrix, or a constant `np.ndarray`.
- `R`: Either a callable function `R(x, u)` that returns the input weight matrix, or a constant `np.ndarray`.

The function returns two callables:

- `P`: A function `P(x, u)` that returns the positive semi-definite solution to the SDRE for a given state `x` and input `u`.
- `K`: A function `K(x, u)` that returns the state feedback gain matrix for a given state `x` and input `u`.

Here's an example of how to use the `sdre_solver`:

```python
import numpy as np
from sdre_solver import sdre_solver

def A(x, u):
    return np.array([[0, 1], [1, -1]])

def B(x, u):
    return np.array([[0], [1]])

Q = np.eye(2)
R = 1

P, K = sdre_solver(A, B, Q, R)

# Example state and input
x = np.array([1, 2])
u = 0

# Compute the solution and gain
P_x = P(x, u)
K_x = K(x, u)

print("Solution P:")
print(P_x)
print("Gain K:")
print(K_x)
