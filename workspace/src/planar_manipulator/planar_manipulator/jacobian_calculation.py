#! /usr/bin/env python3

"""
Calculate forward kinematics of planar manipulator and its Jacobian.
Then find singularities.
"""

import sympy as sp

def main():
    sp.init_printing()

    # Define symbols
    L, q0, q1 = sp.symbols('L q0 q1')

    # Forward kinematics
    x = L * (sp.cos(q0) + sp.cos(q0 + q1))
    y = L * (sp.sin(q0) + sp.sin(q0 + q1))

    # Jacobian matrix
    J = sp.Matrix([[sp.diff(x, q0), sp.diff(x, q1)],
                    [sp.diff(y, q0), sp.diff(y, q1)]])

    # Display the Jacobian
    print("Jacobian matrix:")
    sp.pprint(J)

    # Singularities: when determinant of Jacobian is zero
    det_J = J.det() / L**2  # Normalize by L^2 for simplicity
    det_J = sp.simplify(det_J)
    print("\nDeterminant of Jacobian:")
    sp.pprint(det_J)  # = sin(q1)

    # Singularities at q1 = N * pi, where N is an integer
    # which is what we expected!


if __name__ == "__main__":
    main()
