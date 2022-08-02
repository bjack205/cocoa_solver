//
// Created by Brian Jackson on 8/1/22.
// Copyright (c) 2022 Robotic Exploration Lab. All rights reserved.
//

#pragma once
#include "cocoa.h"

/**
 * @brief Create a new solver for the following linear-time-invariant MPC problem:
 *
 * \f{align}[
 *  \underset{\text{minimize}}{x} &&&
 *      \sum_{k=2}^{N} \frac{1}{2} x_k^T Q x_k + q^T x_k +
 *                     \frac{1}{2} u_{k-1}^T R u_{k-1} + r^T u_{k-1} \\
 *  \text{subject to} &&& A x + B u + f = 0 \\
 *                    &&& G^{(i)} x_k + H^{(i)} u_k + h^{(i)} \in K^{(i)} \leq 0
 * \f}
 *
 * @param num_states number of states
 * @param num_inputs number of inputs
 * @param p size of each conic constraint. Size `(num_constraints,)`.
 * @param num_constraints Total number of conic constraints
 * @param num_horizon horizon length
 * @param Q Quadratic state cost. Size is determined by `is_cost_diagonal`.
 * @param q Linear state cost. Size `(num_states,)`.
 * @param R Quadratic input cost. Size depends on `is_cost_diagonal`.
 * @param r Linear input cost. Size `(num_inputs,)`.
 * @param A State transition matrix. Size `(num_states, num_states)`.
 * @param B Input matrix. Size `(num_states, num_inputs)`.
 * @param f Affine dynamics term. Size `(num_states,).
 * @param is_cost_diagonal Specifies whether Q and R are diagonal.
 * @param err
 * @return
 */
cocoa_Solver *cocoa_NewLTISolver(int num_states, int num_inputs, const int *p,
                                 int num_constraints, int num_horizon,
                                 const cocoa_float *Q, const cocoa_float *q,
                                 const cocoa_float *R, const cocoa_float *r,
                                 const cocoa_float *A, const cocoa_float *B,
                                 const cocoa_float *f,
                                 bool is_cost_diagonal, cocoa_ERRORCODE *err);

cocoa_ERRORCODE cocoa_SetTrackingCost(cocoa_Solver *solver, const cocoa_float *Q,
                                      const cocoa_float *R, const cocoa_float *xref,
                                      const cocoa_float *uref, int k);

/**
 * @brief Shifts the problem by one time step, dropping the data at the first time step and
 * zero-initializing the last time step.
 * @param solver An initialized problem (usually after a solve)
 * @return error code
 */
cocoa_ERRORCODE cocoa_ShiftProblem(cocoa_Solver *solver);

/**
 * @brief Shifts the problem by one time step, dropping the data at the first time step and
 * setting the last time step equal to the data of the previous time step.
 *
 *
 * @param solver An initialized problem (usually after a solve)
 * @return error code
 */
cocoa_ERRORCODE cocoa_ShiftProblemWithCopy(cocoa_Solver *solver);
