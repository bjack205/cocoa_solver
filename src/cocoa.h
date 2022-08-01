//
// Created by Brian Jackson on 8/1/22.
// Copyright (c) 2022 Robotic Exploration Lab. All rights reserved.
//

#pragma once
#include <stdbool.h>

#include "cocoa_types.h"

// TODO: move this into a "constants" header
typedef enum {
  cocoa_OK = 0,
  cocoa_MEMORYERROR = 1,
} cocoa_ERRORCODE;

// Forward-declare the solver
struct cocoa_Solver;
typedef struct cocoa_Solver cocoa_Solver;

/////////////////////////////////////////////
// Construction and Initialization
/////////////////////////////////////////////
/**
 * @brief Create a new cocoa solver instance for solving a COCP:
 *
 * \f{align}[
 *  \underset{\text{minimize}}{x} &&&
 *      \sum_{k=2}^{N} \frac{1}{2} x_k^T Q_k x_k + q_k^T x_k +
 *                     \frac{1}{2} u_{k-1}^T R u_{k-1} + r^T u_{k-1} \\
 *  \text{subject to} &&& A_k x_k + B_k u_k + C_k x_{k+1} + D_k u_{k+1} + f_k = 0 \\
 *                    &&& x_1 = x_\text{init} \\
 *                    &&& G_k^{(i)} x_k + H_k^{(i)} u_k + h_k^{(i)} \in K_k^{(i)} \leq 0
 * \f}
 *
 *
 * Allocates new memory given the input sizes.
 *
 * @param num_states number of states at each time step. Size `(num_horizon,)`.
 * @param num_inputs number of inputs at each time step. Size `(num_horizon-1,)`.
 * @param num_constraints total number of constraints at each time step. Size (`num_horizon,)`.
 * @param num_horizon horizon length (number of knot points, usually odd)
 *                    Valid inputs: [1,INT_MAX)
 * @param use_diagonal_costs Determines whether the quadratic cost matrices are diagonal or dense.
 *                           If true, the interface will assume all the parameters specifying
 *                           the quadratic cost terms `Q` and `R` are vectors of the diagonals,
 *                           instead of dense matrices.
 * @param is_block_diagonal Determines if the cost function for the solver is block diagonal, i.e. that the quadratic
 *                          cross term `Hux` is zero.
 * @param use_explicit_integration If true, assumes explicit integration for the dynamics.
 * @param err error code return. Can be passed a NULL pointer.
 * @return a pointer to a default-initialized solver instance, or NULL otherwise.
 */
cocoa_Solver *cocoa_NewSolver(const int *num_states,
                              const int *num_inputs,
                              const int *num_constraints,
                              int num_horizon,
                              bool use_diagonal_costs,
                              bool is_block_diagonal,
                              bool use_explicit_integration,
                              cocoa_ERRORCODE *err);

cocoa_ERRORCODE cocoa_GetHorizonLength(const cocoa_Solver *solver, int *num_horizon);
cocoa_ERRORCODE cocoa_UsesDiagonalCosts(const cocoa_Solver *solver, bool *use_diagonal_costs);
cocoa_ERRORCODE cocoa_IsBlockDiagonal(const cocoa_Solver *solver, bool *is_block_diagonal);
cocoa_ERRORCODE cocoa_UsesExplicitIntegration(const cocoa_Solver *solver, bool *use_explicit_integration);


/////////////////////////////////////////////
// Cost API
/////////////////////////////////////////////
/**
 * @brief Set the quadratic and affine cost terms on the state for a single time step.
 *
 * @param solver An initialized solver
 * @param Q Quadratic state cost. Size is determined by whether the costs are dense or diagonal.
 * @param q Linear state cost. Size `(num_states,)`.
 * @param k Time step at which to set the cost.
 *          Valid inputs = [-1,num_horizon].
 *          If k = -1, it will copy to all time steps.
 * @return error code
 */
cocoa_ERRORCODE cocoa_SetStateCost(cocoa_Solver *solver,
                                   const cocoa_float *Q,
                                   const cocoa_float *q,
                                   int k);

/**
 * @brief Set the quadratic and affine cost terms on the input for a single time step.
 *
 * @param solver An initialized solver
 * @param R Quadratic input cost. Size is determined by whether the costs are dense or diagonal.
 * @param r Linear input cost. Size `(num_inputs,)`.
 * @param k Time step at which to set the cost.
 *          Valid inputs = [-1,num_horizon).
 *          If k = -1, it will copy to all time steps.
 * @return error code
 */
cocoa_ERRORCODE cocoa_SetInputCost(cocoa_Solver *solver,
                                   const cocoa_float *R,
                                   const cocoa_float *r,
                                   int k);

/**
 * @brief Set the cross-term cost between the states and inputs.
 *
 * @param solver An initialized solver
 * @param H Cross-term cost. Size `(num_inputs, num_states)`.
 * @param k Time step at which to set the cost.
 *          Valid inputs = [-1,num_horizon).
 *          If k = -1, it will copy to all time steps.
 * @return error code
 */
cocoa_ERRORCODE cocoa_SetCrossTermCost(cocoa_Solver *solver, const cocoa_float *Hux, int k);

/////////////////////////////////////////////
// Dynamics API
/////////////////////////////////////////////
/**
 * @brief Set the dynamics at a given time step
 *
 * Will return an error code if `cocoa_UsesExplicitIntegration` is true and `C` and `D` are not NULL.
 *
 * @param solver An initialized solver
 * @param A State transition matrix. Size `(num_states[k+1], num_states[k])`.
 * @param B Input matrix. Size `(num_states[k+1], num_inputs[k])`.
 * @param C Next state matrix. Size (`num_states[k+1], num_states[k+1])`. If NULL, assume identity.
 * @param D Next input matrix. Size (`num_states[k+1], num_inputs[k+1])`. If NULL, assume zero.
 * @param f Constant dynamics term. Size `(num_states[k+1],).
 * @param h Time step (seconds).
 * @param k Time step at which to set the dynamics.
 *          Valid inputs = [-1,num_horizon).
 *          If k = -1, it will copy to all time steps.
 * @return error code
 */
cocoa_ERRORCODE cocoa_SetDynamics(cocoa_Solver *solver,
                                  const cocoa_float *A,
                                  const cocoa_float *B,
                                  const cocoa_float *C,
                                  const cocoa_float *D,
                                  const cocoa_float *f,
                                  cocoa_float h,
                                  int k);

/**
 * @brief Set the initial state for the solver
 * @param solver
 * @param x0
 * @return
 */
cocoa_ERRORCODE cocoa_SetInitialState(cocoa_Solver *solver, const cocoa_float *x0);

cocoa_ERRORCODE cocoa_GetInitialState(cocoa_Solver *solver, cocoa_float *x0);

/////////////////////////////////////////////
// Constraints API
/////////////////////////////////////////////
// TODO: move this into a "constants" header
typedef enum {
  cocoa_ZEROCONE,
  cocoa_NEGATIVEORTHANT,
  cocoa_SECONDORDERCONE,
} cocoa_Cones;
#define cocoa_EQUALITY cocoa_ZEROCONE
#define cocoa_INEQUALITY cocoa_NEGATIVEORTHANT

/**
 * @brief Set the data for the `i`th conic constraint at time step `k`.
 *
 * @param solver An initialized solver
 * @param G The linear state term
 * @param H The linear input term
 * @param h The constant term
 * @param i The constraint index
 * @param k The time step index
 * @param cone The type of cone
 * @return error code
 */
cocoa_ERRORCODE cocoa_SetConstraint(cocoa_Solver *solver,
                                    const cocoa_float *G,
                                    const cocoa_float *H,
                                    const cocoa_float *h,
                                    int i,
                                    int k,
                                    cocoa_Cones cone);

/**
 * @brief Set the relative penalty for the `i`th constraint at time step `k`.
 *
 * These weights are 1 by default. This method can be used to change the relative weights
 * between constraints. The true initial weights are determined by these values and the
 * initial penalty specified in the solver settings.
 *
 * @param solver An initialized solver
 * @param rho The relative penalty weight
 * @param i The constraint index
 * @param k The time step index
 * @return error code
 */
cocoa_ERRORCODE cocoa_SetPenalty(cocoa_Solver *solver, cocoa_float rho, int i, int k);

/////////////////////////////////////////////
// Solve
/////////////////////////////////////////////
cocoa_ERRORCODE cocoa_SetState(cocoa_Solver *solver, const cocoa_float *x, int k);
cocoa_ERRORCODE cocoa_SetInput(cocoa_Solver *solver, const cocoa_float *u, int k);
cocoa_ERRORCODE cocoa_SetDual(cocoa_Solver *solver, const cocoa_float *u, int k, int i);

cocoa_ERRORCODE cocoa_GetState(const cocoa_Solver *solver, cocoa_float *x, int k);
cocoa_ERRORCODE cocoa_GetInput(const cocoa_Solver *solver, cocoa_float *u, int k);
cocoa_ERRORCODE cocoa_GetDual(const cocoa_Solver *solver, cocoa_float *u, int k, int i);

cocoa_ERRORCODE cocoa_Solve(cocoa_Solver *solver);

cocoa_ERRORCODE cocoa_Reset(cocoa_Solver *solver);
cocoa_ERRORCODE cocoa_ResetDuals(cocoa_Solver *solver);
cocoa_ERRORCODE cocoa_ResetPenalties(cocoa_Solver *solver);

// QUESTION: Should these return error codes instead?
typedef enum {
  cocoa_SOLVED = 0,
  cocoa_INFEASIBLE,
  cocoa_MAXITERS,
} cocoa_SOLVESTATUS;

cocoa_SOLVESTATUS cocoa_GetSolveStatus(const cocoa_Solver *solver);

cocoa_float cocoa_PrimalFeasibility(const cocoa_Solver *solver);
cocoa_float cocoa_DualFeasibility(const cocoa_Solver *solver);
cocoa_float cocoa_Stationarity(const cocoa_Solver *solver);
cocoa_float cocoa_Complimentarity(const cocoa_Solver *solver);

int cocoa_Iterations(const cocoa_Solver *solver);
int cocoa_SolveTime(const cocoa_Solver *solver);
int cocoa_PrintSummary(const cocoa_Solver *solver);

/////////////////////////////////////////////
// Options
/////////////////////////////////////////////
cocoa_ERRORCODE cocoa_SetOptionFloat(cocoa_Solver *solver, const char *option, cocoa_float value);
cocoa_ERRORCODE cocoa_SetOptionInt(cocoa_Solver *solver, const char *option, int value);

cocoa_ERRORCODE cocoa_GetOptionFloat(const cocoa_Solver *solver, const char *option, cocoa_float *value);
cocoa_ERRORCODE cocoa_GetOptionInt(const cocoa_Solver *solver, const char *option, int *value);

