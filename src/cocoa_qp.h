//
// Created by Brian Jackson on 8/1/22.
// Copyright (c) 2022 Robotic Exploration Lab. All rights reserved.
//

#pragma once

#include "cocoa.h"

/**
 * @brief Create a solver for the following quadratic program (QP):
 *
 * \f{align}[
 *  \underset{\text{minimize}}{x} &&& \frac{1}{2} x^T P x + q^T x \\
 *  \text{subject to} &&& A x + b = 0 \\
 *                    &&& C x + d \leq 0
 * \f}
 *
 * @param n number of primal variables
 * @param m number of equality constraints
 * @param p number of inequality constraints
 * @param P quadratic cost. Can be either dense or diagonal, controlled by `is_quadratic_cost`.
 * @param q linear cost. Size `(n,)`.
 * @param A Linear equality constraint term. Size `(m,n)`.
 * @param b Constant equality constraint term. Size `(m,)`.
 * @param C Linear inequality constraint term. Size `(p,n)`.
 * @param d Constant inequality constraint term. Size `(p,)`.
 * @param is_quadratic_cost Sets whether the cost is quadratic or dense.
 *                          If true:  `P` should be the vector of diagonals, of size `(n,)`.
 *                          If false: `P` should be a dense matrix of size `(n,n)`.
 * @param err Error code. Can be NULL.
 * @return Pointer to initialized solver, or NULL if `err` is nonzero.
 */
cocoa_Solver *cocoa_NewDenseQPSolver(int n, int m, int p,
                                     const cocoa_float *P, const cocoa_float *q,
                                     const cocoa_float *A, const cocoa_float *b,
                                     const cocoa_float *C, const cocoa_float *d,
                                     bool is_quadratic_cost,
                                     cocoa_ERRORCODE *err);

cocoa_ERRORCODE cocoa_SetPrimals(const cocoa_Solver *solver, const cocoa_float *x);
cocoa_ERRORCODE cocoa_SetDualsEquality(const cocoa_Solver *solver, const cocoa_float *lambda);
cocoa_ERRORCODE cocoa_SetDualsInequality(const cocoa_Solver *solver, const cocoa_float *mu);

cocoa_ERRORCODE cocoa_GetPrimals(const cocoa_Solver *solver, cocoa_float *x);
cocoa_ERRORCODE cocoa_GetDualsEquality(const cocoa_Solver *solver, cocoa_float *lambda);
cocoa_ERRORCODE cocoa_GetDualsInequality(const cocoa_Solver *solver, cocoa_float *mu);

/**
 * @brief Create a solver for the following conically-constraint quadratic program (CCQP):
 *
 * \f{align}[
 *  \underset{\text{minimize}}{x} &&& \frac{1}{2} x^T P x + q^T x \\
 *  \text{subject to} &&& A_i x + b_i \in K_i \\
 * \f}
 *
 * @param n number of primal variables
 * @param m size of each conic constraint. Size `(M,)`.
 * @param M total number of conic constraints
 * @param P quadratic cost. Can be either dense or diagonal, controlled by `is_quadratic_cost`.
 * @param p linear cost
 * @param is_quadratic_cost Sets whether the cost is quadratic or dense.
 *                          If true:  `P` should be the vector of diagonals, of size `(n,)`.
 *                          If false: `P` should be a dense matrix of size `(n,n)`.
 * @param err error code. Can be NULL.
 * @return Pointer to initialized solver, or NULL if `err` is nonzero.
 */
cocoa_Solver *cocoa_NewDenseConicSolver(int n, const int *m, int M,
                                        const cocoa_float *P, const cocoa_float *p,
                                        bool is_quadratic_cost,
                                        cocoa_ERRORCODE *err);
// QUESTION: should we rename the constructors to `cocoa_NewCCQPSolver`, `cocoa_NewQPSolver`, and `cocoa_NewCOCPSolver`?

/**
 * @brief Set the data for the `i`th conic constraint of a CCQP
 * @param solver an initialized CCQP solver, created using `cocoa_NewDenseConicSolver`
 * @param A Linear term for the `i`th conic constraint
 * @param b Constant term for the `i`th conic constraint
 * @param cone The cone for the constraint
 * @param i Constraint index. Valid range: `[0,M)`.
 * @return error code
 */
cocoa_ERRORCODE cocoa_SetConicConstraint(cocoa_Solver *solver,
                                         const cocoa_float *A, const cocoa_float *b,
                                         cocoa_Cones cone,
                                         int i);

cocoa_ERRORCODE cocoa_SetDualsConic(const cocoa_Solver *solver, const cocoa_float *lambda, int i);
cocoa_ERRORCODE cocoa_GetDualsConic(const cocoa_Solver *solver, cocoa_float *lambda, int i);
