//
// Created by Brian Jackson on 8/1/22.
// Copyright (c) 2022 Robotic Exploration Lab. All rights reserved.
//

#pragma once

#include "cocoa.h"

/**
 * @brief Create a new cocoa solver instance, where the number of time steps used for internal
 *        storage may differ from the horizon length.
 *
 * WARNING: This is an advanced method, requiring careful use. Most users should use the `cocoa_NewSolver` method.
 *
 * This constructor allows the user to separately specify the number of time steps used for
 * memory storage and the total number of time steps in the problem. These might differ when:
 * 1. All time steps have the same parameters (same costs, dynamics, and constraints), allowing the solver to only store
 *    one "copy" of these parameters to be referenced by all time steps.
 * 2. The horizon length may change, but is bounded by some maximum number of time steps.
 * The user assigns each time step to a storage index via the `time_step_index_to_storage_index` array. Element `k` of
 * this array should contain an index between 0 and `num_data`, specifying which data should be used for the knot point.
 *
 * This method allows for memory-efficient implementations of a variety of MPC problems, such as hybrid methods where
 * a small number of different "modes" are used, or a regulator problem where the data for all time steps is identical..
 *
 * Note that it is up to the user to keep track of these index assignments. All methods that expect a time step index
 * (usually denoted by an input argument `k`) will use the mapping provided here to redirect to the assigned storage
 * location, so users can inadvertently overwrite data without warning, if used improperly.
 *
 * Use case 2 requires special attention. To ensure proper behavior, the user should instead use `cocoa_NewSolver`,
 * which assumes `num_data` is equal to `num_horizon`. If a shorter horizon length is desired, the user should change
 * the mapping using `cocoa_SetTimeStepToStorageMapping`.
 *
 * @param num_states
 * @param num_inputs
 * @param num_constraints
 * @param num_data
 * @param time_step_index_to_storage_index
 * @param num_horizon
 * @param err
 * @return
 */
cocoa_Solver *cocoa_NewSolverCustomStorage(const int *num_states,
                                           const int *num_inputs,
                                           const int *num_constraints,
                                           int num_data,
                                           const int *time_step_index_to_storage_index,
                                           int num_horizon,
                                           cocoa_ERRORCODE *err);

cocoa_Solver *cocoa_ChangeHorizonLength(cocoa_Solver *solver, int num_horizon);

cocoa_Solver *cocoa_SetTimeStepToStorageMapping(cocoa_Solver *solver, const int *time_step_index_to_storage_index, int num_horizon);

// TODO: add a method that calculates the total number of bytes needed by the solver
// TODO: create a method that initializes the data from a provided data buffer
