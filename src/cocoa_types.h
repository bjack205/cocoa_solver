//
// Created by Brian Jackson on 8/1/22.
// Copyright (c) 2022 Robotic Exploration Lab. All rights reserved.
//

#pragma once

#ifdef COCOA_SINGLE_PRECISION
typedef float cocoa_float;
#define COCOA_USE_SINGLE_PRECISION 1
#else
typedef double cocoa_float;
#define COCOA_USE_SINGLE_PRECISION 0
#endif

/**
 * @brief Check if cocoa is running single precision arithmetic
 * @return true if cocoa was compiled to use single precision floats, false if double.
 */
bool cocoa_use_single_precision(void) { return COCOA_USE_SINGLE_PRECISION; }
