// Simple math helper functions
//
// Copyright (C) 2020  Fred Sundvik
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#pragma once

//////////////////////////////////////////////////////////////////////
// Newton-Raphson root finding
// A fail-safe variant with an interval and bisection if failing
// The function f should return a tuple of the value and derivative
//////////////////////////////////////////////////////////////////////

struct newton_raphson_result
{
    double x;
    double y;
    double dy;
};

typedef void (*nr_callable)
    (struct newton_raphson_result *result, void *user_data); 

void newton_raphson(nr_callable f, double low, double high, double tolerance,
    double maxiter, struct newton_raphson_result *result, void *user_data);