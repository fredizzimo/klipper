// Simple math helper functions
//
// Copyright (C) 2020  Fred Sundvik
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "mathutil.h"
#include <math.h>

void newton_raphson(nr_callable f, double low, double high, double tolerance,
    double maxiter, struct newton_raphson_result *result, void *user_data)
{
    struct newton_raphson_result res_low = {.x=low};
    struct newton_raphson_result res_high = {.x=high};
    f(&res_low, user_data);
    f(&res_high, user_data);
    if (res_low.y == 0.0)
    {
        *result = res_low;
        return;
    }
    if (res_high.y == 0.0)
    {
        *result = res_high;
    }

    // If there's no zero in the range, return the point closest to it
    if (res_low.y < 0.0 && res_high.y < 0.0)
    {
        if (res_low.y > res_high.y)
        {
            *result = res_low;
        }
        else
        {
            *result = res_high;
        }
        return;
    }
    if (res_low.y > 0.0 && res_high.y > 0.0)
    {
        if (res_low.y > res_high.y)
        {
            *result = res_high;
        }
        else
        {
            *result = res_low;
        }
        return;
    }

    double x_low, x_high;
    // Always keep f(x_low) < 0
    if (res_low.y < 0.0)
    {
        x_low = low;
        x_high = high;
    }
    else
    {
        x_low = high;
        x_high = low;
    }

    // Start the search in the middle
    result->x = 0.5 * (low + high);
    double dx = high - low;
    double dx_old = dx;

    f(result, user_data);

    for (int i=0;i<maxiter;i++)
    {
        // Use bisect if newton returns out of range or the change is too small
        double x = result->x;
        double dy = result->dy;
        double y = result->y;
        if (((x - x_high)*dy - y) * ((x - x_low)*dy - y) > 0.0 ||
            fabs(2.0*y) > fabs(dx_old*dy))
        {
            dx_old = dx;
            dx = 0.5 * (x_high - x_low);
            x = x_low + dx;
        }
        // Newton-Raphson
        else
        {
            dx_old = dx;
            dx = y / dy;
            x -= dx;
        }

        // Return if within the tolerance
        if (fabs(dx) < tolerance)
        {
            return;
        }
        result->x = x;

        f(result, user_data);
        // Maintain the bracket
        if (result->y < 0.0)
            x_low = x;
        else
            x_high = x;
    }

    // We are hopefully close enough
}