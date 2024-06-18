/*
 *  Purpose:
 *
 *    NELMIN minimizes a function using the Nelder-Mead algorithm.
 *
 *  Discussion:
 *
 *    This routine seeks the minimum value of a user-specified function.
 *
 *    Simplex function minimisation procedure due to Nelder+Mead(1965),
 *    as implemented by O'Neill(1971, Appl.Statist. 20, 338-45), with
 *    subsequent comments by Chambers+Ertel(1974, 23, 250-1), Benyon(1976,
 *    25, 97) and Hill(1978, 27, 380-2).
 *
 *    The function to be minimized must be defined by a function(al) of
 *    the form
 *
 *      real fn ( const std::array<real,n>& x )
 *
 *    where "real" can be any floating-point type, e.g. double.
 *
 *    This routine does not include a termination test using the
 *    fitting of a quadratic surface.
 *
 *  Licensing:
 *
 *    This code is distributed under the GNU LGPL license.
 *
 *  Author:
 *
 *    Original FORTRAN77 version by R ONeill.
 *    C version by John Burkardt (last modified 28 October 2010).
 *    Port to modern C++ by Piotr Różański. Numerical behaviour unchanged; changes restricted to secondary issues:
 *    a) output variables moved to returned struct type
 *    b) function is now passed as std::function which allows using objects as well as traditional functions
 *    c) floating-point type and number of variables are now template arguments
 *    d) std::array is now used instead of pointers to raw arrays
 *    e) overall comments and code formatting
 *
 *  Reference:
 *
 *    John Nelder, Roger Mead,
 *    A simplex method for function minimization,
 *    Computer Journal,
 *    Volume 7, 1965, pages 308-313.
 *
 *    R ONeill,
 *    Algorithm AS 47:
 *    Function Minimization Using a Simplex Procedure,
 *    Applied Statistics,
 *    Volume 20, Number 3, 1971, pages 338-345.
 *
 *    This C++ version is available at
 *    https://github.com/develancer/nelder-mead/
 */
#ifndef PTR_NELDER_MEAD_H
#define PTR_NELDER_MEAD_H

#include <array>
#include <climits>
#include <functional>

/**
 * Plain data object with output information from the run of nelder_mead routine.
 *
 * @tparam real floating-point type to be used, e.g. double
 * @tparam n the number of variables
 */
template<typename real, int n>
struct nelder_mead_result {
    std::array<real,n> xmin;
    real ynewlo;
    int icount;
    int numres;
    int ifault;
};

/**
 * This routine seeks the minimum value of a user-specified function.
 *
 * @tparam real floating-point type to be used, e.g. double
 * @tparam n the number of variables
 * @param fn the function to be minimized
 * @param start a starting point for the iteration
 * @param reqmin the terminating limit for the variance of function values
 * @param step determines the size and shape of the initial simplex;
 * the relative magnitudes of its elements should reflect the units of the variables
 * @param konvge the convergence check is carried out every konvge iterations
 * @param kcount the maximum number of function evaluations
 * @return structure with output information
 */
template<typename real, int n>
nelder_mead_result<real,n> nelder_mead(
        const std::function<real(const std::array<real,n> &)> &fn,
        std::array<real,n> start,
        real reqmin,
        const std::array<real,n> &step,
        int konvge = 1,
        int kcount = INT_MAX
) {
    const real ccoeff = 0.5;
    real del;
    const real ecoeff = 2.0;
    const real eps = 0.001;
    int ihi;
    int ilo;
    int jcount;
    int l;
    const real rcoeff = 1.0;
    real rq;
    real x;
    real y2star;
    real ylo;
    real ystar;
    real z;

    nelder_mead_result<real,n> result;

    // Check the input parameters.
    if (reqmin <= 0.0 || n < 1 || konvge < 1) {
        result.ifault = 1;
        return result;
    }

    std::array<real,n> p[n + 1];
    std::array<real,n> pstar, p2star, pbar;
    real y[n + 1];

    result.icount = 0;
    result.numres = 0;

    jcount = konvge;
    del = 1.0;
    rq = reqmin * n;

    // Initial or restarted loop.
    for (;;) {
        p[n] = start;
        y[n] = fn(start);
        result.icount++;

        for (int j = 0; j < n; j++) {
            x = start[j];
            start[j] += step[j] * del;
            p[j] = start;
            y[j] = fn(start);
            result.icount++;
            start[j] = x;
        }
        // The simplex construction is complete.

        // Find highest and lowest Y values.
        // YNEWLO = Y(IHI) indicates the vertex of the simplex to be replaced.
        ylo = y[0];
        ilo = 0;

        for (int i = 1; i < n + 1; i++) {
            if (y[i] < ylo) {
                ylo = y[i];
                ilo = i;
            }
        }
        // Inner loop.
        for (;;) {
            if (kcount <= result.icount) {
                break;
            }
            result.ynewlo = y[0];
            ihi = 0;

            for (int i = 1; i < n + 1; i++) {
                if (result.ynewlo < y[i]) {
                    result.ynewlo = y[i];
                    ihi = i;
                }
            }
            // Calculate PBAR, the centroid of the simplex vertices
            // excepting the vertex with Y value YNEWLO.
            for (int i = 0; i < n; i++) {
                z = 0.0;
                for (int j = 0; j < n + 1; j++) {
                    z += p[j][i];
                }
                z -= p[ihi][i];
                pbar[i] = z / n;
            }
            // Reflection through the centroid.
            for (int i = 0; i < n; i++) {
                pstar[i] = pbar[i] + rcoeff * (pbar[i] - p[ihi][i]);
            }
            ystar = fn(pstar);
            result.icount++;

            // Successful reflection, so extension.
            if (ystar < ylo) {
                for (int i = 0; i < n; i++) {
                    p2star[i] = pbar[i] + ecoeff * (pstar[i] - pbar[i]);
                }
                y2star = fn(p2star);
                result.icount++;

                // Check extension.
                if (ystar < y2star) {
                    p[ihi] = pstar;
                    y[ihi] = ystar;
                } else {
                    // Retain extension or contraction.
                    p[ihi] = p2star;
                    y[ihi] = y2star;
                }
            } else {
                // No extension.
                l = 0;
                for (int i = 0; i < n + 1; i++) {
                    if (ystar < y[i]) {
                        l++;
                    }
                }

                if (1 < l) {
                    p[ihi] = pstar;
                    y[ihi] = ystar;
                } else if (l == 0) {
                    // Contraction on the Y(IHI) side of the centroid.
                    for (int i = 0; i < n; i++) {
                        p2star[i] = pbar[i] + ccoeff * (p[ihi][i] - pbar[i]);
                    }
                    y2star = fn(p2star);
                    result.icount++;
                    // Contract the whole simplex.
                    if (y[ihi] < y2star) {
                        for (int j = 0; j < n + 1; j++) {
                            for (int i = 0; i < n; i++) {
                                p[j][i] = (p[j][i] + p[ilo][i]) * 0.5;
                            }
                            result.xmin = p[j];
                            y[j] = fn(result.xmin);
                            result.icount++;
                        }
                        ylo = y[0];
                        ilo = 0;

                        for (int i = 1; i < n + 1; i++) {
                            if (y[i] < ylo) {
                                ylo = y[i];
                                ilo = i;
                            }
                        }
                        continue;
                    } else {
                        // Retain contraction.
                        p[ihi] = p2star;
                        y[ihi] = y2star;
                    }
                } else if (l == 1) {
                    // Contraction on the reflection side of the centroid.
                    for (int i = 0; i < n; i++) {
                        p2star[i] = pbar[i] + ccoeff * (pstar[i] - pbar[i]);
                    }
                    y2star = fn(p2star);
                    result.icount++;
                    // Retain reflection?
                    if (y2star <= ystar) {
                        p[ihi] = p2star;
                        y[ihi] = y2star;
                    } else {
                        p[ihi] = pstar;
                        y[ihi] = ystar;
                    }
                }
            }
            // Check if YLO improved.
            if (y[ihi] < ylo) {
                ylo = y[ihi];
                ilo = ihi;
            }
            jcount--;

            if (0 < jcount) {
                continue;
            }
            // Check to see if minimum reached.
            if (result.icount <= kcount) {
                jcount = konvge;

                z = 0.0;
                for (int i = 0; i < n + 1; i++) {
                    z += y[i];
                }
                x = z / (n + 1);

                z = 0.0;
                for (int i = 0; i < n + 1; i++) {
                    real yx = y[i] - x;
                    z += yx * yx;
                }

                if (z <= rq) {
                    break;
                }
            }
        }
        // Factorial tests to check that YNEWLO is a local minimum.
        result.xmin = p[ilo];
        result.ynewlo = y[ilo];

        if (kcount < result.icount) {
            result.ifault = 2;
            break;
        }

        result.ifault = 0;

        for (int i = 0; i < n; i++) {
            del = step[i] * eps;
            result.xmin[i] += del;
            z = fn(result.xmin);
            result.icount++;
            if (z < result.ynewlo) {
                result.ifault = 2;
                break;
            }
            result.xmin[i] -= del + del;
            z = fn(result.xmin);
            result.icount++;
            if (z < result.ynewlo) {
                result.ifault = 2;
                break;
            }
            result.xmin[i] += del;
        }

        if (result.ifault == 0) {
            break;
        }
        // Restart the procedure.
        start = result.xmin;
        del = eps;
        result.numres++;
    }

    return result;
}

#endif // PTR_NELDER_MEAD_H