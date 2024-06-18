#include "mex.h"
#include "nelder_mead.h"

// The gateway function
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    // Check for proper number of arguments
    if (nrhs != 2) {
        mexErrMsgIdAndTxt("MyToolbox:add_numbers:nrhs", "Two inputs required.");
    }
    if (nlhs != 1) {
        mexErrMsgIdAndTxt("MyToolbox:add_numbers:nlhs", "One output required.");
    }

    // Ensure both inputs are scalar double
    if (!mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) ||
        !mxIsDouble(prhs[1]) || mxIsComplex(prhs[1]) ||
        mxGetNumberOfElements(prhs[0]) != 1 ||
        mxGetNumberOfElements(prhs[1]) != 1) {
        mexErrMsgIdAndTxt("MyToolbox:add_numbers:notScalar", "Input must be a scalar.");
    }

    // Get the input values
    double input1 = mxGetScalar(prhs[0]);
    double input2 = mxGetScalar(prhs[1]);

    // Perform the addition
    double result = input1 + input2;

    // Set the output pointer to the output result
    plhs[0] = mxCreateDoubleScalar(result);
}
