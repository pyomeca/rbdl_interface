#include "biorbd.h"

// MATLAB INTERFACE
///
/// \brief Entry point for the muscod application
///
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] )
{
    functionHub(nlhs, plhs, nrhs, prhs);
}
