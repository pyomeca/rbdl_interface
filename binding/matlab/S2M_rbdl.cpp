#include "biorbd.h"

// MATLAB INTERFACE
///
/// \brief Entry point for the muscod application
///
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] )
{
    mexWarnMsgIdAndTxt("MATLAB:Obsolete",
                       "S2M_rbdl is obsolete, you should change it to biorbd");
    functionHub(nlhs, plhs, nrhs, prhs);
}

