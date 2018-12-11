/*
 * Filter Coefficients (C Source) generated by the Filter Design and Analysis Tool
 * Generated by MATLAB(R) 9.0 and the Signal Processing Toolbox 7.2.
 * Generated on: 29-Nov-2018 14:05:25
 */

/*
 * Discrete-Time IIR Filter (real)
 * -------------------------------
 * Filter Structure    : Direct-Form II, Second-Order Sections
 * Number of Sections  : 8
 * Stable              : Yes
 * Linear Phase        : No
 */

/* General type conversion for MATLAB generated C-code  */
#include "tmwtypes.h"
/* 
 * Expected path to tmwtypes.h 
 * C:\Program Files\MATLAB\R2016a\extern\include\tmwtypes.h 
 */
#define MWSPT_NSEC 17
const int NL[MWSPT_NSEC] = { 1,3,1,3,1,3,1,3,1,3,1,3,1,3,1,2,1 };
const real64_T NUM[MWSPT_NSEC][3] = {
  {
      0.998754233947,                 0,                 0 
  },
  {
                   1,                -2,                 1 
  },
  {
       0.99639155023,                 0,                 0 
  },
  {
                   1,                -2,                 1 
  },
  {
     0.9941949840577,                 0,                 0 
  },
  {
                   1,                -2,                 1 
  },
  {
      0.992257820601,                 0,                 0 
  },
  {
                   1,                -2,                 1 
  },
  {
     0.9906613050596,                 0,                 0 
  },
  {
                   1,                -2,                 1 
  },
  {
     0.9894716772811,                 0,                 0 
  },
  {
                   1,                -2,                 1 
  },
  {
     0.9887378743758,                 0,                 0 
  },
  {
                   1,                -2,                 1 
  },
  {
     0.9942282955142,                 0,                 0 
  },
  {
                   1,                -1,                 0 
  },
  {
                   1,                 0,                 0 
  }
};
const int DL[MWSPT_NSEC] = { 1,3,1,3,1,3,1,3,1,3,1,3,1,3,1,2,1 };
const real64_T DEN[MWSPT_NSEC][3] = {
  {
                   1,                 0,                 0 
  },
  {
                   1,   -1.997441150923,   0.9975757848648 
  },
  {
                   1,                 0,                 0 
  },
  {
                   1,   -1.992715942736,   0.9928502581838 
  },
  {
                   1,                 0,                 0 
  },
  {
                   1,   -1.988322958442,   0.9884569777884 
  },
  {
                   1,                 0,                 0 
  },
  {
                   1,   -1.984448762095,   0.9845825203084 
  },
  {
                   1,                 0,                 0 
  },
  {
                   1,   -1.981255838619,    0.981389381619 
  },
  {
                   1,                 0,                 0 
  },
  {
                   1,   -1.978876663244,   0.9790100458801 
  },
  {
                   1,                 0,                 0 
  },
  {
                   1,   -1.977409106893,   0.9775423906103 
  },
  {
                   1,                 0,                 0 
  },
  {
                   1,  -0.9884565910284,                 0 
  },
  {
                   1,                 0,                 0 
  }
};
