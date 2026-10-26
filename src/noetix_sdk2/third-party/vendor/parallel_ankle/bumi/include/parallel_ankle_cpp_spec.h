//
// File: parallel_ankle_cpp_spec.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 09-Oct-2025 20:38:19
//
#ifndef PARALLEL_ANKLE_CPP_SPEC_H
#define PARALLEL_ANKLE_CPP_SPEC_H

// Include Files
#ifdef PARALLEL_ANKLE_FD_XIL_BUILD
#if defined(_MSC_VER) || defined(__LCC__)
#define PARALLEL_ANKLE_FD_DLL_EXPORT   __declspec(dllimport)
#else
#define PARALLEL_ANKLE_FD_DLL_EXPORT   __attribute__ ((visibility("default")))
#endif

#elif defined(BUILDING_PARALLEL_ANKLE_FD)
#if defined(_MSC_VER) || defined(__LCC__)
#define PARALLEL_ANKLE_FD_DLL_EXPORT   __declspec(dllexport)
#else
#define PARALLEL_ANKLE_FD_DLL_EXPORT   __attribute__ ((visibility("default")))
#endif

#else
#define PARALLEL_ANKLE_FD_DLL_EXPORT
#endif
#endif

//
// File trailer for parallel_ankle_cpp_spec.h
//
// [EOF]
//
