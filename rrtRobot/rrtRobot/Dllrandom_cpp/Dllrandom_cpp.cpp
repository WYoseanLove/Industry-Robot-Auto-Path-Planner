// Dllrandom_cpp.cpp : Defines the exported functions for the DLL.
//

#include "pch.h"
#include "framework.h"
#include "Dllrandom_cpp.h"


// This is an example of an exported variable
DLLRANDOMCPP_API int nDllrandomcpp=0;

// This is an example of an exported function.
DLLRANDOMCPP_API int fnDllrandomcpp(void)
{
    return 0;
}

// This is the constructor of a class that has been exported.
CDllrandomcpp::CDllrandomcpp()
{
    return;
}
