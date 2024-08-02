// The following ifdef block is the standard way of creating macros which make exporting
// from a DLL simpler. All files within this DLL are compiled with the DLLRANDOMCPP_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see
// DLLRANDOMCPP_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef DLLRANDOMCPP_EXPORTS
#define DLLRANDOMCPP_API __declspec(dllexport)
#else
#define DLLRANDOMCPP_API __declspec(dllimport)
#endif

// This class is exported from the dll
class DLLRANDOMCPP_API CDllrandomcpp {
public:
	CDllrandomcpp(void);
	// TODO: add your methods here.
};

extern DLLRANDOMCPP_API int nDllrandomcpp;

DLLRANDOMCPP_API int fnDllrandomcpp(void);
