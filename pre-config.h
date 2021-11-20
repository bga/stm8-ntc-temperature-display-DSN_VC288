//#pragma once

#define STM8S103

#define F_CPU_PRESCALER 0
#define F_CPU 16000000UL

//# to define xINTn_C macros in stdint.h
#define __STDC_CONSTANT_MACROS

//# enumerated type mixed with another enumerated type
#pragma diag_suppress=Pa089
