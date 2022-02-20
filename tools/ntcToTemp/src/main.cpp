#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <!cpp/Math/Fp/log2fix.h>
#include <!cpp/common.h>

#define BGA__MCU__SENSOR__ANALOG__NTC_THERMISTOR__DEBUG_INPECT(exprArg) ::std::cerr << BGA__STR(exprArg) << " = " << (exprArg) << :: std::endl;

#include "../../../common-src/NtcThermistor.h"

const char* const help = (""
	"\n"   "%s rIce_ohm oneDivB r_ohm"
	"\n\t" "rIce_ohm - NTC resistance at 0C(273.15K)"
	"\n\t" "oneDivB - NTC exp coefficient"
	"\n\t" "r_ohm - NTC resistance"
	"\n"   ""
);

typedef U16 Um9_25;
typedef FI16 FI10_6;

int main(int argc, char* argv[]) {
	if(argc <= 3) {
		printf(help, argv[0]);
		return 0;
	};
	
	#define DEF_AND_TRY_PARSE_UINT(nameArg, argvNoArg) UInt nameArg; do { \
		if(1 != sscanf(argv[(argvNoArg)], "%u", &nameArg)) { \
			fprintf(stderr, "Can not parse unsigned int " BGA__STR(nameArg) " at command line pos " BGA__STR((argvNoArg) - 1) "\n" ); \
			return (argvNoArg); \
		} \
	} while(0)
	DEF_AND_TRY_PARSE_UINT(rIce_ohm, 1);
	DEF_AND_TRY_PARSE_UINT(oneDivB, 2);
	DEF_AND_TRY_PARSE_UINT(r_ohm, 3);
	#undef DEF_AND_TRY_PARSE_UINT
	
	::Bga::Mcu::Sensor::Analog::NtcThermistor<U16> t;

	t.m_rIce_ohm = rIce_ohm; 
	t.m_oneDivB = Um9_25(oneDivB); 

	FI16 ret = t.convert(r_ohm);
	
	printf("fixed_int_value FI10_6(%d)\n", int(ret));
	printf("float_value %lf\n", double(ret) / (1 << 6));
	
	return 0;
}

