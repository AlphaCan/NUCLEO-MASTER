#ifndef _FUNCTION_
#define _FUNCTION_

#include <stdio.h>
#include <string.h>
#include "ethercat.h"

#define APP_DEBUG


#ifdef APP_DEBUG
#define debug_printf(format,...) printf(">>Time: %s, Line: %05d, Function: %s: loginfo: "format"", __TIME__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
#else
#define debug_PRINT(...) do{}while(0)
#endif




int nex_mastaer_init(void);




#endif
