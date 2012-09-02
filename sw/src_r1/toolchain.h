#ifndef __TOOLCHAIN_H_
#define __TOOLCHAIN_H_

#ifdef __CC_ARM
	#define restrict
	#define __restrict
#elif defined ( __GNUC__ )
	
#endif

#endif
