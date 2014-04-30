#ifndef _FAKE_IMPL_H
#define _FAKE_IMPL_H

/* Somewhat functional implementations of various functions for test purposes.
 * These implementations are not fully functional, but they allow for example
 * for tests to run without having to initialize the complete runtime environ-
 * ment.
 * The #define's are set through helpers/makedeps, depending on which header
 * files appear in a STUB directive in the test source.
 */


#ifdef STUB_RTE_MALLOC_H

#include <stdlib.h>
#include <string.h>
void * rte_zmalloc (const char *type, size_t size, unsigned align)
{
	char *mem = malloc(size);
	memset(mem, 0, size);

	return mem;
}

#endif



#endif  // _FAKE_IMPL_H
