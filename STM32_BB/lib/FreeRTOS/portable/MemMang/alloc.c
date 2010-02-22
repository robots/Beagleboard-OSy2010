/* this is a glue between newlib and FreeRTOS heap 2 allocator ! */
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"

typedef struct A_BLOCK_LINK
{
	struct A_BLOCK_LINK *pxNextFreeBlock;	/*<< The next free block in the list. */
	size_t xBlockSize;						/*<< The size of the free block. */
} xBlockLink;

static const unsigned short  heapSTRUCT_SIZE	= ( sizeof( xBlockLink ) + portBYTE_ALIGNMENT - ( sizeof( xBlockLink ) % portBYTE_ALIGNMENT ) );

_PTR _realloc_r(struct _reent *re, _PTR ptr, size_t size) {
	xBlockLink *block;
	void *tmp;

	tmp = pvPortMalloc(size);

	if (tmp == NULL) return NULL;
	
	block = ptr;
	block -= heapSTRUCT_SIZE;

	memcpy(tmp, ptr, block->xBlockSize);

	vPortFree(ptr);
	
	return tmp;
}

_PTR _calloc_r(struct _reent *re, size_t num, size_t size) {
	return pvPortMalloc(num*size);
}

_PTR _malloc_r(struct _reent *re, size_t size) {
	return pvPortMalloc(size);
}

_VOID _free_r(struct _reent *re, _PTR ptr) {
	vPortFree(ptr);
}

