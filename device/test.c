#include <stdio.h>
#include <stdint.h>

struct test{
	uintptr_t tx_base;
	uintptr_t rx_base;
}__attribute__((packed));

#define offset(bar4,a)	\
	(a - bar4 == 8)
int main()
{
	struct test t;
	
	if((&t.rx_base - &t.tx_base) == 1){
		printf("yes");
	}
	return 0;
}

