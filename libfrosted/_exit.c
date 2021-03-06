
/*
 * Frosted version of exit.
 */

#include "frosted_api.h"
#include "syscall_table.h"

extern int (**__syscall__)(int rc);

void exit(int rc)
{
    __syscall__[SYS_EXIT](rc);
}
