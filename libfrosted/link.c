/*
 * Stub version of link.
 */

#include "frosted_api.h"
#include <errno.h>
#undef errno
extern int errno;

int link(char *existing, char *new)
{
  errno = ENOSYS;
  return -1;
}

