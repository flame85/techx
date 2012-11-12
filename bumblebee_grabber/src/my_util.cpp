#include <stdlib.h>
#include <sys/time.h>
#include "my_util.h"
double getSecofNow()
{
  struct timeval tim;
  gettimeofday(&tim, NULL);
  return tim.tv_sec+(tim.tv_usec/1000000.0);
}
