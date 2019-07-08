#include <libdill.h>
#include <stdio.h>
#include <stdlib.h>

coroutine int worker(const char *text) {
  while(1) {
    printf("%s\n", text);
    msleep(now() + random() % 1000);
  }
  return 0;
}

int main() {
  go(worker("Hello!"));
  go(worker("World!"));
  printf( "waiting\n" );
  msleep(now() + 5000);
  printf( "ready\n" );
  return 0;
}
