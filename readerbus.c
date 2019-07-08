#include <assert.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <stdio.h>
#include <nanomsg/nn.h>
#include <nanomsg/pipeline.h>
#include <nanomsg/bus.h>
#include <sys/time.h>

int main (const int argc, const char **argv) {

  int sock = nn_socket( AF_SP, NN_BUS );
  assert( sock >= 0 );
  assert( nn_connect(sock, "ipc:///tmp/nanotest.ipc") );

  printf( "Start listening\n" );
  int c  = 0;
  int go = 1;
  char buf[512];
  while( go ) {
    memset( buf, 0, sizeof(buf) );
    int   bytes = nn_recv( sock, &buf, 512, 0 );
    assert( bytes >= 0 );
    if ( bytes > 0 ) {
      printf( "RECEIVED %i, %5i \"%s\"\n", bytes, c, (char*)buf );
    }
    ++c;
  }

  return 0;
}
