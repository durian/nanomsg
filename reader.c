#include <assert.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <stdio.h>
#include <nanomsg/nn.h>
#include <nanomsg/pipeline.h>
#include <nanomsg/pubsub.h>
#include <sys/time.h>

int main (const int argc, const char **argv) {

  /*int sock = nn_socket( AF_SP, NN_PULL );
  assert( sock >= 0 );
  assert( nn_bind(sock, "ipc:///tmp/nanotest.ipc") );*/

  int sock = nn_socket( AF_SP, NN_SUB );
  assert( sock >= 0 );
  assert( nn_bind(sock, "ipc:///tmp/nanotest.ipc") );

  nn_setsockopt( sock, NN_SUB, NN_SUB_SUBSCRIBE, "GPS", 3 );

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
