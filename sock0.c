#include <assert.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <stdio.h>
#include <nanomsg/nn.h>
#include <nanomsg/pipeline.h>
#include <sys/time.h>

// This one receives 1 second worth of messages from sock1.

long long unsigned int ts() {
  struct timeval tv;
  gettimeofday(&tv,NULL);
  return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

int main (const int argc, const char **argv) {
  int sock = nn_socket( AF_SP, NN_PULL );
  assert( sock >= 0 );
  assert( nn_bind(sock, "ipc:///tmp/nanotest.ipc") );

  printf( "Start listening\n" );
  long long unsigned int t0 = ts();
  long long unsigned int t1 = 0;
  int c  = 0;
  int go = 1;
  while( go ) {
    void *buf   = NULL;
    int   bytes = nn_recv( sock, &buf, NN_MSG, 0 );
    assert( bytes >= 0 );
    //printf( "NODE0: RECEIVED %5i \"%s\"\n", c, buf );
    nn_freemsg(buf);
    t1 = ts() - t0;
    ++c;
    if ( t1 > 1000000 ) { // 1 secs
      go = 0;
    }
  }
  float tt = t1 / 1000000.0;
  printf( "TOTAL %i, %llu microsecs %f secs  %.6f\n", c, t1, tt, tt/c );
  return 0;
}
