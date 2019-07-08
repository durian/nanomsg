#include <assert.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <stdio.h>
#include <nanomsg/nn.h>
#include <nanomsg/pipeline.h>

// Start ./sock1 in one window, then start ./sock0 
// in another. This one sends messages as fast as
// it can.

char buf[8196];

int main (const int argc, const char **argv) {
  char *msg    = "hi there";
  int   sz_msg = strlen(msg) + 1; // '\0' too
  int   sock   = nn_socket (AF_SP, NN_PUSH);

  assert(sock >= 0);
  assert( nn_connect(sock, "ipc:///tmp/nanotest.ipc") >= 0 );

  int i = 0;
  while (1) {
    //printf ("SENDING %i \"%s\"\n", i, msg); // slows down a lot
    int bytes = nn_send(sock, msg, sz_msg, 0);
    assert(bytes == sz_msg);
    ++i;
  }
  return nn_shutdown(sock, 0);
}

