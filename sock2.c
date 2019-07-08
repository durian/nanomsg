#include <assert.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <stdio.h>
#include <nanomsg/nn.h>
#include <nanomsg/pubsub.h>

int main (const int argc, const char **argv) {
  int pub = nn_socket (AF_SP, NN_PUB);
  int sub = nn_socket (AF_SP, NN_SUB);
  int nbytes;
  void *buf = NULL;

  nn_setsockopt (sub, NN_SUB, NN_SUB_SUBSCRIBE, "foo", 3);
  nn_setsockopt (sub, NN_SUB, NN_SUB_SUBSCRIBE, "bar", 3);

  nbytes = nn_send (pub, "foo|Hello!", 10, NN_DONTWAIT);
  assert(nbytes == 10);
  printf( "nbytes %i\n", nbytes );

  nbytes = nn_recv (sub, &buf, NN_MSG, 0);
  assert (nbytes == 10);
  printf( "nbytes %i\n", nbytes );
  nn_freemsg (buf);
  
  nbytes = nn_send (pub, "baz|World!", 10, NN_DONTWAIT);
  
  /* Message is not delivered because if matches no subscription. */
  nbytes = nn_recv(sub, &buf, NN_MSG, NN_DONTWAIT);
  printf( "nbytes %i\n", nbytes );

  return 0;
}
