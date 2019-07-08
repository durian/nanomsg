#include <assert.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <stdio.h>
#include <nanomsg/nn.h>
#include <nanomsg/pipeline.h>
#include <pthread.h>
#include <stdlib.h>

// see github pthread_demo

struct t_data {
  int sock;
  int n;
  long d;
  char *id;
} t_data;

void *fly(void *the_data) {
  char buf[128];
  struct t_data *my_data = (struct t_data*)the_data;
  int sock = my_data->sock;
  int n = my_data->n;
  long d = my_data->d;
  char *id = my_data->id;
  int max_tries = 10;
  long pause_tries = 1000000;
  int tries = 0;

  while ( n-- > 0 ) {
    printf ("SENDING %s %i\n", id, n); // slows down a lot
    memset( buf, 0, sizeof(buf) );
    snprintf( buf, 128, "Hi %i %s", n, id );
    tries = 0;
    while ( tries++ < max_tries ) {
      int bytes = nn_send(sock, buf, strlen(buf), 0 ); //NN_DONTWAIT);
      if ( bytes < 0 ) {
	//       Cannot nn_send, errno 11 (Resource temporarily unavailable)
	printf( "Cannot nn_send, try %i, errno %i (%s)\n", tries, errno, nn_strerror(errno) );
	// sleep, return, wait, count?
	usleep( pause_tries );
      } else {
	break;
      }
    }
    usleep(d);
  }
}

int main( int argc, char *argv[] ) {
 int  max_t = 10;
 long d     = 100000; // 1/10 s
 int  msgs  = 10;
 int  c;

 while ( (c = getopt(argc, argv, "d:m:t:")) != -1) {
    switch (c) {
    case 'd':
      d = atol(optarg);
      break;
    case 'm':
      msgs = atoi(optarg);
      break;
    case 't':
      max_t = atoi(optarg);
      break;
    }
  }
 
  int sock = nn_socket(AF_SP, NN_PUSH);
  if ( sock < 0 ) {
    printf( "Cannot open nn_socket, errno %i (%s)\n", errno, nn_strerror(errno) );
    return 1;
  }
  /*
    char tmp[128];
    memset( tmp, 0, sizeof(tmp) );
    snprintf( tmp, 128, "ipc:///tmp/nanotest%i.ipc", i );
    printf( "%s\n", tmp );
  */
  int res = nn_connect(sock, "ipc:///tmp/nanotest0.ipc");
    if ( res < 0 ) {
    printf( "Cannot nn_connect, errno %i (%s)\n", errno, nn_strerror(errno) );
    return 1;
  }

  char buf[128];

  pthread_t t[max_t];
  struct t_data *foo[max_t];

  int i;
  for( i = 0; i < max_t; i++) {
    foo[i] = malloc(sizeof(struct t_data));
    foo[i]->sock = sock;
    foo[i]->n = msgs;
    foo[i]->d = d;
    char tmp[8];
    snprintf( tmp, 8, "id-%i", i );
    foo[i]->id = strdup(tmp);
    pthread_create( &t[i], NULL, fly, (void*)foo[i] );
  }

  for( i = 0; i < max_t; i++) {
    pthread_join(t[i], NULL);
  }

  sleep( 10 );
  return nn_shutdown(sock, 0);
}

