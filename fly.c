/*!
  \file fly.c

  \brief Client which connects to spider, gets channel,
  starts sending messages.

  Protocol:
  Initialise communication over control socket.
  client: send        REQCHANNEL
  server: reply (0)   NOCHANNEL                  no channel was available
                (1)   ipc:///tmp/nanotest0.ipc   physical endpoint for communication
  client: send messages on url
  server shuts down connection after 5 seconds of no traffic.

*/
#include <assert.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <stdio.h>
#include <nanomsg/nn.h>
#include <nanomsg/pipeline.h>
#include <nanomsg/reqrep.h>
#include <stdlib.h>

#include <poll.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netinet/tcp.h>

/* DEFINES ---------------------------------------------------------------- */

#include "nanodefs.h"

/* CODE ------------------------------------------------------------------- */

int nnsend( int sock, char *msg ) {
    int  tries   =     10;
    long tries_d = 100000; // 1e6 = 1 second

    while ( tries-- > 0 ) {
      int bytes = nn_send(sock, msg, strlen(msg), NN_DONTWAIT);
      if ( bytes > 0 ) {
	return 0;
      }
      if ( bytes == EAGAIN ) {
	usleep( tries_d );
	continue;
      }
      if ( bytes < 0 ) {
	//printf( "Cannot nn_send, errno %i (%s)\n", errno, strerror(errno) );
	usleep( tries_d );
      }
    } // while tries
    printf( "Not sent\n" );
    return -1;
}

int receive( int sock, char *msg ) {
  int  tries   =     10;
  long tries_d = 100000; // 1e6 = 1 second
  
  while( tries-- > 0 ) {
    int bytes = nn_recv( sock, msg, MSG_SIZE, NN_DONTWAIT );
    if ( bytes > 0 ) {
      return 0;
    }
    if ( bytes < 0 ) {
      usleep( tries_d );
    }
  } // while tries
  printf( "Nothing received\n" );
  return -1;
}

int nnreceive( int sock, char *msg ) {
  struct pollfd  poll_fds[1];  
  int    nnsocket_recvfd;
  size_t sz = sizeof(int);

  nn_getsockopt( sock, NN_SOL_SOCKET, NN_RCVFD, &nnsocket_recvfd, &sz );
  poll_fds[0].fd     = nnsocket_recvfd;
  poll_fds[0].events = POLLIN;

  int poll_state = poll( poll_fds, 1, 5000 );
  if ( poll_state > 0 ) {
    if ( poll_fds[0].revents & POLLIN ) {
      int bytes = nn_recv( sock, msg, MSG_SIZE, NN_DONTWAIT );
      if ( bytes > 0 ) {
	return bytes;
      }
    }
  }
  printf( "Nothing received\n" );
  return -1;
}


int main( int argc, char *argv[] ) {

  long d   = 1000000; /*!< Delay between each message */
  long num = 10;      /*!< Number of messages to send */
  int  c;
  char *endpoint;

  while ( (c = getopt(argc, argv, "d:n:")) != -1) {
    switch (c) {
    case 'd':
      d = atol(optarg);
      break;
    case 'n':
      num = atol(optarg);
      break;
    }
  }

  /*
    Initial server-client communication is through control socket.
  */
  int control_sock = nn_socket( AF_SP, NN_REQ );
  if ( control_sock < 0 ) {
    printf( "Cannot open control socket, errno %i (%s)\n", errno, strerror(errno) );
    return 1;
  }
  int res = nn_connect( control_sock, "ipc:///tmp/control.ipc" );
  if ( res < 0 ) {
    printf( "Cannot connect to control socket, errno %i (%s)\n", errno, strerror(errno) );
    return 1;
  }

  char msg[MSG_SIZE];
  memset( msg, 0, sizeof(msg) );
  snprintf( msg, MSG_SIZE, "REQCHANNEL" );
  int bytes = nnsend( control_sock, msg );
  if ( bytes < 0 ) {
    printf( "Cannot nnsend, errno %i (%s)", errno, strerror(errno) );
    return 1;
  }

  memset( msg, 0, sizeof(msg) );
  bytes = nnreceive( control_sock, msg );
  if ( bytes < 0 ) {
    printf( "Cannot nn_recv, errno %i (%s)\n", errno, strerror(errno) );
    return 1;
  }
  printf( "RECEIVED %s\n", msg );
  //nn_shutdown(control_sock, 0);

  if ( strcmp(msg, "NOCHANNEL") == 0 ) {
    printf( "No channels available\n" );
    return 1;
  }

  // Received msg contains the endpoint
  endpoint = strdup( msg );
  int sock = nn_socket(AF_SP, NN_PUSH);
  if ( sock < 0 ) {
    printf( "Cannot open nn_socket, errno %i (%s)\n", errno, nn_strerror(errno) );
    return 1;
  }
  res = nn_connect(sock, endpoint);
  if ( res < 0 ) {
    printf( "Cannot nn_connect, errno %i (%s)\n", errno, nn_strerror(errno) );
    return 1;
  }

  long i = num;
  sleep(1);
  while ( i-- != 0 ) {
    memset( msg, 0, sizeof(msg) );
    snprintf( msg, MSG_SIZE, "Hi %05li", i );
    if ( d > 10000 ) {
      printf ("SENDING %li \"%s\"\n", i, msg); // slows down a lot
    }
    res = nnsend( sock, msg );
    if ( res < 0 ) {
      // Happens when server shuts down socket because of timeout.
      printf( "Cannot send to server, aborting!\n" );
      break;
    }
    usleep(d);
  } // while i

  printf( "Shutting down.\n" );

  memset( msg, 0, sizeof(msg) );
  snprintf( msg, MSG_SIZE, "ENDCHANNEL %s", endpoint ); // plus endpoint
  bytes = nnsend( control_sock, msg );
  if ( bytes < 0 ) {
    printf( "Cannot nn_recv, errno %i (%s)", errno, strerror(errno) );
    return 1;
  }
  nn_shutdown(control_sock, 0);

  free( endpoint );

  sleep(1);
  return nn_shutdown(sock, 0);
}

