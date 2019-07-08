/*
 * Reads GPS NMEA messages.
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <linux/inotify.h>
#include <string.h>
#include <syslog.h>
#include <stdarg.h>
#include <unistd.h>
#include <dirent.h>
#include <termios.h>

#include <nanomsg/nn.h>
#include <nanomsg/pipeline.h>
#include <nanomsg/reqrep.h>

#define DEBUG 1

// An nmea message may contain a maximum of
// 20 items, each with a maximum length of 16
// A message can be max 20*16, plus 19 commas.
//
#define MAX_ITEMS  20
#define MAX_LENGTH 16
#define GPS_BUFFER ( (MAX_ITEMS * MAX_LENGTH) + MAX_ITEMS )

void LOG(const char* format, ...) {
  va_list args;
  char buf[8192];

  va_start(args,format);

  vsprintf(buf,format,args);
  openlog("gps", LOG_PID | LOG_CONS, LOG_USER);
  syslog(LOG_INFO, "%s", buf);
  closelog();

  printf("%s\n", buf);

  va_end(args);
}

// Split a comma-separated messages into parts.
//
int split( char *s, char *strs[MAX_ITEMS] ) {
  const char delims[] = ",";
  char buf[MAX_LENGTH];
  
  int i = 0;
  do {
    size_t field_len = strcspn(s, delims);
    if ( field_len > MAX_LENGTH ) {
      field_len = MAX_LENGTH;
    }
    sprintf(buf, "%.*s", (int)field_len, s);
    strs[i] = malloc(strlen(buf) + 1);
    strcpy(strs[i], buf);
    i += 1;
    s += field_len;
  } while ( (*s++) && (i < MAX_ITEMS) );
  return i;
}

// $GPRMC,084435.0,A,5617.428119,N,01250.815218,E,0.0,,190517,0.0,E,A*2D
// nmea_to_dec(5617.428119);  -> 56.290470
// nmea_to_dec(01250.815218); -> 12.846920
//
float nmea_to_dec(float nmea_coord) {
  int b;
  float c;
  
  b = nmea_coord / 100; // 51 degrees
  c = (nmea_coord / 100 - b) * 100 ; //(51.536605 - 51)* 100 = 53.6605
  c /= 60; // 53.6605 / 60 = 0.8943417
  c += b; // 0.8943417 + 51 = 51.8943417
  return c;
}

/*
  ALS3-E supports the following GPS related NMEA sequences:
  • GPGGA - Global Positioning System Fix Data, Time, Position and fix related 
    data for a GNSS receiver
  • GPRMC - Recommended minimum data for GPS
  • GPGSV - Detailed satellite data
  • GPGSA - Overall satellite data
  • GPVTG - Vector track and speed over the Ground

  The device prefix "GL" is for GLONASS related data. "GN" refers to GPS and 
  GLONASS together. ALS3-E supports the following GLONASS related NMEA 
  sentences:
  • GLGSV - Detailed satellite data
  • GNGSA - Overall satellite data
  • GNGNS - Positioning System
*/
int parse_nmea( char* nmea ) {
  char *items[MAX_ITEMS];
  char msg[6];

  // We remove the *xx checksum from the end.
  if ( nmea[strlen(nmea)-3] == '*' ) {
    nmea[strlen(nmea)-3] = '\0';
  }
  int num_items = split( nmea, items );

  strncpy(msg, items[0]+1, 5);
  msg[5] = '\0';

#ifdef DEBUG
  LOG( "Message %s, containing %i items", msg, num_items );
#endif

  if ( strcmp("GPGGA", msg) == 0 ) {
    // $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
    //      0         1       2 3        4 5 6  7 
    //                                       | quality of fix
    //
    if ( strcmp( "0", items[6]) == 0 ) {
      // LOG( "GPGGA; invalid fix" );
    } else if ( strcmp( "1", items[6]) == 0 ) {
      // GPS fix
      // LOG( "GPGGA; valid fix" );
    } else if ( strcmp( "2", items[6]) == 0 ) {
      // Diff. GPS fix
    }
  } else if ( strcmp("GPRMC", msg) == 0 ) {
    // $GPRMC,,V,,,,,,,,,,N*53
    // $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
    //      0         1 2
    //                  | Data status (V=navigation receiver warning)
    //
    if ( strcmp( "V", items[2]) == 0 ) {
      // LOG( "GPRMC; invalid data" );
    } else {
      float lat = nmea_to_dec( atof(items[3]) );
      float lon = nmea_to_dec( atof(items[5]) );
      printf( "lat=%f %s lon=%f %s\n", lat, items[4], lon, items[6] );
    }
  } else if ( strcmp("GPGSV", msg) == 0 ) {
    // $GPGSV,x,x,x,x,x,x,x,...*hh
    //        | num of messages
    //          | this message num
    //            | total number of satellites
    //
#ifdef DEBUG
    LOG( "Satellites in view: %s", items[3] );
    int idx;
    for ( idx = 4; idx < num_items; idx += 4 ) {
      LOG( "PRN %s, ELE %s, AZI %s, SNR %s", items[idx], items[idx+1], items[idx+2], items[idx+3] );
    }
#endif
    // This is potential info for cloud/queue/HU
  } else if ( strcmp("GPGSA", msg) == 0 ) {
    // $GPGSA,A,3,,,,,,16,18,,22,24,,,3.6,2.1,2.2*3C
    // 2    = Mode: 1=Fix not available, 2=2D, 3=3D
    // 3-14 = IDs of SVs used in position fix (null for unused fields)
    // 15   = PDOP, 16   = HDOP, 17   = VDOP
    //
#ifdef DEBUG
    LOG( "MODE %s, PDOP %s, HDOP %s, VDOP %s", items[2], items[15], items[16], items[17] );
#endif
    // This is potential info for cloud/queue/HU
  } else if ( strcmp("GPVTG", msg) == 0 ) {
    // $GPVTG,x.x,T,x.x,M,x.x,N,x.x,K,m,*hh
    // This is potential info for cloud/queue/HU
  } else if ( strcmp("GLGSV", msg) == 0 ) {
    // GLONASS
  } else if ( strcmp("GNGSA", msg) == 0 ) {
    // Beidou(?) satellites
  } else if ( strcmp("GNGNS", msg) == 0 ) {
    // GLONASS satellites in view
  } else {
#ifdef DEBUG
    LOG( "Unhandled message %s", msg );
#endif
  }

  // Free allocated memory by split()
  int i;
  for( i = 0; i < num_items; i++ ) {
    free( items[i] );
  }
}

int main( int argc, char *argv[] ) {
  int  gps_fd;
  int  c;
  char *device_name = NULL;
  char buff[GPS_BUFFER];
  int  len;
  int  res;
  int  loop = 1;
  struct timeval timeout;
  fd_set read_fds, write_fds, except_fds;

int control_sock = nn_socket( AF_SP, NN_REQ );
  if ( control_sock < 0 ) {
    printf( "Cannot open controlsocket, errno %i (%s)\n", errno, strerror(errno) );
    return 1;
  }
  res = nn_connect( control_sock, "ipc:///tmp/control.ipc" );
  if ( res < 0 ) {
    printf( "Cannot connect to control socket, errno %i (%s)\n", errno, strerror(errno) );
    return 1;
  }

  char msg[128];
  memset( msg, 0, sizeof(msg) );
  snprintf( msg, 128, "REQCHANNEL" );
  int bytes = nn_send( control_sock, msg, strlen(msg), 0 );
  if ( bytes < 0 ) {
    printf( "Cannot nn_recv, errno %i (%s)", errno, strerror(errno) );
    return 1;
  }

  memset( msg, 0, sizeof(msg) );
  bytes = nn_recv( control_sock, &msg, 128, 0 );
  if ( bytes < 0 ) {
    printf( "Cannot nn_recv, errno %i (%s)\n", errno, strerror(errno) );
    return 1;
  }
  printf( "RECEIVED %s\n", msg );
  nn_shutdown(control_sock, 0);

  if ( strcmp(msg, "NOCHANNEL") == 0 ) {
    printf( "No channels available\n" );
    return 1;
  }

  // msg contains the endpoint

  int sock = nn_socket(AF_SP, NN_PUSH);

  if ( sock < 0 ) {
    printf( "Cannot open nn_socket, errno %i (%s)\n", errno, nn_strerror(errno) );
    return 1;
  }
  res = nn_connect(sock, msg);
  if ( res < 0 ) {
    printf( "Cannot nn_connect, errno %i (%s)\n", errno, nn_strerror(errno) );
    return 1;
  }

  LOG( "Starting." );

  while ( (c = getopt(argc, argv, "d:")) != -1) {
    switch (c) {
    case 'd':
      device_name = strdup(optarg);
      break;
    default:
      LOG("?? getopt returned character code 0%o ??\n", c);
    }
  }

  if ( ! device_name ) {
    device_name = strdup("/dev/ttyACM2");
  }

  gps_fd = open( device_name, O_RDONLY | O_NOCTTY );
  if ( gps_fd == -1 ) {
    LOG( "Can not open device, errno %i", errno );
    return 1;
  }

  // Read loop.
  // We get timeouts sometimes, which can be fixed by closing and
  // re-opening the tty. Closing, however, takes 30 seconds, 30 seconds
  // in which we get no data.
  // Pressing CTRL-C on command line and restarting is instantaneous.
  /*
    May 17 14:43:55 b2qt user.info gps[1763]: $GPRMC,,V,,,,,,,,,,N*53
    May 17 14:43:56 b2qt user.info gps[1763]: timeout
    May 17 14:43:56 b2qt user.info gps[1763]: Closing after timeout.
    May 17 14:44:26 b2qt user.info gps[1763]: Re-opening after timeout.
    May 17 14:44:26 b2qt user.info gps[1763]: Re-opened device.
    May 17 14:44:27 b2qt user.info gps[1763]: $GPGSV,2,1,06,33,,,34,39,,,34,...
  */
  //
  int max_timeout = 10;
  while ( loop ) {

    // Set timeout to 4.0 seconds and read.
    //
    FD_ZERO( &read_fds );
    FD_ZERO( &write_fds );
    FD_ZERO( &except_fds );
    FD_SET( gps_fd, &read_fds );
    timeout.tv_sec  = 4;
    timeout.tv_usec = 0;
    res = select( gps_fd+1, &read_fds, &write_fds, &except_fds, &timeout );

    if ( res == -1 ) {
      LOG( "select error %i", errno );
      loop = 0;
    } else if ( res == 0 ) {
      LOG( "select timeout" );
      if ( --max_timeout == 0 ) {
	loop = 0;
      } else {
	// test -- re-init
	LOG( "Closing after timeout." );
	int cr = close( gps_fd );
	if ( cr == -1 ) {
	  LOG( "Cannot close device, errno %i", errno );
	  return 1;
	}
	sleep(1);
	//
	LOG( "Re-opening after timeout." );
	gps_fd = open( device_name, O_RDONLY | O_NOCTTY  );
	if ( gps_fd == -1 ) {
	  LOG( "Cannot open device, errno %i", errno );
	  return 1;
	}
	LOG( "Re-opened device." );
	// end test
      }
    } else {
      len = read( gps_fd, buff, GPS_BUFFER );
      if ( len == -1 ) {
	LOG( "Cannot read from device, errno %i", errno );
	return 1;
      }
      buff[len] = '\0';
      buff[strcspn(buff, "\r\n")] = 0; // we get single 0A/LFs too, erase'm all
      if ( strlen(buff) > 0 ) { 
#ifdef DEBUG
	LOG( buff ); // spams log files
#endif
	max_timeout = 10; // reset counter
	memset( ans, 0, sizeof(ans) );
	parse_nmea( buff, ans );

	if ( strlen(ans) == 0 ) {
	  //snprintf( ans, MAX_TEXT_LEN, "nothing" );
	} else {
	  int bytes = nn_send(sock, ans, strlen(ans), 0 );//NN_DONTWAIT);
	  if ( bytes < 0 ) {
	    printf( "bytes err=%i (%s)\n", errno, nn_strerror(errno) );
	  }
	  ++msg_num;
	}

      }
    }

  }

  int cr = close( gps_fd ); // takes 30 secs?
  if ( cr == -1 ) {
    LOG( "Cannot close device, errno %i", errno );
  }

  free( device_name );

  LOG( "exit." );
  return 0;
}

/* ---- EOT ---------------------------------------------------------------- */
