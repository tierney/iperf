/*---------------------------------------------------------------
 * Copyright (c) 1999,2000,2001,2002,2003
 * The Board of Trustees of the University of Illinois
 * All Rights Reserved.
 *---------------------------------------------------------------
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software (Iperf) and associated
 * documentation files (the "Software"), to deal in the Software
 * without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit
 * persons to whom the Software is furnished to do
 * so, subject to the following conditions:
 *
 *
 * Redistributions of source code must retain the above
 * copyright notice, this list of conditions and
 * the following disclaimers.
 *
 *
 * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimers in the documentation and/or other materials
 * provided with the distribution.
 *
 *
 * Neither the names of the University of Illinois, NCSA,
 * nor the names of its contributors may be used to endorse
 * or promote products derived from this Software without
 * specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE CONTIBUTORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * ________________________________________________________________
 * National Laboratory for Applied Network Research
 * National Center for Supercomputing Applications
 * University of Illinois at Urbana-Champaign
 * http://www.ncsa.uiuc.edu
 * ________________________________________________________________
 *
 * Dataxfer.cpp
 * by Lucas Nussbaum <lucas.nussbaum@ens-lyon.fr>
 * -------------------------------------------------------------------
 * Functions to send or receive data. Used by both client and server,
 * depending on their role during the experiment.
 * ------------------------------------------------------------------- */

#include "Dataxfer.hpp"
#include "Extractor.h"
#include "PerfSocket.hpp"
#include "Locale.h"
#include "delay.hpp"
#include "List.h"

const double kSecs_to_usecs = 1e6;
const int    kBytes_to_Bits = 8;

/* Send data through a TCP connection */
void data_sendtcp(thread_Settings * mSettings, char * mBuf, Timestamp mEndTime)
{
  Timestamp lastPacketTime;
  unsigned long currLen = 0;
  struct itimerval it;
  max_size_t totLen = 0;

  int err;

  char* readAt = mBuf;

  // Indicates if the stream is readable
  bool canRead = true, mMode_Time = isModeTime( mSettings );
  ReportStruct *reportstruct = NULL;

  // InitReport handles Barrier for multiple Streams
  mSettings->reporthdr = InitReport( mSettings );
  reportstruct = new ReportStruct;
  reportstruct->packetID = 0;

  lastPacketTime.setnow();
  if ( mMode_Time ) {
    memset (&it, 0, sizeof (it));
    it.it_value.tv_sec = (int) (mSettings->mAmount / 100.0);
    it.it_value.tv_usec = (int) 10000 * (mSettings->mAmount -
                                         it.it_value.tv_sec * 100.0);
    err = setitimer( ITIMER_REAL, &it, NULL );
    if ( err != 0 ) {
	    perror("setitimer");
	    exit(1);
    }
  }
  do {
    // Read the next data block from
    // the file if it's file input
    if ( isFileInput( mSettings ) ) {
      Extractor_getNextDataBlock( readAt, mSettings );
      canRead = Extractor_canRead( mSettings ) != 0;
    } else
      canRead = true;

    // perform write
    currLen = write( mSettings->mSock, mBuf, mSettings->mBufLen );
    if ( currLen < 0 ) {
      WARN_errno( currLen < 0, "write2" );
      break;
    }
    totLen += currLen;

    if(mSettings->mInterval > 0) {
      gettimeofday( &(reportstruct->packetTime), NULL );
      reportstruct->packetLen = currLen;
      ReportPacket( mSettings->reporthdr, reportstruct );
    }

    if ( !mMode_Time ) {
      if (mSettings->mAmount >= currLen) {
            mSettings->mAmount -= currLen;
      } else {
          mSettings->mAmount = 0;
      }
     
    }

  } while ( ! (sInterupted  ||
               (!mMode_Time  &&  0 >= mSettings->mAmount)) && canRead );

  // stop timing
  gettimeofday( &(reportstruct->packetTime), NULL );

  // if we're not doing interval reporting, report the entire transfer as one big packet
  if(0.0 == mSettings->mInterval) {
    reportstruct->packetLen = totLen;
    ReportPacket( mSettings->reporthdr, reportstruct );
  }
  CloseReport( mSettings->reporthdr, reportstruct );

  DELETE_PTR( reportstruct );
  EndReport( mSettings->reporthdr );
}

/* Send data through a TCP or UDP connection. If TCP, will call data_sendtcp */
void data_send(thread_Settings * mSettings, char * mBuf, Timestamp mEndTime)
{
  Timestamp lastPacketTime;
  struct UDP_datagram* mBuf_UDP = (struct UDP_datagram*) mBuf;
  long currLen = 0;

  int delay_target = 0;
  int delay = 0;
  int adjust = 0;

  char* readAt = mBuf;

#if HAVE_THREAD
  if ( !isUDP( mSettings ) ) {
    data_sendtcp(mSettings, mBuf, mEndTime);
    return;
  }
#endif

  // Indicates if the stream is readable
  bool canRead = true, mMode_Time = isModeTime( mSettings );

  // setup termination variables
  if ( mMode_Time ) {
    mEndTime.setnow();
    mEndTime.add( mSettings->mAmount / 100.0 );
  }

  if ( isUDP( mSettings ) ) {
    // Due to the UDP timestamps etc, included
    // reduce the read size by an amount
    // equal to the header size

    // compute delay for bandwidth restriction, constrained to [0,1] seconds
    delay_target = (int) ( mSettings->mBufLen * ((kSecs_to_usecs * kBytes_to_Bits)
                                                 / mSettings->mUDPRate) );
    if ( delay_target < 0  ||
         delay_target > (int) 1 * kSecs_to_usecs ) {
      fprintf( stderr, warn_delay_large, delay_target / kSecs_to_usecs );
      delay_target = (int) kSecs_to_usecs * 1;
    }
    if ( isFileInput( mSettings ) ) {
      if ( isCompat( mSettings ) ) {
        Extractor_reduceReadSize( sizeof(struct UDP_datagram), mSettings );
        readAt += sizeof(struct UDP_datagram);
      } else {
        Extractor_reduceReadSize( sizeof(struct UDP_datagram) +
                                  sizeof(struct client_hdr), mSettings );
        readAt += sizeof(struct UDP_datagram) +
                  sizeof(struct client_hdr);
      }
    }
  }

  ReportStruct *reportstruct = NULL;

  // InitReport handles Barrier for multiple Streams
  mSettings->reporthdr = InitReport( mSettings );
  reportstruct = new ReportStruct;
  reportstruct->packetID = 0;

  lastPacketTime.setnow();

  do {

    // Test case: drop 17 packets and send 2 out-of-order:
    // sequence 51, 52, 70, 53, 54, 71, 72
    //switch( datagramID ) {
    //  case 53: datagramID = 70; break;
    //  case 71: datagramID = 53; break;
    //  case 55: datagramID = 71; break;
    //  default: break;
    //}
    gettimeofday( &(reportstruct->packetTime), NULL );

    if ( isUDP( mSettings ) ) {
      // store datagram ID into buffer
      mBuf_UDP->id      = htonl( (reportstruct->packetID)++ );
      mBuf_UDP->tv_sec  = htonl( reportstruct->packetTime.tv_sec );
      mBuf_UDP->tv_usec = htonl( reportstruct->packetTime.tv_usec );

      // delay between writes
      // make an adjustment for how long the last loop iteration took
      // TODO this doesn't work well in certain cases, like 2 parallel streams
      adjust = delay_target + lastPacketTime.subUsec( reportstruct->packetTime );
      lastPacketTime.set( reportstruct->packetTime.tv_sec,
                          reportstruct->packetTime.tv_usec );

      if ( adjust > 0  ||  delay > 0 ) {
        delay += adjust;
      }
    }

    // Read the next data block from
    // the file if it's file input
    if ( isFileInput( mSettings ) ) {
      Extractor_getNextDataBlock( readAt, mSettings );
      canRead = Extractor_canRead( mSettings ) != 0;
    } else
      canRead = true;

    // perform write
    currLen = write( mSettings->mSock, mBuf, mSettings->mBufLen );
    if ( currLen < 0 && errno != ENOBUFS ) {
      WARN_errno( currLen < 0, "write2" );
      break;
    }

    // report packets
    reportstruct->packetLen = currLen;
    ReportPacket( mSettings->reporthdr, reportstruct );

    if ( delay > 0 ) {
      delay_loop( delay );
    }
    if ( !mMode_Time ) {
      mSettings->mAmount -= currLen;
    }

  } while ( ! (sInterupted  ||
               (mMode_Time   &&  mEndTime.before( reportstruct->packetTime ))  ||
               (!mMode_Time  &&  0 >= mSettings->mAmount)) && canRead );

  // stop timing
  gettimeofday( &(reportstruct->packetTime), NULL );
  CloseReport( mSettings->reporthdr, reportstruct );

  if ( isUDP( mSettings ) ) {
    // send a final terminating datagram
    // Don't count in the mTotalLen. The server counts this one,
    // but didn't count our first datagram, so we're even now.
    // The negative datagram ID signifies termination to the server.

    // store datagram ID into buffer
    mBuf_UDP->id      = htonl( -(reportstruct->packetID)  );
    mBuf_UDP->tv_sec  = htonl( reportstruct->packetTime.tv_sec );
    mBuf_UDP->tv_usec = htonl( reportstruct->packetTime.tv_usec );

    if ( isMulticast( mSettings ) ) {
      write( mSettings->mSock, mBuf, mSettings->mBufLen );
    } else {
      write_UDP_FIN(mSettings, mBuf);
    }
  }
  DELETE_PTR( reportstruct );
  EndReport( mSettings->reporthdr );

}

/* -------------------------------------------------------------------
 * Send a datagram on the socket. The datagram's contents should signify
 * a FIN to the application. Keep re-transmitting until an
 * acknowledgement datagram is received.
 * ------------------------------------------------------------------- */

void write_UDP_FIN(thread_Settings * mSettings, char * mBuf) {
  int rc;
  fd_set readSet;
  struct timeval timeout;

  int count = 0;
  while ( count < 10 ) {
    count++;

    // write data
    write( mSettings->mSock, mBuf, mSettings->mBufLen );

    // wait until the socket is readable, or our timeout expires
    FD_ZERO( &readSet );
    FD_SET( mSettings->mSock, &readSet );
    timeout.tv_sec  = 0;
    timeout.tv_usec = 250000; // quarter second, 250 ms

    rc = select( mSettings->mSock+1, &readSet, NULL, NULL, &timeout );
    FAIL_errno( rc == SOCKET_ERROR, "select", mSettings );

    if ( rc == 0 ) {
      // select timed out
      continue;
    } else {
      // socket ready to read
      rc = read( mSettings->mSock, mBuf, mSettings->mBufLen );
      WARN_errno( rc < 0, "read" );
      if ( rc < 0 ) {
        break;
      } else if ( rc >= (int) (sizeof(UDP_datagram) + sizeof(server_hdr)) ) {
        ReportServerUDP( mSettings, (server_hdr*) ((UDP_datagram*)mBuf + 1) );
      }

      return;
    }
  }

  fprintf( stderr, warn_no_ack, mSettings->mSock, count );
}
// end write_UDP_FIN

void data_recv(thread_Settings * mSettings, char * mBuf, Timestamp mEndTime)
{
  long currLen;
  max_size_t totLen = 0;
  struct UDP_datagram* mBuf_UDP  = (struct UDP_datagram*) mBuf;

  ReportStruct *reportstruct = NULL;

  reportstruct = new ReportStruct;
  if ( reportstruct != NULL ) {
    reportstruct->packetID = 0;
    mSettings->reporthdr = InitReport( mSettings );
    do {
      // perform read
      currLen = recv( mSettings->mSock, mBuf, mSettings->mBufLen, 0 );

      if ( isUDP( mSettings ) ) {
        // read the datagram ID and sentTime out of the buffer
        reportstruct->packetID = ntohl( mBuf_UDP->id );
        reportstruct->sentTime.tv_sec = ntohl( mBuf_UDP->tv_sec  );
        reportstruct->sentTime.tv_usec = ntohl( mBuf_UDP->tv_usec );
        reportstruct->packetLen = currLen;
        gettimeofday( &(reportstruct->packetTime), NULL );
      } else {
        totLen += currLen;
	    }

      // terminate when datagram begins with negative index
      // the datagram ID should be correct, just negated
      if ( reportstruct->packetID < 0 ) {
        reportstruct->packetID = -reportstruct->packetID;
        currLen = -1;
      }
	    if ( isUDP (mSettings))
        ReportPacket( mSettings->reporthdr, reportstruct );
    } while ( currLen > 0 );

    // stop timing
    gettimeofday( &(reportstruct->packetTime), NULL );
    if ( !isUDP (mSettings)) {
      reportstruct->packetLen = totLen;
      ReportPacket( mSettings->reporthdr, reportstruct );
    }
    CloseReport( mSettings->reporthdr, reportstruct );

    // send a acknowledgement back only if we're NOT receiving multicast
    if ( isUDP( mSettings ) && !isMulticast( mSettings ) ) {
      // send back an acknowledgement of the terminating datagram
      write_UDP_AckFIN(mSettings, mBuf);
    }
  } else {
    FAIL(1, "Out of memory! Closing server thread\n", mSettings);
  }

  Mutex_Lock( &clients_mutex );
  Iperf_delete( &(mSettings->peer), &clients );
  Mutex_Unlock( &clients_mutex );

  DELETE_PTR( reportstruct );
  EndReport( mSettings->reporthdr );
}

/* -------------------------------------------------------------------
 * Send an AckFIN (a datagram acknowledging a FIN) on the socket,
 * then select on the socket for some time. If additional datagrams
 * come in, probably our AckFIN was lost and they are re-transmitted
 * termination datagrams, so re-transmit our AckFIN.
 * ------------------------------------------------------------------- */
void write_UDP_AckFIN(thread_Settings * mSettings, char * mBuf)
{
  int rc;

  fd_set readSet;
  FD_ZERO( &readSet );

  struct timeval timeout;

  int count = 0;
  while ( count < 10 ) {
    count++;

    UDP_datagram *UDP_Hdr;
    server_hdr *hdr;

    UDP_Hdr = (UDP_datagram*) mBuf;

    if ( mSettings->mBufLen > (int) ( sizeof( UDP_datagram )
                                      + sizeof( server_hdr ) ) ) {
      Transfer_Info *stats = GetReport( mSettings->reporthdr );
      hdr = (server_hdr*) (UDP_Hdr+1);

      hdr->flags        = htonl( HEADER_VERSION1 );
      hdr->total_len1   = htonl( (long) (stats->TotalLen >> 32) );
      hdr->total_len2   = htonl( (long) (stats->TotalLen & 0xFFFFFFFF) );
      hdr->stop_sec     = htonl( (long) stats->endTime );
      hdr->stop_usec    = htonl( (long)((stats->endTime - (long)stats->endTime)
                                        * rMillion));
      hdr->error_cnt    = htonl( stats->cntError );
      hdr->outorder_cnt = htonl( stats->cntOutofOrder );
      hdr->datagrams    = htonl( stats->cntDatagrams );
      hdr->jitter1      = htonl( (long) stats->jitter );
      hdr->jitter2      = htonl( (long) ((stats->jitter - (long)stats->jitter)
                                         * rMillion) );

    }

    // write data
    write( mSettings->mSock, mBuf, mSettings->mBufLen );

    // wait until the socket is readable, or our timeout expires
    FD_SET( mSettings->mSock, &readSet );
    timeout.tv_sec  = 1;
    timeout.tv_usec = 0;

    rc = select( mSettings->mSock+1, &readSet, NULL, NULL, &timeout );
    FAIL_errno( rc == SOCKET_ERROR, "select", mSettings );

    if ( rc == 0 ) {
      // select timed out
      return;
    } else {
      // socket ready to read
      rc = read( mSettings->mSock, mBuf, mSettings->mBufLen );
      WARN_errno( rc < 0, "read" );
      if ( rc <= 0 ) {
        // Connection closed or errored
        // Stop using it.
        return;
      }
    }
  }

  fprintf( stderr, warn_ack_failed, mSettings->mSock, count );
}
// end write_UDP_AckFIN
