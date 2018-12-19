/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

/** \file
 * \brief
 * EtherCAT RAW socket driver.
 *
 * Low level interface functions to send and receive EtherCAT packets.
 * EtherCAT has the property that packets are only send by the master,
 * and the send packets always return in the receive buffer.
 * There can be multiple packets "on the wire" before they return.
 * To combine the received packets with the original send packets a buffer
 * system is installed. The identifier is put in the index item of the
 * EtherCAT header. The index is stored and compared when a frame is received.
 * If there is a match the packet can be combined with the transmit packet
 * and returned to the higher level function.
 *
 * The socket layer can exhibit a reversal in the packet order (rare).
 * If the Tx order is A-B-C the return order could be A-C-B. The indexed buffer
 * will reorder the packets automatically.
 *
 * The "redundant" option will configure two sockets and two NIC interfaces.
 * Slaves are connected to both interfaces, one on the IN port and one on the
 * OUT port. Packets are send via both interfaces. Any one of the connections
 * (also an interconnect) can be removed and the slaves are still serviced with
 * packets. The software layer will detect the possible failure modes and
 * compensate. If needed the packets from interface A are resent through interface B.
 * This layer is fully transparent for the higher layers.
 */


#include <stdio.h>
#include <string.h>
#include "ethercat.h"
#include "nicdrv.h"
#include "osal_win32.h"
#include "eth.h"
#include "function.h"

/** Redundancy modes */
enum
{
    /** No redundancy, single NIC mode */
    ECT_RED_NONE,
    /** Double redundant NIC connecetion */
    ECT_RED_DOUBLE
};

/** Primary source MAC address used for EtherCAT.
 * This address is not the MAC address used from the NIC.
 * EtherCAT does not care about MAC addressing, but it is used here to
 * differentiate the route the packet traverses through the EtherCAT
 * segment. This is needed to fund out the packet flow in redundant
 * confihurations. */
const uint16 priMAC[3] = { 0x0101, 0x0101, 0x0101 };
/** Secondary source MAC address used for EtherCAT. */
const uint16 secMAC[3] = { 0x0404, 0x0404, 0x0404 };

/** second MAC word is used for identification */
#define RX_PRIM priMAC[1]
/** second MAC word is used for identification */
#define RX_SEC secMAC[1]

//static char errbuf[256];

static void nexx_clear_rxbufstat(int *rxbufstat)
{
   int i;
   for(i = 0; i < NEX_MAXBUF; i++)
   {
      rxbufstat[i] = NEX_BUF_EMPTY;
   }
}

/** Basic setup to connect NIC to socket.
 * @param[in] port        = port context struct
 * @param[in] ifname       = Name of NIC device, f.e. "eth0"
 * @param[in] secondary      = if >0 then use secondary stack instead of primary
 * @return >0 if succeeded
 */
int nexx_setupnic(nexx_portt *port)
{
	int i;
//   int i, rval;
//   pcap_t **psock;

//   rval = 0;
//   InitializeCriticalSection(&(port->getindex_mutex));
//   InitializeCriticalSection(&(port->tx_mutex));
//   InitializeCriticalSection(&(port->rx_mutex));
//   port->sockhandle        = NULL;
   port->lastidx           = 0;
   port->redstate          = ECT_RED_NONE;
//   port->stack.sock        = &(port->sockhandle);
   port->stack.txbuf       = &(port->txbuf);
   port->stack.txbuflength = &(port->txbuflength);
   port->stack.tempbuf     = &(port->tempinbuf);
   port->stack.rxbuf       = &(port->rxbuf);
   port->stack.rxbufstat   = &(port->rxbufstat);
   port->stack.rxsa        = &(port->rxsa);
   nexx_clear_rxbufstat(&(port->rxbufstat[0]));
//   psock = &(port->sockhandle);
   
   /* we use pcap socket to send RAW packets in windows user mode*/
//   *psock = pcap_open(ifname, 65536, PCAP_OPENFLAG_PROMISCUOUS |
//                                     PCAP_OPENFLAG_MAX_RESPONSIVENESS |
//                                     PCAP_OPENFLAG_NOCAPTURE_LOCAL, -1, NULL , errbuf);
//   if (NULL == *psock)
//   {
//     printf("interface %s could not open with pcap\n", ifname);
//     return 0;
//   }

    for (i = 0; i < NEX_MAXBUF; i++)
   {
      nex_setupheader(&(port->txbuf[i]));
      port->rxbufstat[i] = NEX_BUF_EMPTY;
   }
   nex_setupheader(&(port->txbuf2));

   return 1;
}

/** Close sockets used
 * @param[in] port        = port context struct
 * @return 0
 */
int nexx_closenic(nexx_portt *port)
{
//   timeEndPeriod(15);

//   if (port->sockhandle != NULL)
//   {
//      DeleteCriticalSection(&(port->getindex_mutex));
//      DeleteCriticalSection(&(port->tx_mutex));
//      DeleteCriticalSection(&(port->rx_mutex));
//      pcap_close(port->sockhandle);
//      port->sockhandle = NULL;
//   }
//   if ((port->redport) && (port->redport->sockhandle != NULL))
//   {
//      pcap_close(port->redport->sockhandle);
//      port->redport->sockhandle = NULL;
//   }

   return 0;
}

/** Fill buffer with ethernet header structure.
 * Destination MAC is always broadcast.
 * Ethertype is always ETH_P_ECAT.
 * @param[out] p = buffer
 */
void nex_setupheader(void *p)
{
   nex_etherheadert *bp;
   bp = p;
   bp->da0 = htons(0xffff);
   bp->da1 = htons(0xffff);
   bp->da2 = htons(0xffff);
   bp->sa0 = htons(priMAC[0]);
   bp->sa1 = htons(priMAC[1]);
   bp->sa2 = htons(priMAC[2]);
   bp->etype = htons(ETH_P_ECAT);
}

/** Get new frame identifier index and allocate corresponding rx buffer.
 * @param[in] port        = port context struct
 * @return new index.
 */
int nexx_getindex(nexx_portt *port)
{
   int idx;
   int cnt;

//   EnterCriticalSection(&(port->getindex_mutex));

   idx = port->lastidx + 1;
   /* index can't be larger than buffer array */
   if (idx >= NEX_MAXBUF)
   {
      idx = 0;
   }
   cnt = 0;
   /* try to find unused index */
   while ((port->rxbufstat[idx] != NEX_BUF_EMPTY) && (cnt < NEX_MAXBUF))
   {
      idx++;
      cnt++;
      if (idx >= NEX_MAXBUF)
      {
         idx = 0;
      }
   }
   port->rxbufstat[idx] = NEX_BUF_ALLOC;
   if (port->redstate != ECT_RED_NONE)
      port->redport->rxbufstat[idx] = NEX_BUF_ALLOC;
   port->lastidx = idx;

//   LeaveCriticalSection(&(port->getindex_mutex));

   return idx;
}

/** Set rx buffer status.
 * @param[in] port        = port context struct
 * @param[in] idx      = index in buffer array
 * @param[in] bufstat   = status to set
 */
void nexx_setbufstat(nexx_portt *port, int idx, int bufstat)
{
   port->rxbufstat[idx] = bufstat;
   if (port->redstate != ECT_RED_NONE)
      port->redport->rxbufstat[idx] = bufstat;
}

/** Transmit buffer over socket (non blocking).
 * @param[in] port        = port context struct
 * @param[in] idx      = index in tx buffer array
 * @param[in] stacknumber   = 0=Primary 1=Secondary stack
 * @return socket send result
 */
int nexx_outframe(nexx_portt *port, int idx, int stacknumber)
{
	debug_printf("program is here:%s\n",__FUNCTION__);
   int lp, rval;
   nex_stackT *stack;

   if (!stacknumber)
   {
      stack = &(port->stack);
   }
   else
   {
      stack = &(port->redport->stack);
   }
   lp = (*stack->txbuflength)[idx];
   (*stack->rxbufstat)[idx] = NEX_BUF_TX;
//   rval = stm_EMAC_send((*stack->txbuf)[idx],lp);
 //  rval = pcap_sendpacket(*stack->sock, (*stack->txbuf)[idx], lp);
   if (rval == HAL_ERROR)
   {
      (*stack->rxbufstat)[idx] = NEX_BUF_EMPTY;
   }

   return rval;
}

/** Transmit buffer over socket (non blocking).
 * @param[in] port        = port context struct
 * @param[in] idx      = index in tx buffer array
 * @return socket send result
 */
int nexx_outframe_red(nexx_portt *port, int idx)
{
	debug_printf("program is here:%s\n",__FUNCTION__);
   nex_comt *datagramP;
   nex_etherheadert *ehp;
   int rval;

   ehp = (nex_etherheadert *)&(port->txbuf[idx]);
   /* rewrite MAC source address 1 to primary */
   ehp->sa1 = htons(priMAC[1]);
   /* transmit over primary socket*/
   rval = nexx_outframe(port, idx, 0);
   if (port->redstate != ECT_RED_NONE)
   {
//      EnterCriticalSection( &(port->tx_mutex) );
      ehp = (nex_etherheadert *)&(port->txbuf2);
      /* use dummy frame for secondary socket transmit (BRD) */
      datagramP = (nex_comt*)&(port->txbuf2[ETH_HEADERSIZE]);
      /* write index to frame */
      datagramP->index = idx;
      /* rewrite MAC source address 1 to secondary */
      ehp->sa1 = htons(secMAC[1]);
      /* transmit over secondary socket */
      port->redport->rxbufstat[idx] = NEX_BUF_TX;
 //     if (pcap_sendpacket(port->redport->sockhandle, (u_char const *)&(port->txbuf2), port->txbuflength2) == HAL_ERROR)
//	if(stm_EMAC_send((unsigned char *)&(port->txbuf2),port->txbuflength2)==HAL_ERROR)
//	   {
//         port->redport->rxbufstat[idx] = NEX_BUF_EMPTY;
//      }
//      LeaveCriticalSection( &(port->tx_mutex) );
   }

   return rval;
}

/** Non blocking read of socket. Put frame in temporary buffer.
 * @param[in] port        = port context struct
 * @param[in] stacknumber = 0=primary 1=secondary stack
 * @return >0 if frame is available and read
 */
static int nexx_recvpkt(nexx_portt *port, int stacknumber)
{
   int lp, bytesrx;
   nex_stackT *stack;
//   struct pcap_pkthdr * header;
	uint32_t length;
   uint8 * pkt_data;
   int res;

   if (!stacknumber)
   {
      stack = &(port->stack);
   }
   else
   {
      stack = &(port->redport->stack);
   }
   lp = sizeof(port->tempinbuf);

//   res = pcap_next_ex(*stack->sock, &header, &pkt_data);
//   stm_EMAC_recv(pkt_data,length);
   if (res <=0 )
   {
     port->tempinbufs = 0;
      return 0;
   }
//   bytesrx = header->len;
   bytesrx = length;
   if (bytesrx > lp)
   {
      bytesrx = lp;
   }
   memcpy(*stack->tempbuf, pkt_data, bytesrx);
   port->tempinbufs = bytesrx;
   return (bytesrx > 0);
}

/** Non blocking receive frame function. Uses RX buffer and index to combine
 * read frame with transmitted frame. To compensate for received frames that
 * are out-of-order all frames are stored in their respective indexed buffer.
 * If a frame was placed in the buffer previously, the function retreives it
 * from that buffer index without calling nex_recvpkt. If the requested index
 * is not already in the buffer it calls nex_recvpkt to fetch it. There are
 * three options now, 1 no frame read, so exit. 2 frame read but other
 * than requested index, store in buffer and exit. 3 frame read with matching
 * index, store in buffer, set completed flag in buffer status and exit.
 *
 * @param[in] port        = port context struct
 * @param[in] idx         = requested index of frame
 * @param[in] stacknumber  = 0=primary 1=secondary stack
 * @return Workcounter if a frame is found with corresponding index, otherwise
 * NEX_NOFRAME or NEX_OTHERFRAME.
 */
int nexx_inframe(nexx_portt *port, int idx, int stacknumber)
{
   uint16  l;
   int     rval;
   int     idxf;
   nex_etherheadert *ehp;
   nex_comt *ecp;
   nex_stackT *stack;
   nex_bufT *rxbuf;

   if (!stacknumber)
   {
      stack = &(port->stack);
   }
   else
   {
      stack = &(port->redport->stack);
   }
   rval = NEX_NOFRAME;
   rxbuf = &(*stack->rxbuf)[idx];
   /* check if requested index is already in buffer ? */
   if ((idx < NEX_MAXBUF) && ((*stack->rxbufstat)[idx] == NEX_BUF_RCVD))
   {
      l = (*rxbuf)[0] + ((uint16)((*rxbuf)[1] & 0x0f) << 8);
      /* return WKC */
      rval = ((*rxbuf)[l] + ((uint16)(*rxbuf)[l + 1] << 8));
      /* mark as completed */
      (*stack->rxbufstat)[idx] = NEX_BUF_COMPLETE;
   }
   else
   {
//      EnterCriticalSection(&(port->rx_mutex));
      /* non blocking call to retrieve frame from socket */
      if (nexx_recvpkt(port, stacknumber))
      {
         rval = NEX_OTHERFRAME;
         ehp =(nex_etherheadert*)(stack->tempbuf);
         /* check if it is an EtherCAT frame */
         if (ehp->etype == htons(ETH_P_ECAT))
         {
            ecp =(nex_comt*)(&(*stack->tempbuf)[ETH_HEADERSIZE]);
            l = etohs(ecp->elength) & 0x0fff;
            idxf = ecp->index;
            /* found index equals reqested index ? */
            if (idxf == idx)
            {
               /* yes, put it in the buffer array (strip ethernet header) */
               memcpy(rxbuf, &(*stack->tempbuf)[ETH_HEADERSIZE], (*stack->txbuflength)[idx] - ETH_HEADERSIZE);
               /* return WKC */
               rval = ((*rxbuf)[l] + ((uint16)((*rxbuf)[l + 1]) << 8));
               /* mark as completed */
               (*stack->rxbufstat)[idx] = NEX_BUF_COMPLETE;
               /* store MAC source word 1 for redundant routing info */
               (*stack->rxsa)[idx] = ntohs(ehp->sa1);
            }
            else
            {
               /* check if index exist and someone is waiting for it */
               if (idxf < NEX_MAXBUF && (*stack->rxbufstat)[idxf] == NEX_BUF_TX)
               {
                  rxbuf = &(*stack->rxbuf)[idxf];
                  /* put it in the buffer array (strip ethernet header) */
                  memcpy(rxbuf, &(*stack->tempbuf)[ETH_HEADERSIZE], (*stack->txbuflength)[idxf] - ETH_HEADERSIZE);
                  /* mark as received */
                  (*stack->rxbufstat)[idxf] = NEX_BUF_RCVD;
                  (*stack->rxsa)[idxf] = ntohs(ehp->sa1);
               }
               else
               {
                  /* strange things happend */
               }
            }
         }
      }
 //     LeaveCriticalSection( &(port->rx_mutex) );

   }

   /* WKC if mathing frame found */
   return rval;
}

/** Blocking redundant receive frame function. If redundant mode is not active then
 * it skips the secondary stack and redundancy functions. In redundant mode it waits
 * for both (primary and secondary) frames to come in. The result goes in an decision
 * tree that decides, depending on the route of the packet and its possible missing arrival,
 * how to reroute the original packet to get the data in an other try.
 *
 * @param[in] port        = port context struct
 * @param[in] idx = requested index of frame
 * @param[in] tvs = timeout
 * @return Workcounter if a frame is found with corresponding index, otherwise
 * NEX_NOFRAME.
 */
static int nexx_waitinframe_red(nexx_portt *port, int idx, osal_timert *timer)
{
   osal_timert timer2;
   int wkc  = NEX_NOFRAME;
   int wkc2 = NEX_NOFRAME;
   int primrx, secrx;

   /* if not in redundant mode then always assume secondary is OK */
   if (port->redstate == ECT_RED_NONE)
      wkc2 = 0;
   do
   {
      /* only read frame if not already in */
      if (wkc <= NEX_NOFRAME)
         wkc  = nexx_inframe(port, idx, 0);
      /* only try secondary if in redundant mode */
      if (port->redstate != ECT_RED_NONE)
      {
         /* only read frame if not already in */
         if (wkc2 <= NEX_NOFRAME)
            wkc2 = nexx_inframe(port, idx, 1);
      }
   /* wait for both frames to arrive or timeout */
   } while (((wkc <= NEX_NOFRAME) || (wkc2 <= NEX_NOFRAME)) && !osal_timer_is_expired(timer));
   /* only do redundant functions when in redundant mode */
   if (port->redstate != ECT_RED_NONE)
   {
      /* primrx if the reveived MAC source on primary socket */
      primrx = 0;
      if (wkc > NEX_NOFRAME) primrx = port->rxsa[idx];
      /* secrx if the reveived MAC source on psecondary socket */
      secrx = 0;
      if (wkc2 > NEX_NOFRAME) secrx = port->redport->rxsa[idx];

      /* primary socket got secondary frame and secondary socket got primary frame */
      /* normal situation in redundant mode */
      if ( ((primrx == RX_SEC) && (secrx == RX_PRIM)) )
      {
         /* copy secondary buffer to primary */
         memcpy(&(port->rxbuf[idx]), &(port->redport->rxbuf[idx]), port->txbuflength[idx] - ETH_HEADERSIZE);
         wkc = wkc2;
      }
      /* primary socket got nothing or primary frame, and secondary socket got secondary frame */
      /* we need to resend TX packet */
      if ( ((primrx == 0) && (secrx == RX_SEC)) ||
           ((primrx == RX_PRIM) && (secrx == RX_SEC)) )
      {
         /* If both primary and secondary have partial connection retransmit the primary received
          * frame over the secondary socket. The result from the secondary received frame is a combined
          * frame that traversed all slaves in standard order. */
         if ( (primrx == RX_PRIM) && (secrx == RX_SEC) )
         {
            /* copy primary rx to tx buffer */
            memcpy(&(port->txbuf[idx][ETH_HEADERSIZE]), &(port->rxbuf[idx]), port->txbuflength[idx] - ETH_HEADERSIZE);
         }
         osal_timer_start (&timer2, NEX_TIMEOUTRET);
         /* resend secondary tx */
         nexx_outframe(port, idx, 1);
         do
         {
            /* retrieve frame */
            wkc2 = nexx_inframe(port, idx, 1);
         } while ((wkc2 <= NEX_NOFRAME) && !osal_timer_is_expired(&timer2));
         if (wkc2 > NEX_NOFRAME)
         {
            /* copy secondary result to primary rx buffer */
            memcpy(&(port->rxbuf[idx]), &(port->redport->rxbuf[idx]), port->txbuflength[idx] - ETH_HEADERSIZE);
            wkc = wkc2;
         }
      }
   }

   /* return WKC or NEX_NOFRAME */
   return wkc;
}

/** Blocking receive frame function. Calls nex_waitinframe_red().
 * @param[in] port        = port context struct
 * @param[in] idx = requested index of frame
 * @param[in] timeout = timeout in us
 * @return Workcounter if a frame is found with corresponding index, otherwise
 * NEX_NOFRAME.
 */
int nexx_waitinframe(nexx_portt *port, int idx, int timeout)
{
   int wkc;
   osal_timert timer;

   osal_timer_start (&timer, timeout);
   wkc = nexx_waitinframe_red(port, idx, &timer);

   return wkc;
}

/** Blocking send and recieve frame function. Used for non processdata frames.
 * A datagram is build into a frame and transmitted via this function. It waits
 * for an answer and returns the workcounter. The function retries if time is
 * left and the result is WKC=0 or no frame received.
 *
 * The function calls nex_outframe_red() and nex_waitinframe_red().
 *
 * @param[in] port        = port context struct
 * @param[in] idx      = index of frame
 * @param[in] timeout  = timeout in us
 * @return Workcounter or NEX_NOFRAME
 */
int nexx_srconfirm(nexx_portt *port, int idx, int timeout)
{
	debug_printf("program is here:%s\n",__FUNCTION__);
   int wkc = NEX_NOFRAME;
   osal_timert timer1, timer2;

   osal_timer_start (&timer1, timeout);
   do
   {
      /* tx frame on primary and if in redundant mode a dummy on secondary */
      nexx_outframe_red(port, idx);
      if (timeout < NEX_TIMEOUTRET)
      {
         osal_timer_start (&timer2, timeout);
      }
      else
      {
         /* normally use partial timout for rx */
         osal_timer_start (&timer2, NEX_TIMEOUTRET);
      }
      /* get frame from primary or if in redundant mode possibly from secondary */
      wkc = nexx_waitinframe_red(port, idx, &timer2);
   /* wait for answer with WKC>=0 or otherwise retry until timeout */
   } while ((wkc <= NEX_NOFRAME) && !osal_timer_is_expired (&timer1));
   /* if nothing received, clear buffer index status so it can be used again */
   if (wkc <= NEX_NOFRAME)
   {
      nexx_setbufstat(port, idx, NEX_BUF_EMPTY);
   }

   return wkc;
}


#ifdef NEX_VER1

int nex_setupnic()
{
   return nexx_setupnic(&nexx_port);
}

int nex_closenic(void)
{
   return nexx_closenic(&nexx_port);
}

int nex_getindex(void)
{
   return nexx_getindex(&nexx_port);
}

void nex_setbufstat(int idx, int bufstat)
{
   nexx_setbufstat(&nexx_port, idx, bufstat);
}

int nex_outframe(int idx, int stacknumber)
{
   return nexx_outframe(&nexx_port, idx, stacknumber);
}

int nex_outframe_red(int idx)
{
   return nexx_outframe_red(&nexx_port, idx);
}

int nex_inframe(int idx, int stacknumber)
{
   return nexx_inframe(&nexx_port, idx, stacknumber);
}

int nex_waitinframe(int idx, int timeout)
{
   return nexx_waitinframe(&nexx_port, idx, timeout);
}

int nex_srconfirm(int idx, int timeout)
{
   return nexx_srconfirm(&nexx_port, idx, timeout);
}

#endif
