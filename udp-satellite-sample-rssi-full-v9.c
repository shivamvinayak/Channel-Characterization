/*
#define PRINTF("PARENT ETX IS %d \n", etx);
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "dev/leds.h"
#include "dev/cc2420.h"
#include <string.h>
#include "net/uip-debug.h"
#include "cfs/cfs.h"
#include "cfs/cfs-coffee.h" 

#define DEBUG DEBUG_PRINT
#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define SEND_INTERVAL    0.025* CLOCK_SECOND
#define SEND_INTERVAL_2  0.5*CLOCK_SECOND
#define SEND_INTERVAL_3  1*CLOCK_SECOND
#define MAX_PAYLOAD_LEN  120
#define MAX_NEI          28

static process_event_t transmit_ll_event;
static struct uip_udp_conn *server_conn;
static struct uip_udp_conn *client_conn;
static struct uip_udp_conn *link_learn_conn;
static struct etimer et;

int link_learn_var;
int send_data_var;
int link_learn_done;
int send_data_done;
int mote_id;
int non_zero_nei;
static int retran;
static int no_of_chunks=0;
int pkt_count = 0;
static int k = 0;
static int i = 0;
static int j = 0;
long total_rssi[MAX_NEI]; 
int no_of_pkts_rxd[MAX_NEI];
int pkts_rxd_otg[MAX_NEI]; 
int rssi;

int rec_seq;
int max_pkts;
uint8_t rxd_node_id;
int timer1;
int fd; //for cfs file systems
int stat; // for cfs file status
char fname[5]; // CFS file name
unsigned char rx_strm_buf[1500];
int write_lock = 0 ;

int nebr;
int pos;

typedef struct config_pkt {
	uint8_t type;
	uint8_t node_id;
	uint16_t numofpkts;
} __attribute((packed)) config_pkt;

typedef struct my_mote_id {
    uint8_t type;
	uint8_t mote_id;
	uint16_t seq_no;
} __attribute((packed)) my_mote;
my_mote my_mote_id = {0};

struct neighbor_list {
	uint8_t node_id;
	short avg_rssi;
	short rxd_ll_pkts;
	short rxd_otg_pkts;
} __attribute((packed)) neighbor_in_list[MAX_NEI];

struct neighbor_rx_str {
	uint8_t type;
	char frag;
	short node_id;
	short mymote_id;
	char rx_strm[101];
} __attribute((packed)) neighbor_rx_str;

const unsigned short RESET_PKT = 59;	// reset 
const unsigned short START_LINK_LEARNING_PKT = 63;	// power and num of TX pkts with start LL command
const unsigned short CONFIG_PKT = 51;	// power and no. of pkts info for routing
const unsigned short SEND_LL_LIST = 15;
const unsigned short SEND_PKT_PART = 73
const unsigned short HELLO_PKT = 200;
const unsigned short DATA_PKT = 201;

uip_ipaddr_t ipaddr_dest;
uip_ipaddr_t ipaddr_next;

PROCESS(udp_server_process, "UDP server process");
AUTOSTART_PROCESSES(&udp_server_process);
/*---------------------------------------------------------------------------*/
void set_connection_address(uip_ipaddr_t * ipaddr)
{
	uip_ip6addr(ipaddr, 0xff01, 0, 0, 0, 0, 0, 0, 0x0001);
	uip_ds6_maddr_add(&ipaddr);

}


timeout_handler_multicast(int sequ)
{
	static char buf[40];
	sequ = sequ+1;
	my_mote_id.seq_no = sequ;
	sprintf(buf, "%d", my_mote_id.mote_id);
	#if SEND_TOO_LARGE_PACKET_TO_TEST_FRAGMENTATION
	uip_udp_packet_send(link_learn_conn, &my_mote_id, sizeof(my_mote_id));
	#else /* SEND_TOO_LARGE_PACKET_TO_TEST_FRAGMENTATION */
	uip_udp_packet_send(link_learn_conn, &my_mote_id, sizeof(my_mote_id));
	#endif /* SEND_TOO_LARGE_PACKET_TO_TEST_FRAGMENTATION */

}

/*-------------------------------------------------------------------------*/
static void tcpip_handler(void)
{
	if (uip_newdata()) {
		((char *)uip_appdata)[uip_datalen()] = 0;
		config_pkt *config = (config_pkt *) uip_appdata;
		
		if (config->type == HELLO_PKT) {
			my_mote *rxd_mote_id_struct = (my_mote *) uip_appdata;
			rxd_node_id = rxd_mote_id_struct->mote_id;
			if(rxd_node_id == 0){
				return;
			}
			
			for (i = 0; i < MAX_NEI; i++) {
				if (neighbor_in_list[i].node_id == rxd_node_id) {
					no_of_pkts_rxd[i] = no_of_pkts_rxd[i] + 1;
					rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI)-45;
										
					rec_seq = rxd_mote_id_struct->seq_no;
					rx_strm_buf[rec_seq-1] = rssi;
					
					if (rssi<-88)
						pkts_rxd_otg[i] = pkts_rxd_otg[i] + 1;
						
					//printf("Total %d packets received from the node with ID %d \n",no_of_pkts_rxd[i],neighbor_in_list[i].node_id);
					//PRINT6ADDR(&neighbor_in_list[i].n_ip_addr);
					total_rssi[i] = total_rssi[i] + (rssi);
					neighbor_in_list[i].avg_rssi = (total_rssi[i] / no_of_pkts_rxd[i]);
					neighbor_in_list[i].rxd_ll_pkts = no_of_pkts_rxd[i];
					neighbor_in_list[i].rxd_otg_pkts = pkts_rxd_otg[i];

					printf("Total Recived %d out of %d at index %d with RSSI %d and in outage : %d fro nei %d\n",neighbor_in_list[i].rxd_ll_pkts,rec_seq,i,rssi,neighbor_in_list[i].rxd_otg_pkts,rxd_node_id);
					//printf("Troubleshoot Pkt no. %d, %ld is Dividen and %d is Divisor\n",rec_seq,total_rssi[i],no_of_pkts_rxd[i]);
					break;
				
				} else if (neighbor_in_list[i].node_id == 0) {
					printf("New neighbour is being added :%d at index %d\n",rxd_node_id,i);
					neighbor_in_list[i].node_id = rxd_node_id;
					no_of_pkts_rxd[i] = no_of_pkts_rxd[i] + 1;
					//memset(neighbor_in_list[i].rxd_ll_strm, 0, sizeof neighbor_in_list[i].rxd_ll_strm);
					//if(strlen(neighbor_in_list[i].rxd_ll_strm) > 0)
					//strcpy(neighbor_in_list[i].rxd_ll_strm,"");
					rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI)-45;
                	
					rec_seq = rxd_mote_id_struct->seq_no;
											
					if(i>0){
					
						snprintf(fname,sizeof(fname), "%d", neighbor_in_list[i-1].node_id);
						fd = cfs_open(fname, CFS_WRITE | CFS_APPEND);
						if(fd != -1) {
							rx_strm_buf[sizeof(rx_strm_buf) - 1] = '\0';
							stat = cfs_write(fd, rx_strm_buf, sizeof(rx_strm_buf));
							cfs_close(fd);
							printf("successfully written data to cfs. wrote %i bytes\n",stat);
						} else {
							printf("ERROR: could not write to memory in for neighbor %d\n",neighbor_in_list[i-1].node_id);
						}
					}
				
					
					memset(rx_strm_buf, 0, sizeof(rx_strm_buf));
									
					rx_strm_buf[rec_seq-1] = rssi;
					
					if (rssi<-88)
						pkts_rxd_otg[i] = pkts_rxd_otg[i] + 1;
						
									
					//printf("RSSI of this packet is %d \n",rssi);
					total_rssi[i] = total_rssi[i] + (rssi);
					neighbor_in_list[i].avg_rssi = (total_rssi[i] / no_of_pkts_rxd[i]);			
					neighbor_in_list[i].rxd_ll_pkts = no_of_pkts_rxd[i];
					neighbor_in_list[i].rxd_otg_pkts = pkts_rxd_otg[i];
					printf("Total Recived %d out of %d at index %d with RSSI %d and in outage : %d\n",neighbor_in_list[i].rxd_ll_pkts,rec_seq,i,rssi,rssi/*neighbor_in_list[i].rxd_otg_pkts*/);
					
					break;
				}
			
			}
			return;

		}else if (config->type == START_LINK_LEARNING_PKT) {
			max_pkts = config->numofpkts;
		 	if(link_learn_var == 0)
				link_learn_var = 1;
		 	uip_ipaddr_copy(&server_conn->ripaddr,&UIP_IP_BUF->srcipaddr);
 		 	uip_udp_packet_send(server_conn, neighbor_in_list,sizeof(neighbor_in_list[0]));
		 	return;
	
		}else if (config->type == RESET_PKT) {
			printf("\n Received RESET command....\n");
			for (k = 0; k < MAX_NEI; k++) {
				no_of_pkts_rxd[k] = 0;
				total_rssi[k] = 0;
				pkts_rxd_otg[k] = 0;
				neighbor_in_list[k].rxd_ll_pkts = 0;
				neighbor_in_list[k].avg_rssi = 0;
				neighbor_in_list[k].rxd_otg_pkts =0;
				//memset(neighbor_in_list[k].rxd_ll_strm,'\0',sizeof(neighbor_in_list[k].rxd_ll_strm));
				//snprintf(fname,sizeof(fname), "%d", neighbor_in_list[k].node_id);					
			}
			write_lock = 0;
			memset(rx_strm_buf, 0, sizeof(rx_strm_buf));
			memset(neighbor_rx_str.rx_strm,'\0',sizeof(neighbor_rx_str.rx_strm));
			memset(neighbor_in_list, 0, sizeof(neighbor_in_list));
			if (my_mote_id.mote_id == 0 && config->node_id != 0){
				printf("Confi with moteid %d\n",config->node_id);
				my_mote_id.mote_id = config->node_id;
				my_mote_id.type = HELLO_PKT;
				
			}
			cfs_coffee_format();
			neighbor_rx_str.type = DATA_PKT;
			neighbor_rx_str.mymote_id = config->node_id;
			uip_ipaddr_copy(&server_conn->ripaddr,&UIP_IP_BUF->srcipaddr);
			uip_udp_packet_send(server_conn, neighbor_in_list,sizeof(neighbor_in_list[0]));
			return;
			
		}else if (config->type == SEND_LL_LIST) {
			/*Setting the variable for link learning */
			printf("\n Received SEND LL LIST command....\n");
			if(write_lock==0){
				write_lock++;
				snprintf(fname,sizeof(fname), "%d", neighbor_in_list[i].node_id);
				fd = cfs_open(fname, CFS_WRITE | CFS_APPEND);
				if(fd != -1) {
					rx_strm_buf[sizeof(rx_strm_buf)-1] = '\0';
					stat = cfs_write(fd, rx_strm_buf, sizeof(rx_strm_buf));
					cfs_close(fd);
					printf("successfully written data to cfs. wrote %i bytes at last index %d for %d\n",stat,i,neighbor_in_list[i].node_id);
				} else {
					printf("ERROR: could not write to memory in for neighbor %d\n",neighbor_in_list[i].node_id);
				}
			}
			uip_ipaddr_copy(&server_conn->ripaddr,&UIP_IP_BUF->srcipaddr);
			uip_udp_packet_send(server_conn, neighbor_in_list,sizeof(neighbor_in_list[0]));

			for (i = 0; i <= MAX_NEI; i = i + 1) {
				if (neighbor_in_list[i].node_id == 0) {
					break;
				}
			} /***Now i is the number of non-zero neighbours*/
			non_zero_nei = i;
			process_post(&udp_server_process, transmit_ll_event,NULL);
			
			/*failsafe for RESET
			for (k = 0; k < MAX_NEI; k++) {
				no_of_pkts_rxd[k] = 0;
				pkts_rxd_otg[k] = 0;
				total_rssi[k] = 0;
				neighbor_in_list[k].rxd_ll_pkts = 0;
				neighbor_in_list[k].avg_rssi = 0;
				neighbor_in_list[k].rxd_otg_pkts =0;
				//memset(neighbor_in_list[k].rxd_ll_strm,'\0',sizeof(neighbor_in_list[k].rxd_ll_strm));
				snprintf(fname,sizeof(fname), "%d", neighbor_in_list[k].node_id);
				cfs_remove(fname);								
			}
			memset(rx_strm_buf, 0, sizeof(rx_strm_buf));
			memset(neighbor_rx_str.rx_strm,'\0',sizeof(neighbor_rx_str.rx_strm));
			memset(neighbor_in_list, 0, sizeof(neighbor_in_list));
			if (my_mote_id.mote_id == 0 && config->node_id != 0){
				my_mote_id.mote_id = config->node_id;
				my_mote_id.type = HELLO_PKT;
				
			}
			neighbor_rx_str.type = DATA_PKT;
			neighbor_rx_str.mymote_id = config->node_id;*/
			return;
		}else if (config->type == SEND_PKT_PART) {
			
            nebr = config->node_id;
            pos = config->numofpkts;
			
			uip_ipaddr_copy(&server_conn->ripaddr,&UIP_IP_BUF->srcipaddr);
			uip_udp_packet_send(server_conn, neighbor_in_list,sizeof(neighbor_in_list[0]));
			
			
			
			for (i = 0; i <= MAX_NEI; i = i + 1) {
				if (neighbor_in_list[i].node_id == 0) {
					break;
				}
			} /***Now i is the number of non-zero neighbours*/
			non_zero_nei = i;
			process_post(&udp_server_process, transmit_ll_event_part, NULL);
			return;
		}
		
		/*Restore server connection to allow data from any node */
		memset(&server_conn->ripaddr, 0, sizeof(server_conn->ripaddr));
	}
}

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(udp_server_process, ev, data){

#if UIP_CONF_ROUTER
	uip_ipaddr_t ipaddr;
#endif /* UIP_CONF_ROUTER */

	uip_ipaddr_t mcast_addr;

	PROCESS_BEGIN();
	//static uint8_t txpower;

#if 0
	txpower = CC2420_TXPOWER_MAX;
	cc2420_set_txpower(txpower);
#endif

#if UIP_CONF_ROUTER
	uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
	uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
	uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
#endif /* UIP_CONF_ROUTER */

	//printf("Formatting Coffee FS...\n");
    cfs_coffee_format();
	
	//cc2420_set_channel(26);

	uip_ip6addr(&mcast_addr, 0xff01, 0, 0, 0, 0, 0, 0, 0x0001);
	uip_ds6_maddr_add(&mcast_addr);

	server_conn = udp_new(NULL, UIP_HTONS(3001), NULL);
	udp_bind(server_conn, UIP_HTONS(3000));

	link_learn_conn = udp_new(&mcast_addr, UIP_HTONS(3009), NULL);
	udp_bind(link_learn_conn, UIP_HTONS(3009));

	client_conn = udp_new(&ipaddr, UIP_HTONS(3000), NULL);
	udp_bind(client_conn, UIP_HTONS(3001));

	while (1) {
		PROCESS_YIELD();
		if (ev == tcpip_event) {
			tcpip_handler();

			if (link_learn_var == 1) {
				while (1) {
					etimer_set(&et, SEND_INTERVAL);
					PROCESS_WAIT_UNTIL(etimer_expired(&et));
					timeout_handler_multicast(pkt_count);
					pkt_count = pkt_count + 1;
					// j is the loop variable indicating the number of packets sent
					if (pkt_count >= max_pkts) {
						link_learn_var = 0;
						link_learn_done = 1;
						pkt_count = 0;
						break;
					}
				}

			}

		} else if (ev == transmit_ll_event) {
			//static int no_of_chunks;
			no_of_chunks = non_zero_nei;
			printf("Total Neighbors : %d & %d\n",no_of_chunks,non_zero_nei);
			for (j = 0; j < non_zero_nei; j++) {
				snprintf(fname,sizeof(fname), "%d", neighbor_in_list[j].node_id);
				neighbor_rx_str.node_id = neighbor_in_list[j].node_id;
				//strcpy(neighbor_rx_strm[1].rx_strm,"");
				memset(rx_strm_buf,'\0',sizeof(rx_strm_buf));
				fd = cfs_open(fname, CFS_READ);
				if(fd!=-1) {
				cfs_read(fd, rx_strm_buf, sizeof(rx_strm_buf));
				//printf("Received Stream: %s and Size %d & buff %d\n", rx_strm_buf,strlen(rx_strm_buf),sizeof(rx_strm_buf));
				cfs_close(fd);
				} else {
					printf("ERROR: could not read from memory for neighbor %s or %d at loop %d.\n",fname,neighbor_in_list[j].node_id,j);
				}
				
				//cfs_remove(fname);			
				static int odx;
				/*for(odx=0;odx<1500;odx++){
					printf("%d,", rx_strm_buf[odx]);
				}
				printf("\n\n");*/
				static int cc;	   
				for (retran = 0; retran < 3; retran++){
					uip_udp_packet_send(server_conn,(neighbor_in_list + (j)),sizeof (neighbor_in_list[j]));
					etimer_set(&et, SEND_INTERVAL_3);
					PROCESS_WAIT_UNTIL(etimer_expired(&et));					
				}
				printf("\n*******Sent to Server**********\n,");
				//cc2420_set_channel(26);
				//printf("\nChannel Set to : %d\n",cc2420_get_channel());
				
				for(cc=0;cc<sizeof(rx_strm_buf)/100;cc++){
					//printf("sending runlength fragments : %d + 1\n",sizeof(rx_strm_buf)/100);
					memset(neighbor_rx_str.rx_strm,'\0',sizeof(neighbor_rx_str.rx_strm));
					memcpy(neighbor_rx_str.rx_strm, &rx_strm_buf[cc*100], sizeof(neighbor_rx_str.rx_strm));
					neighbor_rx_str.rx_strm[100]='\0';
					neighbor_rx_str.frag=cc;
					printf("%d fragmnet : ",cc);
					static int ndx;
					for(ndx=0;ndx<100;ndx++){
						printf("%d,", neighbor_rx_str.rx_strm[ndx]);
					}
					printf("\n\n", neighbor_rx_str.rx_strm[ndx]);
					//printf("\nfragment %d for neighbor %d 1st : %d and Last : %d\n",cc,neighbor_rx_str.node_id,neighbor_rx_str.rx_strm[0],neighbor_rx_str.rx_strm[99]);
					//printf("\nfor Chunk %d, data to be sent %s",cc,neighbor_rx_strm[1].rx_strm);
					for (retran = 0; retran < 3; retran++){
						uip_udp_packet_send(server_conn,&neighbor_rx_str, sizeof(neighbor_rx_str));
						printf("\n%d neighbor data Sent out of %d at retrans %d\n",j,no_of_chunks,retran);
						etimer_set(&et, SEND_INTERVAL_2);
						PROCESS_WAIT_UNTIL(etimer_expired(&et));
					}
				}
					
					//uip_udp_packet_send(link_learn_conn,(neighbor_in_list + (j)), sizeof (neighbor_in_list[j]));
					//cc2420_set_channel(15);
					//printf("\nChannel Set to : %d\n",cc2420_get_channel());
				
				
				
				//etimer_set(&et, SEND_INTERVAL_3);
				//PROCESS_WAIT_UNTIL(etimer_expired(&et));
			}
			//printf("\nout of the loop with %d\n",j);
			//if (no_of_chunks == 0)
			//j = 0;
			//now send remainder no. of neighbours
			//uip_udp_packet_send(server_conn,(neighbor_in_list +(j * 2)),rem_neighbours * sizeof(neighbor_in_list[0]));
			printf("Last Chunk Sent\n\n");
			
			etimer_set(&et, SEND_INTERVAL_3);
			PROCESS_WAIT_UNTIL(etimer_expired(&et));
			
			//watchdog_reboot();
			
		} else if (ev == transmit_ll_event_part) {
			
			printf("Total Neighbors : %d \n",non_zero_nei);
			
			for (j = 0; j < non_zero_nei; j++) {
				if(neighbor_in_list[j].node_id == nebr){
					pos = j;
					break;
				}
			}
				
			snprintf(fname,sizeof(fname), "%d", neighbor_in_list[j].node_id);
			neighbor_rx_str.node_id = neighbor_in_list[j].node_id;
			//strcpy(neighbor_rx_strm[1].rx_strm,"");
			memset(rx_strm_buf,'\0',sizeof(rx_strm_buf));
			fd = cfs_open(fname, CFS_READ);
			if(fd!=-1) {
				cfs_read(fd, rx_strm_buf, sizeof(rx_strm_buf));
				//printf("Received Stream: %s and Size %d & buff %d\n", rx_strm_buf,strlen(rx_strm_buf),sizeof(rx_strm_buf));
				cfs_close(fd);
			} else {
					printf("ERROR: could not read from memory for neighbor %s or %d at loop %d.\n",fname,neighbor_in_list[j].node_id,j);
			}
		
			memset(neighbor_rx_str.rx_strm,'\0',sizeof(neighbor_rx_str.rx_strm));
			memcpy(neighbor_rx_str.rx_strm, &rx_strm_buf[pos*100], sizeof(neighbor_rx_str.rx_strm));
			neighbor_rx_str.rx_strm[100]='\0';
			neighbor_rx_str.frag=pos;
			printf("%d fragmnet : ",pos);
			static int ndx;
			for(ndx=0;ndx<100;ndx++){
				printf("%d,", neighbor_rx_str.rx_strm[ndx]);
			}
			printf("\n\n", neighbor_rx_str.rx_strm[ndx]);
			for (retran = 0; retran < 3; retran++){
				uip_udp_packet_send(server_conn,&neighbor_rx_str, sizeof(neighbor_rx_str));
				printf("\n%d neighbor data Sent out of %d at retrans %d\n",j,no_of_chunks,retran);
				etimer_set(&et, SEND_INTERVAL_2);
				PROCESS_WAIT_UNTIL(etimer_expired(&et));
			}
						
		}
	}
	PROCESS_END();
}

	
/*---------------------------------------------------------------------------*/
