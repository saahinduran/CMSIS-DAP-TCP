#if 0
/*
 * tcp_server.c
 *
 *  Created on: Nov 8, 2025
 *      Author: sahin
 */

#include "tcpserverRAW.h"

#include "lwip/tcp.h"

struct tcp_server_struct
{
  u8_t state;             /* current connection state */
  u8_t retries;
  struct tcp_pcb *pcb;    /* pointer on the current tcp_pcb */
  struct pbuf *p;         /* pointer on the received/to be transmitted pbuf */
};

/*  protocol states */
enum tcp_server_states
{
  ES_NONE = 0,
  ES_ACCEPTED,
  ES_RECEIVED,
  ES_CLOSING
};


void tcp_echoserver_init(void)
{
	struct tcp_pcb *tcp_echoserver_pcb;

	/* create new tcp pcb */
	tcp_echoserver_pcb = tcp_new();




	if (tcp_echoserver_pcb != NULL)

	{

		err_t err;

		/* bind echo_pcb to port 7 (ECHO protocol) */

		err = tcp_bind(tcp_echoserver_pcb, IP_ADDR_ANY, 7);

		if (err == ERR_OK)

		{

			/* start tcp listening for echo_pcb */

			tcp_echoserver_pcb = tcp_listen(tcp_echoserver_pcb);

			/* initialize LwIP tcp_accept callback function */

			tcp_accept(tcp_echoserver_pcb, tcp_echoserver_accept);

		}

		else

		{

			/* deallocate the pcb */

			memp_free(MEMP_TCP_PCB, tcp_echoserver_pcb);

		}

	}
}

static err_t tcp_echoserver_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
	/* allocate structure es to maintain tcp connection informations */
	struct tcp_server_struct *es;
	es = (struct tcp_server_struct *)mem_malloc(sizeof(struct tcp_server_struct));
	if (es != NULL)
	{
		es->state = ES_ACCEPTED;

		es->pcb = newpcb;

		es->p = NULL;

		/* pass newly allocated es structure as argument to newpcb */

		tcp_arg(newpcb, es);

		/* initialize lwIP tcp_recv callback function for newpcb */

		tcp_recv(newpcb, tcp_echoserver_recv);

		/* initialize lwIP tcp_err callback function for newpcb */

		tcp_err(newpcb, tcp_echoserver_error);

		/* initialize lwIP tcp_poll callback function for newpcb */

		tcp_poll(newpcb, tcp_echoserver_poll, 1);

		ret_err = ERR_OK;
}
#endif
