/*
 * stm32_lwip_tcp_server.c
 *
 * TCP server example for STM32 using lwIP raw API.
 * - Listens on port 5000.
 * - Receives data, performs user-defined processing, and sends back response.
 *
 * Author: ChatGPT (modified version)
 */

#include "lwip/tcp.h"
#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include <string.h>
#include <stdio.h>

#define TCP_SERVER_PORT 5000

/* Forward declarations */
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void  tcp_server_err(void *arg, err_t err);
static err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static void  tcp_server_close_conn(struct tcp_pcb *tpcb, void *conn);

/* User-defined function to process received data */
static void process_data(const char *input, char *output, size_t out_len)
{
    /* Example: convert input to uppercase */
    size_t len = strlen(input);
    for (size_t i = 0; i < len && i < out_len - 1; i++) {
        output[i] = (char)toupper((unsigned char)input[i]);
    }
    output[len < out_len - 1 ? len : out_len - 1] = '\0';
}

void tcp_server_init(void)
{
    struct tcp_pcb *pcb;
    err_t ret;

    pcb = tcp_new();
    if (pcb == NULL) {
        printf("tcp_server_init: tcp_new failed\n");
        return;
    }

    ret = tcp_bind(pcb, IP_ADDR_ANY, TCP_SERVER_PORT);
    if (ret != ERR_OK) {
        printf("tcp_server_init: tcp_bind failed: %d\n", ret);
        tcp_close(pcb);
        return;
    }

    pcb = tcp_listen(pcb);
    if (pcb == NULL) {
        printf("tcp_server_init: tcp_listen failed\n");
        return;
    }

    tcp_accept(pcb, tcp_server_accept);
    printf("TCP server listening on port %d\n", TCP_SERVER_PORT);
}

static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    LWIP_UNUSED_ARG(arg);
    LWIP_UNUSED_ARG(err);

    tcp_arg(newpcb, newpcb); // Use pcb as connection context
    tcp_recv(newpcb, tcp_server_recv);
    tcp_err(newpcb, tcp_server_err);
    tcp_sent(newpcb, tcp_server_sent);

    printf("New client connected\n");
    return ERR_OK;
}

static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (err != ERR_OK || p == NULL) {
        if (p != NULL) pbuf_free(p);
        if (err == ERR_OK && p == NULL) {
            printf("Client closed connection\n");
            tcp_server_close_conn(tpcb, arg);
        }
        return ERR_OK;
    }

    /* Inform lwIP that data has been received */
    tcp_recved(tpcb, p->tot_len);

    /* Copy data into a local buffer */
    char input[256];
    size_t len = (p->tot_len < sizeof(input) - 1) ? p->tot_len : sizeof(input) - 1;
    pbuf_copy_partial(p, input, len, 0);
    input[len] = '\0';

    printf("Received: %s\n", input);

    /* Process data */
    char response[256];
    process_data(input, response, sizeof(response));

    printf("Responding: %s\n", response);

    /* Send response */
    err_t werr = tcp_write(tpcb, response, strlen(response), TCP_WRITE_FLAG_COPY);
    if (werr == ERR_OK) {
        tcp_output(tpcb);
    } else {
        printf("tcp_write failed: %d\n", werr);
    }

    pbuf_free(p);
    return ERR_OK;
}

static err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    LWIP_UNUSED_ARG(arg);
    LWIP_UNUSED_ARG(tpcb);
    LWIP_UNUSED_ARG(len);
    return ERR_OK;
}

static void tcp_server_err(void *arg, err_t err)
{
    LWIP_UNUSED_ARG(err);
    printf("Connection aborted or reset\n");
}

static void tcp_server_close_conn(struct tcp_pcb *tpcb, void *conn)
{
    tcp_arg(tpcb, NULL);
    tcp_sent(tpcb, NULL);
    tcp_recv(tpcb, NULL);
    tcp_err(tpcb, NULL);
    tcp_close(tpcb);
    (void)conn;
}

/* Example usage:
   - Call tcp_server_init() after MX_LWIP_Init() completes.
   - The server listens on port 5000.
   - When a client sends data, process_data() transforms it (example: uppercase) and returns it. */
