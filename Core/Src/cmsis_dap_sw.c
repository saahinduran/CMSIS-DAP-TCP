#if 1
/*
 * stm32_lwip_dap_server_raw.c
 *
 * TCP server example for STM32 using lwIP RAW API.
 * Option A: Pure RAW API, no tcpip_callback required.
 *
 * Behavior and important notes:
 * - Listens on port 5000.
 * - On incoming packet: copies packet to the connection RX buffer and sets rx_ready.
 * - All DAP processing (process_dap_command) is executed from the lwIP tcpip_thread
 *   via the TCP poll callback (dap_poll). **This means process_dap_command() MUST be
 *   non-blocking and fast** (it must not call blocking OS primitives or sleep). If
 *   you need to perform long blocking JTAG operations, move them into a separate
 *   thread/task and use a non-blocking handover pattern (not covered here).
 * - TX is attempted from the poll handler (tcpip_thread). If tcp_write returns
 *   ERR_MEM, the code will retry on subsequent polls or after ack via dap_sent().
 *
 * This file intentionally avoids tcpip_callback/tcpip_try_callback so it can be
 * used in projects where tcpip_callback isn't available or desired.
 *
 * Usage:
 *  - Call dap_server_init() after MX_LWIP_Init().
 *  - Implement process_dap_command() (weak) but ensure it is fast and non-blocking.
 *
 * Author: ChatGPT (modified)
 */

#include "lwip/tcp.h"
#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define DAP_TCP_PORT    5000
#define DAP_RX_BUF_SZ   2048
#define DAP_TX_BUF_SZ   2048

/* connection state (single connection example) */
struct dap_conn {
    struct tcp_pcb *pcb;
    /* RX side (written by tcp_recv callback inside tcpip_thread) */
    volatile bool rx_ready;        /* set when a packet is copied into rx_buf */
    uint16_t rx_len;
    uint8_t  rx_buf[DAP_RX_BUF_SZ];

    /* TX side */
    volatile bool tx_pending;      /* set when tx_buf contains data to send */
    volatile bool tx_in_progress;  /* true while tcp_write was successful and waiting for ack */
    uint16_t tx_len;
    uint8_t  tx_buf[DAP_TX_BUF_SZ];
};

static struct dap_conn *g_conn = NULL; /* global pointer to current connection (or NULL) */
static struct tcp_pcb *g_listen_pcb = NULL;

/* Forward declarations */
static err_t dap_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t dap_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void  dap_err(void *arg, err_t err);
static err_t dap_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static void  dap_close_conn(struct dap_conn *conn);
static err_t dap_poll(void *arg, struct tcp_pcb *tpcb);

/* Weak user-implemented function: process command in rx_buf and fill tx_buf.
 * IMPORTANT: This will be called from tcpip_thread (via tcp_poll) in this design.
 * Therefore it MUST be fast and non-blocking. Return tx_len (0..DAP_TX_BUF_SZ).
 */
__attribute__((weak)) uint16_t process_dap_command(const uint8_t *rx, uint16_t rx_len, uint8_t *tx, uint16_t max_tx_len)
{
    /* Default example: echo the data back (replace with your CMSIS-DAP/JTAG code) */
    uint16_t n = (rx_len < max_tx_len) ? rx_len : max_tx_len;
    memcpy(tx, rx, n);
    return n;
}

/* Initialize the DAP TCP server. Call after lwIP stack is up (MX_LWIP_Init). */
void dap_server_init(void)
{
    err_t ret;
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) {
        printf("dap_server_init: tcp_new failed\n");
        return;
    }

    ret = tcp_bind(pcb, IP_ADDR_ANY, DAP_TCP_PORT);
    if (ret != ERR_OK) {
        printf("dap_server_init: tcp_bind failed %d\n", ret);
        tcp_close(pcb);
        return;
    }

    g_listen_pcb = tcp_listen(pcb);
    if (!g_listen_pcb) {
        printf("dap_server_init: tcp_listen failed\n");
        return;
    }

    tcp_accept(g_listen_pcb, dap_accept);
    printf("DAP TCP server listening on port %d\n", DAP_TCP_PORT);
}

/* Accept callback: allocate per-connection state (single connection; closes previous) */
static err_t dap_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    LWIP_UNUSED_ARG(arg);
    LWIP_UNUSED_ARG(err);

    /* If we already have a connection, close it to accept the new one */
    if (g_conn) {
        dap_close_conn(g_conn);
        g_conn = NULL;
    }

    struct dap_conn *conn = (struct dap_conn *)mem_malloc(sizeof(struct dap_conn));
    if (!conn) {
        printf("dap_accept: mem_malloc failed\n");
        return ERR_MEM;
    }
    memset(conn, 0, sizeof(*conn));

    conn->pcb = newpcb;
    conn->rx_ready = false;
    conn->tx_pending = false;
    conn->tx_in_progress = false;

    g_conn = conn;

    tcp_arg(newpcb, conn);
    tcp_recv(newpcb, dap_recv);
    tcp_err(newpcb, dap_err);
    tcp_sent(newpcb, dap_sent);

    /* Set a poll callback to run periodically in tcpip_thread. The poll interval
       parameter is in multiples of the TCP coarse timer (typically 500 ms). Use
       a small interval if you need low latency; be careful about CPU usage. */
    tcp_poll(newpcb, dap_poll, 1);

    printf("DAP: client connected\n");
    return ERR_OK;
}

/* Receive callback: copy incoming packet into RX buffer and set rx_ready flag.
 * This callback runs in tcpip_thread context.
 */
static err_t dap_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    struct dap_conn *conn = (struct dap_conn *)arg;

    if (err != ERR_OK) {
        if (p) pbuf_free(p);
        return err;
    }

    if (p == NULL) {
        /* remote closed */
        printf("DAP: client closed connection\n");
        dap_close_conn(conn);
        g_conn = NULL;
        return ERR_OK;
    }

    /* If RX buffer already contains unprocessed data, drop the incoming packet */
    if (conn->rx_ready) {
        printf("DAP: rx buffer busy, dropping packet (%u bytes)\n", p->tot_len);
        pbuf_free(p);
        return ERR_OK;
    }

    /* Limit to buffer size */
    uint16_t copy_len = (p->tot_len < DAP_RX_BUF_SZ) ? p->tot_len : (DAP_RX_BUF_SZ);
    pbuf_copy_partial(p, conn->rx_buf, copy_len, 0);
    conn->rx_len = copy_len;

    /* Inform lwIP that we have consumed the data, so window advances */
    tcp_recved(tpcb, p->tot_len);

    /* Free pbuf */
    pbuf_free(p);

    /* Mark ready for processing by the poll callback (runs in tcpip_thread) */
    conn->rx_ready = true;

    return ERR_OK;
}

/* Poll callback: runs in tcpip_thread periodically. We perform the processing here
 * (call process_dap_command) and attempt to send the response using tcp_write.
 */
static err_t dap_poll(void *arg, struct tcp_pcb *tpcb)
{
    struct dap_conn *conn = (struct dap_conn *)arg;
    if (!conn) return ERR_OK;

    /* If RX ready, process it (processing runs in tcpip_thread) */
    if (conn->rx_ready) {
        conn->rx_ready = false; /* consume */

        /* Call user handler directly (MUST be non-blocking and fast) */
        uint16_t tx_len = process_dap_command(conn->rx_buf, conn->rx_len, conn->tx_buf, DAP_TX_BUF_SZ);
        if (tx_len > 0) {
            conn->tx_len = (tx_len <= DAP_TX_BUF_SZ) ? tx_len : DAP_TX_BUF_SZ;
            conn->tx_pending = true;
        }
    }

    /* If there is pending TX and not already in progress, try to send */
    if (conn->tx_pending && !conn->tx_in_progress) {
        err_t werr = tcp_write(conn->pcb, conn->tx_buf, conn->tx_len, TCP_WRITE_FLAG_COPY);
        if (werr == ERR_OK) {
            conn->tx_in_progress = true;
            conn->tx_pending = false;
            tcp_output(conn->pcb);
        } else if (werr == ERR_MEM) {
            /* send buffer full; we'll retry on next poll or on dap_sent callback */
            /* leave tx_pending true */
        } else {
            printf("dap_poll: tcp_write error %d\n", werr);
            conn->tx_pending = false;
        }
    }

    return ERR_OK;
}

/* Sent callback: called when previously queued data was acknowledged. Use this to
 * clear tx_in_progress and retry if tx_pending became true while sending.
 */
static err_t dap_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    LWIP_UNUSED_ARG(len);
    struct dap_conn *conn = (struct dap_conn *)arg;
    if (!conn) return ERR_OK;

    conn->tx_in_progress = false;

    /* If application queued another response while we were sending, the poll will
     * pick it up on the next interval. We can also attempt immediate retry here. */
    if (conn->tx_pending) {
        /* try immediate send (we are in tcpip_thread) */
        err_t werr = tcp_write(conn->pcb, conn->tx_buf, conn->tx_len, TCP_WRITE_FLAG_COPY);
        if (werr == ERR_OK) {
            conn->tx_in_progress = true;
            conn->tx_pending = false;
            tcp_output(conn->pcb);
        }
    }
    return ERR_OK;
}

/* Error callback: connection aborted */
static void dap_err(void *arg, err_t err)
{
    LWIP_UNUSED_ARG(err);
    struct dap_conn *conn = (struct dap_conn *)arg;
    if (!conn) return;
    printf("dap_err: connection aborted\n");
    /* conn->pcb is already freed by lwIP when err callback fires; free conn structure */
    mem_free(conn);
    g_conn = NULL;
}

/* Close and cleanup connection (call from tcpip_thread). */
static void dap_close_conn(struct dap_conn *conn)
{
    if (!conn) return;
    if (conn->pcb) {
        tcp_arg(conn->pcb, NULL);
        tcp_recv(conn->pcb, NULL);
        tcp_err(conn->pcb, NULL);
        tcp_sent(conn->pcb, NULL);
        tcp_poll(conn->pcb, NULL, 0);
        tcp_close(conn->pcb);
        conn->pcb = NULL;
    }
    mem_free(conn);
}

/* End of file */
#endif
