/*
 * Copyright (c) 2015, Yanzi Networks AB.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Original authors:
 *         Joakim Eriksson <joakime@sics.se>
 *         Niclas Finne <nfi@sics.se>
 *         Joel Hoglund <joel@sics.se>
 */

#define SYS_LOG_DOMAIN "lib/lwm2m_rd_client"
#define SYS_LOG_LEVEL CONFIG_SYS_LOG_LWM2M_LEVEL
#include <logging/sys_log.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <init.h>
#include <misc/printk.h>
#include <misc/stack.h>
#include <net/net_pkt.h>
#include <net/zoap.h>
#include <net/lwm2m.h>

/* Mbed TLS */
#if defined(CONFIG_LWM2M_SECURITY_DTLS_SUPPORT)
/* TODO: Review what is strictly required */
#include CONFIG_MBEDTLS_CFG_FILE
#include "mbedtls/platform.h"
#include "mbedtls/net.h"
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#if defined(MBEDTLS_DEBUG_C)
#include "mbedtls/debug.h"
#define DEBUG_THRESHOLD 0
#endif

#if defined(MBEDTLS_KEY_EXCHANGE__SOME__PSK_ENABLED)
/* TODO: Retrieve PSK ID and PSK from partition */
const char psk_id[] = "0970f19b28b24451925523604651de23";
const unsigned char psk[] = {
	0xa5, 0x2b, 0xa5, 0x05, 0x05, 0xae, 0x44, 0x15,
	0x87, 0x7b, 0x7a, 0xd6, 0x81, 0xdf, 0x08, 0x26
};
#endif

/* TODO: Use device name if available */
const char *pers = "perso_data";
/* TODO: find proper initial and maximum retransmit timeout values */
#define DTLS_INITIAL_TIMEOUT	5000 /* 5 seconds */
#define DTLS_MAXIMUM_TIMEOUT	60000 /* 60 seconds */

struct dtls_context {
	struct net_context *net_ctx;
	struct net_pkt *rx_pkt;
	struct k_sem rx_sem;
	int remaining;
};

struct dtls_timing_context {
	u32_t snapshot;
	u32_t int_ms;
	u32_t fin_ms;
};

mbedtls_ssl_context ssl_ctx;
static struct dtls_context dtls_udp_ctx;

/* TODO: find better way to retrive the size required */
#define ZOAP_BUF_SIZE 256
static u8_t dtls_rx_buf[ZOAP_BUF_SIZE];

#endif /* CONFIG_LWM2M_SECURITY_DTLS_SUPPORT */

#include "lwm2m_object.h"
#include "lwm2m_engine.h"

#define LWM2M_RD_CLIENT_URI "rd"

#define SECONDS_TO_UPDATE_EARLY	2
#define STATE_MACHINE_UPDATE_INTERVAL 500

#define LWM2M_PEER_PORT		CONFIG_LWM2M_PEER_PORT
#define LWM2M_BOOTSTRAP_PORT	CONFIG_LWM2M_BOOTSTRAP_PORT

/* The states for the RD client state machine */
/*
 * When node is unregistered it ends up in UNREGISTERED
 * and this is going to be there until use X or Y kicks it
 * back into INIT again
 */
enum sm_engine_state {
	ENGINE_INIT,
	ENGINE_SECURITY_HANDSHAKE,
	ENGINE_DO_BOOTSTRAP,
	ENGINE_BOOTSTRAP_SENT,
	ENGINE_BOOTSTRAP_DONE,
	ENGINE_DO_REGISTRATION,
	ENGINE_REGISTRATION_SENT,
	ENGINE_REGISTRATION_DONE,
	ENGINE_UPDATE_SENT,
	ENGINE_DEREGISTER,
	ENGINE_DEREGISTER_SENT,
	ENGINE_DEREGISTER_FAILED,
	ENGINE_DEREGISTERED
};

struct lwm2m_rd_client_info {
	u16_t lifetime;
	struct net_context *net_ctx;
	struct sockaddr bs_server;
	struct sockaddr reg_server;
	u8_t engine_state;
	u8_t use_bootstrap;
	u8_t has_bs_server_info;
	u8_t use_registration;
	u8_t has_registration_info;
	u8_t registered;
	u8_t bootstrapped; /* bootstrap done */
	u8_t trigger_update;

	s64_t last_update;

	char ep_name[33];
	char server_ep[32];
};

static K_THREAD_STACK_DEFINE(lwm2m_rd_client_thread_stack,
			     CONFIG_LWM2M_RD_CLIENT_STACK_SIZE);
struct k_thread lwm2m_rd_client_thread_data;

#if defined(CONFIG_LWM2M_SECURITY_DTLS_SUPPORT)
/* DTLS RX */
static K_THREAD_STACK_DEFINE(lwm2m_dtls_read_thread_stack, 3000);
static struct k_thread lwm2m_dtls_read_thread_data;
static struct k_sem dtls_read_sem;
#endif

/* HACK: remove when engine transactions are ready */
#define NUM_PENDINGS	CONFIG_LWM2M_ENGINE_MAX_PENDING
#define NUM_REPLIES	CONFIG_LWM2M_ENGINE_MAX_REPLIES

extern struct zoap_pending pendings[NUM_PENDINGS];
extern struct zoap_reply replies[NUM_REPLIES];
extern struct k_delayed_work retransmit_work;

/* buffers */
static char query_buffer[64]; /* allocate some data for queries and updates */
static u8_t client_data[256]; /* allocate some data for the RD */

#define CLIENT_INSTANCE_COUNT	CONFIG_LWM2M_RD_CLIENT_INSTANCE_COUNT
static struct lwm2m_rd_client_info clients[CLIENT_INSTANCE_COUNT];
static int client_count;

#if defined(CONFIG_LWM2M_SECURITY_DTLS_SUPPORT)
static void dtls_debug(void *ctx, int level,
		     const char *file, int line, const char *str)
{
	const char *p, *basename;

	/* Extract basename from file */
	for (p = basename = file; *p != '\0'; p++) {
		if (*p == '/' || *p == '\\') {
			basename = p + 1;
		}
	}

	mbedtls_printf("%s:%04d: |%d| %s", basename, line, level, str);
}

void dtls_timing_set_delay(void *data, u32_t int_ms, u32_t fin_ms)
{
	struct dtls_timing_context *ctx = (struct dtls_timing_context *)data;

	ctx->int_ms = int_ms;
	ctx->fin_ms = fin_ms;

	if (fin_ms != 0) {
		ctx->snapshot = k_uptime_get_32();
	}
}

int dtls_timing_get_delay(void *data)
{
	struct dtls_timing_context *ctx = (struct dtls_timing_context *)data;
	unsigned long elapsed_ms;

	if (ctx->fin_ms == 0) {
		return -1;
	}

	elapsed_ms = k_uptime_get_32() - ctx->snapshot;

	if (elapsed_ms >= ctx->fin_ms) {
		return 2;
	}

	if (elapsed_ms >= ctx->int_ms) {
		return 1;
	}

	return 0;
}

static int entropy_source(void *data, unsigned char *output, size_t len,
			  size_t *olen)
{
	u32_t seed;

	ARG_UNUSED(data);

	seed = sys_rand32_get();
	if (len > sizeof(seed)) {
		len = sizeof(seed);
	}

	memcpy(output, &seed, len);

	*olen = len;

	return 0;
}

static void dtls_receive(struct net_context *ctx, struct net_pkt *pkt,
			int status, void *user_data)
{
	struct dtls_context *dtls_ctx = user_data;

	ARG_UNUSED(ctx);
	ARG_UNUSED(status);

	dtls_ctx->rx_pkt = pkt;
	/* Unblock mbedtls read */
	k_sem_give(&dtls_ctx->rx_sem);
}

int dtls_tx(void *context, const unsigned char *buf, size_t size)
{
	struct net_context *ctx;
	struct net_pkt *send_pkt;
	int ret, len;

	ctx = clients[0].net_ctx;
	send_pkt = net_pkt_get_tx(ctx, K_FOREVER);
	if (!send_pkt) {
		return MBEDTLS_ERR_SSL_ALLOC_FAILED;
	}

	ret = net_pkt_append_all(send_pkt, size, (u8_t *) buf, K_FOREVER);
	if (!ret) {
		return MBEDTLS_ERR_SSL_INTERNAL_ERROR;
	}

	len = net_pkt_get_len(send_pkt);
	SYS_LOG_DBG("TX sent to [%s], pkt len: %d",
			sprint_ip_addr(&clients[0].reg_server), len);
	/* TODO: context here needs to be the current opened one
	 * since there can only be one DTLS connection at a time,
	 * which can be either bootstrap or registration.
	 * Needs to change to use the internal struct to handle the current
	 * context being used. */
	ret = net_context_sendto(send_pkt, &clients[0].reg_server,
				 NET_SOCKADDR_MAX_SIZE,
				 NULL, K_FOREVER, NULL, NULL);
	if (ret < 0) {
		SYS_LOG_ERR("DTLS TX: ERROR");
		net_pkt_unref(send_pkt);
		return MBEDTLS_ERR_SSL_INTERNAL_ERROR;
	} else {
		return len;
	}
}

int dtls_rx(void *context, unsigned char *buf, size_t size)
{
	struct dtls_context *dtls_ctx = context;
	struct net_buf *rx_buf = NULL;
	u16_t read_bytes;
	u8_t *ptr;
	int pos;
	int len;
	int rc;

	k_sem_take(&dtls_ctx->rx_sem, K_FOREVER);

	read_bytes = net_pkt_appdatalen(dtls_ctx->rx_pkt);
	SYS_LOG_INF("DTLS RX: read_bytes: %d, allocated size: %d", read_bytes, size);
	if (read_bytes > size) {
		SYS_LOG_ERR("DTLS RX: alloc failure");
		return MBEDTLS_ERR_SSL_ALLOC_FAILED;
	}

	SYS_LOG_INF("RX Source IP: [%s]", sprint_ip_addr(&clients[0].reg_server));

	ptr = net_pkt_appdata(dtls_ctx->rx_pkt);
	rx_buf = dtls_ctx->rx_pkt->frags;
	len = rx_buf->len - (ptr - rx_buf->data);
	pos = 0;

	while (rx_buf) {
		memcpy(buf + pos, ptr, len);
		pos += len;

		rx_buf = rx_buf->frags;
		if (!rx_buf) {
			break;
		}

		ptr = rx_buf->data;
		len = rx_buf->len;
	}

	net_pkt_unref(dtls_ctx->rx_pkt);
	dtls_ctx->rx_pkt = NULL;

	if (read_bytes != pos) {
		return MBEDTLS_ERR_SSL_INTERNAL_ERROR;
	}

	rc = read_bytes;
	dtls_ctx->remaining = 0;

	return rc;
}

static void lwm2m_dtls_read_service(void)
{
	struct net_pkt *pkt;
	struct net_context *net_ctx;
	int len;
	int ret = 0;

	while (true) {
		k_sem_take(&dtls_read_sem, K_FOREVER);
		SYS_LOG_DBG("dtls read thread unblocked");

	again:
		net_ctx = clients[0].net_ctx;
		memset(dtls_rx_buf, 0, ZOAP_BUF_SIZE);

		do {
			ret = mbedtls_ssl_read(&ssl_ctx,
					dtls_rx_buf, ZOAP_BUF_SIZE - 1);
		} while (ret == MBEDTLS_ERR_SSL_WANT_READ ||
			 ret == MBEDTLS_ERR_SSL_WANT_WRITE);

		if (ret <= 0) {
			switch (ret) {
			case MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY:
				SYS_LOG_INF("Connection closed gracefully");
				break;

			case MBEDTLS_ERR_NET_CONN_RESET:
				SYS_LOG_ERR("Connection was reset by peer");
				break;

			default:
				SYS_LOG_ERR("SSL read error: %d", ret);
				break;
			}

			continue;
		}

		pkt = net_pkt_get_tx(net_ctx, K_FOREVER);
		if (!pkt) {
			SYS_LOG_ERR("Unable to get TX packet, not enough memory.");
			continue;
		}

		/* Add dtls buffer to pkt */
		len = ret;
		ret = net_pkt_append_all(pkt, len, dtls_rx_buf, K_FOREVER);
		if (!ret) {
			SYS_LOG_ERR("Unable to store DATA buffer, not enough memory.");
			net_pkt_unref(pkt);
			continue;
		}
		net_pkt_set_appdatalen(pkt, len);
		net_pkt_set_appdata(pkt, pkt->frags->data);

		/* Call back the non DTLS udp_receive function from engine */
		udp_receive(net_ctx, pkt, 0, &clients[0].reg_server);

		goto again;
	}
}
#endif /* CONFIG_LWM2M_SECURITY_DTLS_SUPPORT */

static void set_sm_state(int index, u8_t state)
{
	/* TODO: add locking? */
	clients[index].engine_state = state;
}

static u8_t get_sm_state(int index)
{
	/* TODO: add locking? */
	return clients[index].engine_state;
}

static int find_clients_index(const struct sockaddr *addr,
				  bool check_bs_server)
{
	int index = -1, i;

	for (i = 0; i < client_count; i++) {
		if (check_bs_server) {
			if (memcmp(addr, &clients[i].bs_server,
				   sizeof(addr)) == 0) {
				index = i;
				break;
			}
		} else {
			if (memcmp(addr, &clients[i].reg_server,
				   sizeof(addr)) == 0) {
				index = i;
				break;
			}
		}
	}

	return index;
}

/* force re-update with remote peer(s) */
void engine_trigger_update(void)
{
	int index;

	for (index = 0; index < client_count; index++) {
		/* TODO: add locking? */
		clients[index].trigger_update = 1;
	}
}

/* state machine reply callbacks */

static int do_bootstrap_reply_cb(const struct zoap_packet *response,
				 struct zoap_reply *reply,
				 const struct sockaddr *from)
{
	u8_t code;
	int index;

	code = zoap_header_get_code(response);
	SYS_LOG_DBG("Bootstrap callback (code:%u.%u)",
		    ZOAP_RESPONSE_CODE_CLASS(code),
		    ZOAP_RESPONSE_CODE_DETAIL(code));

	index = find_clients_index(from, true);
	if (index < 0) {
		SYS_LOG_ERR("Bootstrap client index not found.");
		return 0;
	}

	if (code == ZOAP_RESPONSE_CODE_CHANGED) {
		SYS_LOG_DBG("Considered done!");
		set_sm_state(index, ENGINE_BOOTSTRAP_DONE);
	} else if (code == ZOAP_RESPONSE_CODE_NOT_FOUND) {
		SYS_LOG_ERR("Failed: NOT_FOUND.  Not Retrying.");
		set_sm_state(index, ENGINE_DO_REGISTRATION);
	} else if (code == ZOAP_RESPONSE_CODE_FORBIDDEN) {
		SYS_LOG_ERR("Failed: 4.03 - Forbidden.  Not Retrying.");
		set_sm_state(index, ENGINE_DO_REGISTRATION);
	} else {
		/* TODO: Read payload for error message? */
		SYS_LOG_ERR("Failed with code %u.%u. Retrying ...",
			    ZOAP_RESPONSE_CODE_CLASS(code),
			    ZOAP_RESPONSE_CODE_DETAIL(code));
		set_sm_state(index, ENGINE_INIT);
	}

	return 0;
}

static int do_registration_reply_cb(const struct zoap_packet *response,
				    struct zoap_reply *reply,
				    const struct sockaddr *from)
{
	struct zoap_option options[2];
	u8_t code;
	int ret, index;

	code = zoap_header_get_code(response);
	SYS_LOG_DBG("Registration callback (code:%u.%u)",
		    ZOAP_RESPONSE_CODE_CLASS(code),
		    ZOAP_RESPONSE_CODE_DETAIL(code));

	index = find_clients_index(from, false);
	if (index < 0) {
		SYS_LOG_ERR("Registration client index not found.");
		return 0;
	}

	/* check state and possibly set registration to done */
	if (code == ZOAP_RESPONSE_CODE_CREATED) {
		ret = zoap_find_options(response, ZOAP_OPTION_LOCATION_PATH,
					options, 2);
		if (ret < 0) {
			return ret;
		}

		if (ret < 2) {
			SYS_LOG_ERR("Unexpected endpoint data returned.");
			return -EINVAL;
		}

		/* option[0] should be "rd" */

		if (options[1].len + 1 > sizeof(clients[index].server_ep)) {
			SYS_LOG_ERR("Unexpected length of query: "
				    "%u (expected %zu)\n",
				    options[1].len,
				    sizeof(clients[index].server_ep));
			return -EINVAL;
		}

		memcpy(clients[index].server_ep, options[1].value,
		       options[1].len);
		clients[index].server_ep[options[1].len] = '\0';
		set_sm_state(index, ENGINE_REGISTRATION_DONE);
		clients[index].registered = 1;
		SYS_LOG_INF("Registration Done (EP='%s')",
			    clients[index].server_ep);

		return 0;
	} else if (code == ZOAP_RESPONSE_CODE_NOT_FOUND) {
		SYS_LOG_ERR("Failed: NOT_FOUND.  Not Retrying.");
		set_sm_state(index, ENGINE_REGISTRATION_DONE);
		return 0;
	} else if (code == ZOAP_RESPONSE_CODE_FORBIDDEN) {
		SYS_LOG_ERR("Failed: 4.03 - Forbidden.  Not Retrying.");
		set_sm_state(index, ENGINE_REGISTRATION_DONE);
		return 0;
	}

	/* TODO: Read payload for error message? */
	/* Possible error response codes: 4.00 Bad request & 4.03 Forbidden */
	SYS_LOG_ERR("failed with code %u.%u. Re-init network",
		    ZOAP_RESPONSE_CODE_CLASS(code),
		    ZOAP_RESPONSE_CODE_DETAIL(code));
	set_sm_state(index, ENGINE_INIT);
	return 0;
}

static int do_update_reply_cb(const struct zoap_packet *response,
			      struct zoap_reply *reply,
			      const struct sockaddr *from)
{
	u8_t code;
	int index;

	code = zoap_header_get_code(response);
	SYS_LOG_DBG("Update callback (code:%u.%u)",
		    ZOAP_RESPONSE_CODE_CLASS(code),
		    ZOAP_RESPONSE_CODE_DETAIL(code));

	index = find_clients_index(from, false);
	if (index < 0) {
		SYS_LOG_ERR("Registration client index not found.");
		return 0;
	}

	/* If NOT_FOUND just continue on */
	if ((code == ZOAP_RESPONSE_CODE_CHANGED) ||
	    (code == ZOAP_RESPONSE_CODE_CREATED)) {
		set_sm_state(index, ENGINE_REGISTRATION_DONE);
		SYS_LOG_DBG("Update Done");
		return 0;
	}

	/* TODO: Read payload for error message? */
	/* Possible error response codes: 4.00 Bad request & 4.04 Not Found */
	SYS_LOG_ERR("Failed with code %u.%u. Retrying registration",
		    ZOAP_RESPONSE_CODE_CLASS(code),
		    ZOAP_RESPONSE_CODE_DETAIL(code));
	clients[index].registered = 0;
	set_sm_state(index, ENGINE_DO_REGISTRATION);

	return 0;
}

static int do_deregister_reply_cb(const struct zoap_packet *response,
				  struct zoap_reply *reply,
				  const struct sockaddr *from)
{
	u8_t code;
	int index;

	code = zoap_header_get_code(response);
	SYS_LOG_DBG("Deregister callback (code:%u.%u)",
		    ZOAP_RESPONSE_CODE_CLASS(code),
		    ZOAP_RESPONSE_CODE_DETAIL(code));

	index = find_clients_index(from, false);
	if (index < 0) {
		SYS_LOG_ERR("Registration clients index not found.");
		return 0;
	}

	if (code == ZOAP_RESPONSE_CODE_DELETED) {
		clients[index].registered = 0;
		SYS_LOG_DBG("Deregistration success");
		set_sm_state(index, ENGINE_DEREGISTERED);
	} else {
		SYS_LOG_ERR("failed with code %u.%u",
			    ZOAP_RESPONSE_CODE_CLASS(code),
			    ZOAP_RESPONSE_CODE_DETAIL(code));
		if (get_sm_state(index) == ENGINE_DEREGISTER_SENT) {
			set_sm_state(index, ENGINE_DEREGISTER_FAILED);
		}
	}

	return 0;
}

/* state machine step functions */

static int sm_do_init(int index)
{
	SYS_LOG_DBG("RD Client started with endpoint "
		    "'%s' and client lifetime %d",
		    clients[index].ep_name,
		    clients[index].lifetime);
	/* Zephyr has joined network already */
	clients[index].has_registration_info = 1;
	clients[index].registered = 0;
	clients[index].bootstrapped = 0;
	clients[index].trigger_update = 0;
#if defined(CONFIG_LWM2M_BOOTSTRAP_SERVER)
	clients[index].use_bootstrap = 1;
#else
	clients[index].use_registration = 1;
#endif
	if (clients[index].lifetime == 0) {
		clients[index].lifetime = CONFIG_LWM2M_ENGINE_DEFAULT_LIFETIME;
	}

	set_sm_state(index, ENGINE_SECURITY_HANDSHAKE);

	return 0;
}

static int sm_do_sec_handshake(int index)
{
	int ret = 0;

#if defined(CONFIG_LWM2M_SECURITY_DTLS_SUPPORT)
	SYS_LOG_DBG("Initializing DTLS");

	mbedtls_entropy_context entropy;
	mbedtls_ctr_drbg_context ctr_drbg;
	mbedtls_ssl_config conf;
	struct dtls_timing_context timer;

	mbedtls_ctr_drbg_init(&ctr_drbg);
	mbedtls_platform_set_printf(printk);
	mbedtls_ssl_init(&ssl_ctx);
	mbedtls_ssl_config_init(&conf);
	mbedtls_entropy_init(&entropy);
	mbedtls_entropy_add_source(&entropy, entropy_source, NULL,
				   MBEDTLS_ENTROPY_MAX_GATHER,
				   MBEDTLS_ENTROPY_SOURCE_STRONG);

	ret = mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy,
				    (const unsigned char *)pers, strlen(pers));
	if (ret != 0) {
		SYS_LOG_ERR("mbedtls_ctr_drbg_seed failed (-0x%x)", -ret);
		goto cleanup;
	}

	ret = mbedtls_ssl_config_defaults(&conf,
					  MBEDTLS_SSL_IS_CLIENT,
					  MBEDTLS_SSL_TRANSPORT_DATAGRAM,
					  MBEDTLS_SSL_PRESET_DEFAULT);
	if (ret != 0) {
		SYS_LOG_ERR("mbedtls_ssl_config_defaults"
				" failed (-0x%x)", -ret);
		goto cleanup;
	}

#if defined(MBEDTLS_DEBUG_C)
	mbedtls_debug_set_threshold(DEBUG_THRESHOLD);
#endif
	mbedtls_ssl_conf_rng(&conf, mbedtls_ctr_drbg_random, &ctr_drbg);
	mbedtls_ssl_conf_dbg(&conf, dtls_debug, NULL);
	ret = mbedtls_ssl_setup(&ssl_ctx, &conf);
	if (ret != 0) {
		SYS_LOG_ERR("mbedtls_ssl_setup failed (-0x%x)", -ret);
		goto cleanup;
	}

	/* Mbed only allows one set of PSK/PSK_ID */
#if defined(MBEDTLS_KEY_EXCHANGE__SOME__PSK_ENABLED)
	ret = mbedtls_ssl_conf_psk(&conf, psk, sizeof(psk),
				(const unsigned char *) psk_id,
				sizeof(psk_id) - 1);
	if (ret != 0) {
		SYS_LOG_ERR("mbedtls_ssl_conf_psk failed (-0x%x)", -ret);
		goto cleanup;
	}
#endif
	mbedtls_ssl_conf_handshake_timeout(&conf,
			DTLS_INITIAL_TIMEOUT, DTLS_MAXIMUM_TIMEOUT);
	mbedtls_ssl_set_timer_cb(&ssl_ctx, &timer, dtls_timing_set_delay,
				 dtls_timing_get_delay);
	mbedtls_ssl_set_bio(&ssl_ctx, &dtls_udp_ctx, dtls_tx, dtls_rx, NULL);

	SYS_LOG_DBG("DTLS starting security handshake");
	do {
		ret = mbedtls_ssl_handshake(&ssl_ctx);
	} while (ret == MBEDTLS_ERR_SSL_WANT_READ ||
		 ret == MBEDTLS_ERR_SSL_WANT_WRITE);
	if (ret != 0) {
		SYS_LOG_ERR("mbedtls_ssl_handshake failed -(0x%x)\n", -ret);
		goto cleanup;
	}
	SYS_LOG_DBG("DTLS handshake completed");
	k_sem_give(&dtls_read_sem);

#endif /* CONFIG_LWM2M_SECURITY_DTLS_SUPPORT */

	/* Do bootstrap or registration */
	if (clients[index].use_bootstrap) {
		set_sm_state(index, ENGINE_DO_BOOTSTRAP);
	} else {
		set_sm_state(index, ENGINE_DO_REGISTRATION);
	}

	return 0;

#if defined(CONFIG_LWM2M_SECURITY_DTLS_SUPPORT)
cleanup:
	mbedtls_ssl_free(&ssl_ctx);
	mbedtls_ssl_config_free(&conf);
	mbedtls_ctr_drbg_free(&ctr_drbg);
	mbedtls_entropy_free(&entropy);
	mbedtls_ssl_close_notify(&ssl_ctx);

	return ret;
#endif /* CONFIG_LWM2M_SECURITY_DTLS_SUPPORT */
}

static int sm_do_bootstrap(int index)
{
	struct zoap_packet request;
	struct net_pkt *pkt = NULL;
	struct zoap_pending *pending;
	int ret = 0;

	if (clients[index].use_bootstrap &&
	    clients[index].bootstrapped == 0 &&
	    clients[index].has_bs_server_info) {

		ret = zoap_init_message(clients[index].net_ctx,
					&request, &pkt, ZOAP_TYPE_CON,
					ZOAP_METHOD_POST, 0, NULL, 0,
					replies, do_bootstrap_reply_cb);
		if (ret) {
			goto cleanup;
		}

		zoap_add_option(&request, ZOAP_OPTION_URI_PATH,
				"bs", strlen("bs"));

		snprintf(query_buffer, sizeof(query_buffer) - 1,
			 "ep=%s", clients[index].ep_name);
		zoap_add_option(&request, ZOAP_OPTION_URI_QUERY,
				query_buffer, strlen(query_buffer));

		/* log the bootstrap attempt */
		SYS_LOG_DBG("Register ID with bootstrap server [%s] as '%s'",
			    sprint_ip_addr(&clients[index].bs_server),
			    query_buffer);

		pending = zoap_pending_next_unused(pendings, NUM_PENDINGS);
		if (!pending) {
			SYS_LOG_ERR("Unable to find a free pending to track "
				    "retransmissions.");
			ret = -ENOMEM;
			goto cleanup;
		}

		ret = zoap_pending_init(pending, &request,
					&clients[index].bs_server);
		if (ret < 0) {
			SYS_LOG_ERR("Unable to initialize a pending "
				    "retransmission (err:%d).", ret);
			goto cleanup;
		}

		ret = udp_sendto(pkt, &clients[index].bs_server);
		if (ret < 0) {
			SYS_LOG_ERR("Error sending LWM2M packet (err:%d).",
				    ret);
			goto cleanup;
		}

		zoap_pending_cycle(pending);
		k_delayed_work_submit(&retransmit_work, pending->timeout);
		set_sm_state(index, ENGINE_BOOTSTRAP_SENT);
	}

cleanup:
	if (pkt) {
		net_pkt_unref(pkt);
	}

	return ret;
}

static int sm_bootstrap_done(int index)
{
	/* TODO: Fix this */
	/* check that we should still use bootstrap */
	if (clients[index].use_bootstrap) {
#ifdef CONFIG_LWM2M_SECURITY_OBJ_SUPPORT
		int i;

		SYS_LOG_DBG("*** Bootstrap - checking for server info ...");

		/* get the server URI */
		if (sec_data->server_uri_len > 0) {
			/* TODO: Write endpoint parsing function */
#if 0
			if (!parse_endpoint(sec_data->server_uri,
					    sec_data->server_uri_len,
					    &clients[index].reg_server)) {
#else
			if (true) {
#endif
				SYS_LOG_ERR("Failed to parse URI!");
			} else {
				clients[index].has_registration_info = 1;
				clients[index].registered = 0;
				clients[index].bootstrapped++;
			}
		} else {
			SYS_LOG_ERR("** failed to parse URI");
		}

		/* if we did not register above - then fail this and restart */
		if (clients[index].bootstrapped == 0) {
			/* Not ready - Retry with the bootstrap server again */
			set_sm_state(index, ENGINE_DO_BOOTSTRAP);
		} else {
			set_sm_state(index, ENGINE_DO_REGISTRATION);
		}
	} else {
#endif
		set_sm_state(index, ENGINE_DO_REGISTRATION);
	}

	return 0;
}

static int sm_send_registration(int index, bool send_obj_support_data,
				zoap_reply_t reply_cb)
{
	struct zoap_packet request;
	struct net_pkt *pkt = NULL;
	struct zoap_pending *pending;
	u8_t *payload;
	u16_t client_data_len, len;
	int ret = 0;

	/* remember the last reg time */
	clients[index].last_update = k_uptime_get();
	ret = zoap_init_message(clients[index].net_ctx,
				&request, &pkt, ZOAP_TYPE_CON,
				ZOAP_METHOD_POST, 0, NULL, 0,
				replies, reply_cb);
	if (ret) {
		goto cleanup;
	}

	zoap_add_option(&request, ZOAP_OPTION_URI_PATH,
			LWM2M_RD_CLIENT_URI,
			strlen(LWM2M_RD_CLIENT_URI));

	if (!clients[index].registered) {
		/* include client endpoint in URI QUERY on 1st registration */
		snprintf(query_buffer, sizeof(query_buffer) - 1,
			 "ep=%s", clients[index].ep_name);
		zoap_add_option(&request, ZOAP_OPTION_URI_QUERY,
				query_buffer, strlen(query_buffer));
	} else {
		/* include server endpoint in URI PATH otherwise */
		zoap_add_option(&request, ZOAP_OPTION_URI_PATH,
				clients[index].server_ep,
				strlen(clients[index].server_ep));
	}

	snprintf(query_buffer, sizeof(query_buffer) - 1,
		 "lt=%d", clients[index].lifetime);
	zoap_add_option(&request, ZOAP_OPTION_URI_QUERY,
			query_buffer, strlen(query_buffer));
	/* TODO: add supported binding query string */

	if (send_obj_support_data) {
		/* generate the rd data */
		client_data_len = lwm2m_engine_get_rd_data(client_data,
							   sizeof(client_data));
		payload = zoap_packet_get_payload(&request, &len);
		if (!payload) {
			ret = -EINVAL;
			goto cleanup;
		}

		memcpy(payload, client_data, client_data_len);
		ret = zoap_packet_set_used(&request, client_data_len);
		if (ret) {
			goto cleanup;
		}
	}

	pending = zoap_pending_next_unused(pendings, NUM_PENDINGS);
	if (!pending) {
		SYS_LOG_ERR("Unable to find a free pending to track "
			    "retransmissions.");
		ret = -ENOMEM;
		goto cleanup;
	}

	ret = zoap_pending_init(pending, &request,
				&clients[index].reg_server);
	if (ret < 0) {
		SYS_LOG_ERR("Unable to initialize a pending "
			    "retransmission (err:%d).", ret);
		goto cleanup;
	}

	/* log the registration attempt */
	SYS_LOG_DBG("registration sent [%s]",
		    sprint_ip_addr(&clients[index].reg_server));

	ret = udp_sendto(pkt, &clients[index].reg_server);
	if (ret < 0) {
		SYS_LOG_ERR("Error sending LWM2M packet (err:%d).",
			    ret);
		goto cleanup;
	}

	zoap_pending_cycle(pending);
	k_delayed_work_submit(&retransmit_work, pending->timeout);

cleanup:
	if (pkt) {
		net_pkt_unref(pkt);
	}

	return ret;
}

static int sm_do_registration(int index)
{
	int ret = 0;

	if (clients[index].use_registration &&
	    !clients[index].registered &&
	    clients[index].has_registration_info) {
		ret = sm_send_registration(index, true,
					   do_registration_reply_cb);
		if (!ret) {
			set_sm_state(index, ENGINE_REGISTRATION_SENT);
		} else {
			SYS_LOG_ERR("Registration err: %d", ret);
		}
	}

	return ret;
}

static int sm_registration_done(int index)
{
	int ret = 0;
	bool forced_update;

	/* check for lifetime seconds - 1 so that we can update early */
	if (clients[index].registered &&
	    (clients[index].trigger_update ||
	     ((clients[index].lifetime - SECONDS_TO_UPDATE_EARLY) <=
	      (k_uptime_get() - clients[index].last_update) / 1000))) {
		forced_update = clients[index].trigger_update;
		clients[index].trigger_update = 0;
		ret = sm_send_registration(index, forced_update,
					   do_update_reply_cb);
		if (!ret) {
			set_sm_state(index, ENGINE_UPDATE_SENT);
		} else {
			SYS_LOG_ERR("Registration update err: %d", ret);
		}
	}

	return ret;
}

static int sm_do_deregister(int index)
{
	struct zoap_packet request;
	struct net_pkt *pkt = NULL;
	struct zoap_pending *pending;
	int ret;

	ret = zoap_init_message(clients[index].net_ctx,
				&request, &pkt, ZOAP_TYPE_CON,
				ZOAP_METHOD_DELETE, 0, NULL, 0,
				replies, do_deregister_reply_cb);
	if (ret) {
		goto cleanup;
	}

	zoap_add_option(&request, ZOAP_OPTION_URI_PATH,
			clients[index].server_ep,
			strlen(clients[index].server_ep));

	pending = zoap_pending_next_unused(pendings, NUM_PENDINGS);
	if (!pending) {
		SYS_LOG_ERR("Unable to find a free pending to track "
			    "retransmissions.");
		ret = -ENOMEM;
		goto cleanup;
	}

	ret = zoap_pending_init(pending, &request,
				&clients[index].reg_server);
	if (ret < 0) {
		SYS_LOG_ERR("Unable to initialize a pending "
			    "retransmission (err:%d).", ret);
		goto cleanup;
	}

	SYS_LOG_INF("Deregister from '%s'", clients[index].server_ep);

	ret = udp_sendto(pkt, &clients[index].reg_server);
	if (ret < 0) {
		SYS_LOG_ERR("Error sending LWM2M packet (err:%d).",
			    ret);
		goto cleanup;
	}

	zoap_pending_cycle(pending);
	k_delayed_work_submit(&retransmit_work, pending->timeout);
	set_sm_state(index, ENGINE_DEREGISTER_SENT);

cleanup:
	if (pkt) {
		net_pkt_unref(pkt);
	}

	return ret;
}

static void lwm2m_rd_client_service(void)
{
	int index;

	while (true) {
		for (index = 0; index < client_count; index++) {
			STACK_ANALYZE("rd_service", lwm2m_rd_client_thread_stack);

			switch (get_sm_state(index)) {

			case ENGINE_INIT:
				sm_do_init(index);
				break;

			case ENGINE_SECURITY_HANDSHAKE:
				sm_do_sec_handshake(index);
				break;

			case ENGINE_DO_BOOTSTRAP:
				sm_do_bootstrap(index);
				break;

			case ENGINE_BOOTSTRAP_SENT:
				/* wait for bootstrap to be done */
				break;

			case ENGINE_BOOTSTRAP_DONE:
				sm_bootstrap_done(index);
				break;

			case ENGINE_DO_REGISTRATION:
				sm_do_registration(index);
				break;

			case ENGINE_REGISTRATION_SENT:
				/* wait registration to be done */
				break;

			case ENGINE_REGISTRATION_DONE:
				sm_registration_done(index);
				break;

			case ENGINE_UPDATE_SENT:
				/* wait update to be done */
				break;

			case ENGINE_DEREGISTER:
				sm_do_deregister(index);
				break;

			case ENGINE_DEREGISTER_SENT:
				break;

			case ENGINE_DEREGISTER_FAILED:
				break;

			case ENGINE_DEREGISTERED:
				break;

			default:
				SYS_LOG_ERR("Unhandled state: %d",
					    get_sm_state(index));

			}

			k_yield();
		}

		/*
		 * TODO: calculate the diff between the start of the loop
		 * and subtract that from the update interval
		 */
		k_sleep(K_MSEC(STATE_MACHINE_UPDATE_INTERVAL));
	}
}

static bool peer_addr_exist(struct sockaddr *peer_addr)
{
	bool ret = false;
	int i;

	/* look for duplicate peer_addr */
	for (i = 0; i < client_count; i++) {
#if defined(CONFIG_NET_IPV6)
		if (peer_addr->family == AF_INET6 && net_ipv6_addr_cmp(
				&net_sin6(&clients[i].bs_server)->sin6_addr,
				&net_sin6(peer_addr)->sin6_addr)) {
			ret = true;
			break;
		}

		if (peer_addr->family == AF_INET6 && net_ipv6_addr_cmp(
				&net_sin6(&clients[i].reg_server)->sin6_addr,
				&net_sin6(peer_addr)->sin6_addr)) {
			ret = true;
			break;
		}
#endif

#if defined(CONFIG_NET_IPV4)
		if (peer_addr->family == AF_INET && net_ipv4_addr_cmp(
				&net_sin(&clients[i].bs_server)->sin_addr,
				&net_sin(peer_addr)->sin_addr)) {
			ret = true;
			break;
		}

		if (peer_addr->family == AF_INET && net_ipv4_addr_cmp(
				&net_sin(&clients[i].reg_server)->sin_addr,
				&net_sin(peer_addr)->sin_addr)) {
			ret = true;
			break;
		}
#endif
	}

	return ret;
}

static void set_ep_ports(int index)
{
#if defined(CONFIG_NET_IPV6)
	if (clients[index].bs_server.family == AF_INET6) {
		net_sin6(&clients[index].bs_server)->sin6_port =
			htons(LWM2M_BOOTSTRAP_PORT);
	}

	if (clients[index].reg_server.family == AF_INET6) {
		net_sin6(&clients[index].reg_server)->sin6_port =
			htons(LWM2M_PEER_PORT);
	}
#endif

#if defined(CONFIG_NET_IPV4)
	if (clients[index].bs_server.family == AF_INET) {
		net_sin(&clients[index].bs_server)->sin_port =
			htons(LWM2M_BOOTSTRAP_PORT);
	}

	if (clients[index].reg_server.family == AF_INET) {
		net_sin(&clients[index].reg_server)->sin_port =
			htons(LWM2M_PEER_PORT);
	}
#endif
}

int lwm2m_rd_client_start(struct net_context *net_ctx,
			  struct sockaddr *peer_addr,
			  const char *ep_name)
{
	int index;
	int ret = 0;

	if (client_count + 1 > CLIENT_INSTANCE_COUNT) {
		return -ENOMEM;
	}

	if (peer_addr_exist(peer_addr)) {
		return -EEXIST;
	}

	/* Server Peer IP information */
	/* TODO: use server URI data from security */
	index = client_count;
	client_count++;
	clients[index].net_ctx = net_ctx;
	memcpy(&clients[index].reg_server, peer_addr, sizeof(struct sockaddr));
	memcpy(&clients[index].bs_server, peer_addr, sizeof(struct sockaddr));
	set_ep_ports(index);
	strncpy(clients[index].ep_name, ep_name,
		sizeof(clients[index].ep_name)-1);

#if defined(CONFIG_LWM2M_SECURITY_DTLS_SUPPORT)
	/* TODO: Might need one context per RD connection */
	k_sem_init(&dtls_udp_ctx.rx_sem, 0, UINT_MAX);
	dtls_udp_ctx.rx_pkt = NULL;
	dtls_udp_ctx.remaining = 0;
	dtls_udp_ctx.net_ctx = net_ctx;

	net_context_recv(net_ctx, dtls_receive, K_NO_WAIT, &dtls_udp_ctx);
	if (ret) {
		/* TODO: Abort nicely */
		SYS_LOG_ERR("Could not set receive for net context (err:%d)", ret);
	}
#endif

	SYS_LOG_INF("LWM2M Client: %s", clients[index].ep_name);

	return 0;
}

static int lwm2m_rd_client_init(struct device *dev)
{
	k_thread_create(&lwm2m_rd_client_thread_data,
			&lwm2m_rd_client_thread_stack[0],
			K_THREAD_STACK_SIZEOF(lwm2m_rd_client_thread_stack),
			(k_thread_entry_t) lwm2m_rd_client_service,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);
	SYS_LOG_DBG("LWM2M RD client thread started");

#if defined(CONFIG_LWM2M_SECURITY_DTLS_SUPPORT)
	k_sem_init(&dtls_read_sem, 0, UINT_MAX);
	k_thread_create(&lwm2m_dtls_read_thread_data,
			&lwm2m_dtls_read_thread_stack[0],
			K_THREAD_STACK_SIZEOF(lwm2m_dtls_read_thread_stack),
			(k_thread_entry_t) lwm2m_dtls_read_service,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);
	SYS_LOG_DBG("LWM2M DTLS read thread started");
#endif

	return 0;
}

SYS_INIT(lwm2m_rd_client_init, APPLICATION,
	 CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
