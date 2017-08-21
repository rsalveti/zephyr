/*
 * Copyright (c) 2017 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * TODO:
 * Support PULL transfer method (from server)
 */

#define SYS_LOG_DOMAIN "lwm2m_obj_firmware_pull"
#define SYS_LOG_LEVEL CONFIG_SYS_LOG_LWM2M_LEVEL
#include <logging/sys_log.h>

#include <stdio.h>
#include <net/zoap.h>
#include <net/net_core.h>
#include <net/net_pkt.h>
#include <net/udp.h>

#include "lwm2m_object.h"
#include "lwm2m_engine.h"

#define STATE_IDLE		0
#define STATE_CONNECTING	1

#define PACKAGE_URI_LEN				255

#define BUF_ALLOC_TIMEOUT K_SECONDS(1)

static u8_t transfer_state;
static struct k_work firmware_work;
static char firmware_uri[PACKAGE_URI_LEN];
static struct sockaddr firmware_addr;
static struct net_context *firmware_net_ctx;
static struct k_delayed_work retransmit_work;

#define NUM_PENDINGS	CONFIG_LWM2M_ENGINE_MAX_PENDING
#define NUM_REPLIES	CONFIG_LWM2M_ENGINE_MAX_REPLIES
static struct zoap_pending pendings[NUM_PENDINGS];
static struct zoap_reply replies[NUM_REPLIES];
static struct zoap_block_context firmware_block_ctx;

extern void set_update_state_machine(u8_t state, u8_t result);

static void
firmware_udp_receive(struct net_context *ctx, struct net_pkt *pkt, int status,
		     void *user_data)
{
	lwm2m_udp_receive(ctx, pkt, pendings, NUM_PENDINGS,
			  replies, NUM_REPLIES, true, NULL);
}

static void retransmit_request(struct k_work *work)
{
	struct zoap_pending *pending;
	int r;

	pending = zoap_pending_next_to_expire(pendings, NUM_PENDINGS);
	if (!pending) {
		return;
	}

	r = lwm2m_udp_sendto(pending->pkt, &pending->addr);
	if (r < 0) {
		return;
	}

	if (!zoap_pending_cycle(pending)) {
		zoap_pending_clear(pending);
		return;
	}

	k_delayed_work_submit(&retransmit_work, pending->timeout);
}

static int transfer_request(struct zoap_block_context *ctx,
			    const u8_t *token, u8_t tkl,
			    zoap_reply_t reply_cb)
{
	struct zoap_packet request;
	struct net_pkt *pkt = NULL;
	struct zoap_pending *pending = NULL;
	struct zoap_reply *reply = NULL;
	int ret;

	ret = lwm2m_init_message(firmware_net_ctx, &request, &pkt,
				 ZOAP_TYPE_CON, ZOAP_METHOD_GET,
				 0, token, tkl);
	if (ret) {
		goto cleanup;
	}

#if defined(CONFIG_LWM2M_FIRMWARE_UPDATE_PULL_COAP_PROXY_SUPPORT)
	char *uri_path =
		CONFIG_LWM2M_FIRMWARE_UPDATE_PULL_COAP_PROXY_URI_PATH;
	ret = zoap_add_option(&request, ZOAP_OPTION_URI_PATH,
			uri_path, strlen(uri_path));
	if (ret < 0) {
		SYS_LOG_ERR("Error adding URI_PATH '%s'", uri_path);
		goto cleanup;
	}
#else
	/* hard code URI path here -- should be pulled from package_uri */
	ret = zoap_add_option(&request, ZOAP_OPTION_URI_PATH,
			"large-create", sizeof("large-create") - 1);
	ret = zoap_add_option(&request, ZOAP_OPTION_URI_PATH,
			"1", sizeof("1") - 1);
	if (ret < 0) {
		SYS_LOG_ERR("Error adding URI_QUERY 'large'");
		goto cleanup;
	}
#endif
	ret = zoap_add_block2_option(&request, ctx);
	if (ret) {
		SYS_LOG_ERR("Unable to add block2 option.");
		goto cleanup;
	}
#if defined(CONFIG_LWM2M_FIRMWARE_UPDATE_PULL_COAP_PROXY_SUPPORT)
	ret = zoap_add_option(&request, ZOAP_OPTION_PROXY_URI,
			firmware_uri, strlen(firmware_uri));
	if (ret < 0) {
		SYS_LOG_ERR("Error adding PROXY_URI '%s'", firmware_uri);
		goto cleanup;
	}
#endif

	pending = lwm2m_init_message_pending(&request, &firmware_addr,
					     pendings, NUM_PENDINGS);
	if (!pending) {
		ret = -ENOMEM;
		goto cleanup;
	}

	/* set the reply handler */
	if (reply_cb) {
		reply = zoap_reply_next_unused(replies, NUM_REPLIES);
		if (!reply) {
			SYS_LOG_ERR("No resources for waiting for replies.");
			ret = -ENOMEM;
			goto cleanup;
		}

		zoap_reply_init(reply, &request);
		reply->reply = reply_cb;
	}

	/* send request */
	ret = lwm2m_udp_sendto(pkt, &firmware_addr);
	if (ret < 0) {
		SYS_LOG_ERR("Error sending LWM2M packet (err:%d).",
			    ret);
		goto cleanup;
	}

	zoap_pending_cycle(pending);
	k_delayed_work_submit(&retransmit_work, pending->timeout);
	return 0;

cleanup:
	lwm2m_init_message_cleanup(pkt, pending, reply);
	set_update_state_machine(STATE_IDLE, RESULT_UPDATE_FAILED);
	return ret;
}

static int transfer_empty_ack(u16_t mid)
{
	struct zoap_packet request;
	struct net_pkt *pkt = NULL;
	int ret;

	ret = lwm2m_init_message(firmware_net_ctx, &request, &pkt,
			ZOAP_TYPE_ACK, ZOAP_CODE_EMPTY, mid, NULL, 0);
	if (ret) {
		goto cleanup;
	}

	ret = lwm2m_udp_sendto(pkt, &firmware_addr);
	if (ret < 0) {
		SYS_LOG_ERR("Error sending LWM2M packet (err:%d).",
			    ret);
		goto cleanup;
	}

	return 0;

cleanup:
	lwm2m_init_message_cleanup(pkt, NULL, NULL);
	return ret;
}

static int
do_firmware_transfer_reply_cb(const struct zoap_packet *response,
			      struct zoap_reply *reply,
			      const struct sockaddr *from)
{
	int ret;
	size_t transfer_offset = 0;
	u16_t payload_len;
	u8_t *payload;
	struct zoap_packet *check_response = (struct zoap_packet *)response;
	lwm2m_engine_set_data_cb_t callback;
	bool last_block = false;
	int rclass, rdetail;

	/* If separated response (ACK) return and wait for response */
	if (zoap_header_get_type(response) == ZOAP_TYPE_ACK) {
		return 0;
	}

	/* Send back ACK so the server knows we received the pkt */
	ret = transfer_empty_ack(zoap_header_get_id(check_response));
	if (ret < 0) {
		SYS_LOG_ERR("Error transmitting ACK");
		return ret;
	}

	/* Check for valid pkts by checking response code */
	rclass = ZOAP_RESPONSE_CODE_CLASS(zoap_header_get_code(response));
	rdetail = ZOAP_RESPONSE_CODE_DETAIL(zoap_header_get_code(response));
	if (rclass != 2) {
		SYS_LOG_ERR("Invalid response code: %d.%d, download failed",
				rclass, rdetail);
		set_update_state_machine(STATE_IDLE, RESULT_CONNECTION_LOST);
		return -EINVAL;
	}

	ret = zoap_update_from_block(check_response, &firmware_block_ctx);
	if (ret < 0) {
		SYS_LOG_ERR("Error from block update: %d", ret);
		return ret;
	}
	SYS_LOG_DBG("Block: total: %zd, current: %zd",
				firmware_block_ctx.total_size,
				firmware_block_ctx.current);

	payload = zoap_packet_get_payload(check_response, &payload_len);
	if (payload_len > 0) {
		transfer_offset = zoap_next_block(response,
						  &firmware_block_ctx);
		last_block = transfer_offset == 0;
		SYS_LOG_DBG("pkt payload len %d, last block: %d",
					payload_len, last_block);

		/* application callback */
		callback = lwm2m_firmware_get_write_cb();
		if (callback) {
			/* TODO: define return values and set state machine */
			ret = callback(0, payload, payload_len, last_block,
					firmware_block_ctx.total_size);
			if (ret < 0) {
				SYS_LOG_ERR("Callback failure (%d)", ret);
				return ret;
			}
		}
	}

	if (last_block) {
		set_update_state_machine(STATE_DOWNLOADED, RESULT_DEFAULT);
	} else {
		ret = transfer_request(&firmware_block_ctx, zoap_next_token(),
				8, do_firmware_transfer_reply_cb);
	}

	return ret;
}

static enum zoap_block_size default_block_size(void)
{
	switch (CONFIG_LWM2M_FIRMWARE_UPDATE_PULL_COAP_BLOCK_SIZE) {
	case 16:
		return ZOAP_BLOCK_16;
	case 32:
		return ZOAP_BLOCK_32;
	case 64:
		return ZOAP_BLOCK_64;
	case 128:
		return ZOAP_BLOCK_128;
	case 256:
		return ZOAP_BLOCK_256;
	case 512:
		return ZOAP_BLOCK_512;
	case 1024:
		return ZOAP_BLOCK_1024;
	}

	return ZOAP_BLOCK_256;
}

static void firmware_transfer(struct k_work *work)
{
#if defined(CONFIG_NET_IPV6)
	static struct sockaddr_in6 any_addr6 = { .sin6_addr = IN6ADDR_ANY_INIT,
						.sin6_family = AF_INET6 };
#endif
#if defined(CONFIG_NET_IPV4)
	static struct sockaddr_in any_addr4 = { .sin_addr = INADDR_ANY_INIT,
					       .sin_family = AF_INET };
#endif
	struct net_if *iface;
	char *server_addr;
	int ret, port, family;

	/* Server Peer IP information */
	/* TODO: use parser on server addr (with and without proxy) */
#if defined(CONFIG_LWM2M_FIRMWARE_UPDATE_PULL_COAP_PROXY_SUPPORT)
	server_addr = CONFIG_LWM2M_FIRMWARE_UPDATE_PULL_COAP_PROXY_ADDR;
	port = CONFIG_LWM2M_FIRMWARE_UPDATE_PULL_COAP_PROXY_PORT;
#else
	/* HACK: use firmware_uri directly as IP address */
	server_addr = firmware_uri;
	/* TODO: hard code port for now */
	port = 5685;
#endif
#if defined(CONFIG_NET_IPV6)
	family = AF_INET6;
#endif
#if defined(CONFIG_NET_IPV4)
	family = AF_INET;
#endif

#if defined(CONFIG_NET_IPV6)
	if (family == AF_INET6) {
		firmware_addr.sa_family = family;
		net_addr_pton(firmware_addr.sa_family, server_addr,
			      &net_sin6(&firmware_addr)->sin6_addr);
		net_sin6(&firmware_addr)->sin6_port = htons(port);
	}
#endif

#if defined(CONFIG_NET_IPV4)
	if (family == AF_INET) {
		firmware_addr.sa_family = family;
		net_addr_pton(firmware_addr.sa_family, server_addr,
			      &net_sin(&firmware_addr)->sin_addr);
		net_sin(&firmware_addr)->sin_port = htons(port);
	}
#endif

	ret = net_context_get(firmware_addr.sa_family, SOCK_DGRAM, IPPROTO_UDP,
			      &firmware_net_ctx);
	if (ret) {
		NET_ERR("Could not get an UDP context (err:%d)", ret);
		return;
	}

	iface = net_if_get_default();
	if (!iface) {
		NET_ERR("Could not find default interface");
		goto cleanup;
	}

#if defined(CONFIG_NET_IPV6)
	if (firmware_addr.sa_family == AF_INET6) {
		ret = net_context_bind(firmware_net_ctx,
				       (struct sockaddr *)&any_addr6,
				       sizeof(any_addr6));
	}
#endif

#if defined(CONFIG_NET_IPV4)
	if (firmware_addr.sa_family == AF_INET) {
		ret = net_context_bind(firmware_net_ctx,
				       (struct sockaddr *)&any_addr4,
				       sizeof(any_addr4));
	}
#endif

	if (ret) {
		NET_ERR("Could not bind the UDP context (err:%d)", ret);
		goto cleanup;
	}

	SYS_LOG_DBG("Attached to server %s, port %d", server_addr, port);
	ret = net_context_recv(firmware_net_ctx, firmware_udp_receive, 0, NULL);
	if (ret) {
		SYS_LOG_ERR("Could not set receive for net context (err:%d)",
			    ret);
		goto cleanup;
	}

	/* reset block transfer context */
	zoap_block_transfer_init(&firmware_block_ctx, default_block_size(), 0);

	set_update_state_machine(STATE_DOWNLOADING, RESULT_DEFAULT);
	transfer_request(&firmware_block_ctx, zoap_next_token(), 8,
			 do_firmware_transfer_reply_cb);
	return;

cleanup:
	if (firmware_net_ctx) {
		net_context_put(firmware_net_ctx);
	}
	set_update_state_machine(STATE_IDLE, RESULT_UPDATE_FAILED);
}

/* TODO: */
int lwm2m_firmware_cancel_transfer(void)
{
	return 0;
}

int lwm2m_firmware_start_transfer(char *package_uri)
{
	/* free up old context */
	if (firmware_net_ctx) {
		net_context_put(firmware_net_ctx);
	}

	if (transfer_state == STATE_IDLE) {
		k_work_init(&firmware_work, firmware_transfer);
		k_delayed_work_init(&retransmit_work, retransmit_request);

		/* start file transfer work */
		strncpy(firmware_uri, package_uri, PACKAGE_URI_LEN - 1);
		k_work_submit(&firmware_work);
		return 0;
	}

	return -1;
}
