/*
 * Copyright (c) 2017 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LWM2M_ENGINE_H
#define LWM2M_ENGINE_H

#include "lwm2m_object.h"

#define ZOAP_RESPONSE_CODE_CLASS(x)	(x >> 5)
#define ZOAP_RESPONSE_CODE_DETAIL(x)	(x & 0x1F)

/* TODO: */
#define NOTIFY_OBSERVER(o, i, r)	engine_notify_observer(o, i, r)
#define NOTIFY_OBSERVER_PATH(path)	engine_notify_observer_path(path)

char *sprint_ip_addr(const struct sockaddr *addr);
char *sprint_token(const u8_t *token, u8_t tkl);

int engine_notify_observer(u16_t obj_id, u16_t obj_inst_id, u16_t res_id);
int engine_notify_observer_path(struct lwm2m_obj_path *path);

void engine_register_obj(struct lwm2m_engine_obj *obj);
void engine_unregister_obj(struct lwm2m_engine_obj *obj);
struct lwm2m_engine_obj_field *
get_engine_obj_field(struct lwm2m_engine_obj *obj, int res_id);
int  engine_create_obj_inst(u16_t obj_id, u16_t obj_inst_id,
			    struct lwm2m_engine_obj_inst **obj_inst);
int  engine_delete_obj_inst(u16_t obj_id, u16_t obj_inst_id);
int  get_or_create_engine_obj(struct lwm2m_engine_context *context,
			      struct lwm2m_engine_obj_inst **obj_inst,
			      u8_t *created);

int check_perm(u16_t obj_id, u16_t res_id, u8_t perm);
int zoap_init_message(struct net_context *net_ctx, struct zoap_packet *zpkt,
		      struct net_pkt **pkt, u8_t type, u8_t code, u16_t mid,
		      const u8_t *token, u8_t tkl, zoap_reply_t reply_cb);
u16_t lwm2m_engine_get_rd_data(u8_t *client_data, u16_t size);

int engine_read_handler(struct lwm2m_engine_obj_inst *obj_inst,
			struct lwm2m_engine_res_inst *res,
			struct lwm2m_engine_obj_field *obj_field,
			struct lwm2m_engine_context *context);
int engine_write_handler(struct lwm2m_engine_obj_inst *obj_inst,
			 struct lwm2m_engine_res_inst *res,
			 struct lwm2m_engine_obj_field *obj_field,
			 struct lwm2m_engine_context *context);
int engine_write_attr_handler(struct lwm2m_engine_obj *obj,
			      struct lwm2m_engine_context *context);
int engine_exec_handler(struct lwm2m_engine_obj *obj,
			struct lwm2m_engine_context *context);
int engine_delete_handler(struct lwm2m_engine_obj *obj,
			  struct lwm2m_engine_context *context);

#endif /* LWM2M_ENGINE_H */
