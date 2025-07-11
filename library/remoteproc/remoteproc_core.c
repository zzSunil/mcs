/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2023. All rights reserved
 *
 * SPDX-License-Identifier: MulanPSL-2.0
 */

#include <stdio.h>
#include <syslog.h>
#include <metal/alloc.h>
#include <metal/io.h>
#include <metal/cache.h>
#include <openamp/remoteproc.h>
#include <openamp/remoteproc_loader.h>

#include <mica/mica.h>

#include "remoteproc/remoteproc_module.h"

#define CPU_OFF_FUNCID     0x84000002

/*
 * Related operations for remote processor, including start/stop/notify callbacks
 */
extern const struct remoteproc_ops rproc_bare_metal_ops;
extern const struct remoteproc_ops rproc_jailhouse_ops;

METAL_DECLARE_LIST(g_client_list);

static int store_open(void *store, const char *path, const void **image_data)
{
	long fsize;
	struct img_store *image = store;

	image->file = fopen(path, "r");
	if (!image->file) {
		syslog(LOG_ERR, "Cannot open the file:%s\n", path);
		return -EINVAL;
	}

	fseek(image->file, 0, SEEK_END);
	fsize = ftell(image->file);
	fseek(image->file, 0, SEEK_SET);

	image->buf = malloc(fsize + 1);
	if (!image->buf) {
		fclose(image->file);
		return -ENOMEM;
	}

	*image_data = image->buf;

	return fread(image->buf, 1, fsize, image->file);
}

static void store_close(void *store)
{
	struct img_store *image = store;

	free(image->buf);
	fclose(image->file);
}

static int store_load(void *store, size_t offset, size_t size,
		      const void **data, metal_phys_addr_t pa,
		      struct metal_io_region *io, char is_blocking)
{
	struct img_store *image = store;
	char *tmp;

	if (pa == METAL_BAD_PHYS) {
		if (data == NULL) {
			syslog(LOG_ERR, "%s failed: data is NULL while pa is ANY\n", __func__);
			return -EINVAL;
		}

		tmp = realloc(image->buf, size);
		if (!tmp)
			return -ENOMEM;

		image->buf = tmp;
		*data = tmp;
	} else {
		tmp = metal_io_phys_to_virt(io, pa);
		if (!tmp)
			return -EINVAL;
	}

	fseek(image->file, offset, SEEK_SET);

	return fread(tmp, 1, size, image->file);
}

/*
 * Image store operations.
 * @open: open the "firmware" to prepare loading
 * @close: close the "firmware" to clean up after loading
 * @load: load the firmware contents to target memory
 */
static const struct image_store_ops mem_image_store_ops = {
	.open     = store_open,
	.close    = store_close,
	.load     = store_load,
	.features = SUPPORT_SEEK,
};

int create_client(struct mica_client *client)
{
	struct remoteproc *rproc;
	const struct remoteproc_ops *ops;

	if (client->ped == BARE_METAL)
	{
		DEBUG_PRINT("create_client register a baremetal ops!\n");
		ops = &rproc_bare_metal_ops;
	}
	else if (client->ped == JAILHOUSE)
		ops = &rproc_jailhouse_ops;
	else
		return -EINVAL;
	
	DEBUG_PRINT("create client going to call remoteproc_init for baremetal ops!\n");
	rproc = remoteproc_init(&client->rproc, ops, client);
	if (!rproc) {
		syslog(LOG_ERR, "remoteproc init failed\n");
		return -EINVAL;
	}
	DEBUG_PRINT("create client call remoteproc_init for baremetal ops success!!\n");
	
	metal_list_add_tail(&g_client_list, &client->node);
	metal_list_init(&client->services);
	DEBUG_PRINT("create client add to g_client_list success! and init service list success!\n");
	return 0;
}

int load_client_image(struct mica_client *client)
{
	int ret;
	struct remoteproc *rproc = &client->rproc;
	struct img_store store = { 0 };
	const void *img_data;

	ret = store_open(&store, client->path, &img_data);
	if (ret <= 0) {
		syslog(LOG_ERR, "failed to open firmware %d", ret);
		return -EINVAL;
	}

	ret = remoteproc_config(rproc, &store);
	if (ret) {
		syslog(LOG_ERR, "remoteproc config failed, ret:%d", ret);
		store_close(&store);
		return -EINVAL;
	}
	store_close(&store);

	/* If the remote is already in a running state, skip the load */
	if (rproc->rsc_table && rproc->state == RPROC_READY) {
		DEBUG_PRINT("remote is ready, no need to load it");
		return ret;
	}

	ret = remoteproc_load(rproc, client->path, &store, &mem_image_store_ops, NULL);
	if (!rproc->rsc_table) {
		syslog(LOG_ERR, "failed to parse rsc table, please check the rsctable\n");
		return -EINVAL;
	}

	return ret;
}

int start_client(struct mica_client *client)
{
	struct remoteproc *rproc = &client->rproc;

	return remoteproc_start(rproc);
}

void stop_client(struct mica_client *client)
{
	if (client != NULL)
		remoteproc_shutdown(&client->rproc);
}

void destory_client(struct mica_client *client)
{
	/*
	 * To get the updated list in remoteproc_remove(),
	 * the node must be deleted first.
	 */
	if (client != NULL) {
		metal_list_del(&client->node);
		remoteproc_remove(&client->rproc);
	}
}

const char *show_client_status(struct mica_client *client)
{
	struct resource_table *rsc_table;
	/* Match with rproc_state */
	static const char * const client_status[RPROC_LAST] = {
		[RPROC_OFFLINE]		= "Offline",
		[RPROC_CONFIGURED]	= "Configured",
		[RPROC_READY]		= "Ready",
		[RPROC_RUNNING]		= "Running",
		[RPROC_SUSPENDED]	= "Suspended",
		[RPROC_ERROR]		= "Error",
		[RPROC_STOPPED]		= "Stopped",
	};

	if (client->rproc.state >= RPROC_OFFLINE && client->rproc.state < RPROC_LAST) {
		if (client->rproc.state == RPROC_RUNNING) {
			rsc_table = client->rproc.rsc_table;
			metal_cache_invalidate(rsc_table->reserved, sizeof(rsc_table->reserved));
			if (rsc_table->reserved[0] == CPU_OFF_FUNCID) { /* check rproc offline */
				mica_stop(client);
				return client_status[RPROC_OFFLINE];
			}
		}

		return client_status[client->rproc.state];
	}
	else
		return NULL;
}
