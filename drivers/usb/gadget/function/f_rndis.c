/*
 * f_rndis.c -- RNDIS link function driver
 *
 * Copyright (C) 2003-2005,2008 David Brownell
 * Copyright (C) 2003-2004 Robert Schwebel, Benedikt Spranger
 * Copyright (C) 2008 Nokia Corporation
 * Copyright (C) 2009 Samsung Electronics
 *                    Author: Michal Nazarewicz (mina86@mina86.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2018 KYOCERA Corporation
 * (C) 2019 KYOCERA Corporation
 */

/* #define VERBOSE_DEBUG */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/etherdevice.h>

#include <linux/atomic.h>

#include "u_ether.h"
#include "u_ether_configfs.h"
#include "u_rndis.h"
#include "rndis.h"
#include "configfs.h"

#ifdef CONFIG_MTK_MD_DIRECT_TETHERING_SUPPORT
#include "port_ipc.h"
#include "ccci_ipc_msg_id.h"
#include "mtk_gadget.h"
#include "pkt_track.h"
#endif

#define F_RNDIS_LOG "[USB_RNDIS]"

/*
 * This function is an RNDIS Ethernet port -- a Microsoft protocol that's
 * been promoted instead of the standard CDC Ethernet.  The published RNDIS
 * spec is ambiguous, incomplete, and needlessly complex.  Variants such as
 * ActiveSync have even worse status in terms of specification.
 *
 * In short:  it's a protocol controlled by (and for) Microsoft, not for an
 * Open ecosystem or markets.  Linux supports it *only* because Microsoft
 * doesn't support the CDC Ethernet standard.
 *
 * The RNDIS data transfer model is complex, with multiple Ethernet packets
 * per USB message, and out of band data.  The control model is built around
 * what's essentially an "RNDIS RPC" protocol.  It's all wrapped in a CDC ACM
 * (modem, not Ethernet) veneer, with those ACM descriptors being entirely
 * useless (they're ignored).  RNDIS expects to be the only function in its
 * configuration, so it's no real help if you need composite devices; and
 * it expects to be the first configuration too.
 *
 * There is a single technical advantage of RNDIS over CDC Ethernet, if you
 * discount the fluff that its RPC can be made to deliver: it doesn't need
 * a NOP altsetting for the data interface.  That lets it work on some of the
 * "so smart it's stupid" hardware which takes over configuration changes
 * from the software, and adds restrictions like "no altsettings".
 *
 * Unfortunately MSFT's RNDIS drivers are buggy.  They hang or oops, and
 * have all sorts of contrary-to-specification oddities that can prevent
 * them from working sanely.  Since bugfixes (or accurate specs, letting
 * Linux work around those bugs) are unlikely to ever come from MSFT, you
 * may want to avoid using RNDIS on purely operational grounds.
 *
 * Omissions from the RNDIS 1.0 specification include:
 *
 *   - Power management ... references data that's scattered around lots
 *     of other documentation, which is incorrect/incomplete there too.
 *
 *   - There are various undocumented protocol requirements, like the need
 *     to send garbage in some control-OUT messages.
 *
 *   - MS-Windows drivers sometimes emit undocumented requests.
 */

static unsigned int rndis_dl_max_pkt_per_xfer = 10;
module_param(rndis_dl_max_pkt_per_xfer, uint, 0644);
MODULE_PARM_DESC(rndis_dl_max_pkt_per_xfer,
	"Maximum packets per transfer for DL aggregation");

static unsigned int rndis_ul_max_pkt_per_xfer = 1;
module_param(rndis_ul_max_pkt_per_xfer, uint, 0644);
MODULE_PARM_DESC(rndis_ul_max_pkt_per_xfer,
	"Maximum packets per transfer for UL aggregation");

static unsigned int f_rndis_debug;
module_param(f_rndis_debug, uint, 0644);
MODULE_PARM_DESC(f_rndis_debug,
		"f_rndis debug flag");
#define F_RNDIS_DBG(fmt, args...) \
	pr_notice("F_RNDIS,%s, " fmt, __func__, ## args)

static struct f_rndis *_rndis;
static spinlock_t rndis_lock;

struct f_rndis {
	struct gether			port;
	u8				ctrl_id, data_id;
	u8				ethaddr[ETH_ALEN];
	u32				vendorID;
	const char			*manufacturer;
	struct rndis_params		*params;
	int				config;

	struct usb_ep			*notify;
	struct usb_request		*notify_req;
	atomic_t			notify_count;

#ifdef CONFIG_MTK_MD_DIRECT_TETHERING_SUPPORT
	u8				direct_state; /* direct_state_enum */
	u8				network_type; /* rndis_network_type */
	struct usb_ctrlrequest	ctrl_req;
#endif
};

static inline struct f_rndis *func_to_rndis(struct usb_function *f)
{
	return container_of(f, struct f_rndis, port.func);
}

/* peak (theoretical) bulk transfer rate in bits-per-second */
static unsigned int bitrate(struct usb_gadget *g)
{
	if (gadget_is_superspeed(g) && g->speed == USB_SPEED_SUPER)
		return 13 * 1024 * 8 * 1000 * 8;
	else if (gadget_is_dualspeed(g) && g->speed == USB_SPEED_HIGH)
		return 13 * 512 * 8 * 1000 * 8;
	else
		return 19 * 64 * 1 * 1000 * 8;
}

/*-------------------------------------------------------------------------*/

/*
 */

#define RNDIS_STATUS_INTERVAL_MS	32
#define STATUS_BYTECOUNT		8	/* 8 bytes data */


/* interface descriptor: */

static struct usb_interface_descriptor rndis_control_intf = {
	.bLength =		sizeof rndis_control_intf,
	.bDescriptorType =	USB_DT_INTERFACE,

	/* .bInterfaceNumber = DYNAMIC */
	/* status endpoint is optional; this could be patched later */
	.bNumEndpoints =	1,
#ifdef CONFIG_USB_G_ANDROID
	.bInterfaceClass =	USB_CLASS_COMM,
	.bInterfaceSubClass =   USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol =   USB_CDC_ACM_PROTO_VENDOR,
#else
	.bInterfaceClass =	USB_CLASS_WIRELESS_CONTROLLER,
	.bInterfaceSubClass =	0x01,
	.bInterfaceProtocol =	0x03,
#endif
	/* .iInterface = DYNAMIC */
};

static struct usb_cdc_header_desc header_desc = {
	.bLength =		sizeof header_desc,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_HEADER_TYPE,

	.bcdCDC =		cpu_to_le16(0x0110),
};

static struct usb_cdc_call_mgmt_descriptor call_mgmt_descriptor = {
	.bLength =		sizeof call_mgmt_descriptor,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_CALL_MANAGEMENT_TYPE,

	.bmCapabilities =	0x00,
	.bDataInterface =	0x01,
};

static struct usb_cdc_acm_descriptor rndis_acm_descriptor = {
	.bLength =		sizeof rndis_acm_descriptor,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_ACM_TYPE,

	.bmCapabilities =	0x00,
};

static struct usb_cdc_union_desc rndis_union_desc = {
	.bLength =		sizeof(rndis_union_desc),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_UNION_TYPE,
	/* .bMasterInterface0 =	DYNAMIC */
	/* .bSlaveInterface0 =	DYNAMIC */
};

/* the data interface has two bulk endpoints */

static struct usb_interface_descriptor rndis_data_intf = {
	.bLength =		sizeof rndis_data_intf,
	.bDescriptorType =	USB_DT_INTERFACE,

	/* .bInterfaceNumber = DYNAMIC */
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_CDC_DATA,
	.bInterfaceSubClass =	0,
	.bInterfaceProtocol =	0,
	/* .iInterface = DYNAMIC */
};


static struct usb_interface_assoc_descriptor
rndis_iad_descriptor = {
	.bLength =		sizeof rndis_iad_descriptor,
	.bDescriptorType =	USB_DT_INTERFACE_ASSOCIATION,

	.bFirstInterface =	0, /* XXX, hardcoded */
	.bInterfaceCount = 2,	/* control + data */
#ifdef CONFIG_USB_G_ANDROID
	.bFunctionClass =	USB_CLASS_COMM,
	.bFunctionSubClass =	USB_CDC_SUBCLASS_ETHERNET,
	.bFunctionProtocol =	USB_CDC_PROTO_NONE,
#else
	.bFunctionClass =	USB_CLASS_WIRELESS_CONTROLLER,
	.bFunctionSubClass =	0x01,
	.bFunctionProtocol =	0x03,
#endif
	/* .iFunction = DYNAMIC */
};

/* full speed support: */

static struct usb_endpoint_descriptor fs_notify_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	cpu_to_le16(STATUS_BYTECOUNT),
	.bInterval =		RNDIS_STATUS_INTERVAL_MS,
};

static struct usb_endpoint_descriptor fs_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor fs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *eth_fs_function[] = {
	(struct usb_descriptor_header *) &rndis_iad_descriptor,

	/* control interface matches ACM, not Ethernet */
	(struct usb_descriptor_header *) &rndis_control_intf,
	(struct usb_descriptor_header *) &header_desc,
	(struct usb_descriptor_header *) &call_mgmt_descriptor,
	(struct usb_descriptor_header *) &rndis_acm_descriptor,
	(struct usb_descriptor_header *) &rndis_union_desc,
	(struct usb_descriptor_header *) &fs_notify_desc,

	/* data interface has no altsetting */
	(struct usb_descriptor_header *) &rndis_data_intf,
	(struct usb_descriptor_header *) &fs_in_desc,
	(struct usb_descriptor_header *) &fs_out_desc,
	NULL,
};

/* high speed support: */

static struct usb_endpoint_descriptor hs_notify_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	cpu_to_le16(STATUS_BYTECOUNT),
	.bInterval =		USB_MS_TO_HS_INTERVAL(RNDIS_STATUS_INTERVAL_MS)
};

static struct usb_endpoint_descriptor hs_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_endpoint_descriptor hs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_descriptor_header *eth_hs_function[] = {
	(struct usb_descriptor_header *) &rndis_iad_descriptor,

	/* control interface matches ACM, not Ethernet */
	(struct usb_descriptor_header *) &rndis_control_intf,
	(struct usb_descriptor_header *) &header_desc,
	(struct usb_descriptor_header *) &call_mgmt_descriptor,
	(struct usb_descriptor_header *) &rndis_acm_descriptor,
	(struct usb_descriptor_header *) &rndis_union_desc,
	(struct usb_descriptor_header *) &hs_notify_desc,

	/* data interface has no altsetting */
	(struct usb_descriptor_header *) &rndis_data_intf,
	(struct usb_descriptor_header *) &hs_in_desc,
	(struct usb_descriptor_header *) &hs_out_desc,
	NULL,
};

/* super speed support: */

static struct usb_endpoint_descriptor ss_notify_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	cpu_to_le16(STATUS_BYTECOUNT),
	.bInterval =		USB_MS_TO_HS_INTERVAL(RNDIS_STATUS_INTERVAL_MS)
};

static struct usb_ss_ep_comp_descriptor ss_intr_comp_desc = {
	.bLength =		sizeof ss_intr_comp_desc,
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	/* the following 3 values can be tweaked if necessary */
	/* .bMaxBurst =		0, */
	/* .bmAttributes =	0, */
	.wBytesPerInterval =	cpu_to_le16(STATUS_BYTECOUNT),
};

static struct usb_endpoint_descriptor ss_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(1024),
};

static struct usb_endpoint_descriptor ss_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor ss_bulk_comp_desc = {
	.bLength =		sizeof ss_bulk_comp_desc,
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	/* the following 2 values can be tweaked if necessary */
	/* .bMaxBurst =		0, */
	/* .bmAttributes =	0, */
};

static struct usb_descriptor_header *eth_ss_function[] = {
	(struct usb_descriptor_header *) &rndis_iad_descriptor,

	/* control interface matches ACM, not Ethernet */
	(struct usb_descriptor_header *) &rndis_control_intf,
	(struct usb_descriptor_header *) &header_desc,
	(struct usb_descriptor_header *) &call_mgmt_descriptor,
	(struct usb_descriptor_header *) &rndis_acm_descriptor,
	(struct usb_descriptor_header *) &rndis_union_desc,
	(struct usb_descriptor_header *) &ss_notify_desc,
	(struct usb_descriptor_header *) &ss_intr_comp_desc,

	/* data interface has no altsetting */
	(struct usb_descriptor_header *) &rndis_data_intf,
	(struct usb_descriptor_header *) &ss_in_desc,
	(struct usb_descriptor_header *) &ss_bulk_comp_desc,
	(struct usb_descriptor_header *) &ss_out_desc,
	(struct usb_descriptor_header *) &ss_bulk_comp_desc,
	NULL,
};

/* string descriptors: */

static struct usb_string rndis_string_defs[] = {
	[0].s = "RNDIS Communications Control",
	[1].s = "RNDIS Ethernet Data",
	[2].s = "RNDIS",
	{  } /* end of list */
};

static struct usb_gadget_strings rndis_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		rndis_string_defs,
};

static struct usb_gadget_strings *rndis_strings[] = {
	&rndis_string_table,
	NULL,
};

extern u8 kc_serialnumber[20];

/*-------------------------------------------------------------------------*/

static void set_eth_addr(unsigned char *eth_addr, unsigned char *buff)
{
	u64 num = 0;
	int i, len;

	len = strlen(buff);
	if (len >= 10) {
		len = 10;
	}

	for (i = 0 ; i < len ; i++) {
		num *= 0x10;
		num += buff[i] & 0x0f;
	}

	eth_addr[0] = 0x06;
	for (i = 5 ; i > 0 ; i-- ){
		eth_addr[i] =(char)( num & 0xff );
		num >>= 8;
	}

	return;
}

static int android_set_imacaddr(unsigned char *eth_addr)
{
	static char *usb_imei = NULL;

	if (!eth_addr)
		return -EINVAL;

	usb_imei = (char*)kc_serialnumber;

	if (usb_imei != NULL && strlen(usb_imei) >= 10) {
		set_eth_addr(eth_addr, usb_imei);
	} else {
		eth_addr[0]=0x06;
		eth_addr[1]=0x11;
		eth_addr[2]=0x22;
		eth_addr[3]=0x33;
		eth_addr[4]=0x44;
		eth_addr[5]=0x55;
	}
	return 0;
}

#ifdef CONFIG_MTK_MD_DIRECT_TETHERING_SUPPORT
static void rndis_resume_data_control(struct f_rndis *rndis)
{
	usb_ep_resume_control(rndis->port.in_ep);
	usb_ep_resume_control(rndis->port.out_ep);
}

static void rndis_activate_direct_tethering(struct usb_function *f,
						bool suspend)
{
	struct f_rndis *rndis = func_to_rndis(f);
	struct eth_dev *dev;
	ufpm_activate_md_func_req_t activate_req;

	rndis->direct_state = DIRECT_STATE_ACTIVATING;
	dev = rndis->port.ioport;

	memset(&activate_req, 0, sizeof(ufpm_activate_md_func_req_t));

	activate_req.mode = UFPM_FUNC_MODE_TETHER;
	activate_req.address = usb_gadget_get_address(f->config->cdev->gadget);
	activate_req.configuration = f->config->bConfigurationValue;


	if (gadget_is_superspeed(f->config->cdev->gadget))
		activate_req.speed = USBC_USB_SPEED_USB30;
	else if (gadget_is_dualspeed(f->config->cdev->gadget))
		activate_req.speed = USBC_USB_SPEED_USB20;
	else
		activate_req.speed = USBC_USB_SPEED_USB11;

	/* TODO: U3 endpoint is double but U2 endpoint might not be double */
	activate_req.ap_usb_map[0].type = ENDPOINT_USED | USB_MAP_TYPE_ENDPOINT
							| ENDPOINT_FIFO_DOUBLE
							| ENDPOINT_BULK
							| ENDPOINT_DIR_IN;
	activate_req.ap_usb_map[0].map = rndis->port.in_ep->address & 0x0F;
	activate_req.ap_usb_map[0].maxPktSize = rndis->port.in_ep->maxpacket;

	activate_req.ap_usb_map[1].type = ENDPOINT_USED | USB_MAP_TYPE_ENDPOINT
							| ENDPOINT_FIFO_DOUBLE
							| ENDPOINT_BULK
							| ENDPOINT_DIR_OUT;
	activate_req.ap_usb_map[1].map = rndis->port.out_ep->address;
	activate_req.ap_usb_map[1].maxPktSize = rndis->port.out_ep->maxpacket;

	memcpy(&activate_req.tethering_meta_info.host_mac_addr,
		   dev->host_mac, (sizeof(u8) * 6));
	memcpy(&activate_req.tethering_meta_info.device_mac_addr,
		   dev->net->dev_addr, (sizeof(u8) * 6));
	rndis_get_pkt_info(rndis->params,
	&activate_req.tethering_meta_info.init_cmplt_max_packets_per_transfer,
	&activate_req.tethering_meta_info.init_cmplt_max_transfer_size);
	activate_req.tethering_meta_info.init_cmplt_packet_alignment_factor = 0;
	/* defined by spec. */
	activate_req.tethering_meta_info.init_msg_max_transfer_size = 0x4000;
	rndis_get_net_stats(rndis->params,
				&activate_req.tethering_meta_info.net_stats);

	if (suspend) {
		usb_ep_suspend_control(rndis->port.in_ep);
		usb_ep_suspend_control(rndis->port.out_ep);
	}

	if (pkt_track_activate_md_fast_path(&activate_req) < 0) {
		usb_ep_resume_control(rndis->port.in_ep);
		usb_ep_resume_control(rndis->port.out_ep);
		rndis->direct_state = DIRECT_STATE_DEACTIVATED;
	}

	pr_info("%s, state: %d\n", __func__, rndis->direct_state);
}

static int rndis_deactivate_direct_tethering(struct usb_function *f)
{
	struct f_rndis *rndis = func_to_rndis(f);
	ufpm_md_fast_path_common_req_t req;
	int ret;

	req.mode = UFPM_FUNC_MODE_TETHER;

	ret = pkt_track_deactivate_md_fast_path(&req);

	if (ret >= 0)
		rndis->direct_state = DIRECT_STATE_DEACTIVATING;

	pr_info("%s, state:%d, ret:%d\n", __func__, rndis->direct_state, ret);

	return ret;
}

static int rndis_send_md_response(struct usb_function *f,
		struct rndis_params *params, u32 ep0_data_len, void *ep0Buffer)
{

	rndis_resp_t *r;
	struct usb_composite_dev *cdev = f->config->cdev;
	unsigned long flags;

	spin_lock_irqsave(&cdev->lock, flags);

	if (cdev->gadget->speed == USB_SPEED_UNKNOWN) {
		pr_notice("%s, USB_SPEED_UNKNOWN, directly return\n", __func__);
		spin_unlock_irqrestore(&cdev->lock, flags);
		return 0;
	}

	pr_debug("%s\n", __func__);

	if (!params->dev) {
		spin_unlock_irqrestore(&cdev->lock, flags);
		return -ENOTSUPP;
	}

	r = rndis_add_md_response(params, ep0_data_len);

	if (!r) {
		pr_notice("rndis_add_md_response return NULL\n");
		spin_unlock_irqrestore(&cdev->lock, flags);
		return -ENOMEM;
	}

	memcpy(r->buf, ep0Buffer, ep0_data_len);

	params->resp_avail(params->v);
	spin_unlock_irqrestore(&cdev->lock, flags);
	return 0;
}

static int rndis_handle_md_msg(struct usb_function *f, int msg_id, void *data)
{
	struct f_rndis *rndis = func_to_rndis(f);
	int handled = 0;
	struct local_para *local_para_ptr = (struct local_para *)data;
	ufpm_md_fast_path_common_rsp_t *rsp;
	ufpm_deactivate_md_func_rsp_t *deactivate_rsp;
	ufpm_send_ap_ep0_msg_t *ep0_msg;

	pr_debug("%s msg id: 0x%x, state: %d\n", __func__,
		msg_id, rndis->direct_state);

	switch (msg_id) {
	case IPC_MSG_ID_UFPM_ENABLE_MD_FAST_PATH_RSP:
		rsp = (ufpm_md_fast_path_common_rsp_t *)&local_para_ptr->data[0]
			;
		if (rsp->mode == UFPM_FUNC_MODE_TETHER) {
			if (rndis->direct_state == DIRECT_STATE_ENABLING) {
				if (rsp->result) {
					rndis->direct_state =
					DIRECT_STATE_DEACTIVATED;
					if (rndis->network_type ==
						RNDIS_NETWORK_TYPE_MOBILE) {
						/* Activate direct tethering */
						rndis_activate_direct_tethering(f, true);
					}
				} else {
					rndis->network_type = RNDIS_NETWORK_TYPE_NON_MOBILE;
					rndis->direct_state = DIRECT_STATE_NONE;
				}
			}
			handled = 1;
		}
		break;

	case IPC_MSG_ID_UFPM_DISABLE_MD_FAST_PATH_RSP:
		rsp = (ufpm_md_fast_path_common_rsp_t *)&local_para_ptr->data[0];
		if (rsp->mode == UFPM_FUNC_MODE_TETHER) {
			rndis->direct_state = DIRECT_STATE_NONE;
			handled = 1;
		}
		break;

	case IPC_MSG_ID_UFPM_ACTIVATE_MD_FAST_PATH_RSP:
		rsp = (ufpm_md_fast_path_common_rsp_t *)&local_para_ptr->data[0];
		if (rsp->mode == UFPM_FUNC_MODE_TETHER) {
			if (rndis->direct_state == DIRECT_STATE_ACTIVATING) {
				rndis->direct_state = DIRECT_STATE_ACTIVATED;
				pr_info("%s, switch to state: %d\n", __func__,
					rndis->direct_state);
			}
			handled = 1;
		}
		break;

	case IPC_MSG_ID_UFPM_DEACTIVATE_MD_FAST_PATH_RSP:
		deactivate_rsp = (ufpm_deactivate_md_func_rsp_t *)&local_para_ptr->data[0];
		if (deactivate_rsp->mode == UFPM_FUNC_MODE_TETHER) {
			/* Update network stats */
			rndis_set_net_stats(rndis->params,
				&deactivate_rsp->tethering_meta_info.net_stats);
			handled = 1;
		}
		break;

	case IPC_MSG_ID_UFPM_DEACTIVATE_MD_FAST_PATH_IND:
		pr_info("%s, switch to state: %d in FLIGHT MODE!\n", __func__,
			rndis->direct_state);
		rndis_set_direct_tethering(f, false);
		break;

	case IPC_MSG_ID_UFPM_SEND_MD_USB_EP0_RSP:
	case IPC_MSG_ID_UFPM_SEND_AP_USB_EP0_IND:
		ep0_msg = (ufpm_send_ap_ep0_msg_t *)&local_para_ptr->data[0];
		if (ep0_msg->mode == UFPM_FUNC_MODE_TETHER) {
			if (rndis->direct_state == DIRECT_STATE_ACTIVATED)
				rndis_send_md_response(f, rndis->params,
					ep0_msg->ep0_data_len, &ep0_msg->ep0Buffer);
			handled = 1;
		}
		break;
	default:
		break;
	}

	return handled;
}

static int rndis_md_fast_path_disable(struct usb_function *f)
{
	struct f_rndis *rndis = func_to_rndis(f);
	ufpm_md_fast_path_common_req_t req;
	int ret = 0;

	req.mode = UFPM_FUNC_MODE_TETHER;
	/* Call AP Packet Tracking module's API */
	ret = pkt_track_disable_md_fast_path(&req);
	if (ret < 0) {
		pr_notice("pkt_track_disable_md_fast_path fail, ret=%d\n", ret);
		rndis->direct_state = DIRECT_STATE_NONE;
	} else
		rndis->direct_state = DIRECT_STATE_DISABLING;

	return ret;
}

static int rndis_md_fast_path_enable(struct usb_function *f)
{
	struct f_rndis *rndis = func_to_rndis(f);
	ufpm_enable_md_func_req_t req;
	struct ccci_emi_info emi_info;
	int ret = 0;

	pr_info("%s enable direct tethering\n", __func__);

	ret = ccci_get_emi_info(0, &emi_info);
	if (ret < 0) {
		pr_notice("ccci_get_emi_info fail, ret=%d\n", ret);
		return ret;
	}

	memset(&req, 0, sizeof(req));

	req.mode = UFPM_FUNC_MODE_TETHER;
	req.mpuInfo.apUsbDomainId = emi_info.ap_domain_id;
	req.mpuInfo.mdCldmaDomainId = emi_info.md_domain_id;
	req.mpuInfo.memBank0BaseAddr = emi_info.ap_view_bank0_base;
	req.mpuInfo.memBank0Size = emi_info.bank0_size;
	req.mpuInfo.memBank4BaseAddr = emi_info.ap_view_bank4_base;
	req.mpuInfo.memBank4Size = emi_info.bank4_size;

	pr_info("memBank0BaseAddr=0x%llx\n",
			  req.mpuInfo.memBank0BaseAddr);
	pr_info("memBank0Size=0x%llx\n",
			  req.mpuInfo.memBank0Size);
	pr_info("memBank4BaseAddr=0x%llx\n",
			  req.mpuInfo.memBank4BaseAddr);
	pr_info("memBank4Size=0x%llx\n",
			  req.mpuInfo.memBank4Size);

	/* Enable direct tethering */
	ret = pkt_track_enable_md_fast_path(&req);
	if (ret < 0)
		pr_notice("pkt_track_enable_md_fast_path fail, ret=%d\n", ret);
	else
		rndis->direct_state = DIRECT_STATE_ENABLING;

	return ret;
}


int rndis_get_direct_tethering_state(struct usb_function *f)
{
	struct f_rndis *rndis = func_to_rndis(f);

	return rndis->direct_state;
}

void rndis_set_direct_tethering(struct usb_function *f, bool direct)
{
	struct f_rndis *rndis = func_to_rndis(f);

	pr_info("%s: %s direct tethering\n", __func__,
		 (direct ? "enable" : "disable"));

	if (rndis != NULL) {
		if (direct) {
			if (rndis->network_type == RNDIS_NETWORK_TYPE_NONE ||
				rndis->network_type == RNDIS_NETWORK_TYPE_NON_MOBILE) {
				if (rndis->direct_state == DIRECT_STATE_ENABLING ||
					rndis->direct_state == DIRECT_STATE_DEACTIVATING) {
					rndis->network_type = RNDIS_NETWORK_TYPE_MOBILE;
				} else if (rndis->direct_state == DIRECT_STATE_NONE ||
						   rndis->direct_state == DIRECT_STATE_DEACTIVATED) {
					/* Activate direct tethering */
					rndis->network_type = RNDIS_NETWORK_TYPE_MOBILE;
					rndis_md_fast_path_enable(f);
				}
			}
		} else {
			if (rndis->network_type == RNDIS_NETWORK_TYPE_MOBILE) {
				if (rndis->direct_state == DIRECT_STATE_ACTIVATING) {
					rndis->network_type = RNDIS_NETWORK_TYPE_NON_MOBILE;
				} else if (rndis->direct_state == DIRECT_STATE_ACTIVATED) {
					/* Deactivate direct tethering */
					rndis->network_type = RNDIS_NETWORK_TYPE_NON_MOBILE;
					rndis_deactivate_direct_tethering(f);
					rndis_resume_data_control(rndis);
					rndis->direct_state = DIRECT_STATE_DEACTIVATED;
					rx_fill(rndis->port.ioport, GFP_KERNEL);
					pr_info("%s, rx_fill done!!\n",
						__func__);
				}
			}
		}
	}
}

int rndis_md_msg_hdlr(struct ipc_ilm *ilm)
{
	struct f_rndis *rndis = _rndis;
	unsigned long flags;

	spin_lock_irqsave(&rndis_lock, flags);
	rndis = _rndis;

	if (!rndis) {
		pr_notice("%s():rndis is NULL.\n", __func__);
		spin_unlock_irqrestore(&rndis_lock, flags);
		return -EFAULT;
	}

	/* hadle the msg from md */
	rndis_handle_md_msg(&rndis->port.func, ilm->msg_id,
			(void *)ilm->local_para_ptr);

	spin_unlock_irqrestore(&rndis_lock, flags);
	return 0;
}
#endif

static struct sk_buff *rndis_add_header(struct gether *port,
					struct sk_buff *skb)
{
	struct sk_buff *skb2;
	struct rndis_packet_msg_type *header = NULL;
	struct f_rndis *rndis = func_to_rndis(&port->func);

	if (rndis->port.multi_pkt_xfer) {
		if (port->header) {
			header = port->header;
			memset(header, 0, sizeof(*header));
			header->MessageType = cpu_to_le32(RNDIS_MSG_PACKET);
			header->MessageLength = cpu_to_le32(skb->len +
				sizeof(*header));
			header->DataOffset = cpu_to_le32(36);
			header->DataLength = cpu_to_le32(skb->len);
			pr_debug("MessageLength:%d DataLength:%d\n",
				header->MessageLength,
				header->DataLength);
			return skb;
		}
		pr_notice("RNDIS header is NULL.\n");
		return NULL;

		} else {
			skb2 = skb_realloc_headroom(skb,
			sizeof(struct rndis_packet_msg_type));
			if (skb2)
			rndis_add_hdr(skb2);

			dev_kfree_skb(skb);
			return skb2;
		}

}


static void rndis_response_available(void *_rndis)
{
	struct f_rndis			*rndis = _rndis;
	struct usb_request		*req = rndis->notify_req;
	struct usb_composite_dev	*cdev = rndis->port.func.config->cdev;
	__le32				*data = req->buf;
	int				status;

	if (atomic_inc_return(&rndis->notify_count) != 1)
		return;

	/* Send RNDIS RESPONSE_AVAILABLE notification; a
	 * USB_CDC_NOTIFY_RESPONSE_AVAILABLE "should" work too
	 *
	 * This is the only notification defined by RNDIS.
	 */
	data[0] = cpu_to_le32(1);
	data[1] = cpu_to_le32(0);

	status = usb_ep_queue(rndis->notify, req, GFP_ATOMIC);
	if (status) {
		atomic_dec(&rndis->notify_count);
		DBG(cdev, "notify/0 --> %d\n", status);
	}
}

static void rndis_response_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_rndis			*rndis;
	int				status = req->status;
	struct usb_composite_dev	*cdev;
	struct usb_ep *notify_ep;

	spin_lock(&rndis_lock);
	rndis = _rndis;
	if (!rndis || !rndis->notify) {
		pr_notice("%s():rndis is NULL.\n", __func__);
		spin_unlock(&rndis_lock);
		return;
	}

	if (!rndis->port.func.config || !rndis->port.func.config->cdev) {
		pr_notice("%s(): cdev or config is NULL.\n", __func__);
		spin_unlock(&rndis_lock);
		return;
	}

	cdev = rndis->port.func.config->cdev;

	/* after TX:
	 *  - USB_CDC_GET_ENCAPSULATED_RESPONSE (ep0/control)
	 *  - RNDIS_RESPONSE_AVAILABLE (status/irq)
	 */
	switch (status) {
	case -ECONNRESET:
	case -ESHUTDOWN:
		/* connection gone */
		atomic_set(&rndis->notify_count, 0);
		goto out;
	default:
		DBG(cdev, "RNDIS %s response error %d, %d/%d\n",
			ep->name, status,
			req->actual, req->length);
		/* FALLTHROUGH */
	case 0:
		if (ep != rndis->notify)
			goto out;

		/* handle multiple pending RNDIS_RESPONSE_AVAILABLE
		 * notifications by resending until we're done
		 */
		if (atomic_dec_and_test(&rndis->notify_count))
			goto out;
		notify_ep = rndis->notify;
		spin_unlock(&rndis_lock);
		status = usb_ep_queue(notify_ep, req, GFP_ATOMIC);
		if (status) {
			spin_lock(&rndis_lock);
			if (!_rndis)
				goto out;
			atomic_dec(&rndis->notify_count);
			DBG(cdev, "notify/1 --> %d\n", status);
			spin_unlock(&rndis_lock);
		}
		return;
	}
out:
	spin_unlock(&rndis_lock);
}

static void rndis_command_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_rndis			*rndis;
	struct usb_composite_dev	*cdev;
	int				status;
	rndis_init_msg_type		*buf;

	spin_lock(&rndis_lock);
	rndis = _rndis;
	if (!rndis || !rndis->notify) {
		pr_notice("%s():rndis is NULL.\n", __func__);
		spin_unlock(&rndis_lock);
		return;
	}

	if (!rndis->port.func.config || !rndis->port.func.config->cdev) {
		pr_notice("%s(): cdev or config is NULL.\n", __func__);
		spin_unlock(&rndis_lock);
		return;
	}

	cdev = rndis->port.func.config->cdev;

	/* received RNDIS command from USB_CDC_SEND_ENCAPSULATED_COMMAND */
//	spin_lock(&dev->lock);
	status = rndis_msg_parser(rndis->params, (u8 *) req->buf);
	if (status < 0)
		pr_err("RNDIS command error %d, %d/%d\n",
			status, req->actual, req->length);

	buf = (rndis_init_msg_type *)req->buf;

	if (buf->MessageType == RNDIS_MSG_INIT) {
		if (buf->MaxTransferSize > 2048) {
			rndis->port.multi_pkt_xfer = 1;
			rndis->port.dl_max_transfer_len = buf->MaxTransferSize;
			gether_update_dl_max_xfer_size(&rndis->port,
					rndis->port.dl_max_transfer_len);
		} else
			rndis->port.multi_pkt_xfer = 0;
		pr_info("%s: MaxTransferSize: %d : Multi_pkt_txr: %s\n",
				__func__, buf->MaxTransferSize,
				rndis->port.multi_pkt_xfer ? "enabled" :
							    "disabled");
		if (rndis_dl_max_pkt_per_xfer <= 1)
			rndis->port.multi_pkt_xfer = 0;
	}
	spin_unlock(&rndis_lock);
}

static int
rndis_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct f_rndis		*rndis = func_to_rndis(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_request	*req = cdev->req;
	int			value = -EOPNOTSUPP;
	u16			w_index = le16_to_cpu(ctrl->wIndex);
	u16			w_value = le16_to_cpu(ctrl->wValue);
	u16			w_length = le16_to_cpu(ctrl->wLength);

	spin_lock(&rndis_lock);

	if (!rndis || !rndis->notify) {
		pr_notice("%s():rndis is NULL.\n", __func__);
		spin_unlock(&rndis_lock);
		return -EFAULT;
	}

	/* composite driver infrastructure handles everything except
	 * CDC class messages; interface activation uses set_alt().
	 */
	switch ((ctrl->bRequestType << 8) | ctrl->bRequest) {

	/* RNDIS uses the CDC command encapsulation mechanism to implement
	 * an RPC scheme, with much getting/setting of attributes by OID.
	 */
	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_SEND_ENCAPSULATED_COMMAND:
		if (w_value || w_index != rndis->ctrl_id)
			goto invalid;
		/* read the request; process it later */
		value = w_length;
		req->complete = rndis_command_complete;
		req->context = rndis;

#ifdef CONFIG_MTK_MD_DIRECT_TETHERING_SUPPORT
		memcpy(&rndis->ctrl_req, ctrl, sizeof(struct usb_ctrlrequest));
#endif
		/* later, rndis_response_available() sends a notification */
		break;

	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_GET_ENCAPSULATED_RESPONSE:
		if (w_value || w_index != rndis->ctrl_id)
			goto invalid;
		else {
			u8 *buf;
			u32 n;
			u32 MsgType, MsgLength, MsgID;
			__le32 *tmp;

			/* return the result */
			buf = rndis_get_next_response(rndis->params, &n);
			if (buf) {
				memcpy(req->buf, buf, n);
				req->complete = rndis_response_complete;
				req->context = rndis;

				tmp = (__le32 *)buf;
				MsgType   = get_unaligned_le32(tmp++);
				MsgLength = get_unaligned_le32(tmp++);
				MsgID = get_unaligned_le32(tmp++);

				rndis_free_response(rndis->params, buf);
				value = n;
			}
			/* else stalls ... spec says to avoid that */
		}
		break;

	default:
invalid:
		VDBG(cdev, "invalid control req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
	}

	/* respond with data transfer or status phase? */
	if (value >= 0) {
		DBG(cdev, "rndis req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
		req->zero = (value < w_length);
		req->length = value;

		spin_unlock(&rndis_lock);
		value = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		spin_lock(&rndis_lock);

		if (value < 0)
			ERROR(cdev, "rndis response on err %d\n", value);
	}

	spin_unlock(&rndis_lock);

	/* device either stalls (value < 0) or reports success */
	return value;
}


static int rndis_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct f_rndis		*rndis = func_to_rndis(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	/* we know alt == 0 */

	if (intf == rndis->ctrl_id) {
		VDBG(cdev, "reset rndis control %d\n", intf);
		usb_ep_disable(rndis->notify);

		if (!rndis->notify->desc) {
			F_RNDIS_DBG("init rndis ctrl %d\n", intf);
			VDBG(cdev, "init rndis ctrl %d\n", intf);
			if (config_ep_by_speed(cdev->gadget, f, rndis->notify))
				goto fail;
		}
		usb_ep_enable(rndis->notify);

	} else if (intf == rndis->data_id) {
		struct net_device	*net;

		if (rndis->port.in_ep->enabled) {
			DBG(cdev, "reset rndis\n");
			gether_disconnect(&rndis->port);
		}

		if (!rndis->port.in_ep->desc || !rndis->port.out_ep->desc) {
			DBG(cdev, "init rndis\n");
			if (config_ep_by_speed(cdev->gadget, f,
						rndis->port.in_ep) ||
			    config_ep_by_speed(cdev->gadget, f,
					       rndis->port.out_ep)) {
				rndis->port.in_ep->desc = NULL;
				rndis->port.out_ep->desc = NULL;
				goto fail;
			}
		}

		/* Avoid ZLPs; they can be troublesome. */
		rndis->port.is_zlp_ok = false;

		/* RNDIS should be in the "RNDIS uninitialized" state,
		 * either never activated or after rndis_uninit().
		 *
		 * We don't want data to flow here until a nonzero packet
		 * filter is set, at which point it enters "RNDIS data
		 * initialized" state ... but we do want the endpoints
		 * to be activated.  It's a strange little state.
		 *
		 * REVISIT the RNDIS gadget code has done this wrong for a
		 * very long time.  We need another call to the link layer
		 * code -- gether_updown(...bool) maybe -- to do it right.
		 */
		rndis->port.cdc_filter = 0;

		DBG(cdev, "RNDIS RX/TX early activation ... \n");
		net = gether_connect(&rndis->port);
		if (IS_ERR(net))
			return PTR_ERR(net);

		rndis_set_param_dev(rndis->params, net,
				&rndis->port.cdc_filter);
	} else
		goto fail;

	return 0;
fail:
	return -EINVAL;
}

static void rndis_disable(struct usb_function *f)
{
	struct f_rndis		*rndis = func_to_rndis(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	unsigned long flags;

#ifdef CONFIG_MTK_MD_DIRECT_TETHERING_SUPPORT
	if (rndis->direct_state == DIRECT_STATE_ACTIVATING ||
		rndis->direct_state == DIRECT_STATE_ACTIVATED) {
		rndis->network_type = RNDIS_NETWORK_TYPE_NONE;
		/* Deactivating direct tethering */
		rndis_deactivate_direct_tethering(f);
		pr_info("%s, rndis_resume_data_control\n", __func__);
		rndis_resume_data_control(rndis);
		pr_info("%s, rndis_resume_data_control done\n", __func__);
		rndis->direct_state = DIRECT_STATE_DEACTIVATED;
		pr_info("%s, DIRECT_STATE_DEACTIVATED before rx_fill\n",
			 __func__);
		rx_fill(rndis->port.ioport, GFP_KERNEL);
		pr_info("%s, rx_fill done!!\n", __func__);
	}
#endif

	if (!rndis->notify->enabled)
		return;

	DBG(cdev, "rndis deactivated\n");
	spin_lock_irqsave(&rndis_lock, flags);
	rndis_uninit(rndis->params);
	spin_unlock_irqrestore(&rndis_lock, flags);
	gether_disconnect(&rndis->port);

	usb_ep_disable(rndis->notify);
}

/*-------------------------------------------------------------------------*/

/*
 * This isn't quite the same mechanism as CDC Ethernet, since the
 * notification scheme passes less data, but the same set of link
 * states must be tested.  A key difference is that altsettings are
 * not used to tell whether the link should send packets or not.
 */

static void rndis_open(struct gether *geth)
{
	struct f_rndis		*rndis = func_to_rndis(&geth->func);
	struct usb_composite_dev *cdev = geth->func.config->cdev;

	F_RNDIS_DBG("\n");
	DBG(cdev, "%s\n", __func__);

	rndis_set_param_medium(rndis->params, RNDIS_MEDIUM_802_3,
				bitrate(cdev->gadget) / 100);
	rndis_signal_connect(rndis->params);
}

static void rndis_close(struct gether *geth)
{
	struct f_rndis		*rndis = func_to_rndis(&geth->func);

	F_RNDIS_DBG("\n");
	DBG(geth->func.config->cdev, "%s\n", __func__);

	rndis_set_param_medium(rndis->params, RNDIS_MEDIUM_802_3, 0);
	rndis_signal_disconnect(rndis->params);
}

/*-------------------------------------------------------------------------*/

/* Some controllers can't support RNDIS ... */
static inline bool can_support_rndis(struct usb_configuration *c)
{
	/* everything else is *presumably* fine */
	return true;
}

/* ethernet function driver setup/binding */

static int
rndis_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_rndis		*rndis = func_to_rndis(f);
	struct usb_string	*us;
	int			status;
	struct usb_ep		*ep;

	struct f_rndis_opts *rndis_opts;

	if (!can_support_rndis(c))
		return -EINVAL;

	rndis_opts = container_of(f->fi, struct f_rndis_opts, func_inst);

	if (cdev->use_os_string) {
		f->os_desc_table = kzalloc(sizeof(*f->os_desc_table),
					   GFP_KERNEL);
		if (!f->os_desc_table)
			return -ENOMEM;
		f->os_desc_n = 1;
		f->os_desc_table[0].os_desc = &rndis_opts->rndis_os_desc;
	}

	/*
	 * in drivers/usb/gadget/configfs.c:configfs_composite_bind()
	 * configurations are bound in sequence with list_for_each_entry,
	 * in each configuration its functions are bound in sequence
	 * with list_for_each_entry, so we assume no race condition
	 * with regard to rndis_opts->bound access
	 */
	 /* if (!rndis_opts->bound) { */
	if (rndis_opts) {
	if (!rndis_opts->bound) {
		gether_set_gadget(rndis_opts->net, cdev->gadget);
		status = gether_register_netdev(rndis_opts->net);
		if (status)
			goto fail;
		rndis_opts->bound = true;
	}
	}

	us = usb_gstrings_attach(cdev, rndis_strings,
				 ARRAY_SIZE(rndis_string_defs));
	if (IS_ERR(us)) {
		status = PTR_ERR(us);
		goto fail;
	}
	rndis_control_intf.iInterface = us[0].id;
	rndis_data_intf.iInterface = us[1].id;
	rndis_iad_descriptor.iFunction = us[2].id;

	/* allocate instance-specific interface IDs */
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	rndis->ctrl_id = status;
	rndis_iad_descriptor.bFirstInterface = status;

	rndis_control_intf.bInterfaceNumber = status;
	rndis_union_desc.bMasterInterface0 = status;

	if (cdev->use_os_string)
		f->os_desc_table[0].if_id =
			rndis_iad_descriptor.bFirstInterface;

	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	rndis->data_id = status;

	rndis_data_intf.bInterfaceNumber = status;
	rndis_union_desc.bSlaveInterface0 = status;

	status = -ENODEV;

	/* allocate instance-specific endpoints */
	ep = usb_ep_autoconfig(cdev->gadget, &fs_in_desc);
	if (!ep)
		goto fail;
	rndis->port.in_ep = ep;

	ep = usb_ep_autoconfig(cdev->gadget, &fs_out_desc);
	if (!ep)
		goto fail;
	rndis->port.out_ep = ep;

	/* NOTE:  a status/notification endpoint is, strictly speaking,
	 * optional.  We don't treat it that way though!  It's simpler,
	 * and some newer profiles don't treat it as optional.
	 */
	ep = usb_ep_autoconfig(cdev->gadget, &fs_notify_desc);
	if (!ep)
		goto fail;
	rndis->notify = ep;

	status = -ENOMEM;

	/* allocate notification request and buffer */
	rndis->notify_req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!rndis->notify_req)
		goto fail;
#if defined(CONFIG_64BIT) && defined(CONFIG_MTK_LM_MODE)
	rndis->notify_req->buf = kmalloc(STATUS_BYTECOUNT,
					GFP_KERNEL | GFP_DMA);
#else
	rndis->notify_req->buf = kmalloc(STATUS_BYTECOUNT, GFP_KERNEL);
#endif
	if (!rndis->notify_req->buf)
		goto fail;
	rndis->notify_req->length = STATUS_BYTECOUNT;
	rndis->notify_req->context = rndis;
	rndis->notify_req->complete = rndis_response_complete;

	/* support all relevant hardware speeds... we expect that when
	 * hardware is dual speed, all bulk-capable endpoints work at
	 * both speeds
	 */
	hs_in_desc.bEndpointAddress = fs_in_desc.bEndpointAddress;
	hs_out_desc.bEndpointAddress = fs_out_desc.bEndpointAddress;
	hs_notify_desc.bEndpointAddress = fs_notify_desc.bEndpointAddress;

	ss_in_desc.bEndpointAddress = fs_in_desc.bEndpointAddress;
	ss_out_desc.bEndpointAddress = fs_out_desc.bEndpointAddress;
	ss_notify_desc.bEndpointAddress = fs_notify_desc.bEndpointAddress;

	status = usb_assign_descriptors(f, eth_fs_function, eth_hs_function,
			eth_ss_function, NULL);
	if (status)
		goto fail;

	rndis->port.open = rndis_open;
	rndis->port.close = rndis_close;

	rndis_set_param_medium(rndis->params, RNDIS_MEDIUM_802_3, 0);
	rndis_set_host_mac(rndis->params, rndis->ethaddr);
	rndis_set_max_pkt_xfer(rndis->params, rndis_ul_max_pkt_per_xfer);

	if (rndis->manufacturer && rndis->vendorID &&
			rndis_set_param_vendor(rndis->params, rndis->vendorID,
					       rndis->manufacturer)) {
		status = -EINVAL;
		goto fail_free_descs;
	}

	/* NOTE:  all that is done without knowing or caring about
	 * the network link ... which is unavailable to this code
	 * until we're activated via set_alt().
	 */

	INFO(cdev, "RNDIS: %s speed IN/%s OUT/%s NOTIFY/%s\n",
			gadget_is_superspeed(c->cdev->gadget) ? "super" :
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			rndis->port.in_ep->name, rndis->port.out_ep->name,
			rndis->notify->name);
	return 0;

fail_free_descs:
	usb_free_all_descriptors(f);
fail:
	kfree(f->os_desc_table);
	f->os_desc_n = 0;

	if (rndis->notify_req) {
		kfree(rndis->notify_req->buf);
		usb_ep_free_request(rndis->notify, rndis->notify_req);
	}

	ERROR(cdev, "%s: can't bind, err %d\n", f->name, status);

	return status;
}

#ifdef CONFIG_USB_G_ANDROID
static void
rndis_old_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_rndis	*rndis = func_to_rndis(f);
	unsigned long flags;

	F_RNDIS_DBG("\n");

#ifdef CONFIG_MTK_MD_DIRECT_TETHERING_SUPPORT
	if (rndis->direct_state == DIRECT_STATE_ENABLING ||
		rndis->direct_state == DIRECT_STATE_DEACTIVATED ||
		rndis->direct_state == DIRECT_STATE_DEACTIVATING) {
		rndis_md_fast_path_disable(f);
	}
#endif

	rndis_deregister(rndis->params);  /* FIXME Need to check*/

	usb_free_all_descriptors(f);

	kfree(rndis->notify_req->buf);
	usb_ep_free_request(rndis->notify, rndis->notify_req);

	spin_lock_irqsave(&rndis_lock, flags);
	kfree(rndis);
	_rndis = NULL;
	spin_unlock_irqrestore(&rndis_lock, flags);
}

int
rndis_bind_config_vendor(struct usb_configuration *c, u8 ethaddr[ETH_ALEN],
		u32 vendorID, const char *manufacturer, struct eth_dev *dev)
{
	struct f_rndis	*rndis;
	int status;
	rndis_params *params;

	/* allocate and initialize one new instance */
	status = -ENOMEM;
	rndis = kzalloc(sizeof(*rndis), GFP_KERNEL);
	if (!rndis)
		goto fail;

	_rndis = rndis;

	ether_addr_copy(rndis->ethaddr, ethaddr);
	rndis->vendorID = vendorID;
	rndis->manufacturer = manufacturer;

	rndis->port.ioport = dev;
	/* RNDIS activates when the host changes this filter */
	rndis->port.cdc_filter = 0;

	/* RNDIS has special (and complex) framing */
	rndis->port.header_len = sizeof(struct rndis_packet_msg_type);
	rndis->port.wrap = rndis_add_header;
	rndis->port.unwrap = rndis_rm_hdr;
	rndis->port.ul_max_pkts_per_xfer = rndis_ul_max_pkt_per_xfer;
	rndis->port.dl_max_pkts_per_xfer = rndis_dl_max_pkt_per_xfer;

	rndis->port.func.name = "rndis";
	/* descriptors are per-instance copies */
	rndis->port.func.bind = rndis_bind;
	/* note here use rndis_old_unbind */
	rndis->port.func.unbind = rndis_old_unbind;
	rndis->port.func.set_alt = rndis_set_alt;
	rndis->port.func.setup = rndis_setup;
	rndis->port.func.disable = rndis_disable;

#ifdef CONFIG_MTK_MD_DIRECT_TETHERING_SUPPORT
	rndis->direct_state = DIRECT_STATE_NONE;
	rndis->network_type = RNDIS_NETWORK_TYPE_NON_MOBILE;
#endif

	spin_lock_init(&rndis_lock);

	params = rndis_register(rndis_response_available, rndis);
	if (params == NULL) {
		kfree(rndis);
		return status;
	}
	rndis->params = params;

	status = usb_add_function(c, &rndis->port.func);
	if (status)
		kfree(rndis);
fail:

	F_RNDIS_DBG("done, status %d\n", status);
	return status;
}
#endif
void rndis_borrow_net(struct usb_function_instance *f, struct net_device *net)
{
	struct f_rndis_opts *opts;

	opts = container_of(f, struct f_rndis_opts, func_inst);
	if (opts->bound)
		gether_cleanup(netdev_priv(opts->net));
	else
		free_netdev(opts->net);
	opts->borrowed_net = opts->bound = true;
	opts->net = net;
}
EXPORT_SYMBOL_GPL(rndis_borrow_net);

static inline struct f_rndis_opts *to_f_rndis_opts(struct config_item *item)
{
	return container_of(to_config_group(item), struct f_rndis_opts,
			    func_inst.group);
}

/* f_rndis_item_ops */
USB_ETHERNET_CONFIGFS_ITEM(rndis);

/* f_rndis_opts_dev_addr */
USB_ETHERNET_CONFIGFS_ITEM_ATTR_DEV_ADDR(rndis);

/* f_rndis_opts_host_addr */
USB_ETHERNET_CONFIGFS_ITEM_ATTR_HOST_ADDR(rndis);

/* f_rndis_opts_qmult */
USB_ETHERNET_CONFIGFS_ITEM_ATTR_QMULT(rndis);

/* f_rndis_opts_ifname */
USB_ETHERNET_CONFIGFS_ITEM_ATTR_IFNAME(rndis);

static struct configfs_attribute *rndis_attrs[] = {
	&rndis_opts_attr_dev_addr,
	&rndis_opts_attr_host_addr,
	&rndis_opts_attr_qmult,
	&rndis_opts_attr_ifname,
	NULL,
};

static struct config_item_type rndis_func_type = {
	.ct_item_ops	= &rndis_item_ops,
	.ct_attrs	= rndis_attrs,
	.ct_owner	= THIS_MODULE,
};

static void rndis_free_inst(struct usb_function_instance *f)
{
	struct f_rndis_opts *opts;

	opts = container_of(f, struct f_rndis_opts, func_inst);
	if (!opts->borrowed_net) {
		if (opts->bound)
			gether_cleanup(netdev_priv(opts->net));
		else
			free_netdev(opts->net);
	}

	kfree(opts->rndis_interf_group);	/* single VLA chunk */
	kfree(opts);
}

static struct usb_function_instance *rndis_alloc_inst(void)
{
	struct f_rndis_opts *opts;
	struct usb_os_desc *descs[1];
	char *names[1];
	struct config_group *rndis_interf_group;

	opts = kzalloc(sizeof(*opts), GFP_KERNEL);
	if (!opts)
		return ERR_PTR(-ENOMEM);
	opts->rndis_os_desc.ext_compat_id = opts->rndis_ext_compat_id;

	mutex_init(&opts->lock);
	opts->func_inst.free_func_inst = rndis_free_inst;
	opts->net = gether_setup_name_default("rndis");
	if (IS_ERR(opts->net)) {
		struct net_device *net = opts->net;
		kfree(opts);
		return ERR_CAST(net);
	}
	INIT_LIST_HEAD(&opts->rndis_os_desc.ext_prop);

	descs[0] = &opts->rndis_os_desc;
	names[0] = "rndis";
	config_group_init_type_name(&opts->func_inst.group, "",
				    &rndis_func_type);
	rndis_interf_group =
		usb_os_desc_prepare_interf_dir(&opts->func_inst.group, 1, descs,
					       names, THIS_MODULE);
	if (IS_ERR(rndis_interf_group)) {
		rndis_free_inst(&opts->func_inst);
		return ERR_CAST(rndis_interf_group);
	}
	opts->rndis_interf_group = rndis_interf_group;

	return &opts->func_inst;
}

static void rndis_free(struct usb_function *f)
{
	struct f_rndis *rndis;
	struct f_rndis_opts *opts;
	unsigned long flags;

	rndis = func_to_rndis(f);
	rndis_deregister(rndis->params);
	opts = container_of(f->fi, struct f_rndis_opts, func_inst);

	spin_lock_irqsave(&rndis_lock, flags);
	kfree(rndis);
	_rndis = NULL;
	spin_unlock_irqrestore(&rndis_lock, flags);

	mutex_lock(&opts->lock);
	opts->refcnt--;
	mutex_unlock(&opts->lock);
}

static void rndis_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_rndis		*rndis = func_to_rndis(f);

	kfree(f->os_desc_table);
	f->os_desc_n = 0;
	usb_free_all_descriptors(f);

	kfree(rndis->notify_req->buf);
	usb_ep_free_request(rndis->notify, rndis->notify_req);
}

static struct usb_function *rndis_alloc(struct usb_function_instance *fi)
{
	struct f_rndis	*rndis;
	struct f_rndis_opts *opts;
	struct rndis_params *params;

	/* allocate and initialize one new instance */
	rndis = kzalloc(sizeof(*rndis), GFP_KERNEL);
	if (!rndis)
		return ERR_PTR(-ENOMEM);

	_rndis = rndis;

	opts = container_of(fi, struct f_rndis_opts, func_inst);
	mutex_lock(&opts->lock);
	opts->refcnt++;

	gether_get_host_addr_u8(opts->net, rndis->ethaddr);

	android_set_imacaddr(rndis->ethaddr);
	pr_debug("%s HOST MAC: %pM\n", __func__, rndis->ethaddr);

	rndis->vendorID = opts->vendor_id;
	rndis->manufacturer = opts->manufacturer;

	rndis->port.ioport = netdev_priv(opts->net);
	mutex_unlock(&opts->lock);
	/* RNDIS activates when the host changes this filter */
	rndis->port.cdc_filter = 0;

	/* RNDIS has special (and complex) framing */
	rndis->port.header_len = sizeof(struct rndis_packet_msg_type);
	rndis->port.wrap = rndis_add_header;
	rndis->port.unwrap = rndis_rm_hdr;
	rndis->port.ul_max_pkts_per_xfer = rndis_ul_max_pkt_per_xfer;
	rndis->port.dl_max_pkts_per_xfer = rndis_dl_max_pkt_per_xfer;

	rndis->port.func.name = "rndis";
	/* descriptors are per-instance copies */
	rndis->port.func.bind = rndis_bind;
	rndis->port.func.unbind = rndis_unbind;
	rndis->port.func.set_alt = rndis_set_alt;
	rndis->port.func.setup = rndis_setup;
	rndis->port.func.disable = rndis_disable;
	rndis->port.func.free_func = rndis_free;

#ifdef CONFIG_MTK_MD_DIRECT_TETHERING_SUPPORT
	rndis->direct_state = DIRECT_STATE_NONE;
	rndis->network_type = RNDIS_NETWORK_TYPE_NON_MOBILE;
#endif

	spin_lock_init(&rndis_lock);

	params = rndis_register(rndis_response_available, rndis);
	if (IS_ERR(params)) {
		kfree(rndis);
		return ERR_CAST(params);
	}
	rndis->params = params;

	return &rndis->port.func;
}

DECLARE_USB_FUNCTION_INIT(rndis, rndis_alloc_inst, rndis_alloc);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Brownell");
