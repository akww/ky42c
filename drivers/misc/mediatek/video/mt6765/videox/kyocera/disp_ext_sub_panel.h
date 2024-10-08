/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2020 KYOCERA Corporation
 * (C) 2022 KYOCERA Corporation
 *
 * drivers/video/msm/mdss/disp_ext_sub_panel.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
*/
#ifndef DISP_EXT_SUB_PANEL_H
#define DISP_EXT_SUB_PANEL_H

#include <linux/kdisp.h>
#include "disp_ext_spi.h"
#include <linux/fb_kc.h>

typedef enum {
	DISP_SUB_STATE_PWR_OFF,
	DISP_SUB_STATE_PWR_ON,
	DISP_SUB_STATE_OFF,
	DISP_SUB_STATE_ON,
#ifdef CONFIG_DISP_EXT_SUB_ALWAYS_ON
	DISP_SUB_STATE_ALLWAYSON
#endif /* CONFIG_DISP_EXT_SUB_ALWAYS_ON */
} disp_ext_sub_state_type;

typedef enum {
	DISP_SUB_MODEM_STATE_ON,
	DISP_SUB_MODEM_STATE_OFF,
	DISP_SUB_MODEM_STATE_MAX
} disp_ext_sub_modem_state_type;

typedef enum {
	DISP_SUB_CTRL_CMD =  0x01,
	DISP_SUB_CTRL_WAIT = 0x10,
	DISP_SUB_CTRL_SIG =  0x20,
	DISP_SUB_CTRL_MAX
} disp_ext_sub_ctrl_kind_type;

typedef enum {
	DISP_SUB_SIG_VH = 0,
	DISP_SUB_SIG_RESET,
	DISP_SUB_SIG_VOLED,
	DISP_SUB_SIG_SPI_CS,
	DISP_SUB_SIG_SPI_CLK,
	DISP_SUB_SIG_SPI_DATA,
	DISP_SUB_SIG_SPI_RS,
	DISP_SUB_SIG_MAX
} disp_ext_sub_sig_kind_type;

struct disp_ext_sub_cmd_hdr {
	unsigned char ctrl_kind;
	unsigned char payload_len;
};

struct  disp_ext_sub_cmd_detail {
	struct  disp_ext_sub_cmd_hdr cmd_hdr;
	void *  payload_p;
};

struct disp_ext_sub_cmds {
	char *  buf_p;
	unsigned int  blen;
	unsigned int  cmd_cnt;
	struct disp_ext_sub_cmd_detail* cmd_p;
};

#ifdef CONFIG_DISP_EXT_SUB_ALWAYS_ON
typedef struct
{
	uint32_t result;
} disp_ext_sub_mdmres_type;

typedef struct
{
	uint32_t seq;
	uint32_t request;
	union {
		disp_ext_sub_mdmres_type  resp;
	};
} disp_ext_sub_linux2mdm_rx_type;

typedef struct
{
	uint32_t mode;
	struct fb_var_subdispinfo disp_info;
} disp_ext_sub_dispctl_type;

typedef struct
{
	uint32_t timeadj;
} disp_ext_sub_timeradj_type;

typedef struct
{
	uint32_t level;
} disp_ext_sub_loglvl_type;

typedef struct
{
	uint32_t seq;
	uint32_t request;
	union {
		disp_ext_sub_dispctl_type  dispctl;
		disp_ext_sub_timeradj_type timardj;
		disp_ext_sub_loglvl_type loglvl;
		uint32_t param;
		uint8_t buf[56];
	};
} disp_ext_sub_linux2mdm_tx_type;

struct disp_ext_sub_smd_data {
	int                     request_updated;
	disp_ext_sub_linux2mdm_tx_type tx;
	disp_ext_sub_linux2mdm_rx_type rx;

	const char*             port_name;
	struct smd_channel*     smd_ch;
	bool                    smd_opened;
	void (*cb_func)(void *, unsigned);
	struct completion       completion;
	struct mutex            mutex;
};

typedef struct
{
	uint32_t seq;
	uint32_t request;
} disp_ext_sub_mdm2linux_rx_type;

typedef struct
{
	uint32_t seq;
	uint32_t request;
	uint32_t result;
	uint32_t padding;
} disp_ext_sub_mdm2linux_tx_type;

struct disp_ext_sub_smd_m2l_data {
	struct work_struct      work;

	disp_ext_sub_mdm2linux_tx_type tx;
	disp_ext_sub_mdm2linux_rx_type rx;

	const char*             port_name;
	struct smd_channel*     smd_ch;
	bool                    smd_opened;
	void (*cb_func)(void *, unsigned);
};

struct disp_ext_sub_mdm_notify_data {
	struct notifier_block  nb;
	struct work_struct     work;
	struct completion      completion;
};
#endif /* CONFIG_DISP_EXT_SUB_ALWAYS_ON */

struct disp_ext_sub_pinctrl_res {
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
};

struct disp_ext_sub_pdata {
	u32  xres;
	u32  yres;
	u32  bpp;
	int  rst_gpio;
	struct disp_ext_spi_data  spi_data;
	int  vh_gpio;
	int  voled_gpio;
	int  rs_gpio;
	struct disp_ext_sub_cmds  pwron_cmd;
	struct disp_ext_sub_cmds  on_cmd;
	struct disp_ext_sub_cmds  post_on_cmd;
	struct disp_ext_sub_cmds  off_cmd;
	struct disp_ext_sub_cmds  pwroff_cmd;
	struct disp_ext_sub_cmds  ram_wr_cmd;
	disp_ext_sub_state_type state;
#ifdef CONFIG_DISP_EXT_SUB_ALWAYS_ON
	disp_ext_sub_modem_state_type modem_state;
#endif /* CONFIG_DISP_EXT_SUB_ALWAYS_ON */
	struct pinctrl            *pinctrl;
	struct disp_ext_sub_pinctrl_res pin_res;
	bool first_update;
#ifdef CONFIG_DISP_EXT_SUB_ALWAYS_ON
	struct disp_ext_sub_smd_data  smd_data;
	struct disp_ext_sub_smd_m2l_data  smd_m2l_data;
	struct disp_ext_sub_mdm_notify_data  mdm_notif;
#endif /* CONFIG_DISP_EXT_SUB_ALWAYS_ON */
	struct fb_var_subdispinfo subdispinfo;
	int  current_device_elec_vol;
	struct regulator *vdd;
};

int disp_ext_sub_panel_set_status(disp_ext_sub_state_type next_status, struct disp_ext_sub_pdata *pdata);
int disp_ext_sub_panel_update(struct fb_var_screeninfo *var, struct fb_info *info, uint8_t* apps_img_p);
int disp_ext_sub_set_cmd(void * payload_p, unsigned char payload_len, struct disp_ext_sub_pdata *pdata);
int disp_ext_sub_set_cmd2(void * payload_p, unsigned char payload_len, struct disp_ext_sub_pdata *pdata);
int disp_ext_sub_panel_bus_init(struct disp_ext_sub_pdata *pdata);
int disp_ext_sub_panel_signal_init(struct disp_ext_sub_pdata *pdata);
int disp_ext_sub_panel_pinctrl_init(struct platform_device *pdev, struct disp_ext_sub_pdata *pdata);
int disp_ext_sub_get_panel_dt(struct device_node * np, struct disp_ext_sub_pdata *pdata);
int disp_ext_sub_get_signal_dt(struct device_node * np, struct disp_ext_sub_pdata *pdata);
int disp_ext_sub_get_seq_dt(struct device_node * np, struct disp_ext_sub_pdata *pdata);
int disp_ext_sub_get_spi_dt(struct device_node * np, struct device *dev, struct disp_ext_spi_data *sdata);
int disp_ext_sub_subdispinfo_init(struct disp_ext_sub_pdata *pdata);
int disp_ext_sub_get_vreg_dt(struct device *dev, struct disp_ext_sub_pdata *pdata);
int disp_ext_sub_set_subdispinfo(struct disp_ext_sub_pdata *pdata, struct fb_var_subdispinfo *psubdispinfo);
int disp_ext_sub_get_subdispinfo(struct disp_ext_sub_pdata *pdata);
int disp_ext_sub_set_data(void * payload_p, unsigned char payload_len, struct disp_ext_sub_pdata *pdata);
int disp_ext_sub_get_battery_temp(void);
int disp_ext_sub_panel_reset_cntrl(int onoff);
int disp_ext_sub_set_user_contrast(struct disp_ext_sub_pdata *pdata, int value, bool force_set);

struct disp_ext_sub_funcs {
	int (*pdisp_ext_sub_panel_set_status)(disp_ext_sub_state_type next_status, struct disp_ext_sub_pdata *pdata);
	int (*pdisp_ext_sub_panel_update)(struct fb_var_screeninfo *var, struct fb_info *info, uint8_t* apps_img_p);
	int (*pdisp_ext_sub_set_cmd)(void * payload_p, unsigned char payload_len, struct disp_ext_sub_pdata *pdata);
	int (*pdisp_ext_sub_set_cmd2)(void * payload_p, unsigned char payload_len, struct disp_ext_sub_pdata *pdata);
	int (*pdisp_ext_sub_panel_bus_init)(struct disp_ext_sub_pdata *pdata);
	int (*pdisp_ext_sub_panel_signal_init)(struct disp_ext_sub_pdata *pdata);
	int (*pdisp_ext_sub_panel_pinctrl_init)(struct platform_device *pdev, struct disp_ext_sub_pdata *pdata);
	int (*pdisp_ext_sub_get_panel_dt)(struct device_node * np, struct disp_ext_sub_pdata *pdata);
	int (*pdisp_ext_sub_get_signal_dt)(struct device_node * np, struct disp_ext_sub_pdata *pdata);
	int (*pdisp_ext_sub_get_seq_dt)(struct device_node * np, struct disp_ext_sub_pdata *pdata);
	int (*pdisp_ext_sub_get_spi_dt)(struct device_node * np, struct device *dev, struct disp_ext_spi_data *sdata);
	int (*pdisp_ext_sub_subdispinfo_init)(struct disp_ext_sub_pdata *pdata);
	int (*pdisp_ext_sub_get_vreg_dt)(struct device *dev, struct disp_ext_sub_pdata *pdata);
	int (*pdisp_ext_sub_set_subdispinfo)(struct disp_ext_sub_pdata *pdata, struct fb_var_subdispinfo *psubdispinfo);
	int (*pdisp_ext_sub_get_subdispinfo)(struct disp_ext_sub_pdata *pdata);
	int (*pdisp_ext_sub_set_data)(void * payload_p, unsigned char payload_len, struct disp_ext_sub_pdata *pdata);
	int (*pdisp_ext_sub_get_battery_temp)(void);
	int (*pdisp_ext_sub_panel_reset_cntrl)(int onoff);
	int (*pdisp_ext_sub_set_user_contrast)(struct disp_ext_sub_pdata *pdata, int value, bool force_set);
};

int disp_ext_sub_get_panel_detect(struct device_node * np, struct disp_ext_sub_pdata *pdata);

void st7571_disp_ext_sub_func_register(struct disp_ext_sub_funcs *func);
void ld7032_disp_ext_sub_func_register(struct disp_ext_sub_funcs *func);

extern struct disp_ext_sub_funcs kc_disp_ext_sub_func;

#endif /* DISP_EXT_SUB_PANEL_H */
