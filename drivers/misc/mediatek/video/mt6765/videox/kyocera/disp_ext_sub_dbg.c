/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 * (C) 2022 KYOCERA Corporation
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of_device.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/dynamic_debug.h>

#include <linux/fb.h>

#include "disp_ext_sub_ctrl.h"

static void disp_ext_sub_seq_data_dump( struct disp_ext_sub_cmds* cmd_p );

void disp_ext_sub_img_dump( void* img_p, uint32_t size, uint32_t byte_size )
{
	//dynamic_hex_dump(byte_size == 1 ? "subdisp raw image " : "subdisp fb image ", DUMP_PREFIX_NONE, 16, byte_size, img_p, size, false);
	return;
}

void  disp_ext_sub_seq_log( uint8_t *data_p, uint32_t size )
{
	int i;

	pr_debug("%s start\n",__func__);

	if( (!data_p) || (!size) ){
		pr_err("%s end - seq data invalid\n",__func__);
		return;
	}

	for( i=0; i<size; i++ ){
		pr_debug("data[%d]:[ %02x ]\n",i,(int)data_p[i]);
	}

	pr_debug("%s end\n",__func__);
	return;
}

void disp_ext_sub_pdata_dump( struct disp_ext_sub_pdata *pdata )
{
	struct disp_ext_sub_cmds* cmd_p;

	pr_debug("%s start\n",__func__);

	if(!pdata){
		pr_err("%s end\n",__func__);
		return;
	}

	pr_debug("xres:[%d]\n",(int)pdata->xres);
	pr_debug("yres:[%d]\n",(int)pdata->yres);

	pr_debug("rst_gpio:[%d]\n",(int)pdata->rst_gpio);
	pr_debug("cs_gpio:[%d]\n",(int)pdata->spi_data.cs_gpio);
	pr_debug("clk_gpio:[%d]\n",(int)pdata->spi_data.clk_gpio);
	pr_debug("data_gpio:[%d]\n",(int)pdata->spi_data.data_gpio);
	pr_debug("rs_gpio:[%d]\n",(int)pdata->spi_data.rs_gpio);

	cmd_p = &(pdata->pwron_cmd);
	pr_debug("PowerON command ---------------- \n");
	disp_ext_sub_seq_data_dump(cmd_p);

	cmd_p = &(pdata->on_cmd);
	pr_debug("ON command --------------------- \n");
	disp_ext_sub_seq_data_dump(cmd_p);

	cmd_p = &(pdata->post_on_cmd);
	pr_debug("Post ON command ---------------- \n");
	disp_ext_sub_seq_data_dump(cmd_p);

	cmd_p = &(pdata->off_cmd);
	pr_debug("OFF command -------------------- \n");
	disp_ext_sub_seq_data_dump(cmd_p);

	cmd_p = &(pdata->pwroff_cmd);
	pr_debug("PowerOFF command --------------- \n");
	disp_ext_sub_seq_data_dump(cmd_p);

	cmd_p = &(pdata->ram_wr_cmd);
	pr_debug("RAM Write command -------------- \n");
	disp_ext_sub_seq_data_dump(cmd_p);

	pr_debug("%s end\n",__func__);

	return;
}

static void disp_ext_sub_seq_data_dump( struct disp_ext_sub_cmds* cmd_p )
{
	struct  disp_ext_sub_cmd_detail* cmd_detail_p;
	int i;
	int j;
	unsigned char* data_p;

	if( cmd_p ) {
		cmd_detail_p = cmd_p->cmd_p;

		pr_debug("buf_p:[%x],blen:[%d],cmd_cnt:[%d]\n",
				(int)cmd_p->buf_p,(int)cmd_p->blen,(int)cmd_p->cmd_cnt );

		for( i=0; i<cmd_p->cmd_cnt; i++) {
			pr_debug("ctrl_kind:[%d],payload_len:[%d] :: ",
					(int)(cmd_detail_p[i].cmd_hdr.ctrl_kind),
					(int)(cmd_detail_p[i].cmd_hdr.payload_len) );

			data_p = (unsigned char*)cmd_detail_p[i].payload_p;
			for( j=0;
				 j<cmd_detail_p[i].cmd_hdr.payload_len;
				 j++ ) {
				pr_debug("0x%02x", (int)(*data_p));
				data_p++;
			}
			pr_debug("\n");
		}
	}

	return;
}
