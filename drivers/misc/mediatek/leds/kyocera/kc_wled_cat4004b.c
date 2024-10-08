/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2022 KYOCERA Corporation
 */

#define pr_fmt(fmt)	"WLED %s: " fmt, __func__

#include <linux/module.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include "kc_wled.h"

#define EN_HI() gpio_set_value(gpio, 1)
#define EN_LO() gpio_set_value(gpio, 0)

#define EN_SET_RISING_EDGE() \
	EN_LO(); \
	udelay(10); \
	EN_HI()

#define EN_SETUP() \
	EN_HI(); \
	usleep_range(10,10)

#define CAT4004B_CURRENT_ST_MAX CAT4004B_CURRENT_ST_31

typedef enum {
  CAT4004B_CURRENT_ST_0,	/*    0 mA */
  CAT4004B_CURRENT_ST_1,	/*  0.8 mA */
  CAT4004B_CURRENT_ST_2,	/*  1.6 mA */
  CAT4004B_CURRENT_ST_3,	/*  2.4 mA */
  CAT4004B_CURRENT_ST_4,	/*  3.2 mA */
  CAT4004B_CURRENT_ST_5,	/*  4.0 mA */
  CAT4004B_CURRENT_ST_6,	/*  4.8 mA */
  CAT4004B_CURRENT_ST_7,	/*  5.6 mA */
  CAT4004B_CURRENT_ST_8,	/*  6.5 mA */
  CAT4004B_CURRENT_ST_9,	/*  7.3 mA */
  CAT4004B_CURRENT_ST_10,	/*  8.1 mA */
  CAT4004B_CURRENT_ST_11,	/*  8.9 mA */
  CAT4004B_CURRENT_ST_12,	/*  9.7 mA */
  CAT4004B_CURRENT_ST_13,	/* 10.5 mA */
  CAT4004B_CURRENT_ST_14,	/* 11.3 mA */
  CAT4004B_CURRENT_ST_15,	/* 12.1 mA */
  CAT4004B_CURRENT_ST_16,	/* 12.9 mA */
  CAT4004B_CURRENT_ST_17,	/* 13.7 mA */
  CAT4004B_CURRENT_ST_18,	/* 14.5 mA */
  CAT4004B_CURRENT_ST_19,	/* 15.3 mA */
  CAT4004B_CURRENT_ST_20,	/* 16.1 mA */
  CAT4004B_CURRENT_ST_21,	/* 16.9 mA */
  CAT4004B_CURRENT_ST_22,	/* 17.7 mA */
  CAT4004B_CURRENT_ST_23,	/* 18.5 mA */
  CAT4004B_CURRENT_ST_24,	/* 19.4 mA */
  CAT4004B_CURRENT_ST_25,	/* 20.2 mA */
  CAT4004B_CURRENT_ST_26,	/* 21.0 mA */
  CAT4004B_CURRENT_ST_27,	/* 21.8 mA */
  CAT4004B_CURRENT_ST_28,	/* 22.6 mA */
  CAT4004B_CURRENT_ST_29,	/* 23.4 mA */
  CAT4004B_CURRENT_ST_30,	/* 24.2 mA */
  CAT4004B_CURRENT_ST_31,	/* 25.0 mA */
  CAT4004B_CURRENT_ST_NUM
} current_state_type;

static const current_state_type current_state_tbl[WLED_BRIGHTNESS_NUM] = {
	CAT4004B_CURRENT_ST_0,	//0
	CAT4004B_CURRENT_ST_1,	//1
	CAT4004B_CURRENT_ST_1,	//2
	CAT4004B_CURRENT_ST_1,	//3
	CAT4004B_CURRENT_ST_1,	//4
	CAT4004B_CURRENT_ST_1,	//5
	CAT4004B_CURRENT_ST_1,	//6
	CAT4004B_CURRENT_ST_1,	//7
	CAT4004B_CURRENT_ST_1,	//8
	CAT4004B_CURRENT_ST_1,	//9
	CAT4004B_CURRENT_ST_1,	//10
	CAT4004B_CURRENT_ST_1,	//11
	CAT4004B_CURRENT_ST_1,	//12
	CAT4004B_CURRENT_ST_1,	//13
	CAT4004B_CURRENT_ST_1,	//14
	CAT4004B_CURRENT_ST_1,	//15
	CAT4004B_CURRENT_ST_2,	//16
	CAT4004B_CURRENT_ST_2,	//17
	CAT4004B_CURRENT_ST_2,	//18
	CAT4004B_CURRENT_ST_2,	//19
	CAT4004B_CURRENT_ST_2,	//20
	CAT4004B_CURRENT_ST_2,	//21
	CAT4004B_CURRENT_ST_2,	//22
	CAT4004B_CURRENT_ST_2,	//23
	CAT4004B_CURRENT_ST_2,	//24
	CAT4004B_CURRENT_ST_2,	//25
	CAT4004B_CURRENT_ST_2,	//26
	CAT4004B_CURRENT_ST_2,	//27
	CAT4004B_CURRENT_ST_2,	//28
	CAT4004B_CURRENT_ST_2,	//29
	CAT4004B_CURRENT_ST_2,	//30
	CAT4004B_CURRENT_ST_2,	//31
	CAT4004B_CURRENT_ST_3,	//32
	CAT4004B_CURRENT_ST_3,	//33
	CAT4004B_CURRENT_ST_3,	//34
	CAT4004B_CURRENT_ST_3,	//35
	CAT4004B_CURRENT_ST_3,	//36
	CAT4004B_CURRENT_ST_3,	//37
	CAT4004B_CURRENT_ST_3,	//38
	CAT4004B_CURRENT_ST_3,	//39
	CAT4004B_CURRENT_ST_4,	//40
	CAT4004B_CURRENT_ST_4,	//41
	CAT4004B_CURRENT_ST_4,	//42
	CAT4004B_CURRENT_ST_4,	//43
	CAT4004B_CURRENT_ST_4,	//44
	CAT4004B_CURRENT_ST_4,	//45
	CAT4004B_CURRENT_ST_4,	//46
	CAT4004B_CURRENT_ST_4,	//47
	CAT4004B_CURRENT_ST_5,	//48
	CAT4004B_CURRENT_ST_5,	//49
	CAT4004B_CURRENT_ST_5,	//50
	CAT4004B_CURRENT_ST_5,	//51
	CAT4004B_CURRENT_ST_5,	//52
	CAT4004B_CURRENT_ST_5,	//53
	CAT4004B_CURRENT_ST_5,	//54
	CAT4004B_CURRENT_ST_5,	//55
	CAT4004B_CURRENT_ST_5,	//56
	CAT4004B_CURRENT_ST_5,	//57
	CAT4004B_CURRENT_ST_5,	//58
	CAT4004B_CURRENT_ST_5,	//59
	CAT4004B_CURRENT_ST_5,	//60
	CAT4004B_CURRENT_ST_5,	//61
	CAT4004B_CURRENT_ST_5,	//62
	CAT4004B_CURRENT_ST_5,	//63
	CAT4004B_CURRENT_ST_6,	//64
	CAT4004B_CURRENT_ST_6,	//65
	CAT4004B_CURRENT_ST_6,	//66
	CAT4004B_CURRENT_ST_6,	//67
	CAT4004B_CURRENT_ST_6,	//68
	CAT4004B_CURRENT_ST_6,	//69
	CAT4004B_CURRENT_ST_6,	//70
	CAT4004B_CURRENT_ST_6,	//71
	CAT4004B_CURRENT_ST_6,	//72
	CAT4004B_CURRENT_ST_6,	//73
	CAT4004B_CURRENT_ST_6,	//74
	CAT4004B_CURRENT_ST_6,	//75
	CAT4004B_CURRENT_ST_6,	//76
	CAT4004B_CURRENT_ST_6,	//77
	CAT4004B_CURRENT_ST_6,	//78
	CAT4004B_CURRENT_ST_6,	//79
	CAT4004B_CURRENT_ST_7,	//80
	CAT4004B_CURRENT_ST_7,	//81
	CAT4004B_CURRENT_ST_7,	//82
	CAT4004B_CURRENT_ST_7,	//83
	CAT4004B_CURRENT_ST_7,	//84
	CAT4004B_CURRENT_ST_7,	//85
	CAT4004B_CURRENT_ST_7,	//86
	CAT4004B_CURRENT_ST_7,	//87
	CAT4004B_CURRENT_ST_7,	//88
	CAT4004B_CURRENT_ST_7,	//89
	CAT4004B_CURRENT_ST_7,	//90
	CAT4004B_CURRENT_ST_7,	//91
	CAT4004B_CURRENT_ST_7,	//92
	CAT4004B_CURRENT_ST_7,	//93
	CAT4004B_CURRENT_ST_7,	//94
	CAT4004B_CURRENT_ST_7,	//95
	CAT4004B_CURRENT_ST_8,	//96
	CAT4004B_CURRENT_ST_8,	//97
	CAT4004B_CURRENT_ST_8,	//98
	CAT4004B_CURRENT_ST_8,	//99
	CAT4004B_CURRENT_ST_8,	//100
	CAT4004B_CURRENT_ST_8,	//101
	CAT4004B_CURRENT_ST_8,	//102
	CAT4004B_CURRENT_ST_8,	//103
	CAT4004B_CURRENT_ST_8,	//104
	CAT4004B_CURRENT_ST_8,	//105
	CAT4004B_CURRENT_ST_8,	//106
	CAT4004B_CURRENT_ST_8,	//107
	CAT4004B_CURRENT_ST_8,	//108
	CAT4004B_CURRENT_ST_8,	//109
	CAT4004B_CURRENT_ST_8,	//110
	CAT4004B_CURRENT_ST_8,	//111
	CAT4004B_CURRENT_ST_9,	//112
	CAT4004B_CURRENT_ST_9,	//113
	CAT4004B_CURRENT_ST_9,	//114
	CAT4004B_CURRENT_ST_9,	//115
	CAT4004B_CURRENT_ST_9,	//116
	CAT4004B_CURRENT_ST_9,	//117
	CAT4004B_CURRENT_ST_9,	//118
	CAT4004B_CURRENT_ST_9,	//119
	CAT4004B_CURRENT_ST_9,	//120
	CAT4004B_CURRENT_ST_9,	//121
	CAT4004B_CURRENT_ST_9,	//122
	CAT4004B_CURRENT_ST_9,	//123
	CAT4004B_CURRENT_ST_9,	//124
	CAT4004B_CURRENT_ST_9,	//125
	CAT4004B_CURRENT_ST_9,	//126
	CAT4004B_CURRENT_ST_9,	//127
	CAT4004B_CURRENT_ST_10,	//128
	CAT4004B_CURRENT_ST_10,	//129
	CAT4004B_CURRENT_ST_10,	//130
	CAT4004B_CURRENT_ST_10,	//131
	CAT4004B_CURRENT_ST_10,	//132
	CAT4004B_CURRENT_ST_10,	//133
	CAT4004B_CURRENT_ST_10,	//134
	CAT4004B_CURRENT_ST_10,	//135
	CAT4004B_CURRENT_ST_10,	//136
	CAT4004B_CURRENT_ST_10,	//137
	CAT4004B_CURRENT_ST_10,	//138
	CAT4004B_CURRENT_ST_10,	//139
	CAT4004B_CURRENT_ST_10,	//140
	CAT4004B_CURRENT_ST_10,	//141
	CAT4004B_CURRENT_ST_10,	//142
	CAT4004B_CURRENT_ST_10,	//143
	CAT4004B_CURRENT_ST_10,	//144
	CAT4004B_CURRENT_ST_10,	//145
	CAT4004B_CURRENT_ST_10,	//146
	CAT4004B_CURRENT_ST_10,	//147
	CAT4004B_CURRENT_ST_10,	//148
	CAT4004B_CURRENT_ST_10,	//149
	CAT4004B_CURRENT_ST_10,	//150
	CAT4004B_CURRENT_ST_10,	//151
	CAT4004B_CURRENT_ST_11,	//152
	CAT4004B_CURRENT_ST_11,	//153
	CAT4004B_CURRENT_ST_11,	//154
	CAT4004B_CURRENT_ST_11,	//155
	CAT4004B_CURRENT_ST_11,	//156
	CAT4004B_CURRENT_ST_11,	//157
	CAT4004B_CURRENT_ST_11,	//158
	CAT4004B_CURRENT_ST_11,	//159
	CAT4004B_CURRENT_ST_11,	//160
	CAT4004B_CURRENT_ST_11,	//161
	CAT4004B_CURRENT_ST_11,	//162
	CAT4004B_CURRENT_ST_11,	//163
	CAT4004B_CURRENT_ST_11,	//164
	CAT4004B_CURRENT_ST_11,	//165
	CAT4004B_CURRENT_ST_11,	//166
	CAT4004B_CURRENT_ST_11,	//167
	CAT4004B_CURRENT_ST_12,	//168
	CAT4004B_CURRENT_ST_12,	//169
	CAT4004B_CURRENT_ST_12,	//170
	CAT4004B_CURRENT_ST_12,	//171
	CAT4004B_CURRENT_ST_12,	//172
	CAT4004B_CURRENT_ST_12,	//173
	CAT4004B_CURRENT_ST_12,	//174
	CAT4004B_CURRENT_ST_12,	//175
	CAT4004B_CURRENT_ST_12,	//176
	CAT4004B_CURRENT_ST_12,	//177
	CAT4004B_CURRENT_ST_12,	//178
	CAT4004B_CURRENT_ST_12,	//179
	CAT4004B_CURRENT_ST_12,	//180
	CAT4004B_CURRENT_ST_12,	//181
	CAT4004B_CURRENT_ST_12,	//182
	CAT4004B_CURRENT_ST_12,	//183
	CAT4004B_CURRENT_ST_12,	//184
	CAT4004B_CURRENT_ST_12,	//185
	CAT4004B_CURRENT_ST_12,	//186
	CAT4004B_CURRENT_ST_12,	//187
	CAT4004B_CURRENT_ST_12,	//188
	CAT4004B_CURRENT_ST_12,	//189
	CAT4004B_CURRENT_ST_12,	//190
	CAT4004B_CURRENT_ST_12,	//191
	CAT4004B_CURRENT_ST_13,	//192
	CAT4004B_CURRENT_ST_13,	//193
	CAT4004B_CURRENT_ST_13,	//194
	CAT4004B_CURRENT_ST_13,	//195
	CAT4004B_CURRENT_ST_13,	//196
	CAT4004B_CURRENT_ST_13,	//197
	CAT4004B_CURRENT_ST_13,	//198
	CAT4004B_CURRENT_ST_13,	//199
	CAT4004B_CURRENT_ST_14,	//200
	CAT4004B_CURRENT_ST_14,	//201
	CAT4004B_CURRENT_ST_14,	//202
	CAT4004B_CURRENT_ST_14,	//203
	CAT4004B_CURRENT_ST_15,	//204
	CAT4004B_CURRENT_ST_15,	//205
	CAT4004B_CURRENT_ST_15,	//206
	CAT4004B_CURRENT_ST_15,	//207
	CAT4004B_CURRENT_ST_15,	//208
	CAT4004B_CURRENT_ST_15,	//209
	CAT4004B_CURRENT_ST_15,	//210
	CAT4004B_CURRENT_ST_15,	//211
	CAT4004B_CURRENT_ST_16,	//212
	CAT4004B_CURRENT_ST_16,	//213
	CAT4004B_CURRENT_ST_16,	//214
	CAT4004B_CURRENT_ST_16,	//215
	CAT4004B_CURRENT_ST_17,	//216
	CAT4004B_CURRENT_ST_17,	//217
	CAT4004B_CURRENT_ST_17,	//218
	CAT4004B_CURRENT_ST_17,	//219
	CAT4004B_CURRENT_ST_17,	//220
	CAT4004B_CURRENT_ST_17,	//221
	CAT4004B_CURRENT_ST_17,	//222
	CAT4004B_CURRENT_ST_17,	//223
	CAT4004B_CURRENT_ST_18,	//224
	CAT4004B_CURRENT_ST_18,	//225
	CAT4004B_CURRENT_ST_18,	//226
	CAT4004B_CURRENT_ST_18,	//227
	CAT4004B_CURRENT_ST_19,	//228
	CAT4004B_CURRENT_ST_19,	//229
	CAT4004B_CURRENT_ST_19,	//230
	CAT4004B_CURRENT_ST_19,	//231
	CAT4004B_CURRENT_ST_20,	//232
	CAT4004B_CURRENT_ST_20,	//233
	CAT4004B_CURRENT_ST_20,	//234
	CAT4004B_CURRENT_ST_20,	//235
	CAT4004B_CURRENT_ST_21,	//236
	CAT4004B_CURRENT_ST_21,	//237
	CAT4004B_CURRENT_ST_21,	//238
	CAT4004B_CURRENT_ST_21,	//239
	CAT4004B_CURRENT_ST_22,	//240
	CAT4004B_CURRENT_ST_22,	//241
	CAT4004B_CURRENT_ST_22,	//242
	CAT4004B_CURRENT_ST_22,	//243
	CAT4004B_CURRENT_ST_23,	//244
	CAT4004B_CURRENT_ST_23,	//245
	CAT4004B_CURRENT_ST_23,	//246
	CAT4004B_CURRENT_ST_23,	//247
	CAT4004B_CURRENT_ST_24,	//248
	CAT4004B_CURRENT_ST_24,	//249
	CAT4004B_CURRENT_ST_24,	//250
	CAT4004B_CURRENT_ST_24,	//251
	CAT4004B_CURRENT_ST_25,	//252
	CAT4004B_CURRENT_ST_25,	//253
	CAT4004B_CURRENT_ST_25,	//254
	CAT4004B_CURRENT_ST_25,	//255
};

static DEFINE_SPINLOCK(cat4004b_lock);

static int current_state = CAT4004B_CURRENT_ST_0;

unsigned long wled_stat_aboot = WLED_BRIGHTNESS_DEFAULT;
static int __init cat4004b_set_stat_aboot(char *buf)
{
	int ret;
	
	pr_debug("[IN]\n");
	ret = kstrtoul(buf, 0, &wled_stat_aboot);
	if (ret != 0) {
		pr_err("[OUT] cat4004b_set_stat_aboot param error buf = %s ret = %d\n",buf,ret);
	}
	pr_err("[OUT] wled_stat_aboot=0x%lx\n", wled_stat_aboot);
	return 0;
}
early_param("led_stat", cat4004b_set_stat_aboot);


void cat4004b_init(void)
{
	current_state = current_state_tbl[wled_stat_aboot];
	pr_notice("[OUT] cat4004b_init=%d\n", current_state);
	pr_err("[OUT] cat4004b_init=%d\n", current_state);
}


void cat4004b_work(int level, enum kc_wled_table tbl, int gpio)
{
	int new_current_state;
	int shift_cnt = 0;
	unsigned long flags;

	new_current_state = current_state_tbl[level];
	pr_debug("%s: level = %d new_current_state=%d\n", __func__, level, new_current_state);
	
	if (current_state == new_current_state) {
		pr_debug("do not set because it is same value level:%d tbl:%d current_state:%d\n", level, tbl, current_state);
		return;
	}

	if (current_state < new_current_state) {
		shift_cnt = CAT4004B_CURRENT_ST_NUM - (new_current_state - current_state);

		if (current_state == 0) {
			EN_SETUP();
			shift_cnt--;
			pr_debug("shift_cnt-- to %d level:%d current_state:%d\n", shift_cnt, level, current_state);
		}
	} else {
		shift_cnt = current_state - new_current_state;
	}
	pr_debug("shift_cnt:%d\n", shift_cnt);

	spin_lock_irqsave(&cat4004b_lock, flags);
	for (; shift_cnt != 0; shift_cnt--) {
		EN_SET_RISING_EDGE();
		udelay(10);
	}
	spin_unlock_irqrestore(&cat4004b_lock, flags);

	if (new_current_state == 0) {
		EN_LO();
		pr_debug("CAT4004B OFF current_state:%d\n", current_state);
	}
	pr_debug("level:%d tbl:%d current_state old:%d new:%d\n", level, tbl, current_state, new_current_state);
	current_state = new_current_state;

	return;
}

