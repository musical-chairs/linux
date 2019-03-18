// SPDX-License-Identifier: GPL-2.0
/*
 *   Focusrite Scarlett 18i20 Gen 2 Driver for ALSA
 *
 *   Copyright (c) 2018-2019 by Geoffrey D. Bennett
 *
 *   Based on the Scarlett (Gen 1) Driver for ALSA:
 *
 *   Copyright (c) 2013 by Tobias Hoffmann
 *   Copyright (c) 2013 by Robin Gareus <robin at gareus.org>
 *   Copyright (c) 2002 by Takashi Iwai <tiwai at suse.de>
 *   Copyright (c) 2014 by Chris J Arges <chris.j.arges at canonical.com>
 *
 *   Many codes borrowed from audio.c by
 *     Alan Cox (alan at lxorguk.ukuu.org.uk)
 *     Thomas Sailer (sailer at ife.ee.ethz.ch)
 *
 *   Code cleanup:
 *   David Henningsson <david.henningsson at canonical.com>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 */

/* Mixer Interface for the Focusrite Scarlett 18i20 Gen 2 audio
 * interface. Based on the Gen 1 driver and rewritten.
 */

/* The protocol was reverse engineered by looking at the communication
 * between Focusrite Control 2.3.4 and the Focusrite(R) Scarlett 18i20
 * (firmware 1083) using usbmon in July-August 2018.
 *
 * This ALSA mixer gives access to:
 *  - input, output, mixer-matrix muxes
 *  - 18x10 mixer-matrix gain stages
 *  - gain/volume controls
 *  - level meters
 *
 * <ditaa>
 *    /--------------\    18chn            20chn     /--------------\
 *    | Hardware  in +--+------\    /-------------+--+ ALSA PCM out |
 *    \--------------/  |      |    |             |  \--------------/
 *                      |      |    |    /-----\  |
 *                      |      |    |    |     |  |
 *                      |      v    v    v     |  |
 *                      |   +---------------+  |  |
 *                      |    \ Matrix  Mux /   |  |
 *                      |     +-----+-----+    |  |
 *                      |           |          |  |
 *                      |           |18chn     |  |
 *                      |           |          |  |
 *                      |           |     10chn|  |
 *                      |           v          |  |
 *                      |     +------------+   |  |
 *                      |     | Mixer      |   |  |
 *                      |     |     Matrix |   |  |
 *                      |     |            |   |  |
 *                      |     | 18x10 Gain |   |  |
 *                      |     |   stages   |   |  |
 *                      |     +-----+------+   |  |
 *                      |           |          |  |
 *                      |18chn      |10chn     |  |20chn
 *                      |           |          |  |
 *                      |           +----------/  |
 *                      |           |             |
 *                      v           v             v
 *                      ===========================
 *               +---------------+       +--—------------+
 *                \ Output  Mux /         \ Capture Mux /
 *                 +---+---+---+           +-----+-----+
 *                     |   |                     |
 *                10chn|   |                     |18chn
 *                     |   |                     |
 *  /--------------\   |   |                     |   /--------------\
 *  | S/PDIF, ADAT |<--/   |10chn                \-->| ALSA PCM in  |
 *  | Hardware out |       |                         \--------------/
 *  \--------------/       |
 *                         v
 *                  +-------------+    switch per channel to select
 *                  | Master Gain |<-- front panel volume knob or
 *                  +------+------+    individual software gain
 *                         |
 *                         |10chn
 *  /--------------\       |
 *  | Analogue     |<------/
 *  | Hardware out |
 *  \--------------/
 * </ditaa>
 *
 */

#include <linux/slab.h>
#include <linux/usb.h>

#include <sound/control.h>
#include <sound/tlv.h>

#include "usbaudio.h"
#include "mixer.h"
#include "helper.h"

#include "mixer_scarlett_gen2.h"

/* some gui mixers can't handle negative ctl values */
#define SCARLETT_VOLUME_BIAS 127

/* mixer range from -80dB to +6dB in 0.5dB steps */
#define SCARLETT_MIXER_MIN_DB -80
#define SCARLETT_MIXER_BIAS (-SCARLETT_MIXER_MIN_DB * 2)
#define SCARLETT_MIXER_MAX_DB 6
#define SCARLETT_MIXER_MAX_VALUE \
	((SCARLETT_MIXER_MAX_DB - SCARLETT_MIXER_MIN_DB) * 2)

/* map from (dB + 80) * 2 to mixer value
 * for dB in 0 .. 172: int(8192 * pow(10, ((dB - 160) / 2 / 20)))
 */
static const __u16 scarlett2_mixer_values[173] = {
	0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2,
	2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 8, 8,
	9, 9, 10, 10, 11, 12, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
	23, 24, 25, 27, 29, 30, 32, 34, 36, 38, 41, 43, 46, 48, 51,
	54, 57, 61, 65, 68, 73, 77, 81, 86, 91, 97, 103, 109, 115,
	122, 129, 137, 145, 154, 163, 173, 183, 194, 205, 217, 230,
	244, 259, 274, 290, 307, 326, 345, 365, 387, 410, 434, 460,
	487, 516, 547, 579, 614, 650, 689, 730, 773, 819, 867, 919,
	973, 1031, 1092, 1157, 1225, 1298, 1375, 1456, 1543, 1634,
	1731, 1833, 1942, 2057, 2179, 2308, 2445, 2590, 2744, 2906,
	3078, 3261, 3454, 3659, 3876, 4105, 4349, 4606, 4879, 5168,
	5475, 5799, 6143, 6507, 6892, 7301, 7733, 8192, 8677, 9191,
	9736, 10313, 10924, 11571, 12257, 12983, 13752, 14567, 15430,
	16345
};

/* Maximum number of analogue outputs */
#define SCARLETT2_CONTROLS_MAX 10

/* Maximum number of inputs to the mixer */
#define SCARLETT2_INPUT_MIX_MAX 18

/* Maximum number of outputs from the mixer */
#define SCARLETT2_OUTPUT_MIX_MAX 10

/* Maximum size of the data in the USB mux assignment message:
 * 18 inputs, 20 outputs, 18 matrix inputs, 8 spare
 */
#define SCARLETT2_MUX_MAX 64

/* Number of meters:
 * 18 inputs, 20 outputs, 18 matrix inputs
 */
#define SCARLETT2_NUM_METERS 56

/* Hardware port types:
 * - None (no input to mux)
 * - Analogue I/O
 * - S/PDIF I/O
 * - ADAT I/O
 * - Mixer I/O
 * - PCM I/O
 */
enum {
	SCARLETT2_PORT_TYPE_NONE = 0,
	SCARLETT2_PORT_TYPE_ANALOGUE = 1,
	SCARLETT2_PORT_TYPE_SPDIF = 2,
	SCARLETT2_PORT_TYPE_ADAT = 3,
	SCARLETT2_PORT_TYPE_MIX = 4,
	SCARLETT2_PORT_TYPE_PCM = 5,
	SCARLETT2_PORT_TYPE_COUNT = 6,
};

/* Count of total I/O and number available at each sample rate */
enum {
	SCARLETT2_PORT_IN = 0,
	SCARLETT2_PORT_OUT = 1,
	SCARLETT2_PORT_OUT_44 = 2,
	SCARLETT2_PORT_OUT_88 = 3,
	SCARLETT2_PORT_OUT_176 = 4,
	SCARLETT2_PORT_DIRECTIONS = 5,
};

#define SCARLETT2_BUTTON_COUNT 2

static const char *const scarlett2_button_names[SCARLETT2_BUTTON_COUNT] = {
	"Mute", "Dim"
};

/* Description of each hardware port type:
 * - id: hardware ID for this port type
 * - num: number of sources/destinations of this port type
 * - src_descr: printf format string for mux input selections
 * - src_num_offset: added to channel number for the fprintf
 * - dst_descr: printf format string for mixer controls
 */
struct scarlett2_ports {
	__u16 id;
	int num[SCARLETT2_PORT_DIRECTIONS];
	const char * const src_descr;
	int src_num_offset;
	const char * const dst_descr;
};

struct scarlett2_device_info {
	const char * const line_out_descrs[SCARLETT2_CONTROLS_MAX];
	struct scarlett2_ports ports[SCARLETT2_PORT_TYPE_COUNT];
};

struct scarlett2_chip_data {
	struct mutex usb_mutex; /* prevent sending concurrent USB requests */
	struct mutex data_mutex; /* lock access to this data */
	const struct scarlett2_device_info *info;
	int num_mux_srcs;
	__u16 scarlett2_seq;
	__u8 vol_updated;
	__u8 master_vol;
	__u8 vol[SCARLETT2_CONTROLS_MAX];
	__u8 vol_sw_hw_switch[SCARLETT2_CONTROLS_MAX];
	__u8 buttons[SCARLETT2_BUTTON_COUNT];
	struct snd_kcontrol *master_vol_ctl;
	struct snd_kcontrol *vol_ctls[SCARLETT2_CONTROLS_MAX];
	struct snd_kcontrol *button_ctls[SCARLETT2_CONTROLS_MAX];
	__u8 mux[SCARLETT2_MUX_MAX];
	__u8 mix[SCARLETT2_INPUT_MIX_MAX * SCARLETT2_OUTPUT_MIX_MAX];
};

struct __attribute__((__packed__)) scarlett2_usb_volume_status {
	__u8 buttons[SCARLETT2_BUTTON_COUNT]; /* mute & dim buttons */
	__u8 pad1;
	__s16 sw_vol[10]; /* software volume setting */
	__s16 hw_vol[10]; /* actual volume of output inc. dim (-18dB) */
	__u16 pad2[5];
	__u8 sw_hw_switch[10]; /* sw (0) or hw (1) controlled */
	__u16 pad3[3];
	__s16 master_vol; /* front panel volume knob */
};

/*** Model-specific data ***/

static const struct scarlett2_device_info s18i20_gen2_info = {
	.line_out_descrs = {
		"Monitor L",
		"Monitor R",
		NULL,
		NULL,
		NULL,
		NULL,
		"Headphones 1 L",
		"Headphones 1 R",
		"Headphones 2 L",
		"Headphones 2 R",
	},

	.ports = {
		{
			.id = 0x000,
			.num = { 1, 0, 8, 8, 6 },
			.src_descr = "Off",
			.src_num_offset = 0,
		},
		{
			.id = 0x080,
			.num = { 8, 10, 10, 10, 10 },
			.src_descr = "Analogue %d",
			.src_num_offset = 1,
			.dst_descr = "Analogue Output %02d Playback"
		},
		{
			/* S/PDIF outputs aren't available at 192KHz
			 * but are included in the USB mux I/O
			 * assignment message anyway
			 */
			.id = 0x180,
			.num = { 2, 2, 2, 2, 2 },
			.src_descr = "S/PDIF %d",
			.src_num_offset = 1,
			.dst_descr = "S/PDIF Output %d Playback"
		},
		{
			.id = 0x200,
			.num = { 8, 8, 8, 4, 0 },
			.src_descr = "ADAT %d",
			.src_num_offset = 1,
			.dst_descr = "ADAT Output %d Playback"
		},
		{
			.id = 0x300,
			.num = { 10, 18, 18, 18, 18 },
			.src_descr = "Mix %c",
			.src_num_offset = 65,
			.dst_descr = "Mixer Input %02d Capture"
		},
		{
			.id = 0x600,
			.num = { 20, 18, 18, 14, 10 },
			.src_descr = "PCM %d",
			.src_num_offset = 1,
			.dst_descr = "PCM %02d Capture"
		},
	},
};

/* get the starting port index number for a given port type/direction */
static int scarlett2_get_port_start_num(const struct scarlett2_ports *ports,
					int direction, int port_type)
{
	int i, num = 0;

	for (i = 0; i < port_type; i++)
		num += ports[i].num[direction];

	return num;
}

/*** USB Interactions ***/

/* Vendor-Specific Interface, Endpoint, MaxPacketSize, Interval */
#define SCARLETT2_USB_VENDOR_SPECIFIC_INTERFACE 5
#define SCARLETT2_USB_INTERRUPT_ENDPOINT 4
#define SCARLETT2_USB_INTERRUPT_MAX_DATA 64
#define SCARLETT2_USB_INTERRUPT_INTERVAL 3

/* Interrupt flags for volume and mute/dim button changes */
#define SCARLETT2_USB_INTERRUPT_VOL_CHANGE 0x400000
#define SCARLETT2_USB_INTERRUPT_BUTTON_CHANGE 0x200000

/* Commands for sending/receiving requests/responses */
#define SCARLETT2_USB_VENDOR_SPECIFIC_CMD_REQ 2
#define SCARLETT2_USB_VENDOR_SPECIFIC_CMD_RESP 3

#define SCARLETT2_USB_INIT_SEQ 0x00000000
#define SCARLETT2_USB_GET_METER_LEVELS 0x00001001
#define SCARLETT2_USB_SET_MIX 0x00002002
#define SCARLETT2_USB_SET_MUX 0x00003002
#define SCARLETT2_USB_GET_DATA 0x00800000
#define SCARLETT2_USB_SET_DATA_1 0x00800001
#define SCARLETT2_USB_SET_DATA_2 0x00800002

#define SCARLETT2_USB_VOLUME_STATUS_OFFSET 0x31
#define SCARLETT2_USB_LINE_OUT_VOLUME_OFFSET 0x34
#define SCARLETT2_USB_SW_HW_SWITCH_OFFSET 0x66

#define SCARLETT2_USB_LINE_OUT_VOLUME_SET_MAGIC 1
#define SCARLETT2_USB_SW_HW_SWITCH_SET_MAGIC 3
#define SCARLETT2_USB_METER_LEVELS_GET_MAGIC 1

/* Send a proprietary format request to the Scarlett interface. */
static int scarlett2_usb(
	struct usb_mixer_interface *mixer, __u32 cmd,
	void *req_data, __u16 req_size, void *resp_data, __u16 resp_size)
{
	struct scarlett2_chip_data *private = mixer->chip->private;

	/* proprietary request/response format */
	struct scarlett2_usb_packet {
		__u32 cmd;
		__u16 size;
		__u16 seq;
		__u32 error;
		__u32 pad;
		__u8 data[];
	};

	__u16 req_buf_size =
		sizeof(struct scarlett2_usb_packet) + req_size;
	__u16 resp_buf_size =
		sizeof(struct scarlett2_usb_packet) + resp_size;

	struct scarlett2_usb_packet *req, *resp;

	int seq, err;
	__u32 resp_cmd;
	__u32 resp_seq;
	__u16 size;

	mutex_lock(&private->usb_mutex);

	/* build request message and send it */

	req = kmalloc(req_buf_size, GFP_KERNEL);
	if (!req) {
		mutex_unlock(&private->usb_mutex);
		return -ENOMEM;
	}

	/* sequence must go up by 1 for each request */
	seq = private->scarlett2_seq++;

	req->cmd = cpu_to_le32(cmd);
	req->size = cpu_to_le16(req_size);
	req->seq = cpu_to_le16(seq);
	req->error = 0;
	req->pad = 0;
	if (req_size)
		memcpy(req->data, req_data, req_size);

	err = snd_usb_ctl_msg(mixer->chip->dev,
			usb_sndctrlpipe(mixer->chip->dev, 0),
			SCARLETT2_USB_VENDOR_SPECIFIC_CMD_REQ,
			USB_RECIP_INTERFACE | USB_TYPE_CLASS | USB_DIR_OUT,
			0,
			SCARLETT2_USB_VENDOR_SPECIFIC_INTERFACE,
			req,
			req_buf_size);

	kfree(req);

	if (err != req_buf_size) {
		snd_printk(KERN_ERR "Scarlett Gen 2 USB request result "
			   "command %x was %d\n", cmd, err);
		mutex_unlock(&private->usb_mutex);
		return -EINVAL;
	}

	/* send a second message to get the response */

	resp = kmalloc(resp_buf_size, GFP_KERNEL);
	if (!resp) {
		mutex_unlock(&private->usb_mutex);
		return -ENOMEM;
	}

	err = snd_usb_ctl_msg(mixer->chip->dev,
			usb_sndctrlpipe(mixer->chip->dev, 0),
			SCARLETT2_USB_VENDOR_SPECIFIC_CMD_RESP,
			USB_RECIP_INTERFACE | USB_TYPE_CLASS | USB_DIR_IN,
			0,
			SCARLETT2_USB_VENDOR_SPECIFIC_INTERFACE,
			resp,
			resp_buf_size);

	/* validate the response */

	if (err != resp_buf_size) {
		snd_printk(KERN_ERR "Scarlett Gen 2 USB response result "
			   "command %x was %d\n", cmd, err);
		kfree(resp);
		mutex_unlock(&private->usb_mutex);
		return -EINVAL;
	}

	resp_cmd = le32_to_cpu(resp->cmd);
	resp_seq = le16_to_cpu(resp->seq);
	size = le16_to_cpu(resp->size);

	if (resp_cmd != cmd ||
	    resp_seq != seq ||
	    size != resp_size ||
	    resp->error ||
	    resp->pad) {
		snd_printk(KERN_ERR "Scarlett Gen 2 USB invalid response; "
			   "cmd tx/rx %d/%d seq %d/%d size %d/%d "
			   "error %d pad %d\n",
			   cmd, resp_cmd, seq, resp_seq, size, resp_size,
			   resp->error, resp->pad);
		kfree(resp);
		mutex_unlock(&private->usb_mutex);
		return -EINVAL;
	}

	if (resp_size > 0)
		memcpy(resp_data, resp->data, resp_size);

	kfree(resp);
	mutex_unlock(&private->usb_mutex);
	return 0;
}

/* Send a USB message to set data (currently volume level or switch) */
static int scarlett2_usb_set(
	struct usb_mixer_interface *mixer,
	int index, int size, int magic_value, int value)
{
	struct __attribute__((__packed__)) {
		__u32 index;
		__u32 bytes;
		__s32 value;
	} req;
	__u32 req2;
	int err;

	/* send first message with the data */
	req.index = cpu_to_le32(index);
	req.bytes = cpu_to_le32(size);
	req.value = cpu_to_le32(value);
	err = scarlett2_usb(mixer, SCARLETT2_USB_SET_DATA_1,
			    &req, sizeof(__u32) * 2 + size, NULL, 0);
	if (err < 0)
		return err;

	/* send second message with the magic value */
	req2 = cpu_to_le32(magic_value);
	err = scarlett2_usb(mixer, SCARLETT2_USB_SET_DATA_2,
			    &req2, sizeof(req2), NULL, 0);
	if (err < 0)
		return err;

	return 0;
}

/* Send a USB message to get data; result placed in *buf */
static int scarlett2_usb_get(
	struct usb_mixer_interface *mixer,
	__u32 cmd, int offset, void *buf, int size)
{
	struct __attribute__((__packed__)) {
		__u32 offset;
		__u32 size;
	} req;
	int err;

	/* send first message with the data */
	req.offset = cpu_to_le32(offset);
	req.size = cpu_to_le32(size);
	err = scarlett2_usb(mixer, cmd, &req, sizeof(req), buf, size);
	if (err < 0)
		return err;

	return 0;
}

/* Send a USB message to get volume status; result placed in *buf */
static int scarlett2_usb_get_volume_status(
	struct usb_mixer_interface *mixer,
	struct scarlett2_usb_volume_status *buf)
{
	return scarlett2_usb_get(
		mixer,
		SCARLETT2_USB_GET_DATA,
		SCARLETT2_USB_VOLUME_STATUS_OFFSET,
		buf, sizeof(*buf));
}

/* Send a USB message to set the software volume for an analogue output */
static int scarlett2_usb_set_line_output_volume(
	struct usb_mixer_interface *mixer,
	int line_num, int volume)
{
	int offset = SCARLETT2_USB_LINE_OUT_VOLUME_OFFSET + line_num * 2;

	return scarlett2_usb_set(
		mixer, offset, 2,
		SCARLETT2_USB_LINE_OUT_VOLUME_SET_MAGIC, volume);
}

/* Send a USB message to set software (0) or hardware (1) volume for
 * an analogue output
 */
static int scarlett2_usb_set_monitor_volume_switch(
	struct usb_mixer_interface *mixer, int line_num, int value)
{
	int offset = SCARLETT2_USB_SW_HW_SWITCH_OFFSET + line_num;

	return scarlett2_usb_set(
		mixer, offset, 1,
		SCARLETT2_USB_SW_HW_SWITCH_SET_MAGIC, value);
}

/* Send a USB message to set the volumes for all inputs of one mix
 * (values obtained from private->mix[])
 */
static int scarlett2_usb_set_mix(struct usb_mixer_interface *mixer,
				     int mix_num)
{
	struct scarlett2_chip_data *private = mixer->chip->private;
	const struct scarlett2_device_info *info = private->info;

	struct __attribute__((__packed__)) {
		__u16 mix_num;
		__u16 data[SCARLETT2_INPUT_MIX_MAX];
	} req;

	int i, j, err;
	int num_mixer_in =
		info->ports[SCARLETT2_PORT_TYPE_MIX].num[SCARLETT2_PORT_OUT];

	req.mix_num = cpu_to_le16(mix_num);

	for (i = 0, j = mix_num * num_mixer_in; i < num_mixer_in; i++, j++)
		req.data[i] = cpu_to_le16(
			scarlett2_mixer_values[private->mix[j]]
		);

	err = scarlett2_usb(mixer, SCARLETT2_USB_SET_MIX,
			    &req, (num_mixer_in + 1) * sizeof(__u16),
			    NULL, 0);
	if (err < 0)
		return err;

	return 0;
}

/* Convert a port number index (per info->ports) to a hardware ID */
static __u32 scarlett2_mux_src_num_to_id(const struct scarlett2_ports *ports,
					 int num)
{
	int port_type;

	for (port_type = 0;
	     port_type < SCARLETT2_PORT_TYPE_COUNT;
	     port_type++) {
		if (num < ports[port_type].num[SCARLETT2_PORT_IN])
			return ports[port_type].id | num;
		num -= ports[port_type].num[SCARLETT2_PORT_IN];
	}

	/* Oops */
	return 0;
}

/* Send USB messages to set mux inputs */
static int scarlett2_usb_set_mux(struct usb_mixer_interface *mixer)
{
	struct scarlett2_chip_data *private = mixer->chip->private;
	const struct scarlett2_device_info *info = private->info;
	const struct scarlett2_ports *ports = info->ports;
	int rate, port_dir_rate;

	static const int assignment_order[SCARLETT2_PORT_TYPE_COUNT] = {
		SCARLETT2_PORT_TYPE_PCM,
		SCARLETT2_PORT_TYPE_ANALOGUE,
		SCARLETT2_PORT_TYPE_SPDIF,
		SCARLETT2_PORT_TYPE_ADAT,
		SCARLETT2_PORT_TYPE_MIX,
		SCARLETT2_PORT_TYPE_NONE,
	};

	struct __attribute__((__packed__)) {
		__u16 pad;
		__u16 num;
		__u32 data[SCARLETT2_MUX_MAX];
	} req;

	req.pad = 0;

	/* mux settings for each rate */
	for (rate = 0, port_dir_rate = SCARLETT2_PORT_OUT_44;
	     port_dir_rate <= SCARLETT2_PORT_OUT_176;
	     rate++, port_dir_rate++) {
		int order_num, i, err;

		req.num = cpu_to_le16(rate);

		for (order_num = 0, i = 0;
		     order_num < SCARLETT2_PORT_TYPE_COUNT;
		     order_num++) {
			int port_type = assignment_order[order_num];
			int j = scarlett2_get_port_start_num(ports,
							     SCARLETT2_PORT_OUT,
							     port_type);
			int port_id = ports[port_type].id;
			int channel;

			for (channel = 0;
			     channel < ports[port_type].num[port_dir_rate];
			     channel++, i++, j++)
				/* lower 12 bits for the destination and
				 * next 12 bits for the source
				 */
				req.data[i] = !port_id
					? 0
					: cpu_to_le32(
						port_id |
						channel |
						scarlett2_mux_src_num_to_id(
							ports, private->mux[j]
						) << 12
					  );

			/* skip private->mux[j] entries not output */
			j += ports[port_type].num[SCARLETT2_PORT_OUT] -
			     ports[port_type].num[port_dir_rate];
		}

		err = scarlett2_usb(mixer, SCARLETT2_USB_SET_MUX,
				    &req, (i + 1) * sizeof(__u32),
				    NULL, 0);
		if (err < 0)
			return err;
	}

	return 0;
}

/* Send USB message to get meter levels */
static int scarlett2_usb_get_meter_levels(struct usb_mixer_interface *mixer,
					  __u16 *levels)
{
	struct __attribute__((__packed__)) {
		__u16 pad;
		__u16 num_meters;
		__u32 magic;
	} req;
	__u32 resp[SCARLETT2_NUM_METERS];
	int i, err;

	req.pad = 0;
	req.num_meters = cpu_to_le16(SCARLETT2_NUM_METERS);
	req.magic = cpu_to_le32(SCARLETT2_USB_METER_LEVELS_GET_MAGIC);
	err = scarlett2_usb(mixer, SCARLETT2_USB_GET_METER_LEVELS,
			    &req, sizeof(req), resp, sizeof(resp));
	if (err < 0)
		return err;

	/* copy, convert to u16 */
	for (i = 0; i < SCARLETT2_NUM_METERS; i++)
		levels[i] = resp[i];

	return 0;
}

/*** Control Functions ***/

/* helper function to create a new control */
static int add_new_ctl(struct usb_mixer_interface *mixer,
		       const struct snd_kcontrol_new *ncontrol,
		       int index, int channels, const char *name,
		       struct snd_kcontrol **kctl_return)
{
	struct snd_kcontrol *kctl;
	struct usb_mixer_elem_info *elem;
	int err;

	elem = kzalloc(sizeof(*elem), GFP_KERNEL);
	if (!elem)
		return -ENOMEM;

	elem->head.mixer = mixer;
	elem->control = index;
	elem->head.id = index;
	elem->channels = channels;

	kctl = snd_ctl_new1(ncontrol, elem);
	if (!kctl) {
		kfree(elem);
		return -ENOMEM;
	}
	kctl->private_free = snd_usb_mixer_elem_free;

	strlcpy(kctl->id.name, name, sizeof(kctl->id.name));

	err = snd_usb_mixer_add_control(&elem->head, kctl);
	if (err < 0)
		return err;

	if (kctl_return)
		*kctl_return = kctl;

	return 0;
}

/*** Analogue Line Out Volume Controls ***/

/* first-time initialisation of volumes */
static int scarlett2_init_volumes(struct usb_mixer_interface *mixer)
{
	struct scarlett2_chip_data *private = mixer->chip->private;
	const struct scarlett2_ports *ports = private->info->ports;
	int num_line_out =
		ports[SCARLETT2_PORT_TYPE_ANALOGUE].num[SCARLETT2_PORT_OUT];
	struct scarlett2_usb_volume_status volume_status;
	int err, i;

	err = scarlett2_usb_get_volume_status(mixer, &volume_status);
	if (err < 0)
		return err;

	private->master_vol = clamp(
		volume_status.master_vol + SCARLETT_VOLUME_BIAS,
		0, SCARLETT_VOLUME_BIAS);

	for (i = 0; i < num_line_out; i++) {
		int volume;

		private->vol_sw_hw_switch[i] = !!volume_status.sw_hw_switch[i];

		volume = private->vol_sw_hw_switch[i]
			   ? volume_status.master_vol
			   : volume_status.sw_vol[i];
		volume = clamp(volume + SCARLETT_VOLUME_BIAS,
			       0, SCARLETT_VOLUME_BIAS);
		private->vol[i] = volume;
	}

	for (i = 0; i < SCARLETT2_BUTTON_COUNT; i++)
		private->buttons[i] = !!volume_status.buttons[i];

	return 0;
}

/* update hardware volume controls after receiving notification that
 * they have changed
 */
static int scarlett2_update_volumes(struct usb_mixer_interface *mixer)
{
	struct scarlett2_chip_data *private = mixer->chip->private;
	const struct scarlett2_ports *ports = private->info->ports;
	struct scarlett2_usb_volume_status volume_status;
	int num_line_out =
		ports[SCARLETT2_PORT_TYPE_ANALOGUE].num[SCARLETT2_PORT_OUT];
	int err, i;

	private->vol_updated = 0;

	err = scarlett2_usb_get_volume_status(mixer, &volume_status);
	if (err < 0)
		return err;

	private->master_vol = clamp(
		volume_status.master_vol + SCARLETT_VOLUME_BIAS,
		0, SCARLETT_VOLUME_BIAS);

	for (i = 0; i < num_line_out; i++) {
		if (private->vol_sw_hw_switch[i])
			private->vol[i] = private->master_vol;
	}

	for (i = 0; i < SCARLETT2_BUTTON_COUNT; i++)
		private->buttons[i] = !!volume_status.buttons[i];

	return 0;
}

static int scarlett2_volume_ctl_info(struct snd_kcontrol *kctl,
				     struct snd_ctl_elem_info *uinfo)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = elem->channels;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = SCARLETT_VOLUME_BIAS;
	uinfo->value.integer.step = 1;
	return 0;
}

static int scarlett2_master_volume_ctl_get(struct snd_kcontrol *kctl,
					   struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct scarlett2_chip_data *private = elem->head.mixer->chip->private;

	if (private->vol_updated) {
		mutex_lock(&private->data_mutex);
		scarlett2_update_volumes(elem->head.mixer);
		mutex_unlock(&private->data_mutex);
	}

	ucontrol->value.integer.value[0] = private->master_vol;
	return 0;
}

static int scarlett2_volume_ctl_get(struct snd_kcontrol *kctl,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct scarlett2_chip_data *private = elem->head.mixer->chip->private;
	int index = elem->control;

	if (private->vol_updated) {
		mutex_lock(&private->data_mutex);
		scarlett2_update_volumes(elem->head.mixer);
		mutex_unlock(&private->data_mutex);
	}

	ucontrol->value.integer.value[0] = private->vol[index];
	return 0;
}

static int scarlett2_volume_ctl_put(struct snd_kcontrol *kctl,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct scarlett2_chip_data *private = elem->head.mixer->chip->private;
	int index = elem->control;
	int oval, val, err;

	mutex_lock(&private->data_mutex);

	oval = private->vol[index];
	val = ucontrol->value.integer.value[0];

	if (oval == val) {
		mutex_unlock(&private->data_mutex);
		return 0;
	}

	private->vol[index] = val;
	err = scarlett2_usb_set_line_output_volume(elem->head.mixer,
						   index,
						   val - SCARLETT_VOLUME_BIAS);
	mutex_unlock(&private->data_mutex);
	return err < 0 ? err : 1;
}

static const DECLARE_TLV_DB_MINMAX(
	db_scale_scarlett_gain, -SCARLETT_VOLUME_BIAS * 100, 0
);

static const struct snd_kcontrol_new scarlett2_master_volume_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READ |
		  SNDRV_CTL_ELEM_ACCESS_TLV_READ,
	.name = "",
	.info = scarlett2_volume_ctl_info,
	.get  = scarlett2_master_volume_ctl_get,
	.private_value = 0, /* max value */
	.tlv = { .p = db_scale_scarlett_gain }
};

static const struct snd_kcontrol_new scarlett2_line_out_volume_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
		  SNDRV_CTL_ELEM_ACCESS_TLV_READ,
	.name = "",
	.info = scarlett2_volume_ctl_info,
	.get  = scarlett2_volume_ctl_get,
	.put  = scarlett2_volume_ctl_put,
	.private_value = 0, /* max value */
	.tlv = { .p = db_scale_scarlett_gain }
};

/*** HW/SW Volume Switch Controls ***/

static int scarlett2_sw_hw_enum_ctl_info(struct snd_kcontrol *kctl,
					 struct snd_ctl_elem_info *uinfo)
{
	static const char *const values[2] = {
		"SW", "HW"
	};

	return snd_ctl_enum_info(uinfo, 1, 2, values);
}

static int scarlett2_sw_hw_enum_ctl_get(struct snd_kcontrol *kctl,
					struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct scarlett2_chip_data *private = elem->head.mixer->chip->private;

	ucontrol->value.enumerated.item[0] =
		private->vol_sw_hw_switch[elem->control];
	return 0;
}

static int scarlett2_sw_hw_enum_ctl_put(struct snd_kcontrol *kctl,
					struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct scarlett2_chip_data *private = elem->head.mixer->chip->private;

	int index = elem->control;
	int oval, val, err;

	mutex_lock(&private->data_mutex);

	oval = private->vol_sw_hw_switch[index];
	val = !!ucontrol->value.integer.value[0];

	if (oval == val) {
		mutex_unlock(&private->data_mutex);
		return 0;
	}

	private->vol_sw_hw_switch[index] = val;

	/* Change access mode to RO (hardware controlled volume)
	 * or RW (software controlled volume)
	 */
	if (val)
		private->vol_ctls[index]->vd[0].access &=
			~SNDRV_CTL_ELEM_ACCESS_WRITE;
	else
		private->vol_ctls[index]->vd[0].access |=
			SNDRV_CTL_ELEM_ACCESS_WRITE;

	/* Reset volume to master volume */
	private->vol[index] = private->master_vol;

	/* Set SW volume to current HW volume */
	err = scarlett2_usb_set_line_output_volume(
		elem->head.mixer, index,
		private->master_vol - SCARLETT_VOLUME_BIAS);
	if (err < 0) {
		mutex_unlock(&private->data_mutex);
		return err;
	}

	/* Notify of RO/RW change */
	snd_ctl_notify(elem->head.mixer->chip->card,
		       SNDRV_CTL_EVENT_MASK_INFO,
		       &private->vol_ctls[index]->id);

	/* Send SW/HW switch change to the device */
	err = scarlett2_usb_set_monitor_volume_switch(elem->head.mixer,
						      index, val);
	mutex_unlock(&private->data_mutex);
	return err;
}

static const struct snd_kcontrol_new scarlett2_sw_hw_enum_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "",
	.info = scarlett2_sw_hw_enum_ctl_info,
	.get  = scarlett2_sw_hw_enum_ctl_get,
	.put  = scarlett2_sw_hw_enum_ctl_put,
};

/*** Mute/Dim Controls ***/

static int scarlett2_button_ctl_get(struct snd_kcontrol *kctl,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct scarlett2_chip_data *private = elem->head.mixer->chip->private;

	if (private->vol_updated) {
		mutex_lock(&private->data_mutex);
		scarlett2_update_volumes(elem->head.mixer);
		mutex_unlock(&private->data_mutex);
	}

	ucontrol->value.enumerated.item[0] = private->buttons[elem->control];
	return 0;
}

static const struct snd_kcontrol_new scarlett2_button_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READ,
	.name = "",
	.info = snd_ctl_boolean_mono_info,
	.get  = scarlett2_button_ctl_get
};

/*** Create the volume and volume switch controls ***/

static int scarlett2_add_line_out_ctls(struct usb_mixer_interface *mixer)
{
	struct scarlett2_chip_data *private = mixer->chip->private;
	const struct scarlett2_device_info *info = private->info;
	const struct scarlett2_ports *ports = info->ports;
	int num_line_out =
		ports[SCARLETT2_PORT_TYPE_ANALOGUE].num[SCARLETT2_PORT_OUT];
	int err, i;
	char mx[SNDRV_CTL_ELEM_ID_NAME_MAXLEN];

	/* Add R/O HW volume control */
	snprintf(mx, sizeof(mx), "Master HW Playback Volume");
	err = add_new_ctl(mixer, &scarlett2_master_volume_ctl,
			  0, 1, mx, &private->master_vol_ctl);
	if (err < 0)
		return err;

	/* Add volume controls */
	for (i = 0; i < num_line_out; i++) {

		/* Fader */
		if (info->line_out_descrs[i])
			snprintf(mx, sizeof(mx),
				 "Line %02d (%s) Playback Volume",
				 i + 1, info->line_out_descrs[i]);
		else
			snprintf(mx, sizeof(mx),
				 "Line %02d Playback Volume",
				 i + 1);
		err = add_new_ctl(mixer, &scarlett2_line_out_volume_ctl,
				  i, 1, mx, &private->vol_ctls[i]);
		if (err < 0)
			return err;

		/* Make the fader read-only if the SW/HW switch is set to HW */
		if (private->vol_sw_hw_switch[i])
			private->vol_ctls[i]->vd[0].access &=
				~SNDRV_CTL_ELEM_ACCESS_WRITE;

		/* SW/HW Switch */
		snprintf(mx, sizeof(mx),
			 "Line Out %02d Volume Control Playback Enum",
			 i + 1);
		err = add_new_ctl(mixer, &scarlett2_sw_hw_enum_ctl,
				  i, 1, mx, NULL);
		if (err < 0)
			return err;
	}

	/* Add button controls */
	for (i = 0; i < SCARLETT2_BUTTON_COUNT; i++) {
		err = add_new_ctl(mixer, &scarlett2_button_ctl,
				  i, 1, scarlett2_button_names[i],
				  &private->button_ctls[i]);
		if (err < 0)
			return err;
	}

	return 0;
}

/*** Mixer Volume Controls ***/

static int scarlett2_mixer_ctl_info(struct snd_kcontrol *kctl,
				    struct snd_ctl_elem_info *uinfo)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = elem->channels;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = SCARLETT_MIXER_MAX_VALUE;
	uinfo->value.integer.step = 1;
	return 0;
}

static int scarlett2_mixer_ctl_get(struct snd_kcontrol *kctl,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct scarlett2_chip_data *private = elem->head.mixer->chip->private;

	ucontrol->value.integer.value[0] = private->mix[elem->control];
	return 0;
}

static int scarlett2_mixer_ctl_put(struct snd_kcontrol *kctl,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct scarlett2_chip_data *private = elem->head.mixer->chip->private;
	const struct scarlett2_device_info *info = private->info;
	const struct scarlett2_ports *ports = info->ports;
	int oval, val, num_mixer_in, mix_num, err;

	mutex_lock(&private->data_mutex);

	oval = private->mix[elem->control];
	val = ucontrol->value.integer.value[0];
	num_mixer_in = ports[SCARLETT2_PORT_TYPE_MIX].num[SCARLETT2_PORT_OUT];
	mix_num = elem->control / num_mixer_in;

	if (oval == val) {
		mutex_unlock(&private->data_mutex);
		return 0;
	}

	private->mix[elem->control] = val;
	err = scarlett2_usb_set_mix(elem->head.mixer, mix_num);
	mutex_unlock(&private->data_mutex);
	return err < 0 ? err : 1;
}

static const DECLARE_TLV_DB_MINMAX(
	db_scale_scarlett_mixer,
	SCARLETT_MIXER_MIN_DB * 100,
	SCARLETT_MIXER_MAX_DB * 100
);

static const struct snd_kcontrol_new scarlett2_mixer_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
		  SNDRV_CTL_ELEM_ACCESS_TLV_READ,
	.name = "",
	.info = scarlett2_mixer_ctl_info,
	.get  = scarlett2_mixer_ctl_get,
	.put  = scarlett2_mixer_ctl_put,
	.private_value = SCARLETT_MIXER_MAX_DB, /* max value */
	.tlv = { .p = db_scale_scarlett_mixer }
};

static int scarlett2_add_mixer_ctls(struct usb_mixer_interface *mixer)
{
	struct scarlett2_chip_data *private = mixer->chip->private;
	const struct scarlett2_ports *ports = private->info->ports;
	int err, i, j;
	int index;
	char mx[SNDRV_CTL_ELEM_ID_NAME_MAXLEN];

	int num_inputs = ports[SCARLETT2_PORT_TYPE_MIX].num[SCARLETT2_PORT_OUT];
	int num_outputs = ports[SCARLETT2_PORT_TYPE_MIX].num[SCARLETT2_PORT_IN];

	for (i = 0, index = 0; i < num_outputs; i++) {
		for (j = 0; j < num_inputs; j++, index++) {
			snprintf(mx, sizeof(mx),
				 "Mix %c Input %02d Playback Volume",
				 'A' + i, j + 1);
			err = add_new_ctl(mixer, &scarlett2_mixer_ctl,
					  index, 1, mx, NULL);
			if (err < 0)
				return err;
		}
	}

	return 0;
}

/*** Mux Source Selection Controls ***/

static int scarlett2_mux_src_enum_ctl_info(struct snd_kcontrol *kctl,
					   struct snd_ctl_elem_info *uinfo)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct scarlett2_chip_data *private = elem->head.mixer->chip->private;
	const struct scarlett2_ports *ports = private->info->ports;
	unsigned int item = uinfo->value.enumerated.item;
	int items = private->num_mux_srcs;
	int port_type;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = elem->channels;
	uinfo->value.enumerated.items = items;

	if (item >= items)
		item = uinfo->value.enumerated.item = items - 1;

	for (port_type = 0;
	     port_type < SCARLETT2_PORT_TYPE_COUNT;
	     port_type++) {
		if (item < ports[port_type].num[SCARLETT2_PORT_IN]) {
			sprintf(uinfo->value.enumerated.name,
				ports[port_type].src_descr,
				item + ports[port_type].src_num_offset);
			return 0;
		}
		item -= ports[port_type].num[SCARLETT2_PORT_IN];
	}

	return -EINVAL;
}

static int scarlett2_mux_src_enum_ctl_get(struct snd_kcontrol *kctl,
					  struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct scarlett2_chip_data *private = elem->head.mixer->chip->private;

	ucontrol->value.enumerated.item[0] = private->mux[elem->control];
	return 0;
}

static int scarlett2_mux_src_enum_ctl_put(struct snd_kcontrol *kctl,
					  struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct scarlett2_chip_data *private = elem->head.mixer->chip->private;
	int index = elem->control;
	int oval, val, err;

	mutex_lock(&private->data_mutex);

	oval = private->mux[index];
	val = clamp(ucontrol->value.integer.value[0],
		    0L, private->num_mux_srcs - 1L);

	if (oval == val) {
		mutex_unlock(&private->data_mutex);
		return 0;
	}

	private->mux[index] = val;
	err = scarlett2_usb_set_mux(elem->head.mixer);
	mutex_unlock(&private->data_mutex);
	return err < 0 ? err : 1;
}

static const struct snd_kcontrol_new scarlett2_mux_src_enum_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "",
	.info = scarlett2_mux_src_enum_ctl_info,
	.get  = scarlett2_mux_src_enum_ctl_get,
	.put  = scarlett2_mux_src_enum_ctl_put,
};

static int scarlett2_add_mux_enums(struct usb_mixer_interface *mixer)
{
	struct scarlett2_chip_data *private = mixer->chip->private;
	const struct scarlett2_ports *ports = private->info->ports;
	int port_type, channel, i;

	for (i = 0, port_type = 0;
	     port_type < SCARLETT2_PORT_TYPE_COUNT;
	     port_type++) {
		for (channel = 0;
		     channel < ports[port_type].num[SCARLETT2_PORT_OUT];
		     channel++, i++) {
			int err;
			char mx[SNDRV_CTL_ELEM_ID_NAME_MAXLEN];
			const char *const descr = ports[port_type].dst_descr;

			snprintf(mx, sizeof(mx) - 5, descr, channel + 1);
			strcat(mx, " Enum");

			err = add_new_ctl(mixer,
					  &scarlett2_mux_src_enum_ctl,
					  i, 1, mx, NULL);
			if (err < 0)
				return err;
		}
	}

	return 0;
}

/*** Meter Controls ***/

static int scarlett2_meter_ctl_info(struct snd_kcontrol *kctl,
				    struct snd_ctl_elem_info *uinfo)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = elem->channels;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 4095;
	uinfo->value.integer.step = 1;
	return 0;
}

static int scarlett2_meter_ctl_get(struct snd_kcontrol *kctl,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	__u16 meter_levels[SCARLETT2_NUM_METERS];
	int i, err;

	err = scarlett2_usb_get_meter_levels(elem->head.mixer, meter_levels);
	if (err < 0)
		return err;

	for (i = 0; i < elem->channels; i++)
		ucontrol->value.integer.value[i] = meter_levels[i];

	return 0;
}

static const struct snd_kcontrol_new scarlett2_meter_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_PCM,
	.access = SNDRV_CTL_ELEM_ACCESS_READ | SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	.name = "",
	.info = scarlett2_meter_ctl_info,
	.get  = scarlett2_meter_ctl_get
};

static int scarlett2_add_meter_ctl(struct usb_mixer_interface *mixer)
{
	return add_new_ctl(mixer, &scarlett2_meter_ctl,
			   0, SCARLETT2_NUM_METERS, "Level Meter", NULL);
}

/*** Initialisation ***/

static int scarlett2_count_mux_srcs(const struct scarlett2_ports *ports)
{
	int port_type, count = 0;

	for (port_type = 0;
	     port_type < SCARLETT2_PORT_TYPE_COUNT;
	     port_type++)
		count += ports[port_type].num[SCARLETT2_PORT_IN];

	return count;
}

/* Default routing connects PCM outputs and inputs to Analogue,
 * S/PDIF, then ADAT
 */
static void scarlett2_init_routing(__u8 *mux,
				   const struct scarlett2_ports *ports)
{
	int i, input_num, input_count, port_type;
	int output_num, output_count, port_type_connect_num;

	static const int connect_order[] = {
		SCARLETT2_PORT_TYPE_ANALOGUE,
		SCARLETT2_PORT_TYPE_SPDIF,
		SCARLETT2_PORT_TYPE_ADAT,
		-1
	};

	/* Assign PCM inputs (routing outputs) */
	output_num = scarlett2_get_port_start_num(ports,
						  SCARLETT2_PORT_OUT,
						  SCARLETT2_PORT_TYPE_PCM);
	output_count = ports[SCARLETT2_PORT_TYPE_PCM].num[SCARLETT2_PORT_OUT];

	for (port_type = connect_order[port_type_connect_num = 0];
	     port_type >= 0;
	     port_type = connect_order[++port_type_connect_num]) {
		input_num = scarlett2_get_port_start_num(
			ports, SCARLETT2_PORT_IN, port_type);
		input_count = ports[port_type].num[SCARLETT2_PORT_IN];
		for (i = 0;
		     i < input_count && output_count;
		     i++, output_count--)
			mux[output_num++] = input_num++;
	}

	/* Assign PCM outputs (routing inputs) */
	input_num = scarlett2_get_port_start_num(ports,
						 SCARLETT2_PORT_IN,
						 SCARLETT2_PORT_TYPE_PCM);
	input_count = ports[SCARLETT2_PORT_TYPE_PCM].num[SCARLETT2_PORT_IN];

	for (port_type = connect_order[port_type_connect_num = 0];
	     port_type >= 0;
	     port_type = connect_order[++port_type_connect_num]) {
		output_num = scarlett2_get_port_start_num(
			ports, SCARLETT2_PORT_OUT, port_type);
		output_count = ports[port_type].num[SCARLETT2_PORT_OUT];
		for (i = 0;
		     i < output_count && input_count;
		     i++, input_count--)
			mux[output_num++] = input_num++;
	}
}

/* Initialise private data, routing, sequence number */
static int scarlett2_init_private(struct usb_mixer_interface *mixer,
				  const struct scarlett2_device_info *info)
{
	struct scarlett2_chip_data *private =
		kzalloc(sizeof(struct scarlett2_chip_data), GFP_KERNEL);

	if (!private)
		return -ENOMEM;

	mutex_init(&private->usb_mutex);
	mutex_init(&private->data_mutex);
	private->info = info;
	private->num_mux_srcs = scarlett2_count_mux_srcs(info->ports);
	private->scarlett2_seq = 0;
	mixer->chip->private = private;

	/* Setup default routing */
	scarlett2_init_routing(private->mux, info->ports);

	/* Initialise the sequence number used for the proprietary commands */
	return scarlett2_usb(mixer, SCARLETT2_USB_INIT_SEQ,
			     NULL, 0, NULL, 0);
}

/* Notify on volume change */
static void scarlett2_mixer_interrupt_vol_change(
	struct usb_mixer_interface *mixer)
{
	struct scarlett2_chip_data *private = mixer->chip->private;
	const struct scarlett2_ports *ports = private->info->ports;
	int num_line_out =
		ports[SCARLETT2_PORT_TYPE_ANALOGUE].num[SCARLETT2_PORT_OUT];
	int i;

	private->vol_updated = 1;

	snd_ctl_notify(mixer->chip->card, SNDRV_CTL_EVENT_MASK_VALUE,
		       &private->master_vol_ctl->id);

	for (i = 0; i < num_line_out; i++) {
		if (!private->vol_sw_hw_switch[i])
			continue;
		snd_ctl_notify(mixer->chip->card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &private->vol_ctls[i]->id);
	}
}

/* Notify on button change */
static void scarlett2_mixer_interrupt_button_change(
	struct usb_mixer_interface *mixer)
{
	struct scarlett2_chip_data *private = mixer->chip->private;
	int i;

	private->vol_updated = 1;

	for (i = 0; i < SCARLETT2_BUTTON_COUNT; i++)
		snd_ctl_notify(mixer->chip->card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &private->button_ctls[i]->id);
}

/* Interrupt callback */
static void scarlett2_mixer_interrupt(struct urb *urb)
{
	struct usb_mixer_interface *mixer = urb->context;
	int len = urb->actual_length;
	int ustatus = urb->status;
	__u32 data;

	if (ustatus != 0)
		goto requeue;

	if (len == 8) {
		data = le32_to_cpu(*(__u32 *)urb->transfer_buffer);
		if (data & SCARLETT2_USB_INTERRUPT_VOL_CHANGE)
			scarlett2_mixer_interrupt_vol_change(mixer);
		if (data & SCARLETT2_USB_INTERRUPT_BUTTON_CHANGE)
			scarlett2_mixer_interrupt_button_change(mixer);
	} else {
		snd_printk(KERN_ERR "scarlett mixer interrupt length %d\n",
			   len);
	}

requeue:
	if (ustatus != -ENOENT &&
	    ustatus != -ECONNRESET &&
	    ustatus != -ESHUTDOWN) {
		urb->dev = mixer->chip->dev;
		usb_submit_urb(urb, GFP_ATOMIC);
	}
}

static int scarlett2_mixer_status_create(struct usb_mixer_interface *mixer)
{
	void *transfer_buffer;

	if (mixer->urb) {
		snd_printk(KERN_ERR "scarlett2_mixer_status_create: "
			   "mixer urb already in use!\n");
		return 0;
	}

	mixer->urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!mixer->urb)
		return -ENOMEM;

	transfer_buffer = kmalloc(SCARLETT2_USB_INTERRUPT_MAX_DATA, GFP_KERNEL);
	if (!transfer_buffer)
		return -ENOMEM;

	usb_fill_int_urb(
		mixer->urb,
		mixer->chip->dev,
		usb_rcvintpipe(mixer->chip->dev,
			       SCARLETT2_USB_INTERRUPT_ENDPOINT),
		transfer_buffer,
		SCARLETT2_USB_INTERRUPT_MAX_DATA,
		scarlett2_mixer_interrupt,
		mixer,
		SCARLETT2_USB_INTERRUPT_INTERVAL);

	return usb_submit_urb(mixer->urb, GFP_KERNEL);
}

/* Entry point */
int snd_scarlett_gen2_controls_create(struct usb_mixer_interface *mixer)
{
	const struct scarlett2_device_info *info;
	int err;

	/* only use UAC_VERSION_2 */
	if (!mixer->protocol)
		return 0;

	switch (mixer->chip->usb_id) {
	case USB_ID(0x1235, 0x8201):
		info = &s18i20_gen2_info;
		break;
	default: /* device not (yet) supported */
		return -EINVAL;
	}

	/* Initialise private data, routing, sequence number */
	if ((err = scarlett2_init_private(mixer, info)) < 0)
		return err;

	/* Read volume levels and controls from the interface */
	if ((err = scarlett2_init_volumes(mixer)) < 0)
		return err;

	/* Create the analogue output volume controls */
	if ((err = scarlett2_add_line_out_ctls(mixer)) < 0)
		return err;

	/* Create the input, output, and mixer mux input selections */
	if ((err = scarlett2_add_mux_enums(mixer)) < 0)
		return err;

	/* Create the matrix mixer controls */
	if ((err = scarlett2_add_mixer_ctls(mixer)) < 0)
		return err;

	/* Create the level meter controls */
	if ((err = scarlett2_add_meter_ctl(mixer)) < 0)
		return err;

	/* Set up the interrupt polling */
	if ((err = scarlett2_mixer_status_create(mixer)) < 0)
		return err;

	return 0;
}
