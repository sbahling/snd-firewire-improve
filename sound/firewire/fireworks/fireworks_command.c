/*
 * fireworks_command.c - a part of driver for Fireworks based devices
 *
 * Copyright (c) 2013 Takashi Sakamoto <o-takashi@sakmocchi.jp>
 *
 *
 * This driver is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2.
 *
 * This driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this driver; if not, see <http://www.gnu.org/licenses/>.
 */

#include "./fireworks.h"

/*
 * This driver uses transaction version 1 or later to use extended hardware
 * information. Then too old devices are not available.
 *
 * Each commands are not required to have continuous sequence numbers. This
 * number is just used to match command and response.
 *
 * NOTE: FFADO implementaion is EFC over AVC but device with firmware
 * version 5.5 or later support it but don't use it. This module support a part
 * of commands. Please see FFADO if you want to see whole commands. But I note
 * there are some commands which FFADO don't implement
 */

#define EFW_TRANSACTION_SEQNUM_MIN	(SND_EFW_TRANSACTION_SEQNUM_MAX + 1)
#define EFW_TRANSACTION_SEQNUM_MAX	((u32)~0)

/* for clock source and sampling rate */
struct efc_clock {
	u32 source;
	u32 sampling_rate;
	u32 index;
};

/* command categories */
enum efc_category {
	EFC_CAT_HWINFO		= 0,
	EFC_CAT_TRANSPORT	= 2,
	EFC_CAT_HWCTL		= 3,
};

/* hardware info category commands */
enum efc_cmd_hwinfo {
	EFC_CMD_HWINFO_GET_CAPS		= 0,
	EFC_CMD_HWINFO_GET_POLLED	= 1,
	EFC_CMD_HWINFO_SET_RESP_ADDR	= 2
};

enum efc_cmd_transport {
	EFC_CMD_TRANSPORT_SET_TX_MODE	= 0
};

/* hardware control category commands */
enum efc_cmd_hwctl {
	EFC_CMD_HWCTL_SET_CLOCK		= 0,
	EFC_CMD_HWCTL_GET_CLOCK		= 1,
	EFC_CMD_HWCTL_IDENTIFY		= 5
};

/* return values in response */
enum efr_status {
	EFC_RETVAL_OK			= 0,
	EFC_RETVAL_BAD			= 1,
	EFC_RETVAL_BAD_COMMAND		= 2,
	EFC_RETVAL_COMM_ERR		= 3,
	EFC_RETVAL_BAD_QUAD_COUNT	= 4,
	EFC_RETVAL_UNSUPPORTED		= 5,
	EFC_RETVAL_1394_TIMEOUT		= 6,
	EFC_RETVAL_DSP_TIMEOUT		= 7,
	EFC_RETVAL_BAD_RATE		= 8,
	EFC_RETVAL_BAD_CLOCK		= 9,
	EFC_RETVAL_BAD_CHANNEL		= 10,
	EFC_RETVAL_BAD_PAN		= 11,
	EFC_RETVAL_FLASH_BUSY		= 12,
	EFC_RETVAL_BAD_MIRROR		= 13,
	EFC_RETVAL_BAD_LED		= 14,
	EFC_RETVAL_BAD_PARAMETER	= 15,
	EFC_RETVAL_INCOMPLETE		= 0x80000000
};

static const char *const efr_status_names[] = {
	[EFC_RETVAL_OK]			= "OK",
	[EFC_RETVAL_BAD]		= "bad",
	[EFC_RETVAL_BAD_COMMAND]	= "bad command",
	[EFC_RETVAL_COMM_ERR]		= "comm err",
	[EFC_RETVAL_BAD_QUAD_COUNT]	= "bad quad count",
	[EFC_RETVAL_UNSUPPORTED]	= "unsupported",
	[EFC_RETVAL_1394_TIMEOUT]	= "1394 timeout",
	[EFC_RETVAL_DSP_TIMEOUT]	= "DSP timeout",
	[EFC_RETVAL_BAD_RATE]		= "bad rate",
	[EFC_RETVAL_BAD_CLOCK]		= "bad clock",
	[EFC_RETVAL_BAD_CHANNEL]	= "bad channel",
	[EFC_RETVAL_BAD_PAN]		= "bad pan",
	[EFC_RETVAL_FLASH_BUSY]		= "flash busy",
	[EFC_RETVAL_BAD_MIRROR]		= "bad mirror",
	[EFC_RETVAL_BAD_LED]		= "bad LED",
	[EFC_RETVAL_BAD_PARAMETER]	= "bad parameter",
	[EFC_RETVAL_BAD_PARAMETER + 1]	= "incomplete"
};

static int
efw_transaction(struct snd_efw *efw, unsigned int category,
		unsigned int command,
		const u32 *params, unsigned int param_count,
		void *response, unsigned int response_quadlets)
{
	struct snd_efw_transaction *header;
	__be32 *buf;
	u32 seqnum;
	unsigned int i, buf_bytes;
	int err;

	/* calculate buffer size*/
	buf_bytes = sizeof(struct snd_efw_transaction)
			 + max(param_count, response_quadlets) * 4;

	/* keep buffer */
	buf = kzalloc(buf_bytes, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	/* to keep consistency of sequence number */
	spin_lock(&efw->lock);
	if ((efw->seqnum < EFW_TRANSACTION_SEQNUM_MIN) ||
	    (efw->seqnum >= EFW_TRANSACTION_SEQNUM_MAX - 2))
		efw->seqnum = EFW_TRANSACTION_SEQNUM_MIN;
	else
		efw->seqnum += 2;
	seqnum = efw->seqnum;
	spin_unlock(&efw->lock);

	/* fill efc fields */
	header = (struct snd_efw_transaction *)buf;
	header->length		= sizeof(struct snd_efw_transaction) / 4 +
				  param_count;
	header->version		= 1;
	header->seqnum		= seqnum;
	header->category	= category;
	header->command		= command;
	header->status		= 0;

	/* fill EFC parameters */
	for (i = 0; i < param_count; i++)
		header->params[i] = params[i];

	/* for endian-ness*/
	for (i = 0; i < (buf_bytes / 4); i++)
		buf[i] = cpu_to_be32(buf[i]);

	err = snd_efw_transaction_run(efw->unit, buf, buf_bytes,
				      buf, buf_bytes, seqnum);
	if (err < 0)
		goto end;

	/* for endian-ness */
	for (i = 0; i < (err / 4); i++)
		buf[i] = be32_to_cpu(buf[i]);

	/* check EFC response fields */
	if ((header->version < 1) ||
	    (header->category != category) ||
	    (header->command != command) ||
	    (header->status != EFC_RETVAL_OK)) {
		dev_err(&efw->unit->device, "EFC failed [%u/%u]: %s\n",
			header->category, header->command,
			efr_status_names[header->status]);
		err = -EIO;
		goto end;
	}

	/* fill response buffer */
	if (response != NULL) {
		memset(response, 0, response_quadlets * 4);
		response_quadlets = min(response_quadlets, header->length);
		memcpy(response, header->params, response_quadlets * 4);
	}
end:
	kfree(buf);
	return err;
}

int snd_efw_command_identify(struct snd_efw *efw)
{
	return efw_transaction(efw, EFC_CAT_HWCTL,
			       EFC_CMD_HWCTL_IDENTIFY,
			       NULL, 0, NULL, 0);
}

/*
 * The address in host system for EFC response is changable when the device
 * supports. struct hwinfo.flags includes its flag. The default is
 * INITIAL_MEMORY_SPACE_EFC_RESPONSE
 */
int snd_efw_command_set_resp_addr(struct snd_efw *efw,
				  u16 addr_high, u32 addr_low)
{
	u32 addr[2] = {addr_high, addr_low};

	return efw_transaction(efw, EFC_CAT_HWCTL,
			       EFC_CMD_HWINFO_SET_RESP_ADDR,
			       addr, 2, NULL, 0);
}

/*
 * This is for timestamp processing. In Windows mode, all 32bit fields of second
 * CIP header in AMDTP transmit packet is used for 'presentation timestamp'. In
 * 'no data' packet the value of this field is 0x90ffffff.
 */
int snd_efw_command_set_tx_mode(struct snd_efw *efw,
				enum snd_efw_transport_mode mode)
{
	u32 param = mode;
	return efw_transaction(efw, EFC_CAT_TRANSPORT,
			       EFC_CMD_TRANSPORT_SET_TX_MODE,
			       &param, 1, NULL, 0);
}

int snd_efw_command_get_hwinfo(struct snd_efw *efw,
			       struct snd_efw_hwinfo *hwinfo)
{
	u32 *tmp;
	unsigned int i, count;
	int err;

	err  = efw_transaction(efw, EFC_CAT_HWINFO,
			       EFC_CMD_HWINFO_GET_CAPS,
			       NULL, 0, hwinfo, sizeof(*hwinfo) / 4);
	if (err < 0)
		goto end;

	/* arrangement for endianness */
	count = HWINFO_NAME_SIZE_BYTES / 4;
	tmp = (u32 *)&hwinfo->vendor_name;
	for (i = 0; i < count; i++)
		tmp[i] = cpu_to_be32(tmp[i]);
	tmp = (u32 *)&hwinfo->model_name;
	for (i = 0; i < count; i++)
		tmp[i] = cpu_to_be32(tmp[i]);

	count = sizeof(struct snd_efw_phys_grp) * HWINFO_MAX_CAPS_GROUPS / 4;
	tmp = (u32 *)&hwinfo->phys_out_grps;
	for (i = 0; i < count; i++)
		tmp[i] = cpu_to_be32(tmp[i]);
	tmp = (u32 *)&hwinfo->phys_in_grps;
	for (i = 0; i < count; i++)
		tmp[i] = cpu_to_be32(tmp[i]);

	/* ensure terminated */
	hwinfo->vendor_name[HWINFO_NAME_SIZE_BYTES - 1] = '\0';
	hwinfo->model_name[HWINFO_NAME_SIZE_BYTES  - 1] = '\0';
end:
	return err;
}

int snd_efw_command_get_phys_meters(struct snd_efw *efw,
				    struct snd_efw_phys_meters *meters,
				    unsigned int len)
{
	return efw_transaction(efw, EFC_CAT_HWINFO,
			       EFC_CMD_HWINFO_GET_POLLED,
			       NULL, 0, meters, len / 4);
}

static int
command_get_clock(struct snd_efw *efw, struct efc_clock *clock)
{
	return efw_transaction(efw, EFC_CAT_HWCTL,
			       EFC_CMD_HWCTL_GET_CLOCK,
			       NULL, 0, clock, sizeof(struct efc_clock) / 4);
}

/* give UINT_MAX if set nothing */
static int
command_set_clock(struct snd_efw *efw,
		  unsigned int source, unsigned int rate)
{
	int err;

	struct efc_clock clock = {0};

	/* check arguments */
	if ((source == UINT_MAX) && (rate == UINT_MAX)) {
		err = -EINVAL;
		goto end;
	}

	/* get current status */
	err = command_get_clock(efw, &clock);
	if (err < 0)
		goto end;

	/* no need */
	if ((clock.source == source) && (clock.sampling_rate == rate))
		goto end;

	/* set params */
	if ((source != UINT_MAX) && (clock.source != source))
		clock.source = source;
	if ((rate != UINT_MAX) && (clock.sampling_rate != rate))
		clock.sampling_rate = rate;
	clock.index = 0;

	err = efw_transaction(efw, EFC_CAT_HWCTL,
			      EFC_CMD_HWCTL_SET_CLOCK,
			      (u32 *)&clock, 3, NULL, 0);
	if (err < 0)
		goto end;

	/*
	 * With firmware version 5.8, just after changing clock state, these
	 * parameters are not immediately retrieved by get command. In my
	 * trial, there needs to be 100msec to get changed parameters.
	 */
	msleep(150);
end:
	return err;
}

int snd_efw_command_get_clock_source(struct snd_efw *efw,
				     enum snd_efw_clock_source *source)
{
	int err;
	struct efc_clock clock = {0};

	err = command_get_clock(efw, &clock);
	if (err >= 0)
		*source = clock.source;

	return err;
}

int snd_efw_command_set_clock_source(struct snd_efw *efw,
				     enum snd_efw_clock_source source)
{
	return command_set_clock(efw, source, UINT_MAX);
}

int snd_efw_command_get_sampling_rate(struct snd_efw *efw,
				      unsigned int *rate)
{
	int err;
	struct efc_clock clock = {0};

	err = command_get_clock(efw, &clock);
	if (err >= 0)
		*rate = clock.sampling_rate;

	return err;
}

int
snd_efw_command_set_sampling_rate(struct snd_efw *efw,
				  unsigned int rate)
{
	return command_set_clock(efw, UINT_MAX, rate);
}

