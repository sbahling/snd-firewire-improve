/*
 * motu-protocol-v1.c - a part of driver for MOTU FireWire series
 *
 * Copyright (c) 2015-2017 Takashi Sakamoto <o-takashi@sakamocchi.jp>
 *
 * Licensed under the terms of the GNU General Public License, version 2.
 */

#include "motu.h"

#define V1_CLOCK_STATUS_OFFSET			0x0b00
#define  V1_OPT_IN_IFACE_IS_SPDIF		0x00008000
#define  V1_OPT_OUT_IFACE_IS_SPDIF		0x00004000
#define  V1_FETCH_PCM_FRAMES			0x00000080
#define  V1_CLOCK_SRC_IS_NOT_FROM_ADAT_DSUB	0x00000020
#define  V1_CLOCK_RATE_BASED_ON_48000		0x00000004
#define  V1_CLOCK_SRC_SPDIF_ON_OPT_OR_COAX	0x00000002
#define  V1_CLOCK_SRC_ADAT_ON_OPT_OR_DSUB	0x00000001

static int v1_get_clock_rate(struct snd_motu *motu, unsigned int *rate)
{
	__be32 reg;
	u32 data;
	int index, err;

	err = snd_motu_transaction_read(motu, V1_CLOCK_STATUS_OFFSET, &reg,
					sizeof(reg));
	if (err < 0)
		return err;
	data = be32_to_cpu(reg);

	if (data & V1_CLOCK_RATE_BASED_ON_48000)
		index = 1;
	else
		index = 0;

	*rate = snd_motu_clock_rates[index];

	return 0;
}

static int v1_set_clock_rate(struct snd_motu *motu, unsigned int rate)
{
	__be32 reg;
	u32 data;
	int i, err;

	for (i = 0; i < ARRAY_SIZE(snd_motu_clock_rates); ++i) {
		if (snd_motu_clock_rates[i] == rate)
			break;
	}
	if (i == ARRAY_SIZE(snd_motu_clock_rates))
		return -EINVAL;

	err = snd_motu_transaction_read(motu, V1_CLOCK_STATUS_OFFSET, &reg,
					sizeof(reg));
	if (err < 0)
		return err;
	data = be32_to_cpu(reg);

	data &= ~V1_FETCH_PCM_FRAMES;
	if (rate == 48000)
		data |= V1_CLOCK_RATE_BASED_ON_48000;
	else
		data &= ~V1_CLOCK_RATE_BASED_ON_48000;

	reg = cpu_to_be32(data);
	return snd_motu_transaction_write(motu, V1_CLOCK_STATUS_OFFSET, &reg,
					  sizeof(reg));
}

static int v1_get_clock_source(struct snd_motu *motu,
			       enum snd_motu_clock_source *src)
{
	__be32 reg;
	u32 data;
	int err;

	err = snd_motu_transaction_read(motu, V1_CLOCK_STATUS_OFFSET, &reg,
					sizeof(reg));
	if (err < 0)
		return err;

	data = be32_to_cpu(reg);
	if (data & V1_CLOCK_SRC_ADAT_ON_OPT_OR_DSUB) {
		if (data & V1_CLOCK_SRC_IS_NOT_FROM_ADAT_DSUB)
			*src = SND_MOTU_CLOCK_SOURCE_ADAT_ON_OPT;
		else
			*src = SND_MOTU_CLOCK_SOURCE_ADAT_ON_DSUB;
	} else if (data & V1_CLOCK_SRC_SPDIF_ON_OPT_OR_COAX) {
		if (data & V1_OPT_IN_IFACE_IS_SPDIF)
			*src = SND_MOTU_CLOCK_SOURCE_SPDIF_ON_OPT;
		else
			*src = SND_MOTU_CLOCK_SOURCE_SPDIF_ON_COAX;
	} else {
		*src = SND_MOTU_CLOCK_SOURCE_INTERNAL;
	}

	return 0;
}

static int v1_switch_fetching_mode(struct snd_motu *motu, bool enable)
{
	__be32 reg;
	u32 data;
	int err;

	err = snd_motu_transaction_read(motu, V1_CLOCK_STATUS_OFFSET, &reg,
					sizeof(reg));
	if (err < 0)
		return err;
	data = be32_to_cpu(reg);

	if (enable)
		data |= V1_FETCH_PCM_FRAMES;
	else
		data &= ~V1_FETCH_PCM_FRAMES;

	reg = cpu_to_be32(data);
	return snd_motu_transaction_write(motu, V1_CLOCK_STATUS_OFFSET, &reg,
					  sizeof(reg));
}

static void calculate_fixed_part(struct snd_motu_packet_format *formats,
				 enum amdtp_stream_direction dir,
				 enum snd_motu_spec_flags flags,
				 unsigned char analog_ports)
{
	unsigned char pcm_chunks[3] = {0, 0, 0};
	int i;

	if (dir == AMDTP_IN_STREAM)
		formats->msg_chunks = 2;
	else
		formats->msg_chunks = 0;

	pcm_chunks[0] = analog_ports;
	if (flags & SND_MOTU_SPEC_SUPPORT_CLOCK_X2)
		pcm_chunks[1] = analog_ports;
	if (flags & SND_MOTU_SPEC_SUPPORT_CLOCK_X4)
		pcm_chunks[2] = analog_ports;

	pcm_chunks[0] += 2;
	if (flags & SND_MOTU_SPEC_SUPPORT_CLOCK_X2)
		pcm_chunks[1] += 2;

	for (i = 0; i < 3; ++i)
		formats->fixed_part_pcm_chunks[i] = pcm_chunks[i];
}

static void calculate_differed_part(struct snd_motu_packet_format *formats,
				    enum snd_motu_spec_flags flags,
				    u32 opt_iface_mode_data,
				    u32 opt_iface_mode_mask)
{
	unsigned char pcm_chunks[3] = {0, 0, 0};
	int i;

	/* Packet includes PCM frames from ADAT on optical interface. */
	if (!(opt_iface_mode_data & opt_iface_mode_mask)) {
		pcm_chunks[0] += 8;
		if (flags & SND_MOTU_SPEC_SUPPORT_CLOCK_X2)
			pcm_chunks[1] += 4;
	}

	for (i = 0; i < 3; ++i)
		formats->differed_part_pcm_chunks[i] = pcm_chunks[i];
}

static int v1_cache_packet_formats(struct snd_motu *motu)
{
	__be32 reg;
	u32 opt_iface_mode_data;
	int err;

	err = snd_motu_transaction_read(motu, V1_CLOCK_STATUS_OFFSET, &reg,
					sizeof(reg));
	if (err < 0)
		return err;
	opt_iface_mode_data = be32_to_cpu(reg);

	calculate_fixed_part(&motu->tx_packet_formats, AMDTP_IN_STREAM,
			     motu->spec->flags, motu->spec->analog_in_ports);
	calculate_differed_part(&motu->tx_packet_formats, motu->spec->flags,
				opt_iface_mode_data, V1_OPT_IN_IFACE_IS_SPDIF);

	calculate_fixed_part(&motu->rx_packet_formats, AMDTP_OUT_STREAM,
			     motu->spec->flags, motu->spec->analog_out_ports);
	calculate_differed_part(&motu->rx_packet_formats, motu->spec->flags,
				opt_iface_mode_data, V1_OPT_OUT_IFACE_IS_SPDIF);

	motu->tx_packet_formats.pcm_byte_offset = 4;
	motu->rx_packet_formats.pcm_byte_offset = 4;

	return 0;
}

const struct snd_motu_protocol snd_motu_protocol_v1 = {
	.get_clock_rate		= v1_get_clock_rate,
	.set_clock_rate		= v1_set_clock_rate,
	.get_clock_source	= v1_get_clock_source,
	.switch_fetching_mode	= v1_switch_fetching_mode,
	.cache_packet_formats	= v1_cache_packet_formats,
};
