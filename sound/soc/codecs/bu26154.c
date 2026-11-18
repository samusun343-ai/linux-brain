// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * bu26154.c -- ASoC codec driver for the ROHM BU26154MUV monaural
 * audio codec (as used, among others, in the SHARP Brain series of
 * electronic dictionaries).
 *
 * Written from the public ROHM datasheet (TSZ22111, Rev.002) only.
 *
 * Bus protocol (datasheet, "2-wire serial interface"): the first byte
 * of a write transaction is the register INDEX; the low bit of the
 * INDEX selects the direction of the register access (even = read,
 * odd = write), so the canonical register address is the even INDEX.
 * Data bytes auto-increment the INDEX by two (next register pair).
 * Three register maps are selected through the MAPCON register
 * (INDEX 0x1c, present in every map); this driver uses MAP0 only.
 *
 * Clocking: the codec is SAI clock slave (MST = 0).  Its internal
 * audio clock is taken from the MCLKI pin without the PLL, fed by the
 * SoC at 256/512/1024 x fs (selected through CLKSEL in hw_params).
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#define BU26154_DRV_NAME	"bu26154"

/* MAP0 register (canonical, even) INDEX addresses */
#define BU26154_SR		0x00	/* sampling rate */
#define BU26154_CLK_EN		0x0c	/* clock enable */
#define BU26154_CLK_CTRL	0x0e	/* clock input/output control */
#define BU26154_SOFTRST		0x10	/* software reset */
#define BU26154_RECPLAY		0x12	/* record/playback run control */
#define BU26154_ANALOG_REF	0x20	/* analog reference power mgmt */
#define BU26154_ANALOG_IN	0x22	/* analog input power mgmt */
#define BU26154_DAC_PWR		0x24	/* DAC power management */
#define BU26154_SPK_PWR		0x26	/* speaker amplifier power mgmt */
#define BU26154_AVVOL		0x3a	/* analog volume */
#define BU26154_PDATT		0x3e	/* playback digital attenuator */
#define BU26154_AVMUTE		0x48	/* amplifier volume function */
#define BU26154_SAI_TX		0x60	/* SAI transmitter control */
#define BU26154_SAI_RX		0x62	/* SAI receiver control */
#define BU26154_SAI_MODE	0x64	/* SAI mode select */
#define BU26154_DVFUNC		0x68	/* digital volume function */
#define BU26154_RDVOL		0x6c	/* record digital attenuator */

/* CLK_EN bits */
#define BU26154_CLK_EN_MCLKOE	BIT(4)
#define BU26154_CLK_EN_PLLOE	BIT(3)
#define BU26154_CLK_EN_PLLEN	BIT(2)
#define BU26154_CLK_EN_MCLKEN	BIT(0)

/* CLK_CTRL bits: PLLISEL in [4:3], CLKSEL in [2:0] */
#define BU26154_CLKSEL_MCLK_256FS	0x4
#define BU26154_CLKSEL_MCLK_512FS	0x6
#define BU26154_CLKSEL_MCLK_1024FS	0x7

/* ANALOG_REF bits */
#define BU26154_AREF_HPREN	BIT(7)
#define BU26154_AREF_HPLEN	BIT(6)
#define BU26154_AREF_HPVDDEN	BIT(3)
#define BU26154_AREF_MICBEN	BIT(2)
#define BU26154_AREF_VMIDCON_MASK	0x3
#define BU26154_AREF_VMIDCON_NORMAL	0x2

/* ANALOG_IN bits */
#define BU26154_AIN_PGAEN	BIT(3)
#define BU26154_AIN_ADCREN	BIT(2)
#define BU26154_AIN_ADCEN	BIT(1)

/* DAC_PWR bits */
#define BU26154_DAC_DACREN	BIT(2)
#define BU26154_DAC_DACLEN	BIT(1)

/* SPK_PWR bits */
#define BU26154_SPK_AVREN	BIT(4)
#define BU26154_SPK_SPEN	BIT(1)

/* SAI_TX / SAI_RX fields */
#define BU26154_SAI_PCMF_MASK	(0x3 << 6)
#define BU26154_SAI_PCMF_16BIT	(0x2 << 6)
#define BU26154_SAI_PCMF_24BIT	(0x3 << 6)
#define BU26154_SAI_FMT_FRAME	BIT(5)	/* 0 = LRCLK transfer mode */
#define BU26154_SAI_LSB_FIRST	BIT(4)
#define BU26154_SAI_64FS	BIT(3)
#define BU26154_SAI_RIGHT_J	BIT(2)
#define BU26154_SAI_NO_DELAY	BIT(1)
#define BU26154_SAI_WSL		BIT(0)

/* RECPLAY values */
#define BU26154_RECPLAY_STOP	0x0
#define BU26154_RECPLAY_REC	0x1
#define BU26154_RECPLAY_PLAY	0x2
#define BU26154_RECPLAY_BOTH	0x3

/* DVFUNC bits */
#define BU26154_DVFUNC_DVMUTE	BIT(4)

struct bu26154_priv {
	struct i2c_client *i2c;
	unsigned int sysclk;	/* MCLKI frequency in Hz, 0 = unset */
	unsigned int fmt;	/* SAI format bits for TX/RX regs */
	int playback_active;
	int capture_active;
};

/*
 * The register file is reached through the INDEX protocol described
 * above; provide component-level read/write callbacks instead of a
 * regmap so the odd/even INDEX convention is handled in one place.
 */
static int bu26154_reg_write(void *context, unsigned int reg,
			     unsigned int val)
{
	struct i2c_client *i2c = context;
	u8 buf[2] = { reg | 0x01, val };
	int ret;

	ret = i2c_master_send(i2c, buf, sizeof(buf));
	return ret == sizeof(buf) ? 0 : -EIO;
}

static int bu26154_reg_read(void *context, unsigned int reg,
			    unsigned int *val)
{
	struct i2c_client *i2c = context;
	u8 idx = reg;	/* even INDEX = read pointer */
	u8 data;
	int ret;

	ret = i2c_master_send(i2c, &idx, 1);
	if (ret != 1)
		return -EIO;
	ret = i2c_master_recv(i2c, &data, 1);
	if (ret != 1)
		return -EIO;
	*val = data;
	return 0;
}

static unsigned int bu26154_read(struct snd_soc_component *component,
				 unsigned int reg)
{
	struct bu26154_priv *priv = snd_soc_component_get_drvdata(component);
	unsigned int val = 0;
	int ret;

	ret = bu26154_reg_read(priv->i2c, reg, &val);
	if (ret)
		dev_err(component->dev, "read of reg %#x failed: %d\n",
			reg, ret);
	return val;
}

static int bu26154_write(struct snd_soc_component *component,
			 unsigned int reg, unsigned int val)
{
	struct bu26154_priv *priv = snd_soc_component_get_drvdata(component);

	return bu26154_reg_write(priv->i2c, reg, val);
}

static int bu26154_update_recplay(struct snd_soc_component *component)
{
	struct bu26154_priv *priv = snd_soc_component_get_drvdata(component);
	unsigned int v = BU26154_RECPLAY_STOP;

	if (priv->playback_active && priv->capture_active)
		v = BU26154_RECPLAY_BOTH;
	else if (priv->playback_active)
		v = BU26154_RECPLAY_PLAY;
	else if (priv->capture_active)
		v = BU26154_RECPLAY_REC;

	return snd_soc_component_write(component, BU26154_RECPLAY, v);
}

static const unsigned int bu26154_rates[] = {
	8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000,
};

/* snd_pcm_hw_constraint_list() keeps a pointer to this struct, so it
 * must have static storage duration.
 */
static const struct snd_pcm_hw_constraint_list bu26154_rate_constraint = {
	.list	= bu26154_rates,
	.count	= ARRAY_SIZE(bu26154_rates),
	.mask	= 0,
};

static int bu26154_startup(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
{
	struct bu26154_priv *priv = snd_soc_component_get_drvdata(dai->component);
	int ret;

	ret = snd_pcm_hw_constraint_list(substream->runtime, 0,
					 SNDRV_PCM_HW_PARAM_RATE,
					 &bu26154_rate_constraint);
	if (ret)
		return ret;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		priv->playback_active++;
	else
		priv->capture_active++;
	return bu26154_update_recplay(dai->component);
}

static void bu26154_shutdown(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct bu26154_priv *priv = snd_soc_component_get_drvdata(dai->component);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		priv->playback_active--;
	else
		priv->capture_active--;
	bu26154_update_recplay(dai->component);
}

static int bu26154_set_sysclk(struct snd_soc_dai *dai, int clk_id,
			      unsigned int freq, int dir)
{
	struct bu26154_priv *priv = snd_soc_component_get_drvdata(dai->component);

	priv->sysclk = freq;
	return 0;
}

static int bu26154_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct bu26154_priv *priv = snd_soc_component_get_drvdata(dai->component);
	unsigned int sai = BU26154_SAI_64FS;	/* SAIF runs at 64fs */

	if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBS_CFS)
		return -EINVAL;		/* codec is always clock slave */

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		/* 1-BCLK data delay, left justified in slot, MSB first */
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		sai |= BU26154_SAI_NO_DELAY;
		break;
	default:
		return -EINVAL;
	}

	if ((fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF)
		return -EINVAL;

	priv->fmt = sai;
	return 0;
}

static int bu26154_sr_value(unsigned int rate)
{
	switch (rate) {
	case 8000:	return 0x0;
	case 11025:	return 0x1;
	case 12000:	return 0x2;
	case 16000:	return 0x3;
	case 22050:	return 0x4;
	case 24000:	return 0x5;
	case 32000:	return 0x6;
	case 44100:	return 0x7;
	case 48000:	return 0x8;
	default:	return -EINVAL;
	}
}

static int bu26154_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct bu26154_priv *priv = snd_soc_component_get_drvdata(component);
	unsigned int rate = params_rate(params);
	unsigned int sai = priv->fmt;
	int sr = bu26154_sr_value(rate);
	int ret;

	if (sr < 0)
		return sr;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		sai |= BU26154_SAI_PCMF_16BIT;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		sai |= BU26154_SAI_PCMF_24BIT;
		break;
	default:
		return -EINVAL;
	}

	/* Select the MCLKI divider matching the supplied system clock. */
	if (priv->sysclk) {
		unsigned int clksel;

		switch (priv->sysclk / rate) {
		case 256:	clksel = BU26154_CLKSEL_MCLK_256FS; break;
		case 512:	clksel = BU26154_CLKSEL_MCLK_512FS; break;
		case 1024:	clksel = BU26154_CLKSEL_MCLK_1024FS; break;
		default:
			dev_err(component->dev,
				"unsupported sysclk %u for rate %u\n",
				priv->sysclk, rate);
			return -EINVAL;
		}
		ret = snd_soc_component_write(component, BU26154_CLK_CTRL,
					      clksel);
		if (ret)
			return ret;
		ret = snd_soc_component_write(component, BU26154_CLK_EN,
					      BU26154_CLK_EN_PLLOE |
					      BU26154_CLK_EN_MCLKEN);
		if (ret)
			return ret;
	}

	ret = snd_soc_component_write(component, BU26154_SR, sr);
	if (ret)
		return ret;
	ret = snd_soc_component_write(component, BU26154_SAI_TX, sai);
	if (ret)
		return ret;
	return snd_soc_component_write(component, BU26154_SAI_RX, sai);
}

static int bu26154_set_bias_level(struct snd_soc_component *component,
				  enum snd_soc_bias_level level)
{
	int ret = 0;

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		if (snd_soc_component_get_bias_level(component) ==
		    SND_SOC_BIAS_OFF) {
			/* VMID normal power-up, mic bias, HP LDO on */
			ret = snd_soc_component_write(component,
				BU26154_ANALOG_REF,
				BU26154_AREF_HPREN | BU26154_AREF_HPLEN |
				BU26154_AREF_HPVDDEN | BU26154_AREF_MICBEN |
				BU26154_AREF_VMIDCON_NORMAL);
			if (ret)
				break;
			msleep(50);	/* VMID charge (1 uF cap) */
			snd_soc_component_update_bits(component,
				BU26154_DAC_PWR,
				BU26154_DAC_DACREN | BU26154_DAC_DACLEN,
				BU26154_DAC_DACREN | BU26154_DAC_DACLEN);
			snd_soc_component_update_bits(component,
				BU26154_SPK_PWR,
				BU26154_SPK_AVREN | BU26154_SPK_SPEN,
				BU26154_SPK_AVREN | BU26154_SPK_SPEN);
			snd_soc_component_update_bits(component,
				BU26154_ANALOG_IN,
				BU26154_AIN_PGAEN | BU26154_AIN_ADCREN |
				BU26154_AIN_ADCEN,
				BU26154_AIN_PGAEN | BU26154_AIN_ADCREN |
				BU26154_AIN_ADCEN);
		}
		break;
	case SND_SOC_BIAS_OFF:
		snd_soc_component_write(component, BU26154_RECPLAY,
					BU26154_RECPLAY_STOP);
		snd_soc_component_write(component, BU26154_ANALOG_REF, 0);
		break;
	}
	return ret;
}

/* PDATT/RDVOL: 0x6f and below = mute, 0x70 = -71.5 dB, +0.5 dB per
 * step up to 0xff = 0 dB (datasheet, Playback Digital Attenuator). */
static const DECLARE_TLV_DB_RANGE(bu26154_pdatt_tlv,
	0, 0x6f, TLV_DB_SCALE_ITEM(TLV_DB_GAIN_MUTE, 0, 1),
	0x70, 0xff, TLV_DB_SCALE_ITEM(-7150, 50, 0));

static const struct snd_kcontrol_new bu26154_controls[] = {
	SOC_SINGLE_TLV("Master Playback Volume", BU26154_PDATT,
		       0, 0xff, 0, bu26154_pdatt_tlv),
	SOC_SINGLE("Master Playback Switch", BU26154_DVFUNC,
		   4, 1, 1),	/* DVMUTE, inverted: switch on = unmuted */
	SOC_SINGLE("Speaker Playback Volume", BU26154_AVVOL,
		   0, 0x19, 0),
	SOC_SINGLE_TLV("Master Capture Volume", BU26154_RDVOL,
		       0, 0xff, 0, bu26154_pdatt_tlv),
};

static const struct snd_soc_dapm_widget bu26154_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC", "Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("ADC", "Capture", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("Speaker"),
	SND_SOC_DAPM_OUTPUT("Headphone Jack"),
	SND_SOC_DAPM_INPUT("Mic Jack"),
};

static const struct snd_soc_dapm_route bu26154_dapm_routes[] = {
	{ "Speaker", NULL, "DAC" },
	{ "Headphone Jack", NULL, "DAC" },
	{ "ADC", NULL, "Mic Jack" },
};

static const struct snd_soc_dai_ops bu26154_dai_ops = {
	.startup	= bu26154_startup,
	.shutdown	= bu26154_shutdown,
	.hw_params	= bu26154_hw_params,
	.set_fmt	= bu26154_set_fmt,
	.set_sysclk	= bu26154_set_sysclk,
};

static struct snd_soc_dai_driver bu26154_dai = {
	.name = "bu26154-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_KNOT,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_KNOT,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
	.ops = &bu26154_dai_ops,
};

static const struct snd_soc_component_driver bu26154_component_driver = {
	.name			= BU26154_DRV_NAME,
	.controls		= bu26154_controls,
	.num_controls		= ARRAY_SIZE(bu26154_controls),
	.dapm_widgets		= bu26154_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(bu26154_dapm_widgets),
	.dapm_routes		= bu26154_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(bu26154_dapm_routes),
	.read			= bu26154_read,
	.write			= bu26154_write,
	.set_bias_level		= bu26154_set_bias_level,
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static int bu26154_i2c_probe(struct i2c_client *i2c)
{
	struct bu26154_priv *priv;
	unsigned int val;
	int ret;

	priv = devm_kzalloc(&i2c->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->i2c = i2c;
	i2c_set_clientdata(i2c, priv);

	/* Software reset restores the datasheet reset values. */
	ret = bu26154_reg_write(i2c, BU26154_SOFTRST, 1);
	if (ret)
		return ret;
	udelay(100);
	ret = bu26154_reg_write(i2c, BU26154_SOFTRST, 0);
	if (ret)
		return ret;

	/* Presence check: PDATT resets to 0xff, AVVOL to 0x0a. */
	ret = bu26154_reg_read(i2c, BU26154_PDATT, &val);
	if (ret)
		return ret;
	if (val != 0xff) {
		dev_err(&i2c->dev, "unexpected PDATT value %#x\n", val);
		return -ENODEV;
	}
	ret = bu26154_reg_read(i2c, BU26154_AVVOL, &val);
	if (ret)
		return ret;
	if (val != 0x0a) {
		dev_err(&i2c->dev, "unexpected AVVOL value %#x\n", val);
		return -ENODEV;
	}

	dev_info(&i2c->dev, "BU26154 codec found\n");

	return devm_snd_soc_register_component(&i2c->dev,
					       &bu26154_component_driver,
					       &bu26154_dai, 1);
}

static const struct i2c_device_id bu26154_i2c_id[] = {
	{ "bu26154", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bu26154_i2c_id);

static const struct of_device_id bu26154_of_match[] = {
	{ .compatible = "rohm,bu26154" },
	{ }
};
MODULE_DEVICE_TABLE(of, bu26154_of_match);

static struct i2c_driver bu26154_i2c_driver = {
	.driver = {
		.name = BU26154_DRV_NAME,
		.of_match_table = bu26154_of_match,
	},
	.probe_new = bu26154_i2c_probe,
	.id_table = bu26154_i2c_id,
};
module_i2c_driver(bu26154_i2c_driver);

MODULE_DESCRIPTION("ASoC driver for the ROHM BU26154 codec");
MODULE_AUTHOR("samusun343-ai");
MODULE_LICENSE("GPL");
