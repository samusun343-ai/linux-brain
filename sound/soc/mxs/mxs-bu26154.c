// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * mxs-bu26154.c -- ASoC machine driver for i.MX28 (MXS) boards using
 * the ROHM BU26154 codec, e.g. the SHARP Brain series.
 *
 * The board wiring follows the i.MX28 EVK pattern used by the
 * mxs-sgtl5000 machine driver: SAIF0 is the playback DAI, SAIF1 the
 * capture DAI (clock-slaved to SAIF0), and the SAIF MCLK output feeds
 * the codec's MCLKI pin at 512 x fs.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#include "mxs-saif.h"

static int mxs_bu26154_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	unsigned int rate = params_rate(params);
	u32 mclk = 512 * rate;
	int ret;

	/* Set the codec SYSCLK (provided by SAIF MCLK) */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, mclk, 0);
	if (ret) {
		dev_err(codec_dai->dev, "Failed to set codec sysclk\n");
		return ret;
	}

	/* The SAIF MCLK is the same clock */
	ret = snd_soc_dai_set_sysclk(cpu_dai, MXS_SAIF_MCLK, mclk, 0);
	if (ret) {
		dev_err(cpu_dai->dev, "Failed to set SAIF sysclk\n");
		return ret;
	}

	return 0;
}

static const struct snd_soc_ops mxs_bu26154_hifi_ops = {
	.hw_params = mxs_bu26154_hw_params,
};

#define MXS_BU26154_DAI_FMT (SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | \
			     SND_SOC_DAIFMT_CBS_CFS)

SND_SOC_DAILINK_DEFS(hifi_tx,
	DAILINK_COMP_ARRAY(COMP_EMPTY()),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "bu26154-hifi")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(hifi_rx,
	DAILINK_COMP_ARRAY(COMP_EMPTY()),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "bu26154-hifi")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

static struct snd_soc_dai_link mxs_bu26154_dai[] = {
	{
		.name		= "HiFi Tx",
		.stream_name	= "HiFi Playback",
		.dai_fmt	= MXS_BU26154_DAI_FMT,
		.ops		= &mxs_bu26154_hifi_ops,
		.playback_only	= true,
		SND_SOC_DAILINK_REG(hifi_tx),
	}, {
		.name		= "HiFi Rx",
		.stream_name	= "HiFi Capture",
		.dai_fmt	= MXS_BU26154_DAI_FMT,
		.ops		= &mxs_bu26154_hifi_ops,
		.capture_only	= true,
		SND_SOC_DAILINK_REG(hifi_rx),
	},
};

static const struct snd_soc_dapm_widget mxs_bu26154_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
};

static struct snd_soc_card mxs_bu26154_card = {
	.name		= "mxs_bu26154",
	.owner		= THIS_MODULE,
	.dai_link	= mxs_bu26154_dai,
	.num_links	= ARRAY_SIZE(mxs_bu26154_dai),
	.dapm_widgets	= mxs_bu26154_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(mxs_bu26154_dapm_widgets),
};

static int mxs_bu26154_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &mxs_bu26154_card;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *saif_np[2], *codec_np;
	int i, ret;

	saif_np[0] = of_parse_phandle(np, "saif-controllers", 0);
	saif_np[1] = of_parse_phandle(np, "saif-controllers", 1);
	codec_np = of_parse_phandle(np, "audio-codec", 0);
	if (!saif_np[0] || !saif_np[1] || !codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto err;
	}

	for (i = 0; i < 2; i++) {
		mxs_bu26154_dai[i].cpus->of_node = saif_np[i];
		mxs_bu26154_dai[i].codecs->of_node = codec_np;
		mxs_bu26154_dai[i].platforms->of_node = saif_np[i];
	}

	/*
	 * Claim the SAIF0 MCLK output (512 x fs at 44.1 kHz initially;
	 * hw_params reprograms it per stream). This also marks the
	 * SAIF MCLK as in use so mxs_saif_hw_params() programs the
	 * BITCLK_MULT_RATE divider for the codec's MCLKI.
	 */
	ret = mxs_saif_get_mclk(0, 512 * 44100, 44100);
	if (ret) {
		dev_err(&pdev->dev, "failed to get mclk: %d\n", ret);
		if (ret == -EINVAL)
			ret = -EPROBE_DEFER;
		goto err;
	}

	card->dev = &pdev->dev;
	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register card: %d\n", ret);
		mxs_saif_put_mclk(0);
	}
err:
	of_node_put(saif_np[0]);
	of_node_put(saif_np[1]);
	of_node_put(codec_np);
	return ret;
}

static int mxs_bu26154_remove(struct platform_device *pdev)
{
	mxs_saif_put_mclk(0);
	return 0;
}

static const struct of_device_id mxs_bu26154_dt_ids[] = {
	{ .compatible = "sharp,mxs-audio-bu26154" },
	{ }
};
MODULE_DEVICE_TABLE(of, mxs_bu26154_dt_ids);

static struct platform_driver mxs_bu26154_driver = {
	.probe	= mxs_bu26154_probe,
	.remove	= mxs_bu26154_remove,
	.driver	= {
		.name	= "mxs-audio-bu26154",
		.of_match_table = mxs_bu26154_dt_ids,
	},
};
module_platform_driver(mxs_bu26154_driver);

MODULE_DESCRIPTION("MXS ASoC machine driver for BU26154 codecs");
MODULE_AUTHOR("samusun343-ai");
MODULE_LICENSE("GPL");
