/*
 * Copyright 2018 MSC Technologies GmbH
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <sound/soc.h>

#include "../codecs/da7213.h"
#include "imx-audmux.h"

#define DAI_NAME_SIZE	32

struct imx_da7213_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	char codec_dai_name[DAI_NAME_SIZE];
	char platform_name[DAI_NAME_SIZE];
	struct clk *codec_clk;
	unsigned int clk_frequency;
};

static int imx_da7213_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct snd_soc_card *card = rtd->card;
	struct imx_da7213_data *data = snd_soc_card_get_drvdata(card);
	struct device *dev = rtd->card->dev;
	unsigned int fout;
	int ret = 0;

	switch (params_rate(params)) {
	case 8000:
		fout = DA7213_PLL_FREQ_OUT_98304000;
		break;
	case 11025:
		fout = DA7213_PLL_FREQ_OUT_90316800;
		break;
	case 12000:
		fout = DA7213_PLL_FREQ_OUT_98304000;
		break;
	case 16000:
		fout = DA7213_PLL_FREQ_OUT_98304000;
		break;
	case 22050:
		fout = DA7213_PLL_FREQ_OUT_90316800;
		break;
	case 24000:
		fout = DA7213_PLL_FREQ_OUT_90316800;
		break;
	case 32000:
		fout = DA7213_PLL_FREQ_OUT_98304000;
		break;
	case 44100:
		fout = DA7213_PLL_FREQ_OUT_90316800;
		break;
	case 48000:
		fout = DA7213_PLL_FREQ_OUT_98304000;
		break;
	case 88200:
		fout = DA7213_PLL_FREQ_OUT_90316800;
		break;
	case 96000:
		fout = DA7213_PLL_FREQ_OUT_98304000;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_dai_set_fmt(codec_dai, data->dai.dai_fmt);
	if (ret) {
		dev_err(dev, "failed to set codec dai fmt: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_fmt(cpu_dai, data->dai.dai_fmt);
	if (ret) {
		dev_err(dev, "failed to set cpu dai fmt: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_pll(codec_dai, 0, DA7213_SYSCLK_PLL,
			data->clk_frequency, fout);
	if (ret) {
		dev_err(dev, "failed to start PLL: %d\n", ret);
		return ret;
	}

	return ret;
}

static int imx_da7213_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);

	snd_soc_component_update_bits(codec_dai->component, DA7213_PLL_CTRL, DA7213_PLL_EN, 0);

	return 0;
}

static struct snd_soc_ops imx_da7213_ops = {
	.hw_params = imx_da7213_hw_params,
	.hw_free = imx_da7213_hw_free,
};

static int imx_da7213_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct imx_da7213_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct device *dev = rtd->card->dev;
	int ret;

	data->dai.ops = &imx_da7213_ops,

	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, 0, SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(dev, "failed to set cpu sysclk: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, DA7213_CLKSRC_MCLK,
			data->clk_frequency, SND_SOC_CLOCK_OUT);
	if (ret) {
		dev_err(dev, "could not set codec driver clock params\n");
		return ret;
	}

	return 0;
}

static const struct snd_soc_dapm_widget imx_da7213_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Mic1", NULL),
	SND_SOC_DAPM_MIC("Mic2", NULL),
	SND_SOC_DAPM_MIC("Mic3", NULL),
	SND_SOC_DAPM_MIC("Mic4", NULL),
	SND_SOC_DAPM_LINE("AuxL", NULL),
	SND_SOC_DAPM_LINE("AuxR", NULL),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
};

static int imx_da7213_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *ssi_np, *codec_np;
	struct platform_device *ssi_pdev;
	struct i2c_client *codec_dev;
	struct imx_da7213_data *data = NULL;
	struct snd_soc_dai_link_component *comp;
	unsigned int dai_fmt;
	int ret;

	if (!of_property_read_bool(np, "fsl,no-audmux") ) {
		int int_port, ext_port;
		ret = of_property_read_u32(np, "mux-int-port", &int_port);
		if (ret) {
			dev_err(&pdev->dev, "mux-int-port missing or invalid\n");
			return ret;
		}

		ret = of_property_read_u32(np, "mux-ext-port", &ext_port);
		if (ret) {
			dev_err(&pdev->dev, "mux-ext-port missing or invalid\n");
			return ret;
		}

		/*
		* The port numbering in the hardware manual starts at 1, while
		* the audmux API expects it starts at 0.
		*/
		int_port--;
		ext_port--;
		ret = imx_audmux_v2_configure_port(int_port,
				IMX_AUDMUX_V2_PTCR_SYN |
				IMX_AUDMUX_V2_PTCR_TFSEL(ext_port) |
				IMX_AUDMUX_V2_PTCR_TCSEL(ext_port) |
				IMX_AUDMUX_V2_PTCR_TFSDIR |
				IMX_AUDMUX_V2_PTCR_TCLKDIR,
				IMX_AUDMUX_V2_PDCR_RXDSEL(ext_port));
		if (ret) {
			dev_err(&pdev->dev, "audmux internal port setup failed\n");
			return ret;
		}

		ret = imx_audmux_v2_configure_port(ext_port,
				IMX_AUDMUX_V2_PTCR_SYN,
				IMX_AUDMUX_V2_PDCR_RXDSEL(int_port));
		if (ret) {
			dev_err(&pdev->dev, "audmux external port setup failed\n");
			return ret;
		}
	}

	if (unlikely(of_property_read_bool(np, "cbs_cfs")))
		dai_fmt = SND_SOC_DAIFMT_CBS_CFS;
	else
		dai_fmt = SND_SOC_DAIFMT_CBM_CFM;

	ssi_np = of_parse_phandle(pdev->dev.of_node, "ssi-controller", 0);
	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!ssi_np || !codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	ssi_pdev = of_find_device_by_node(ssi_np);
	if (!ssi_pdev) {
		dev_err(&pdev->dev, "failed to find SSI platform device\n");
		ret = -EPROBE_DEFER;
		goto fail;
	}
	put_device(&ssi_pdev->dev);
	codec_dev = of_find_i2c_device_by_node(codec_np);
	if (!codec_dev) {
		dev_err(&pdev->dev, "failed to find codec platform device now. retrying....\n");
		return -EPROBE_DEFER;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}

	data->codec_clk = clk_get(&codec_dev->dev, NULL);
	if (IS_ERR(data->codec_clk)) {
		ret = PTR_ERR(data->codec_clk);
		goto fail;
	}

	comp = devm_kzalloc(&pdev->dev, 3 * sizeof(*comp), GFP_KERNEL);
	if (!comp) {
		ret = -ENOMEM;
		goto fail_clk_get;
	}

	data->clk_frequency = clk_get_rate(data->codec_clk);
	clk_prepare_enable(data->codec_clk);

	data->dai.cpus		= &comp[0];
	data->dai.codecs	= &comp[1];
	data->dai.platforms	= &comp[2];

	data->dai.num_cpus	= 1;
	data->dai.num_codecs	= 1;
	data->dai.num_platforms	= 1;

	data->dai.name = "HiFi";
	data->dai.stream_name = "HiFi";
	data->dai.codecs->dai_name = "da7213-hifi";
	data->dai.codecs->of_node = codec_np;
	data->dai.cpus->of_node = ssi_np;
	data->dai.platforms->of_node = ssi_np;
	data->dai.init = &imx_da7213_dai_init;
	data->dai.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			dai_fmt;

	data->card.dev = &pdev->dev;
	ret = snd_soc_of_parse_card_name(&data->card, "card-name");
	if (ret)
		goto fail_clk;

	ret = snd_soc_of_parse_audio_routing(&data->card, "audio-routing");
	if (ret)
		goto fail_clk;

	data->card.num_links = 1;
	data->card.owner = THIS_MODULE;
	data->card.dai_link = &data->dai;
	data->card.dapm_widgets = imx_da7213_dapm_widgets;
	data->card.num_dapm_widgets = ARRAY_SIZE(imx_da7213_dapm_widgets);

	platform_set_drvdata(pdev, &data->card);
	snd_soc_card_set_drvdata(&data->card, data);

	ret = devm_snd_soc_register_card(&pdev->dev, &data->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto fail_clk;
	}

	of_node_put(ssi_np);
	of_node_put(codec_np);

	return 0;

fail_clk:
	clk_disable_unprepare(data->codec_clk);
fail_clk_get:
	clk_put(data->codec_clk);
fail:
	of_node_put(ssi_np);
	of_node_put(codec_np);

	return ret;
}

static int imx_da7213_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct imx_da7213_data *data = snd_soc_card_get_drvdata(card);

	clk_disable_unprepare(data->codec_clk);
	clk_put(data->codec_clk);

	return 0;
}

static const struct of_device_id imx_da7213_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-da7213", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_da7213_dt_ids);

static struct platform_driver imx_da7213_driver = {
	.driver = {
		.name = "imx-da7213",
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_da7213_dt_ids,
	},
	.probe = imx_da7213_probe,
	.remove = imx_da7213_remove,
};
module_platform_driver(imx_da7213_driver);

MODULE_DESCRIPTION("Freescale i.MX DA7213 ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-da7213");
