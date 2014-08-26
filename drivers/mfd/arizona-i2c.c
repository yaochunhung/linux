/*
 * Arizona-i2c.c  --  Arizona I2C bus interface
 *
 * Copyright 2012 Wolfson Microelectronics plc
 *
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/of.h>

#include <linux/mfd/arizona/core.h>

#include "arizona.h"

/************************************************************/
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/arizona/pdata.h>
#include <linux/mfd/arizona/registers.h>

/***********WM8280 1.8V REGUATOR*************/
static struct regulator_consumer_supply vflorida1_consumer[] = {
	REGULATOR_SUPPLY("AVDD", "0-001a"),
	REGULATOR_SUPPLY("DBVDD1", "0-001a"),
	REGULATOR_SUPPLY("LDOVDD", "0-001a"),
	REGULATOR_SUPPLY("CPVDD",   "0-001a"),
	REGULATOR_SUPPLY("DBVDD2", "florida-codec"),
	REGULATOR_SUPPLY("DBVDD3", "florida-codec"),
	REGULATOR_SUPPLY("CPVDD",   "florida-codec"),
};

static struct regulator_init_data vflorida1_data = {
		.constraints = {
			.always_on = 1,
		},
		.num_consumer_supplies	=	ARRAY_SIZE(vflorida1_consumer),
		.consumer_supplies	=	vflorida1_consumer,
};

static struct fixed_voltage_config vflorida1_config = {
	.supply_name	= "DC_1V8",
	.microvolts	= 1800000,
	.gpio		= -EINVAL,
	.init_data	= &vflorida1_data,
};

static struct platform_device vflorida1_device = {
	.name = "reg-fixed-voltage",
	.id = PLATFORM_DEVID_AUTO,
	.dev = {
		.platform_data = &vflorida1_config,
	},
};

/***********WM8280 5V REGUATOR*************/
static struct regulator_consumer_supply vflorida2_consumer[] = {
	REGULATOR_SUPPLY("SPKVDDL", "florida-codec"),
	REGULATOR_SUPPLY("SPKVDDR", "florida-codec"),
};

static struct regulator_init_data vflorida2_data = {
		.constraints = {
			.always_on = 1,
		},
		.num_consumer_supplies	=	ARRAY_SIZE(vflorida2_consumer),
		.consumer_supplies	=	vflorida2_consumer,
};

static struct fixed_voltage_config vflorida2_config = {
	.supply_name	= "DC_5V",
	.microvolts	= 3700000,
	.gpio		= -EINVAL,
	.init_data  = &vflorida2_data,
};

static struct platform_device vflorida2_device = {
	.name = "reg-fixed-voltage",
	.id = PLATFORM_DEVID_AUTO,
	.dev = {
		.platform_data = &vflorida2_config,
	},
};

/***********WM8280 Codec Driver platform data*************/
static const struct arizona_micd_range micd_ctp_ranges[] = {
	{ .max =  11, .key = BTN_0 },
	{ .max =  28, .key = BTN_1 },
	{ .max =  54, .key = BTN_2 },
	{ .max = 100, .key = BTN_3 },
	{ .max = 186, .key = BTN_4 },
	{ .max = 430, .key = BTN_5 },
};

static const struct arizona_micd_config micd_modes[] = {
	/*{Acc Det on Micdet2, Use Micbias1 for detection,
	 * Set GPIO to 0 to selecte this polarity}*/
	{ ARIZONA_ACCDET_SRC, 1, 0 },
	/*{Acc Det on Micdet1, Use Micbias2 for detection,
	 * Set GPIO to 1 to selecte this polarity}*/
	{ 0, 2, 1 },
};

static struct arizona_pdata florida_pdata  = {
	.reset = 0, /*No Reset GPIO from AP, use SW reset*/
	.ldoena = 0, /*TODO: Add actual GPIO for LDOEN, use SW Control for now*/
	.irq_flags = IRQF_TRIGGER_LOW | IRQF_ONESHOT,
	.clk32k_src = ARIZONA_32KZ_MCLK2, /*Onboard OSC provides 32K on MCLK2*/
	/*IN1 uses both MICBIAS1 and MICBIAS2 based on jack polarity,
	the below values in dmic_ref only has meaning for DMIC's and not AMIC's*/
	.dmic_ref = {ARIZONA_DMIC_MICBIAS1, ARIZONA_DMIC_MICBIAS3, 0, 0},
	.inmode = {ARIZONA_INMODE_SE, ARIZONA_INMODE_DMIC, 0, 0},
	.gpio_base = 0, /* Base allocated by gpio core*/
	.micd_pol_gpio = 2, /* GPIO3 (offset 2 from gpio_base) of the codec*/
	.micd_configs = micd_modes,
	.num_micd_configs = ARRAY_SIZE(micd_modes),
	.gpsw = 0x3, /*Use the Micbias Clamping Switch to prevent pops*/
	.micd_force_micbias = true,
};

/************************************************************/
static struct i2c_board_info arizona_i2c_device = {
	I2C_BOARD_INFO("wm8280", 0x1A),
	.platform_data = &florida_pdata,
};

static int arizona_i2c_probe(struct i2c_client *i2c,
					  const struct i2c_device_id *id)
{
	struct arizona *arizona;
	const struct regmap_config *regmap_config;
	int ret, type;

	if (i2c->dev.of_node)
		type = arizona_of_get_type(&i2c->dev);
	else
		type = id->driver_data;

	switch (type) {
#ifdef CONFIG_MFD_WM5102
	case WM5102:
		regmap_config = &wm5102_i2c_regmap;
		break;
#endif
#ifdef CONFIG_MFD_FLORIDA
	case WM8280:
	case WM5110:
		regmap_config = &florida_i2c_regmap;
		break;
#endif
#ifdef CONFIG_MFD_WM8997
	case WM8997:
		regmap_config = &wm8997_i2c_regmap;
		break;
#endif
	default:
		dev_err(&i2c->dev, "Unknown device type %ld\n",
			id->driver_data);
		return -EINVAL;
	}

	arizona = devm_kzalloc(&i2c->dev, sizeof(*arizona), GFP_KERNEL);
	if (arizona == NULL)
		return -ENOMEM;

	arizona->regmap = devm_regmap_init_i2c(i2c, regmap_config);
	if (IS_ERR(arizona->regmap)) {
		ret = PTR_ERR(arizona->regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	/***********WM8280 Register Regulator*************/
	platform_device_register(&vflorida1_device);
	platform_device_register(&vflorida2_device);

	arizona->type = id->driver_data;
	arizona->dev = &i2c->dev;
	arizona->irq = i2c->irq;

	pr_debug("%s irq %d\n", __func__, i2c->irq);

	return arizona_dev_init(arizona);
}

static int arizona_i2c_remove(struct i2c_client *i2c)
{
	struct arizona *arizona = dev_get_drvdata(&i2c->dev);
	arizona_dev_exit(arizona);
	return 0;
}

static const struct i2c_device_id arizona_i2c_id[] = {
	{ "wm5102", WM5102 },
	{ "wm8280", WM8280 },
	{ "wm8281", WM8280 },
	{ "wm5110", WM5110 },
	{ "wm8997", WM8997 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, arizona_i2c_id);

static struct i2c_driver arizona_i2c_driver = {
	.driver = {
		.name	= "arizona",
		.owner	= THIS_MODULE,
		.pm	= &arizona_pm_ops,
		.of_match_table	= of_match_ptr(arizona_of_match),
	},
	.probe		= arizona_i2c_probe,
	.remove		= arizona_i2c_remove,
	.id_table	= arizona_i2c_id,
};

static int __init arizona_modinit(void)
{
	int ret = 0;
	struct i2c_adapter *adapter;
	struct i2c_client *client;

	pr_debug("%s Entry\n", __func__);

	adapter = i2c_get_adapter(0);
	if (adapter) {
		client = i2c_new_device(adapter, &arizona_i2c_device);
		if (!client) {
			pr_err("can't create i2c device %s\n",
				arizona_i2c_device.type);
			i2c_put_adapter(adapter);
			return -ENODEV;
		}
	} else {
		pr_err("adapter is NULL\n");
		return -ENODEV;
	}

	ret = i2c_add_driver(&arizona_i2c_driver);

	pr_debug("%s Exit\n", __func__);

	return ret;
}

module_init(arizona_modinit);

static void __exit arizona_modexit(void)
{
	i2c_del_driver(&arizona_i2c_driver);
}

module_exit(arizona_modexit);

MODULE_DESCRIPTION("Arizona I2C bus interface");
MODULE_AUTHOR("Mark Brown <broonie@opensource.wolfsonmicro.com>");
MODULE_LICENSE("GPL");
