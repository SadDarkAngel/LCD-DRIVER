// SPDX-License-Identifier: GPL-2.0+
/*
 * LCD-OLinuXino support for panel driver
 *
 * Copyright (C) 2018 Olimex Ltd.
 *   Author: Stefan Mavrodiev <stefan@olimex.com>
 */

#include <linux/backlight.h>
#include <linux/crc32.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drmP.h>

#include <video/videomode.h>
#include <video/display_timing.h>


#define LCD_OLINUXINO_HEADER_MAGIC	0x4F4CB727
#define LCD_OLINUXINO_DATA_LEN		256

struct lcd_olinuxino_eeprom {
	u32 header;				// 4
	u32 id;					// 8
	u32 revision;				// 12
	u32 serial;				// 16
	struct display_timing timing;		// 128
	struct {
		char name[32];			// 160
		u32 width_mm;			// 164
		u32 height_mm;			// 168
		u32 bpc;			// 172
		u32 bus_format;			// 176
		u32 bus_flag;			// 180
	} info;
	u8 unused[72];				// 252
	u32 checksum;				// 256
} __attribute__((__packed__));

struct lcd_olinuxino {
	struct device *dev;
	struct i2c_client *client;
	struct mutex mutex;

	struct drm_panel panel;
	bool prepared;
	bool enabled;

	struct backlight_device *backlight;
	struct regulator *supply;
	struct gpio_desc *enable_gpio;

	struct lcd_olinuxino_eeprom eeprom;
};

static inline struct lcd_olinuxino *to_lcd_olinuxino(struct drm_panel *panel)
{
	return container_of(panel, struct lcd_olinuxino, panel);
}

static int lcd_olinuxino_disable(struct drm_panel *panel)
{
	struct lcd_olinuxino *lcd = to_lcd_olinuxino(panel);

	if (!lcd->enabled)
		return 0;

	if (lcd->backlight) {
		lcd->backlight->props.power = FB_BLANK_POWERDOWN;
		lcd->backlight->props.state |= BL_CORE_FBBLANK;
		backlight_update_status(lcd->backlight);
	}

	lcd->enabled = false;

	return 0;
}

static int lcd_olinuxino_unprepare(struct drm_panel *panel)
{
	struct lcd_olinuxino *lcd = to_lcd_olinuxino(panel);

	if (!lcd->prepared)
		return 0;

	gpiod_set_value_cansleep(lcd->enable_gpio, 0);
	regulator_disable(lcd->supply);

	lcd->prepared = false;

	return 0;
}

static int lcd_olinuxino_prepare(struct drm_panel *panel)
{
	struct lcd_olinuxino *lcd = to_lcd_olinuxino(panel);
	int ret;

	if (lcd->prepared)
		return 0;

	ret = regulator_enable(lcd->supply);
	if (ret < 0)
		return ret;

	gpiod_set_value_cansleep(lcd->enable_gpio, 1);
	lcd->prepared = true;

	return 0;
}

static int lcd_olinuxino_enable(struct drm_panel *panel)
{
	struct lcd_olinuxino *lcd = to_lcd_olinuxino(panel);

	if (lcd->enabled)
		return 0;

	if (lcd->backlight) {
		lcd->backlight->props.state &= ~BL_CORE_FBBLANK;
		lcd->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(lcd->backlight);
	}

	lcd->enabled = true;

	return 0;
}

static int lcd_olinuxino_get_modes(struct drm_panel *panel)
{
	struct lcd_olinuxino *lcd = to_lcd_olinuxino(panel);
	struct drm_connector *connector = lcd->panel.connector;
	struct display_timing *timing = &lcd->eeprom.timing;
	struct drm_device *drm = lcd->panel.drm;
	struct drm_display_mode *mode;
	struct videomode vm;

	videomode_from_timing(timing, &vm);
	mode = drm_mode_create(drm);
	if (!mode) {
		dev_err(drm->dev, "failed to add mode %ux%u\n",
			timing->hactive.typ, timing->vactive.typ);
			return 0;
	}

	drm_display_mode_from_videomode(&vm, mode);

	mode->type |= DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	memcpy(connector->display_info.name, lcd->eeprom.info.name, 32);
	connector->display_info.width_mm = lcd->eeprom.info.width_mm;
	connector->display_info.height_mm = lcd->eeprom.info.height_mm;
	connector->display_info.bpc = lcd->eeprom.info.bpc;

	drm_display_info_set_bus_formats(&connector->display_info,
					 &lcd->eeprom.info.bus_format, 1);
	connector->display_info.bus_flags = lcd->eeprom.info.bus_flag;

	drm_mode_probed_add(connector, mode);
	return 1;
}

static int lcd_olinuxino_get_timings(struct drm_panel *panel,
				     unsigned int num_timings,
				     struct display_timing *timings)
{
	struct lcd_olinuxino *lcd = to_lcd_olinuxino(panel);

	if (timings)
		timings = &lcd->eeprom.timing;

	return 1;
}

struct drm_panel_funcs lcd_olinuxino_funcs = {
	.disable = lcd_olinuxino_disable,
	.unprepare = lcd_olinuxino_unprepare,
	.prepare = lcd_olinuxino_prepare,
	.enable = lcd_olinuxino_enable,
	.get_modes = lcd_olinuxino_get_modes,
	.get_timings = lcd_olinuxino_get_timings,
};

static int lcd_olinuxino_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *backlight;
	struct lcd_olinuxino *lcd;

	int ret = 0;
	int i;

	u32 checksum;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C ||
				     I2C_FUNC_SMBUS_READ_I2C_BLOCK))
		return -ENOSYS;

	lcd = devm_kzalloc(dev, sizeof(*lcd), GFP_KERNEL);
	if (!lcd)
		return -ENOMEM;

	i2c_set_clientdata(client, lcd);
	lcd->dev = dev;
	lcd->client = client;

	mutex_init(&lcd->mutex);

	/* Copy data into buffer */
	for(i = 0; i < LCD_OLINUXINO_DATA_LEN; i += I2C_SMBUS_BLOCK_MAX) {
		mutex_lock(&lcd->mutex);
		ret = i2c_smbus_read_i2c_block_data(client,
						    i,
						    I2C_SMBUS_BLOCK_MAX,
						    (u8 *)&lcd->eeprom + i);
		mutex_unlock(&lcd->mutex);
		if (ret < 0) {
			dev_err(dev, "Error reading from device at %02x\n", i);
			return ret;
		}
	}

	/* Check configuration checksum */
	checksum = ~crc32(~0, (u8 *)&lcd->eeprom, 252);
	if (checksum != lcd->eeprom.checksum) {
		dev_err(dev, "Configuration checksum does not match!\n");
		dev_dbg(dev, "Calculated checksum: %08x\n", checksum);
		dev_dbg(dev, "Stored checksum: %08x\n", lcd->eeprom.checksum);
		return -EINVAL;
	}

	/* Check magic header */
	if (lcd->eeprom.header != LCD_OLINUXINO_HEADER_MAGIC) {
		dev_err(dev, "Magic header does not match\n");
		return -EINVAL;
	}

	lcd->enabled = false;
	lcd->prepared = false;

	lcd->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(lcd->supply))
		return PTR_ERR(lcd->supply);

	lcd->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(lcd->enable_gpio))
		return PTR_ERR(lcd->enable_gpio);

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		lcd->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!lcd->backlight)
			return -EPROBE_DEFER;
	}

	drm_panel_init(&lcd->panel);
	lcd->panel.dev = dev;
	lcd->panel.funcs = &lcd_olinuxino_funcs;

	ret = drm_panel_add(&lcd->panel);
	if (ret < 0)
		goto free_backlight;

	return 0;

free_backlight:
	if (lcd->backlight)
		put_device(&lcd->backlight->dev);

	return ret;
}

static int lcd_olinuxino_remove(struct i2c_client *client)
{
	struct lcd_olinuxino *panel = i2c_get_clientdata(client);

	drm_panel_detach(&panel->panel);
	drm_panel_remove(&panel->panel);

	lcd_olinuxino_disable(&panel->panel);
	lcd_olinuxino_unprepare(&panel->panel);

	if (panel->backlight)
		put_device(&panel->backlight->dev);

	return 0;
}

static const struct of_device_id lcd_olinuxino_of_ids[] = {
	{ .compatible = "olimex,lcd-olinuxino" },
	{ }
};
MODULE_DEVICE_TABLE(of, lcd_olinuxino_of_ids);

static struct i2c_driver lcd_olinuxino_driver = {
	.driver = {
		.name = "lcd_olinuxino",
		.of_match_table = lcd_olinuxino_of_ids,
	},
	.probe = lcd_olinuxino_probe,
	.remove = lcd_olinuxino_remove,
};

static int __init lcd_olinuxino_init(void)
{
	return i2c_add_driver(&lcd_olinuxino_driver);
}
module_init(lcd_olinuxino_init);

static void __exit lcd_olinuxino_exit(void)
{
	i2c_del_driver(&lcd_olinuxino_driver);
}
module_exit(lcd_olinuxino_exit);

MODULE_AUTHOR("Stefan Mavrodiev <stefan@olimex.com>");
MODULE_DESCRIPTION("LCD-OLinuXino driver");
MODULE_LICENSE("GPL v2");
