#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/acpi.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/interrupt.h>
#include <linux/irq.h> /* For irq_get_irq_data() */
#include <linux/completion.h>
#include <linux/pm_runtime.h>
#include <linux/random.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Fox");
MODULE_DESCRIPTION("MyModule");
MODULE_VERSION("0.1a");

/* BMP280 specific registers */
#define BMP280_REG_HUMIDITY_LSB		0xFE
#define BMP280_REG_HUMIDITY_MSB		0xFD
#define BMP280_REG_TEMP_XLSB		0xFC
#define BMP280_REG_TEMP_LSB		0xFB
#define BMP280_REG_TEMP_MSB		0xFA
#define BMP280_REG_PRESS_XLSB		0xF9
#define BMP280_REG_PRESS_LSB		0xF8
#define BMP280_REG_PRESS_MSB		0xF7

#define BMP280_REG_CONFIG		0xF5
#define BMP280_REG_CTRL_MEAS		0xF4
#define BMP280_REG_STATUS		0xF3
#define BMP280_REG_CTRL_HUMIDITY	0xF2

/* Due to non linear mapping, and data sizes we can't do a bulk read */
#define BMP280_REG_COMP_H1		0xA1
#define BMP280_REG_COMP_H2		0xE1
#define BMP280_REG_COMP_H3		0xE3
#define BMP280_REG_COMP_H4		0xE4
#define BMP280_REG_COMP_H5		0xE5
#define BMP280_REG_COMP_H6		0xE7

#define BMP280_REG_COMP_TEMP_START	0x88
#define BMP280_COMP_TEMP_REG_COUNT	6

#define BMP280_REG_COMP_PRESS_START	0x8E
#define BMP280_COMP_PRESS_REG_COUNT	18

#define BMP280_FILTER_MASK		(BIT(4) | BIT(3) | BIT(2))
#define BMP280_FILTER_OFF		0
#define BMP280_FILTER_2X		BIT(2)
#define BMP280_FILTER_4X		BIT(3)
#define BMP280_FILTER_8X		(BIT(3) | BIT(2))
#define BMP280_FILTER_16X		BIT(4)

#define BMP280_OSRS_HUMIDITY_MASK	(BIT(2) | BIT(1) | BIT(0))
#define BMP280_OSRS_HUMIDITIY_X(osrs_h)	((osrs_h) << 0)
#define BMP280_OSRS_HUMIDITY_SKIP	0
#define BMP280_OSRS_HUMIDITY_1X		BMP280_OSRS_HUMIDITIY_X(1)
#define BMP280_OSRS_HUMIDITY_2X		BMP280_OSRS_HUMIDITIY_X(2)
#define BMP280_OSRS_HUMIDITY_4X		BMP280_OSRS_HUMIDITIY_X(3)
#define BMP280_OSRS_HUMIDITY_8X		BMP280_OSRS_HUMIDITIY_X(4)
#define BMP280_OSRS_HUMIDITY_16X	BMP280_OSRS_HUMIDITIY_X(5)

#define BMP280_OSRS_TEMP_MASK		(BIT(7) | BIT(6) | BIT(5))
#define BMP280_OSRS_TEMP_SKIP		0
#define BMP280_OSRS_TEMP_X(osrs_t)	((osrs_t) << 5)
#define BMP280_OSRS_TEMP_1X		BMP280_OSRS_TEMP_X(1)
#define BMP280_OSRS_TEMP_2X		BMP280_OSRS_TEMP_X(2)
#define BMP280_OSRS_TEMP_4X		BMP280_OSRS_TEMP_X(3)
#define BMP280_OSRS_TEMP_8X		BMP280_OSRS_TEMP_X(4)
#define BMP280_OSRS_TEMP_16X		BMP280_OSRS_TEMP_X(5)

#define BMP280_OSRS_PRESS_MASK		(BIT(4) | BIT(3) | BIT(2))
#define BMP280_OSRS_PRESS_SKIP		0
#define BMP280_OSRS_PRESS_X(osrs_p)	((osrs_p) << 2)
#define BMP280_OSRS_PRESS_1X		BMP280_OSRS_PRESS_X(1)
#define BMP280_OSRS_PRESS_2X		BMP280_OSRS_PRESS_X(2)
#define BMP280_OSRS_PRESS_4X		BMP280_OSRS_PRESS_X(3)
#define BMP280_OSRS_PRESS_8X		BMP280_OSRS_PRESS_X(4)
#define BMP280_OSRS_PRESS_16X		BMP280_OSRS_PRESS_X(5)

#define BMP280_MODE_MASK		(BIT(1) | BIT(0))
#define BMP280_MODE_SLEEP		0
#define BMP280_MODE_FORCED		BIT(0)
#define BMP280_MODE_NORMAL		(BIT(1) | BIT(0))

/* BMP180 specific registers */
#define BMP180_REG_OUT_XLSB		0xF8
#define BMP180_REG_OUT_LSB		0xF7
#define BMP180_REG_OUT_MSB		0xF6

#define BMP180_REG_CALIB_START		0xAA
#define BMP180_REG_CALIB_COUNT		22

#define BMP180_MEAS_SCO			BIT(5)
#define BMP180_MEAS_TEMP		(0x0E | BMP180_MEAS_SCO)
#define BMP180_MEAS_PRESS_X(oss)	((oss) << 6 | 0x14 | BMP180_MEAS_SCO)
#define BMP180_MEAS_PRESS_1X		BMP180_MEAS_PRESS_X(0)
#define BMP180_MEAS_PRESS_2X		BMP180_MEAS_PRESS_X(1)
#define BMP180_MEAS_PRESS_4X		BMP180_MEAS_PRESS_X(2)
#define BMP180_MEAS_PRESS_8X		BMP180_MEAS_PRESS_X(3)

/* BMP180 and BMP280 common registers */
#define BMP280_REG_CTRL_MEAS		0xF4
#define BMP280_REG_RESET		0xE0
#define BMP280_REG_ID			0xD0

#define BMP180_CHIP_ID			0x55
#define BMP280_CHIP_ID			0x58
#define BME280_CHIP_ID			0x60
#define BMP280_SOFT_RESET_VAL		0xB6

/* BMP280 register skipped special values */
#define BMP280_TEMP_SKIPPED		0x80000
#define BMP280_PRESS_SKIPPED		0x80000
#define BMP280_HUMIDITY_SKIPPED		0x8000

static bool bmp280_is_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case BMP280_REG_CONFIG:
	case BMP280_REG_CTRL_HUMIDITY:
	case BMP280_REG_CTRL_MEAS:
	case BMP280_REG_RESET:
		return true;
	default:
		return false;
	};
}

static bool bmp280_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case BMP280_REG_HUMIDITY_LSB:
	case BMP280_REG_HUMIDITY_MSB:
	case BMP280_REG_TEMP_XLSB:
	case BMP280_REG_TEMP_LSB:
	case BMP280_REG_TEMP_MSB:
	case BMP280_REG_PRESS_XLSB:
	case BMP280_REG_PRESS_LSB:
	case BMP280_REG_PRESS_MSB:
	case BMP280_REG_STATUS:
		return true;
	default:
		return false;
	}
}

static int bmp280_runtime_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct bmp280_data *data = iio_priv(indio_dev);
	int ret;

	ret = regulator_disable(data->vdda);
	if (ret)
		return ret;
	return regulator_disable(data->vddd);
}

static int bmp280_runtime_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct bmp280_data *data = iio_priv(indio_dev);
	int ret;

	ret = regulator_enable(data->vddd);
	if (ret)
		return ret;
	ret = regulator_enable(data->vdda);
	if (ret)
		return ret;
	usleep_range(data->start_up_time, data->start_up_time + 100);
	return data->chip_info->chip_config(data);
}

const struct regmap_config bmp280_regmap_config = {
.reg_bits = 8,
.val_bits = 8,

.max_register = BMP280_REG_HUMIDITY_LSB,
.cache_type = REGCACHE_RBTREE,

.writeable_reg = bmp280_is_writeable_reg,
.volatile_reg = bmp280_is_volatile_reg,
};

const struct dev_pm_ops bmp280_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(bmp280_runtime_suspend,
			   bmp280_runtime_resume, NULL)
};

int bmp280_common_remove(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct bmp280_data *data = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	pm_runtime_get_sync(data->dev);
	pm_runtime_put_noidle(data->dev);
	pm_runtime_disable(data->dev);
	regulator_disable(data->vdda);
	regulator_disable(data->vddd);
	return 0;
}

int bmp280_common_probe(struct device *dev,
			struct regmap *regmap,
			unsigned int chip,
			const char *name,
			int irq)
{
int ret;
struct iio_dev *indio_dev;
struct bmp280_data *data;
unsigned int chip_id;
struct gpio_desc *gpiod;

indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
if (!indio_dev)
	return -ENOMEM;

data = iio_priv(indio_dev);
mutex_init(&data->lock);
data->dev = dev;

indio_dev->dev.parent = dev;
indio_dev->name = name;
indio_dev->channels = bmp280_channels;
indio_dev->info = &bmp280_info;
indio_dev->modes = INDIO_DIRECT_MODE;

indio_dev->num_channels = 3;
data->chip_info = &bme280_chip_info;
data->oversampling_press = ilog2(16);
data->oversampling_humid = ilog2(16);
data->oversampling_temp = ilog2(2);
data->start_up_time = 2000;

/* Bring up regulators */
data->vddd = devm_regulator_get(dev, "vddd");
if (IS_ERR(data->vddd)) {
	dev_err(dev, "failed to get VDDD regulator\n");
	return PTR_ERR(data->vddd);
}
ret = regulator_enable(data->vddd);
if (ret) {
	dev_err(dev, "failed to enable VDDD regulator\n");
	return ret;
}
data->vdda = devm_regulator_get(dev, "vdda");
if (IS_ERR(data->vdda)) {
	dev_err(dev, "failed to get VDDA regulator\n");
	ret = PTR_ERR(data->vdda);
	goto out_disable_vddd;
}
ret = regulator_enable(data->vdda);
if (ret) {
	dev_err(dev, "failed to enable VDDA regulator\n");
	goto out_disable_vddd;
}
/* Wait to make sure we started up properly */
usleep_range(data->start_up_time, data->start_up_time + 100);

/* Bring chip out of reset if there is an assigned GPIO line */
gpiod = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
/* Deassert the signal */
if (!IS_ERR(gpiod)) {
	dev_info(dev, "release reset\n");
	gpiod_set_value(gpiod, 0);
}

data->regmap = regmap;
ret = regmap_read(regmap, BMP280_REG_ID, &chip_id);
if (ret < 0)
	goto out_disable_vdda;
if (chip_id != chip) {
	dev_err(dev, "bad chip id: expected %x got %x\n",
		chip, chip_id);
	ret = -EINVAL;
	goto out_disable_vdda;
}

ret = data->chip_info->chip_config(data);
if (ret < 0)
	goto out_disable_vdda;

dev_set_drvdata(dev, indio_dev);

/*
 * Some chips have calibration parameters "programmed into the devices'
 * non-volatile memory during production". Let's read them out at probe
 * time once. They will not change.
 */
if (chip_id  == BMP180_CHIP_ID) {
	ret = bmp180_read_calib(data, &data->calib.bmp180);
	if (ret < 0) {
		dev_err(data->dev,
			"failed to read calibration coefficients\n");
		goto out_disable_vdda;
	}
} else if (chip_id == BMP280_CHIP_ID || chip_id == BME280_CHIP_ID) {
	ret = bmp280_read_calib(data, &data->calib.bmp280, chip_id);
	if (ret < 0) {
		dev_err(data->dev,
			"failed to read calibration coefficients\n");
		goto out_disable_vdda;
	}
}

/*
 * Attempt to grab an optional EOC IRQ - only the BMP085 has this
 * however as it happens, the BMP085 shares the chip ID of BMP180
 * so we look for an IRQ if we have that.
 */
if (irq > 0 || (chip_id  == BMP180_CHIP_ID)) {
	ret = bmp085_fetch_eoc_irq(dev, name, irq, data);
	if (ret)
		goto out_disable_vdda;
}

/* Enable runtime PM */
pm_runtime_get_noresume(dev);
pm_runtime_set_active(dev);
pm_runtime_enable(dev);
/*
 * Set autosuspend to two orders of magnitude larger than the
 * start-up time.
 */
pm_runtime_set_autosuspend_delay(dev, data->start_up_time / 10);
pm_runtime_use_autosuspend(dev);
pm_runtime_put(dev);

ret = iio_device_register(indio_dev);
if (ret)
	goto out_runtime_pm_disable;


return 0;

out_runtime_pm_disable:
pm_runtime_get_sync(data->dev);
pm_runtime_put_noidle(data->dev);
pm_runtime_disable(data->dev);
out_disable_vdda:
regulator_disable(data->vdda);
out_disable_vddd:
regulator_disable(data->vddd);
return ret;
}

/* Regmap configurations */
const struct regmap_config bmp180_regmap_config;
const struct regmap_config bmp280_regmap_config;

/* Probe called from different transports */
int bmp280_common_probe(struct device *dev,
			struct regmap *regmap,
			unsigned int chip,
			const char *name,
			int irq);
int bmp280_common_remove(struct device *dev);

/* PM ops */
const struct dev_pm_ops bmp280_dev_pm_ops;

static int bmp280_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct regmap *regmap;
	const struct regmap_config *regmap_config;

	switch (id->driver_data) {
	case BMP180_CHIP_ID:
		regmap_config = &bmp180_regmap_config;
		break;
	case BMP280_CHIP_ID:
	case BME280_CHIP_ID:
		regmap_config = &bmp280_regmap_config;
		break;
	default:
		return -EINVAL;
	}

	regmap = devm_regmap_init_i2c(client, regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "failed to allocate register map\n");
		return PTR_ERR(regmap);
	}

	return bmp280_common_probe(&client->dev,
				   regmap,
				   id->driver_data,
				   id->name,
				   client->irq);
}

static int bmp280_i2c_remove(struct i2c_client *client)
{
	return bmp280_common_remove(&client->dev);
}

static const struct acpi_device_id bmp280_acpi_i2c_match[] = {
	{"BMP0280", BMP280_CHIP_ID },
	{"BMP0180", BMP180_CHIP_ID },
	{"BMP0085", BMP180_CHIP_ID },
	{"BME0280", BME280_CHIP_ID },
	{ },
};
MODULE_DEVICE_TABLE(acpi, bmp280_acpi_i2c_match);

#ifdef CONFIG_OF
static const struct of_device_id bmp280_of_i2c_match[] = {
	{ .compatible = "bosch,bme280", .data = (void *)BME280_CHIP_ID },
	{ .compatible = "bosch,bmp280", .data = (void *)BMP280_CHIP_ID },
	{ .compatible = "bosch,bmp180", .data = (void *)BMP180_CHIP_ID },
	{ .compatible = "bosch,bmp085", .data = (void *)BMP180_CHIP_ID },
	{ },
};
MODULE_DEVICE_TABLE(of, bmp280_of_i2c_match);
#else
#define bmp280_of_i2c_match NULL
#endif

static const struct i2c_device_id bmp280_i2c_id[] = {
	{"bmp280", BMP280_CHIP_ID },
	{"bmp180", BMP180_CHIP_ID },
	{"bmp085", BMP180_CHIP_ID },
	{"bme280", BME280_CHIP_ID },
	{ },
};
MODULE_DEVICE_TABLE(i2c, bmp280_i2c_id);

static struct i2c_driver bmp280_i2c_driver = {
	.driver = {
		.name	= "bmp280",
		.acpi_match_table = ACPI_PTR(bmp280_acpi_i2c_match),
		.of_match_table = of_match_ptr(bmp280_of_i2c_match),
		.pm = &bmp280_dev_pm_ops,
	},
	.probe		= bmp280_i2c_probe,
	.remove		= bmp280_i2c_remove,
	.id_table	= bmp280_i2c_id,
};
module_i2c_driver(bmp280_i2c_driver);
