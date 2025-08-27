/*
 * Copyright (c) 2024 Croxel, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/services/nus.h>

// sens
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
/* Include the header file of the I2C API */
#include <zephyr/drivers/i2c.h>
/* Include the header file of printk() */
#include <zephyr/sys/printk.h>



#define DEVICE_NAME		CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN		(sizeof(DEVICE_NAME) - 1)
#define SENSOR_BUFF_SIZE    32




// sens
#define SLEEP_TIME_MS 2000
#define CTRLMEAS 0xF4
#define CALIB00	 0x88
#define CALIBP00 0x8E
#define CALIBH00 0xA1
#define ID	     0xD0
// according to docs, read from 0xF7 to 0xFE
#define TEMPMSB	 0xFA
#define PRESMSB  0xF7
#define HUMMSB   0xFD

#define CHIP_ID  0x60
#define SENSOR_CONFIG_VALUE 0x93
/* Get the node identifier of the sensor */
#define I2C_NODE DT_NODELABEL(mysensor)

static int32_t t_fine; // temp compensation

/* Data structure to store BME280 data */
struct bme280_data {
	/* Compensation parameters */
	// for temp
	uint16_t dig_t1;
	int16_t dig_t2;
	int16_t dig_t3;
	// for pressure
	uint16_t dig_p1;
	int16_t dig_p2;
	int16_t dig_p3;
	int16_t dig_p4;
	int16_t dig_p5;
	int16_t dig_p6;
	int16_t dig_p7;
	int16_t dig_p8;
	int16_t dig_p9;
	// for humidity total 8 bytes one byte is half used twice
	uint8_t dig_h1; // 1
	int16_t dig_h2; // 2
	uint8_t dig_h3; // 1
	int16_t dig_h4; // 2
	int16_t dig_h5; // 2
	int8_t  dig_h6; // 1
} bmedata;

enum Sensor_State {
	TEMP,
	HUM,
	PRES
};

enum Sensor_State current_sen_state = TEMP;

// ble fns
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_SRV_VAL),
};

static void notif_enabled(bool enabled, void *ctx)
{
	ARG_UNUSED(ctx);

	printk("%s() - %s\n", __func__, (enabled ? "Enabled" : "Disabled"));
}

static void cycle_sensor() {
	if (current_sen_state == TEMP) {
		current_sen_state = HUM;
	} else if (current_sen_state == HUM) {
		current_sen_state = PRES;
	} else {
		current_sen_state = TEMP;
	}
}

static void received(struct bt_conn *conn, const void *data, uint16_t len, void *ctx)
{
	char message[CONFIG_BT_L2CAP_TX_MTU + 1] = "";

	ARG_UNUSED(conn);
	ARG_UNUSED(ctx);

	memcpy(message, data, MIN(sizeof(message) - 1, len));
	cycle_sensor();
}

struct bt_nus_cb nus_listener = {
	.notif_enabled = notif_enabled,
	.received = received,
};

// sensor fns
// sensor calibration
void bme_calibrationdata(const struct i2c_dt_spec *spec, struct bme280_data *sensor_data_ptr)
{
	/* Put calibration function code */

	// calibration values for temp
	uint8_t values[6];  // 6 = 3 * 2
	int ret = i2c_burst_read_dt(spec, CALIB00, values, 6);  
	if (ret != 0) {
		printk("Failed to read register %x \n", CALIB00);
		return;
 	}

	//calibration values for pressure
	uint8_t p_values[18]; // 18 = 9 * 2
	ret = i2c_burst_read_dt(spec, CALIBP00, p_values, 18);  
	if (ret != 0) {
		printk("Failed to read register %x \n", CALIB00);
		return;
 	}

	//calibration values for humidity
	uint8_t h_values[8]; // 8 bytes needed
	ret = i2c_burst_read_dt(spec, CALIBH00, h_values, 9);  
	if (ret != 0) {
		printk("Failed to read register %x \n", CALIB00);
		return;
 	}
	
	sensor_data_ptr->dig_t1 = ((uint16_t)values[1]) << 8 | values[0];
	sensor_data_ptr->dig_t2 = ((uint16_t)values[3]) << 8 | values[2];
	sensor_data_ptr->dig_t3 = ((uint16_t)values[5]) << 8 | values[4];

	sensor_data_ptr->dig_p1 = ((uint16_t)p_values[1]) << 8 | p_values[0];
	sensor_data_ptr->dig_p2 = ((uint16_t)p_values[3]) << 8 | p_values[2];
	sensor_data_ptr->dig_p3 = ((uint16_t)p_values[5]) << 8 | p_values[4];
	sensor_data_ptr->dig_p4 = ((uint16_t)p_values[7]) << 8 | p_values[6];
	sensor_data_ptr->dig_p5 = ((uint16_t)p_values[9]) << 8 | p_values[8];
	sensor_data_ptr->dig_p6 = ((uint16_t)p_values[11]) << 8 | p_values[10];
	sensor_data_ptr->dig_p7 = ((uint16_t)p_values[13]) << 8 | p_values[12];
	sensor_data_ptr->dig_p8 = ((uint16_t)p_values[15]) << 8 | p_values[14];
	sensor_data_ptr->dig_p9 = ((uint16_t)p_values[17]) << 8 | p_values[16];

	sensor_data_ptr->dig_h1 = (uint16_t)h_values[0];
	sensor_data_ptr->dig_h2 = ((uint16_t)h_values[2]) << 8 | p_values[1];
	sensor_data_ptr->dig_h3 = (uint16_t)h_values[3];
	sensor_data_ptr->dig_h4 = (((uint16_t)h_values[5] & 0x0F) << 8 | h_values[4]);
	if (sensor_data_ptr->dig_h4 & 0x800) {
		sensor_data_ptr->dig_h4 |= 0xF000;
	}
	sensor_data_ptr->dig_h5 = (((uint16_t)h_values[6] << 4) | h_values[5] >> 4);
	if (sensor_data_ptr->dig_h5 & 0x800) {
		sensor_data_ptr->dig_h5 |= 0xF000;
	}
	sensor_data_ptr->dig_h6 = (uint16_t)h_values[7];

}

/* Compensate current temperature using previously stored sensor calibration data */
static int32_t bme280_compensate_temp(struct bme280_data *data, int32_t adc_temp)
{
	int32_t var1, var2;

	var1 = (((adc_temp >> 3) - ((int32_t)data->dig_t1 << 1)) * ((int32_t)data->dig_t2)) >> 11;

	var2 = (((((adc_temp >> 4) - ((int32_t)data->dig_t1)) *
		  ((adc_temp >> 4) - ((int32_t)data->dig_t1))) >>
		 12) *
		((int32_t)data->dig_t3)) >>
	       14;

	t_fine = var1 + var2;
	return ((t_fine) * 5 + 128) >> 8;
}

/* Compensate current pressure using previously stored sensor calibration data */
static int32_t bme280_compensate_pres(struct bme280_data *data, int32_t adc_P)
{
	int64_t var1, var2, p;

    var1 = (int64_t)t_fine - 128000;
    var2 = var1 * var1 * (int64_t)data->dig_p6;
    var2 = var2 + ((var1 * (int64_t)data->dig_p5) << 17);
    var2 = var2 + (((int64_t)data->dig_p4) << 35);
    var1 = ((var1 * var1 * (int64_t)data->dig_p3) >> 8) +
           ((var1 * (int64_t)data->dig_p2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * (int64_t)data->dig_p1) >> 33;

    if (var1 == 0) {
        /* avoid DIV by zero */
        return 0;
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)data->dig_p9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)data->dig_p8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)data->dig_p7) << 4);
    return (uint32_t)p;
}

// BME280_U32_t bme280_compensate_H_int32(BME280_S32_t adc_H)
// {
// BME280_S32_t v_x1_u32r;
// v_x1_u32r = (t_fine – ((BME280_S32_t)76800));
// v_x1_u32r = (((((adc_H << 14) – (((BME280_S32_t)dig_H4) << 20) – (((BME280_S32_t)dig_H5) * v_x1_u32r)) +
// ((BME280_S32_t)16384)) >> 15) * (((((((v_x1_u32r * ((BME280_S32_t)dig_H6)) >> 10) * (((v_x1_u32r *
// ((BME280_S32_t)dig_H3)) >> 11) + ((BME280_S32_t)32768))) >> 10) + ((BME280_S32_t)2097152)) *
// ((BME280_S32_t)dig_H2) + 8192) >> 14));
// v_x1_u32r = (v_x1_u32r – (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((BME280_S32_t)dig_H1)) >> 4));
// v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
// v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
// return (BME280_U32_t)(v_x1_u32r>>12);
// }

static uint32_t bme280_compensate_hum(struct bme280_data *data, int32_t adc_H) {
    int32_t v_x1_u32r;

    v_x1_u32r = t_fine - 76800;
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)data->dig_h4) << 20) - 
                    ((int32_t)data->dig_h5 * v_x1_u32r)) + 16384) >> 15) *
                 (((((((v_x1_u32r * (int32_t)data->dig_h6) >> 10) *
                      ((v_x1_u32r * ((int32_t)data->dig_h3) >> 11) + 32768)) >> 10) + 2097152) *
                    (int32_t)data->dig_h2 + 8192) >> 14));

    v_x1_u32r = v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * 
                              (int32_t)data->dig_h1) >> 4);

    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;

    return (uint32_t)(v_x1_u32r >> 12);
}

int main(void)
{

	// init ble first
	int err;

	printk("Sample - Bluetooth Peripheral NUS\n");

	err = bt_nus_cb_register(&nus_listener, NULL);
	if (err) {
		printk("Failed to register NUS callback: %d\n", err);
		return err;
	}

	err = bt_enable(NULL);
	if (err) {
		printk("Failed to enable bluetooth: %d\n", err);
		return err;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Failed to start advertising: %d\n", err);
		return err;
	}

	printk("BLE Initialization complete\n");

	// sensor
	/* Retrieve the API-specific device structure and make sure that the device is
	 * ready to use  */
	static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);
	if (!device_is_ready(dev_i2c.bus)) {
		printk("I2C bus %s is not ready!\n\r",dev_i2c.bus->name);
		return -1;
	}
	/* Verify it is proper device by reading device id (sensor)*/
	uint8_t id = 0;
	uint8_t regs[] = {ID};  
	int ret = i2c_write_read_dt(&dev_i2c, regs, 1, &id, 1);  
	
	if (ret != 0) {
		printk("Failed to read register %x \n", regs[0]);
	return -1;
	}  
	if (id != CHIP_ID) {
		printk("Invalid chip id! %x \n", id);
	return -1;
	}
	bme_calibrationdata(&dev_i2c, &bmedata);

	/* Setup the sensor by writing the value 0x93 to the Configuration register */
	uint8_t sensor_config[] = {CTRLMEAS, SENSOR_CONFIG_VALUE};
	ret = i2c_write_dt(&dev_i2c, sensor_config, 2);  
	if (ret != 0) {
		printk("Failed to write register %x \n", sensor_config[0]);
		return -1;
 	}


	// buffer for output
	char sensor_reading[SENSOR_BUFF_SIZE];
	uint8_t temp_val[8] = {0};  // press x 3, temp x 3, hum x 2
	while (true) {

		// switch (current_sen_state) {
		// case TEMP:
		// 	strcpy(sensor_reading, "Temp!\n");
		// 	break;
		// case HUM:
		// 	strcpy(sensor_reading, "Hum!\n");
		// 	break;
		// case PRES:
		// 	strcpy(sensor_reading, "Pres!\n");
		// }
		// read
		int ret = i2c_burst_read_dt(&dev_i2c, PRESMSB, temp_val, sizeof(temp_val));  
		if (ret != 0) {
			printk("Failed to read register %x \n", PRESMSB);
			k_msleep(SLEEP_TIME_MS);
			continue;
		}
		/* - Put the data read from registers into actual order (see datasheet) */
		int32_t adc_pres = 
			(temp_val[0] << 12) | (temp_val[1] << 4) | ((temp_val[2] >> 4) & 0x0F);			
		int32_t adc_temp = 
			(temp_val[3] << 12) | (temp_val[4] << 4) | ((temp_val[5] >> 4) & 0x0F);
		int32_t adc_hum = 
			(temp_val[6] << 8) | (temp_val[7]);

		switch (current_sen_state) {
		case TEMP:
			/* - Compensate temperature */
			int32_t comp_temp = bme280_compensate_temp(&bmedata, adc_temp);
			// convert and print
			float temperature = (float)comp_temp / 100.0f;
			double fTemp = (double)temperature * 1.8 + 32;  // Print reading to console
			snprintf(sensor_reading, SENSOR_BUFF_SIZE,
			"T: %8.2fC\n",(double)temperature);
			break;
		case HUM:
			/* - Compensate humidity */
			int32_t comp_hum = bme280_compensate_hum(&bmedata, adc_hum);
			// convert and print
			float humidity = (float)(comp_hum) / 1024.0f;
			snprintf(sensor_reading, SENSOR_BUFF_SIZE, "H: %8.5f%%RH\n", (double)humidity);
			break;
		case PRES:
		/* - Compensate pressure */
			int32_t comp_pres = bme280_compensate_pres(&bmedata, adc_pres);
			// convert and print
			float pressure = (float)(comp_pres) / 256;
			snprintf(sensor_reading, SENSOR_BUFF_SIZE, "P: %8.2fPa\n", (double)pressure);
			break;
		default:
			snprintf(sensor_reading, SENSOR_BUFF_SIZE,"error, unknown state");
		}

		k_sleep(K_SECONDS(1));



		err = bt_nus_send(NULL, sensor_reading, strlen(sensor_reading));
		printk("Data send - Result: %d\n", err);


		if (err < 0 && (err != -EAGAIN) && (err != -ENOTCONN)) {
			return err;
		}
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
