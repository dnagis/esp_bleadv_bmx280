/*
 * bmp280.h
 *
 *  Created on: Feb 8, 2017
 *      Author: kris
 *
 *  This file is part of OpenAirProject-ESP32.
 *
 *  OpenAirProject-ESP32 is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  OpenAirProject-ESP32 is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with OpenAirProject-ESP32.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MAIN_BMP280_H_
#define MAIN_BMP280_H_

#include "oap_common.h"
#include "oap_data_env.h"

#define OAP_BMX280_ENABLED 1
#define OAP_BMX280_I2C_NUM 0
#define OAP_BMX280_ADDRESS 0x77 //Alors là grosse erreur des gars de chez openair project ils avaient mis 76!!!
#define OAP_BMX280_I2C_SDA_PIN 18 //VVNX****** 15 et 2 marche aussi, MAIS avec 15 et 2 j'ai l'impression que flash bloque quand VDD branché...
#define OAP_BMX280_I2C_SCL_PIN 19 //VVNX****** Pas d'astuce particulière pour les numéros de pin...

#define OAP_BMX280_ENABLED_AUX 0
#define OAP_BMX280_I2C_NUM_AUX 0x77
#define OAP_BMX280_ADDRESS_AUX 0x77
#define OAP_BMX280_I2C_SDA_PIN_AUX 25
#define OAP_BMX280_I2C_SCL_PIN_AUX 26

typedef void(*env_callback)(env_data_t*);

typedef struct bmx280_config_t {

	uint8_t i2c_num;
	uint8_t device_addr;
	uint8_t sda_pin;
	uint8_t scl_pin;
	uint8_t sensor_idx;	//sensor number (0 - 1)
	uint32_t interval;
	env_callback callback;

} bmx280_config_t;

esp_err_t bmx280_init(bmx280_config_t* config);

esp_err_t bmx280_set_hardware_config(bmx280_config_t* bmx280_config, uint8_t sensor_idx);

#endif /* MAIN_BMP280_H_ */
