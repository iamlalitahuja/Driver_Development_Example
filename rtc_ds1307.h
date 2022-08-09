/*
 * rtc_ds1307.h
 *
 *  Created on: Jan 16, 2021
 *      Author: lalitahuja
 */

#ifndef MAIN_RTC_DS1307_H_
#define MAIN_RTC_DS1307_H_

#include <stdio.h>
#include "driver/i2c.h"
#include "esp_console.h"
#include "esp_log.h"

#include <time.h>
#define TAG_RTC "I2C_RTC"
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ    /*!< I2C master read */
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 /*!< I2C ack value */
#define NACK_VAL 0x1                /*!< I2C nack value */
#define chip_addr 0x68



// Convert decimal numbers to binary coded decimal
uint8_t decToBcd(uint8_t val);
// Convert binary coded decimal to decimal numbers
uint8_t bcdToDec(uint8_t val);
esp_err_t i2c_master_driver_initialize(void);
int readDS1307time(uint8_t *result);
int writeDS1307time( uint8_t hour, uint8_t minute, uint8_t second, uint8_t date, uint8_t month, uint8_t year, uint8_t dayOfWeek);
long epoch_time();

void gmt_time(char *timeBuffer);

#endif /* MAIN_RTC_DS1307_H_ */
