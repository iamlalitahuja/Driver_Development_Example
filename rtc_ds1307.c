/*
 * rtc_ds1307.c
 *
 *  Created on: Jan 16, 2021
 *      Author: lalitahuja
 */


#include "rtc_ds1307.h"
#include "error.h"


gpio_num_t i2c_gpio_scl = 19;
gpio_num_t i2c_gpio_sda = 21;

uint32_t i2c_frequency = 100000;
i2c_port_t i2c_port = I2C_NUM_0;

esp_err_t i2c_master_driver_initialize(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = i2c_gpio_sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = i2c_gpio_scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = i2c_frequency
    };
    i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    return i2c_param_config(i2c_port, &conf);
}

int readDS1307time(uint8_t *result)
{
	uint8_t len = 8;
    uint8_t data[8] = {0};
    uint8_t data_addr = 0;

//    i2c_master_driver_initialize();
//    i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
	i2c_master_write_byte(cmd, chip_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, data_addr, ACK_CHECK_EN);
	i2c_master_start(cmd);
    i2c_master_write_byte(cmd, chip_addr << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, len, ACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
    	//printf("\n time\n " );
        for (int i = 0; i < len-1; i++) {
        	result[i] = bcdToDec(data[i]);
            //printf("%2d ", result[i] );
            //result0  1    2  		3		4		5		6
            //sec	  min	hour	day		date	month   year
            //6       36 	11  	4  		3  		3 		21
        }
    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG_RTC, "Bus is busy");
        sendError(RTC_BUS_BUSY, "RTC_BUS_BUSY");
        return 1;
    } else {
        ESP_LOGW(TAG_RTC, "Read failed");
        sendError(RTC_READ_FAIL, "RTC_READ_FAIL");
        return 1;
    }
    //i2c_driver_delete(i2c_port);
    return 0;
}



int writeDS1307time( uint8_t hour, uint8_t minute, uint8_t second, uint8_t date, uint8_t month, uint8_t year, uint8_t dayOfWeek)
{
	uint8_t data_addr = 0;
	uint8_t len = 8;
	uint8_t data[8] = {0};

	data[0] = ( decToBcd(second) ) & ~(0x00);	// set seconds
	data[1] = decToBcd(minute); 			// set minutes
	data[2] = decToBcd(hour); 		 		// set hours
	data[3] = decToBcd(dayOfWeek); 			// set day of week (1=Sunday, 7=Saturday)
	data[4] = decToBcd(date);   			// set date (1 to 31)
	data[5] = decToBcd(month);		 		// set month
	data[6] = decToBcd(year); 				// set year (0 to 99)
	data[7] = 0x00;

//    i2c_master_driver_initialize();
//    i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, chip_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data_addr, ACK_CHECK_EN);
    i2c_master_write(cmd, data,len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG_RTC, "Write OK");
    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG_RTC, "Bus is busy");
        return 1;
        sendError(RTC_BUS_BUSY, "RTC_BUS_BUSY");
        return 1;
    } else {
        ESP_LOGW(TAG_RTC, "Write Failed");
        sendError(RTC_WRITE_FAIL, "RTC_WRITE_FAIL");
        return 1;
    }
    //i2c_driver_delete(i2c_port);
    return 0;
}

long epoch_time(){

	uint8_t result[8];
    struct tm t;
    time_t t_of_day;

	if( readDS1307time(result) )
		return 0;

    t.tm_sec   = result[0];
    t.tm_min   = result[1];
    t.tm_hour  = result[2];
    t.tm_mday  = result[4]; 						// Date of the month
    t.tm_mon   = result[5] - 1;         			// Month, where 0 = January
    t.tm_year  = (2000+result[6]) -1900; 			// Year - 1900
    t.tm_isdst = -1;        						// Is DST on? 1 = yes, 0 = no, -1 = unknown
    t_of_day = mktime(&t);

    //printf("seconds since the Epoch: %ld\n", (long) t_of_day);

    return (long) (t_of_day);

}


void gmt_time(char *timeBuffer){

	uint8_t result[8];


	readDS1307time(result);

	sprintf(timeBuffer," %02d:%02d:%02d  %02d/%02d/%02d",result[2],result[1],result[0],result[4],result[5],result[6]);
    //printf("\n gmt_time: %s\n",timeBuffer);

//    return data;

}

uint8_t decToBcd(uint8_t val){
  return( (val/10*16) + (val%10) );
}

uint8_t bcdToDec(uint8_t val){
  return( (val/16*10) + (val%16) );
}

//void app_main(void)
//{
//	// set the initial time here:
//	// DS1307  hours, minutes, seconds, date, month, year, day
//	writeDS1307time(16,0,10,14,01,21,5);
//
//	while(1){
//		readDS1307time();
//
//		vTaskDelay(2000/portTICK_RATE_MS);
//	}
//}

