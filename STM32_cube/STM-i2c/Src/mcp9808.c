/*
 * mcp9808.c
 *
 *  Created on: 1. 7. 2018
 *      Author: David
 */
/*
#define mcp_addr       0x18 //adress of temp
#define mcp_reg_conf   0x01
#define reg_temp       0x30

uint8_t data_write[3];
uint8_t data_read[2];

int setup_temp(){
	uint8_t data_write[3];

	data_write[0] = mcp_reg_conf;
	data_write[1] = 0x00;  // config msb
	data_write[2] = 0x00;  // config lsb
	HAL_I2C_Master_Transmit(&hi2c2,mcp_addr << 1,data_write, 3 ,100);
	return int x=0;
}

int read_data(){

	return int x =0;
}
*/
