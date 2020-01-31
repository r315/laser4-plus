
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
//-------------------------------
//-------------------------------
//CC2500 SPI routines
//-------------------------------
//-------------------------------
#ifdef CC2500_INSTALLED

#include "board.h"
#include "iface_cc2500.h"

#define NOP()

//----------------------
static void CC2500_ReadRegisterMulti(uint8_t address, uint8_t data[], uint8_t length)
{
	CC25_CS_TRUE;
	SPI_Write(CC2500_READ_BURST | address);
	for(uint8_t i = 0; i < length; i++)
		data[i] = SPI_Read();
	CC25_CS_FALSE;
}

//--------------------------------------------
uint8_t CC2500_ReadReg(uint8_t address)
{ 
	uint8_t result;
	CC25_CS_TRUE;
	SPI_Write(CC2500_READ_SINGLE | address);
	result = SPI_Read();  
	CC25_CS_FALSE;
	return(result); 
} 

//------------------------
static void CC2500_WriteRegisterMulti(uint8_t address, const uint8_t data[], uint8_t length)
{
	CC25_CS_TRUE;
	SPI_Write(CC2500_WRITE_BURST | address);
	for(uint8_t i = 0; i < length; i++)
		SPI_Write(data[i]);
	CC25_CS_FALSE;
}

//*********************************************
void CC2500_ReadData(uint8_t *dpbuffer, uint8_t len)
{
	CC2500_ReadRegisterMulti(CC2500_3F_RXFIFO, dpbuffer, len);
}

//------------------------
void CC2500_Strobe(uint8_t state)
{
	CC25_CS_TRUE;
	SPI_Write(state);
	CC25_CS_FALSE;
}

//------------------------
void CC2500_WriteData(uint8_t *dpbuffer, uint8_t len)
{
	CC2500_Strobe(CC2500_SFTX);
	CC2500_WriteRegisterMulti(CC2500_3F_TXFIFO, dpbuffer, len);
	CC2500_Strobe(CC2500_STX);
}

//----------------------------
void CC2500_WriteReg(uint8_t address, uint8_t data)
{
	CC25_CS_TRUE;
	SPI_Write(address); 
	NOP();
	SPI_Write(data);
	CC25_CS_FALSE;
}

//------------------------
void CC2500_SetTxRxMode(uint8_t mode)
{
	if(mode == TX_EN)
	{//from deviation firmware
		CC2500_WriteReg(CC2500_00_IOCFG2, 0x2F);
		CC2500_WriteReg(CC2500_02_IOCFG0, 0x2F | 0x40);
	}
	else
		if (mode == RX_EN)
		{
			CC2500_WriteReg(CC2500_02_IOCFG0, 0x2F);
			CC2500_WriteReg(CC2500_00_IOCFG2, 0x2F | 0x40);
		}
		else
		{
			CC2500_WriteReg(CC2500_02_IOCFG0, 0x2F);
			CC2500_WriteReg(CC2500_00_IOCFG2, 0x2F);
		}
}

//------------------------
/*static void cc2500_resetChip(void)
{
	// Toggle chip select signal
	CC25_CS_FALSE;
	delayMicroseconds(30);
	CC25_CS_TRUE;
	delayMicroseconds(30);
	CC25_CS_FALSE;
	delayMicroseconds(45);
	CC2500_Strobe(CC2500_SRES);
	_delay_ms(100);
}
*/
uint8_t CC2500_Reset(void)
{
	CC2500_Strobe(CC2500_SRES);
	delayMs(1);
	CC2500_SetTxRxMode(TXRX_OFF);
	return CC2500_ReadReg(CC2500_0E_FREQ1) == 0xC4;//check if reset
}
/*
static void CC2500_SetPower_Value(uint8_t power)
{
	const unsigned char patable[8]=	{
		0xC5,  // -12dbm
		0x97, // -10dbm
		0x6E, // -8dbm
		0x7F, // -6dbm
		0xA9, // -4dbm
		0xBB, // -2dbm
		0xFE, // 0dbm
		0xFF // 1.5dbm
	};
	if (power > 7)
		power = 7;
	CC2500_WriteReg(CC2500_3E_PATABLE,  patable[power]);
}
*/

void CC2500_SetPower(uint8_t power)
{
	CC2500_WriteReg(CC2500_3E_PATABLE, power);
}

uint8_t CC2500_ReadStatus(uint8_t sreg){
uint8_t data;
	CC25_CS_TRUE;
	SPI_Write(CC2500_READ_BURST | sreg);
	data = SPI_Read();  
	CC25_CS_FALSE;
	return data;
}

#endif /* CC2500_INSTALLED */