/*
 * max14915.c
 *
 * Created: 19.01.2021 13:19:25
 *  Author: gfcwfzkm
 */ 

#include "max14915.h"

#define MAX14_READ	0
#define MAX14_WRITE	1

#define CRC_MASK	0x1F

#define MAX14_CONFIG1_DEFAULT	(MAX14_R_CONFIG1_FLATCHEN | MAX14_R_CONFIG1_FFILTEREN | MAX14_R_CONFIG1_SLEDSET | MAX14_R_CONFIG1_FLSEDSET)
#define MAX14_MASK_DEFAULT		(MAX14_R_MASK_COMERRM | MAX14_R_MASK_VDDOKM | MAX14_R_MASK_SHTVDDM | MAX14_R_MASK_OWONM | MAX14_R_MASK_OWOFFM | MAX14_R_MASK_CURRLIMM)

/* Maxim's CRC Functions. Ain't pretty, but works I guess...
 * https://www.maximintegrated.com/en/design/technical-documents/app-notes/6/6633.html
 */
uint8_t max14_crc5encode(uint8_t *data)
{
	uint8_t crc5_start = 0x1f;
	uint8_t crc5_poly = 0x15;
	uint8_t crc_result = crc5_start;
	
	// BYTE1
	for (uint8_t i=0; i<8; i++)
	{
		if( ((( data[0]>>(7-i) ) &0x01) ^ ((crc_result & 0x10)>>4)) > 0 ) // IF(XOR(C6;BITAND(D5;2^4)/2^4)
		{
			crc_result = (uint8_t) (crc5_poly ^ ((crc_result<<1) & 0x1f)); // BITXOR($D$1;BITAND((D5*2);31))
		}
		else
		{
			crc_result = (uint8_t)((crc_result<<1) & 0x1f); // shift left, keep only lower 6 bits
		}
	}
	
	// BYTE2
	for (uint8_t i=0; i<8; i++)
	{
		if( ((( data[1]>>(7-i) ) &0x01) ^ ((crc_result & 0x10)>>4)) > 0 ) // IF(XOR(C6;BITAND(D5;2^4)/2^4)
		{
			crc_result = (uint8_t) (crc5_poly ^ ((crc_result<<1) & 0x1f)); // BITXOR($D$1;BITAND((D5*2);31))
		}
		else
		{
			crc_result = (uint8_t)((crc_result<<1) & 0x1f); // shift left, keep only lower 6 bits
		}
	}
	
	// 3 extra bits set to zero
	uint8_t BYTE3=0x00;
	for (uint8_t i=0; i<3; i++)
	{
		if( ((( BYTE3>>(7-i) ) &0x01) ^ ((crc_result & 0x10)>>4)) > 0 ) // IF(XOR(C6;BITAND(D5;2^4)/2^4)
		{
			crc_result = (uint8_t) (crc5_poly ^ ((crc_result<<1) & 0x1f)); // BITXOR($D$1;BITAND((D5*2);31))
		}
		else
		{
			crc_result = (uint8_t)((crc_result<<1) & 0x1f); // shift left, keep only lower 6 bits
		}
	}
	return crc_result;
}

uint8_t max14_crc5_decode(uint8_t *data)
{
	uint8_t crc5_start = 0x1f;
	uint8_t crc5_poly = 0x15;
	uint8_t crc_result = crc5_start;
	
	// BYTE1
	for (int i=2; i<8; i++)
	{
		if( ((( data[0]>>(7-i) ) &0x01) ^ ((crc_result & 0x10)>>4)) > 0 ) // IF(XOR(C6;BITAND(D5;2^4)/2^4)
		{
			crc_result = (uint8_t) (crc5_poly ^ ((crc_result<<1) & 0x1f)); // BITXOR($D$1;BITAND((D5*2);31))
		}
		else
		{
			crc_result = (uint8_t)((crc_result<<1) & 0x1f); // shift left, keep only lower 6 bits
		}
	}
	
	// BYTE2
	for (int i=0; i<8; i++)
	{
		if( ((( data[1]>>(7-i) ) &0x01) ^ ((crc_result & 0x10)>>4)) > 0 ) // IF(XOR(C6;BITAND(D5;2^4)/2^4)
		{
			crc_result = (uint8_t) (crc5_poly ^ ((crc_result<<1) & 0x1f)); // BITXOR($D$1;BITAND((D5*2);31))
		}
		else
		{
			crc_result = (uint8_t)((crc_result<<1) & 0x1f); // shift left, keep only lower 6 bits
		}
	}
	
	// 3 extra bits set to zero
	for (int i=0; i<3; i++)
	{
		if( ((( (data[2] & 0xE0)>>(7-i) ) &0x01) ^ ((crc_result & 0x10)>>4)) > 0 ) // IF(XOR(C6;BITAND(D5;2^4)/2^4)
		{
			crc_result = (uint8_t) (crc5_poly ^ ((crc_result<<1) & 0x1f)); // BITXOR($D$1;BITAND((D5*2);31))
			
		}
		else
		{
			crc_result = (uint8_t)((crc_result<<1) & 0x1f);  // shift left, keep only lower 6 bits
			
		}
	}
	return crc_result;
}

uint8_t max14_readReg(max14_t *dout, uint8_t regAddr)
{
	uint8_t dBuf[3];
	uint8_t dLen = 2;
	
	dBuf[0] = regAddr << 1 | MAX14_READ;
	dBuf[1] = 0;
	dBuf[2] = 0;
	
	if (dout->crc_en == MAX14_CRCEN)
	{
		dBuf[2] = max14_crc5encode(dBuf);
		dLen = 3;
	}
	
	if (dout->startTransaction(dout->ioInterface))					dout->coms_error |= MAX14_COMSERROR;
	if (dout->transceiveBytes(dout->ioInterface, 0, dBuf, dLen))	dout->coms_error |= MAX14_COMSERROR;
	if (dout->endTransaction(dout->ioInterface))					dout->coms_error |= MAX14_COMSERROR;
	
	if ( (dout->crc_en == MAX14_CRCEN) && (max14_crc5_decode(dBuf) != (dBuf[2] & CRC_MASK)) )
	{
		dout->coms_error |= MAX14_CRCERROR;
	}
	
	dout->diag = dBuf[0];
	return dBuf[1];
}

void max14_writeReg(max14_t *dout, uint8_t regAddr, uint8_t val)
{
	uint8_t dBuf[3];
	uint8_t dLen = 2;
	
	dBuf[0] = regAddr << 1 | MAX14_WRITE;
	dBuf[1] = val;
	dBuf[2] = 0;
	
	if (dout->crc_en == MAX14_CRCEN)
	{
		dBuf[2] = max14_crc5encode(dBuf);
		dLen = 3;
	}
	
	if (dout->startTransaction(dout->ioInterface))					dout->coms_error |= MAX14_COMSERROR;
	if (dout->transceiveBytes(dout->ioInterface, 0, dBuf, dLen))	dout->coms_error |= MAX14_COMSERROR;
	if (dout->endTransaction(dout->ioInterface))					dout->coms_error |= MAX14_COMSERROR;
	
	if ( (dout->crc_en == MAX14_CRCEN) && (max14_crc5_decode(dBuf) != (dBuf[2] & CRC_MASK)) )
	{
		dout->coms_error |= MAX14_CRCERROR;
	}
	
	dout->diag = dBuf[0];
}

void max14_initStruct(max14_t *dout, void *ioComs,
				uint8_t (*startTransaction)(void*),
				uint8_t (*transceiveBytes)(void*,uint8_t,uint8_t*,uint16_t),
				uint8_t (*endTransaction)(void*))
{
	dout->ioInterface = ioComs;
	dout->startTransaction = startTransaction;
	dout->transceiveBytes = transceiveBytes;
	dout->endTransaction = endTransaction;
}

enum MAX14_ERROR max14_init(max14_t *dout, enum MAX14_CRCEN crcen)
{
	uint8_t tempVal;
	
	dout->crc_en = crcen;
	dout->coms_error = MAX14_NOERROR;
	
	// Set the device to default register settings, unless POR is set
	tempVal = max14_readReg(dout, MAX14_R_GLOBALERR);
	if (!(tempVal & MAX14_R_GLOBALERR_VINT_UV))
	{
		max14_writeReg(dout, MAX14_R_SETOUT, 0);
		max14_writeReg(dout, MAX14_R_SETFLED, 0);
		max14_writeReg(dout, MAX14_R_SETSLED, 0);
		max14_writeReg(dout, MAX14_R_OWOFFEN, 0);
		max14_writeReg(dout, MAX14_R_OWONEN, 0);
		max14_writeReg(dout, MAX14_R_SHTVDDEN, 0);
		max14_writeReg(dout, MAX14_R_CONFIG1, MAX14_CONFIG1_DEFAULT);
		max14_writeReg(dout, MAX14_R_CONFIG2, 0);
		max14_writeReg(dout, MAX14_R_MASK, MAX14_MASK_DEFAULT);
	}
	
	// Checking if we really are connected to the device, by reading a default-value back
	tempVal = max14_readReg(dout, MAX14_R_CONFIG1);
	if (tempVal != (MAX14_R_CONFIG1_FLATCHEN | MAX14_R_CONFIG1_FFILTEREN | MAX14_R_CONFIG1_SLEDSET | MAX14_R_CONFIG1_FLSEDSET))
	{
		dout->coms_error |= MAX14_COMSERROR;
	}
	
	return dout->coms_error;
}

enum MAX14_GLOBALFAULT max14_getGlobalFault(max14_t *dout)
{
	dout->coms_error = MAX14_NOERROR;
	return max14_readReg(dout, MAX14_R_GLOBALERR);
}

enum MAX14_ERROR max14_setOutput(max14_t *dout, uint8_t val)
{
	dout->coms_error = MAX14_NOERROR;
	max14_writeReg(dout, MAX14_R_SETOUT,val);
	return dout->coms_error;
}

enum MAX14_ERROR max14_setWireBreak(max14_t *dout, uint8_t wirebreakPins)
{
	dout->coms_error = MAX14_NOERROR;
	
	return dout->coms_error;
}