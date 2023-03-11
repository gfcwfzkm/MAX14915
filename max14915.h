/*
 * max14915.h
 *
 * Created: 19.01.2021 13:19:16
 *  Author: gfcwfzkm
 */ 


#ifndef MAX14915_H_
#define MAX14915_H_

#include <inttypes.h>

#define MAX14_R_SETOUT		0x00 // RW
#define MAX14_R_SETFLED		0x01 // RW Fault LED Register
#define MAX14_R_SETSLED		0x02 // RW Status LED Register
#define MAX14_R_INTERRUPT	0x03 // R Interrupt Register
#define MAX14_R_INTERRUPT_COMERR		0x80 // WDT of SPI or SYNCH-pin detected
#define MAX14_R_INTERRUPT_SUPPLYERR		0x40 // Logic-OR of the Voltage Error Registers
#define MAX14_R_INTERRUPT_THRMERR		0x20 // Chip enters thermal shutdown
#define MAX14_R_INTERRUPT_SHTVDDFAULT	0x10 // Short to VDD on any OUT detected
#define MAX14_R_INTERRUPT_OWONFAULT		0x08 // Opem-Wire fault in On-state detected
#define MAX14_R_INTERRUPT_OWOFFFAULT	0x04 // Open-Wire fault in Off-state detected
#define MAX14_R_INTERRUPT_CURRLIMFAULT	0x02 // Over-Current-Limit fault detected
#define MAX14_R_INTERRUPT_OVERLDFAULT	0x01 // Over-Load fault detected
#define MAX14_R_OVLCHF		0x04 // R Thermal Overload Fault Register
#define MAX14_R_CURRLIM		0x05 // R Current Limit Fault Register
#define MAX14_R_OWOFFCHF	0x06 // R Open Wire OFF-State Fault Register
#define MAX14_R_OWONCHF		0x07 // R Open Wire ON-State Fault Register
#define MAX14_R_SHTVDCHF	0x08 // R Short to VDD in OFF-State Fault register
#define MAX14_R_GLOBALERR	0x09 // R Global Error Register
#define MAX14_R_GLOBALERR_WDERR			0x80 // SPI Watchdog Timeout Detected
#define MAX14_R_GLOBALERR_SYNCHERR		0x40 // Synch Watchdog Timeout Detected
#define MAX14_R_GLOBALERR_THRMSHUTD		0x20 // Thermal Shutdown entered
#define MAX14_R_GLOBALERR_VDDUVLO		0x10 // VDD below UVLO_VDD
#define MAX14_R_GLOBALERR_VDDWARN		0x08 // VDD below VDD_WARN
#define MAX14_R_GLOBALERR_VDDNOTGOOD	0x04 // VDD below VDD_GOOD
#define MAX14_R_GLOBALERR_VA_UVLO		0x02 // VA under VA_UVLO
#define MAX14_R_GLOBALERR_VINT_UV		0x01 // Power-On-Reset Signal, Register contents lost
#define MAX14_R_OWOFFEN		0x0A // RW Enable Open Wire detection in OFF state
#define MAX14_R_OWONEN		0x0B // RW Enable Open Wire detection in ON state
#define MAX14_R_SHTVDDEN	0x0C // RW Enable Detection of a Short to VDD
#define MAX14_R_CONFIG1		0x0D // RW Config Register 1
#define MAX14_R_CONFIG1_LEDCURRLIM	0x80 // Mask FLEDs to signal current limiting on a channel
#define MAX14_R_CONFIG1_FLATCHEN	0x40 // Enable latching of diagonistic faults registers
#define MAX14_R_CONFIG1_FILTRLONG	0x20 // Long blanking time of diagnostic fault bits (8ms instead of 4ms)
#define MAX14_R_CONFIG1_FFILTEREN	0x10 // Enable blanking and filtering of diagnostic fault bits
#define MAX14_R_CONFIG1_FLEDSTRECH1	0x08 // Sets the minimum-on time of the FLEDs
#define MAX14_R_CONFIG1_FLEDSTRECH0	0x04 // Sets the minimum-on time of the fLEDs
#define MAX14_R_CONFIG1_SLEDSET		0x02 // Allowing the LEDs to be controlled by the SETSLED register bits
#define MAX14_R_CONFIG1_FLSEDSET	0x01 // Allowing the LEDs to be controlled by the SETFLED register bits
#define MAX14_R_CONFIG2		0x0E // RW Config Register 2
#define MAX14_R_CONFIG2_WDTO1		0x80 // WDTO bits enable and set the timeout for the SPI and SYNCH watchdog
#define MAX14_R_CONFIG2_WDTO0		0x40 // WDTO bits enable and set the timeout for the SPI and SYNCH watchdog
#define MAX14_R_CONFIG2_OWOFFCS1	0x20 // Set the pull-up current source for open-wire OFF detection
#define MAX14_R_CONFIG2_OWOFFCS0	0x10 // Set the pull-up current source for open-wire OFF detection
#define MAX14_R_CONFIG2_SHRTVDDTHR1	0x08 // Set the voltage threshold for short-to-vdd detection
#define MAX14_R_CONFIG2_SHRTVDDTHR0	0x04 // Set the voltage threshold for short-to-vdd detection
#define MAX14_R_CONFIG2_SYNCHWDEN	0x02 // Enables the Watchdog for the SYNCH Pin
#define MAX14_R_CONFIG2_VDDONTHR	0x01 // Sets VDD_GOOD Threshold to 16V instead of 9V
#define MAX14_R_MASK		0x0F // RW FAULT-Pin Mask Register (0 = signaled to fault)
#define MAX14_R_MASK_COMERRM		0x80 // Disable watchdog timeout being signaled on FAULT
#define MAX14_R_MASK_SUPPLYERRM		0x40 // Disable supply errors being signaled on FAULT
#define MAX14_R_MASK_VDDOKM			0x20 // Disable signaling of VDDNOTGOOD & VDDWARN on FAULT
#define MAX14_R_MASK_SHTVDDM		0x10 // Disable short-to-VDD errors being signaled on FAULT
#define MAX14_R_MASK_OWONM			0x08 // Disable Open-Wire-ON errors being signaled on FAULT
#define MAX14_R_MASK_OWOFFM			0x04 // Disable Open-Wire-OFF errors being signaled on FAULT
#define MAX14_R_MASK_CURRLIMM		0x02 // Disable over-current conditions being signaled on FAULT
#define MAX14_R_MASK_OVERLDM		0x01 // Disable overload faults being signaled on FAULT

enum MAX14_CRCEN{
	MAX14_NOCRC = 0,
	MAX14_CRCEN = 1
};

enum MAX14_DIAGNOSTIC{
	MAX14_SHORT_TO_VDD		= 0x20,
	MAX14_OPENWIRE_ON_F		= 0x10,
	MAX14_OPENWIRE_OFF_F	= 0x08,
	MAX14_CURRENT_LIMIT_F	= 0x04,
	MAX14_OVERLOAD_F		= 0x02,
	MAX14_GLOBAL_FAULT		= 0x01,
	MAX14_NO_DIAGNOSTIC_F	= 0
};

enum MAX14_GLOBALFAULT{
	MAX14_WATCHDOG_ERR		= MAX14_R_GLOBALERR_WDERR,
	MAX14_SYNCH_ERR			= MAX14_R_GLOBALERR_SYNCHERR,
	MAX14_THERMAL_SHUTDOWN	= MAX14_R_GLOBALERR_THRMSHUTD,
	MAX14_VDD_UVLO			= MAX14_R_GLOBALERR_VDDUVLO,
	MAX14_VDD_WARN			= MAX14_R_GLOBALERR_VDDWARN,
	MAX14_VDD_NOT_GOOD		= MAX14_R_GLOBALERR_VDDNOTGOOD,
	MAX14_VA_UVLO			= MAX14_R_GLOBALERR_VA_UVLO,
	MAX14_VINT_UV			= MAX14_R_GLOBALERR_VINT_UV
};

enum MAX14_ERROR{
	MAX14_NOERROR	= 0,
	MAX14_COMSERROR	= 1,
	MAX14_CRCERROR	= 2
};

typedef struct
{
	enum MAX14_CRCEN crc_en:1;			// CRC is enabled or not
	enum MAX14_ERROR coms_error;		// Make sure to check coms_error after each operation
	enum MAX14_DIAGNOSTIC diag;			// diagnostics is updated with each SPI transaction to the chip
	void *ioInterface;					// Pointer to the IO/Peripheral Interface library
	// Any return value by the IO interface functions have to return zero when successful or
	// non-zero when not successful.
	uint8_t (*startTransaction)(void*);	// Prepare the IO/Peripheral Interface for a transaction
	uint8_t (*transceiveBytes)(void*,	// Send and receive Bytes from the buffer (SPI only)
	uint8_t,		// Address of the PortExpander (8-Bit Address Format!) (ignored if zero),
	uint8_t*,		// Pointer to send buffer,
	uint16_t);		// Amount of bytes to send
	uint8_t (*endTransaction)(void*);	// Finish the transaction / Release IO/Peripheral
} max14_t;

/**
 * @brief Structure preparation
 * 
 * Sets up the \a max14_t structure and it's function pointers.
 * Beware, that the chip select needs to be handled by your SPI
 * library using the startTransaction and endTransaction functions
 * 
 * @param dout				Pointer to a \a max14_t structure
 * @param ioComs 			Optional void pointer used by your SPI library
 * @param startTransaction 	Function pointer to initiate the SPI transaction
 * @param transceiveBytes	Function pointer to send/receive SPI data
 * @param endTransaction 	Function pointer to end the SPI transaction
 */
void max14_initStruct(max14_t *dout, void *ioComs,
				uint8_t (*startTransaction)(void*),
				uint8_t (*transceiveBytes)(void*,uint8_t,uint8_t*,uint16_t),
				uint8_t (*endTransaction)(void*));

/**
 * @brief Initialise the high-side-switch chip
 * 
 * Checks for a Power-On-Reset (POR) and sets up the chip's registers properly.
 * If no POR encoured yet this function has been called, all registers
 * will be set to their respective POR value.
 * 
 * It also checks if the chip is responding by reading back
 * the configuration register. Returns an error otherwise.
 * 
 * @param dout 				Pointer to a \a max14_t structure
 * @param crcen 			Enabling/disabling CRC with each transaction
 * @return enum MAX14_ERROR Non-Zero if an error encountered. Check \a MAX14_ERROR
 */
enum MAX14_ERROR max14_init(max14_t *dout, enum MAX14_CRCEN crcen);

/**
 * @brief Reads from a register
 * 
 * Just reads the register. Performs CRC checks if enabled, which are
 * stored in dout->coms_error.
 * 
 * @param dout 		Pointer to a \a max14_t structure
 * @param regAddr	Register address to read
 * @return uint8_t	Register value from said address
 */
uint8_t max14_readReg(max14_t *dout, uint8_t regAddr);

/**
 * @brief Writes into a register
 * 
 * Writes a value into a register. Performs CRC checks if enabled,
 * which are stored in dout->coms_error.
 * 
 * @param dout 		Pointer to a \a max14_t structure
 * @param regAddr	Register address to write
 * @param val		Register value to write into said address
 */
void max14_writeReg(max14_t *dout, uint8_t regAddr, uint8_t val);

/**
 * @brief Reads the global fault register
 * 
 * Just reads and returns the global fault register as an enum \a MAX14_GLOBALFAULT.
 * 
 * @param dout 						Pointer to a \a max14_t structure
 * @return enum MAX14_GLOBALFAULT 	Enum containing possible faults & errors
 */
enum MAX14_GLOBALFAULT max14_getGlobalFault(max14_t *dout);

/**
 * @brief Sets the 8 outputs 
 * 
 * Directly sets the out high-side switches
 * 
 * @param dout 					Pointer to a \a max14_t structure
 * @param val 					Value to turn on/off
 * @return enum MAX14_ERROR 	Non-Zero if an error encountered. Check \a MAX14_ERROR
 */
enum MAX14_ERROR max14_setOutput(max14_t *dout, uint8_t val);

/**
 * @brief Sets up wire-break parameters
 * 
 * Configures the wire-break detection feature
 * ! Not yet fully implemented !
 * 
 * @param dout 				Pointer to a \a max14_t structure
 * @param wirebreakPins 	Pins to perform wire-break detection on
 * @return enum MAX14_ERROR Non-Zero if an error encountered. Check \a MAX14_ERROR
 */
//enum MAX14_ERROR max14_setWireBreak(max14_t *dout, uint8_t wirebreakPins);



#endif /* MAX14915_H_ */
