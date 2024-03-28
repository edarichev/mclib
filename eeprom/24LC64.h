// misc draft functions
#ifndef _24LC64_H_INCLUDED_
#define _24LC64_H_INCLUDED_

#if defined (PLATFORM_STM32_HAL)

#define EEPROM_24LC64_PATTERN_SIZE       16
#define EEPROM_24LC64_DEVICE_ADDR        0xA0
#define EEPROM_24LC64_MAX_ADDR           (0x1FFF + 1)

/**
 * A memory test for 24LC64
 * 
 * @param testChar test character for pattern, for example 0xAA or 0x55
 * @returns true if test passed
 */
bool memtest_24LC64(I2C_HandleTypeDef *hi2c, uint8_t testChar)
{
	char wmsgPattern[EEPROM_24LC64_PATTERN_SIZE];
	memset(wmsgPattern, testChar, EEPROM_24LC64_PATTERN_SIZE);
	char buf[EEPROM_24LC64_PATTERN_SIZE];
	for (int addr = 0; 
			addr < EEPROM_24LC64_MAX_ADDR - EEPROM_24LC64_PATTERN_SIZE; 
			addr += EEPROM_24LC64_PATTERN_SIZE) {
		// random access
		HAL_I2C_Mem_Write(hi2c, EEPROM_24LC64_DEVICE_ADDR, addr, I2C_MEMADD_SIZE_16BIT,
				(uint8_t*) wmsgPattern, EEPROM_24LC64_PATTERN_SIZE, HAL_MAX_DELAY);
		while (HAL_I2C_IsDeviceReady(hi2c, EEPROM_24LC64_DEVICE_ADDR, 1, HAL_MAX_DELAY)
				!= HAL_OK)
			;
		memset(buf, 0, EEPROM_24LC64_PATTERN_SIZE);
		HAL_I2C_Mem_Read(hi2c, EEPROM_24LC64_DEVICE_ADDR, addr, I2C_MEMADD_SIZE_16BIT,
				(uint8_t*) buf, EEPROM_24LC64_PATTERN_SIZE, HAL_MAX_DELAY);
		if (memcmp(wmsgPattern, buf, EEPROM_24LC64_PATTERN_SIZE) != 0)
			return false;
	}
	return true; // no errors -> true, test passed
}

#endif // PLATFORM_STM32_HAL

#endif // _24LC64_H_INCLUDED_
