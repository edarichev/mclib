/**
 * spiclient.h
 * 
 * SPI Abstraction

 *  Created on: Mar 20, 2024
 *      Author: de
*/

#ifndef _SPICLIENT_H_INCLUDED_
#define _SPICLIENT_H_INCLUDED_

//////////////////////////// PLATFORM SPECIFIC /////////////////////////////////

#include <type_traits>

#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif

enum SPIConnectionMode : uint8_t
{
    Polling,
};

// Platform specific realization


#if defined(PLATFORM_STM32_HAL)

// Before including this file include the your platform-specific file,
// for example "stm32f4xx_hal.h" for STM32 F4 board

#endif // PLATFORM_STM32_HAL


#endif // _SPICLIENT_H_INCLUDED_
