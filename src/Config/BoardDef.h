/*
 * BoardDef.h
 *
 *  Created on: 30 Jun 2019
 *      Author: David
 */

#ifndef SRC_CONFIG_BOARDDEF_H_
#define SRC_CONFIG_BOARDDEF_H_

#include <Duet3Common.h>								// this file is in the CANlib project because both main and expansion boards need it
#include <RRF3Common.h>

#if defined(EXP3HC)
# include "EXP3HC.h"
#elif defined(TOOL1LC)
# include "TOOL1LC.h"
#elif defined(EXP1XD)
# include "EXP1XD.h"
#elif defined(EXP1HCLv1_0)
# include "EXP1HCLv1_0.h"
#elif defined(SAMMYC21)
# include "SAMMYC21.h"
#elif defined(ATECM)
# include "ATECM.h"
#elif defined(ATEIO)
# include "ATEIO.h"
#elif defined(RPI_PICO)
# include "RPi_Pico.h"
#elif defined(FLY36RRF)
# include "Fly36_RRF.h"
#elif defined(FLYSB2040v1_0)
# include "FlySB2040v1_0.h"
#elif defined(MKSTHR36v1_0)
# include "MKSTHR36v1_0.h"
#elif defined(M23CL)
# include "M23CL.h"
#else
# error Board type not defined
#endif

// Default board features
#ifndef DIFFERENTIAL_STEPPER_OUTPUTS
# define DIFFERENTIAL_STEPPER_OUTPUTS	0
#endif

#ifndef USE_TC_FOR_STEP
# define USE_TC_FOR_STEP				0
#endif

#ifndef SUPPORT_CLOSED_LOOP
# define SUPPORT_CLOSED_LOOP			0
#endif

#if !SUPPORT_DRIVERS
# define HAS_SMART_DRIVERS				0
# define SUPPORT_TMC22xx				0
# define SUPPORT_TMC51xx				0
# define SUPPORT_TMC2160				0
# define SUPPORT_SLOW_DRIVERS			0
constexpr size_t NumDrivers = 0;
#endif

#if !defined(SUPPORT_BME280)
# define SUPPORT_BME280					(SUPPORT_SPI_SENSORS)
#endif

#if !defined(SUPPORT_LIS3DH)
# define SUPPORT_LIS3DH					0
#endif

#endif /* SRC_CONFIG_BOARDDEF_H_ */
