/*
 * board.h
 *
 *  Created on:
 *      Author: 86133
 */

#ifndef BOARD_H_
#define BOARD_H_

#include "F2837xS_Cla_typedefs.h"// F2806x CLA Type definitions
#include "F2837xS_device.h"      // F2806x Headerfile Include File
#include "F2837xS_Examples.h"    // F2806x Examples Include File
#include "main.h"
/*GPIO define
TEST_MODE_GPIO:GPIO53

*/

void boardInit(void);
void boardApp(void);



#endif /* DRIVERS_MCU_TMS320F28062_INC_BOARD_H_ */
