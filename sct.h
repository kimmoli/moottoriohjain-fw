/*
 * sct.h
 *
 *  Created on: 27.6.2013
 *      Author: kimmo.lindholm
 */

#ifndef SCT_H_
#define SCT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "LPC8xx.h"

void SCT_IRQHandler(void);
void initSct();

#ifdef __cplusplus
}
#endif



#endif /* SCT_H_ */
