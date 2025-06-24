/*
 * mtf01WrapperC.h
 *
 *  Created on: Jun 23, 2025
 *      Author: louis
 */

#ifndef INC_SENSORS_MTF01WRAPPERC_H_
#define INC_SENSORS_MTF01WRAPPERC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

#define MTF01_FRAME_SIZE 255

void mtf01CopyFrame(const size_t dmaPos);


#ifdef __cplusplus
}
#endif


#endif /* INC_SENSORS_MTF01WRAPPERC_H_ */
