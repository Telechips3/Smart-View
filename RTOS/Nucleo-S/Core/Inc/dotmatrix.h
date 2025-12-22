/* dotmatrix.h */
#ifndef __DOTMATRIX_H
#define __DOTMATRIX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void dotmatrix_init(void);
void dotmatrix_update(void);
void dotmatrix_process_input(uint8_t rx_data);
void dotmatrix_set_adb_pattern(int16_t bbox_x, uint16_t bbox_w);
#ifdef __cplusplus
}
#endif

#endif
