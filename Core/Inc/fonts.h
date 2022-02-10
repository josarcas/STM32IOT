/*
 * fonts.h
 *
 *  Created on: 25 oct. 2021
 *      Author: JoseCarlos
 */

#ifndef MAIN_FONTS_H_
#define MAIN_FONTS_H_

#include "stdint.h"

extern const uint8_t font8_table[];
extern const uint8_t font12_table[];
extern const uint8_t font16_table[];
extern const uint8_t font20_table[];
extern const uint8_t font24_table[];

typedef enum{
	font8,
	font12,
	font16,
	font20,
	font24
}font_t;



#endif /* MAIN_FONTS_H_ */
