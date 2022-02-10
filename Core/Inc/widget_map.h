/*
 * widget_map.h
 *
 *  Created on: Nov 3, 2021
 *      Author: JoseCarlos
 */

#ifndef INC_WIDGET_MAP_H_
#define INC_WIDGET_MAP_H_

#include "stdint.h"

/*
static const uint8_t wifi_widget[] = {
	0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x03, 0x00, 0x00, 0x02, 0x00, 0x00,
	0x03, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
	0x03, 0x00, 0x02, 0x00, 0x00, 0x01, 0x00,
	0x03, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00,
	0x03, 0x00, 0x02, 0x00, 0x01, 0x00, 0x01,
	0x03, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00,
	0x03, 0x00, 0x02, 0x00, 0x00, 0x01, 0x00,
	0x03, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
	0x00, 0x03, 0x00, 0x00, 0x02, 0x00, 0x00,
	0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
};
*/


static const uint8_t weather[]={
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00,
	0x00, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00,
	0xFF, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
	0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
	0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00,
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00,
};

static const uint8_t bluetooth_icon[]={
	0x00, 0x00, 0x01, 0x01, 0x00,
	0x01, 0x00, 0x01, 0x00, 0x01,
	0x00, 0x01, 0x01, 0x00, 0x01,
	0x00, 0x00, 0x01, 0x01, 0x00,
	0x00, 0x01, 0x01, 0x00, 0x01,
	0x01, 0x00, 0x01, 0x00, 0x01,
	0x00, 0x00, 0x01, 0x01, 0x00,
};



#endif /* INC_WIDGET_MAP_H_ */