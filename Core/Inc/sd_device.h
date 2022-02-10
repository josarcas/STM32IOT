/*
 * sd_device.h
 *
 *  Created on: 3 nov. 2021
 *      Author: JoseCarlos
 */

#ifndef INC_SD_DEVICE_H_
#define INC_SD_DEVICE_H_

/*INCLUDES*******************************************************************************/
#include "stdint.h"
#include "string.h"
#include "stdbool.h"

/*DEFINES********************************************************************************/
#define MAX_LENGH_NAME_FILE 50

/*
#ifndef	TYPE_FILE
#define FILE_TYPE_UNKOWN	0
#define FILE_TYPE_FOLDER	1
#define FILE_TYPE_TXT		10
#define FILE_TYPE_BMP		11
#define FILE_TYPE_WAV		12
#endif
*/

typedef enum{
	unkown,
	folder,
	txt,
	bmp,
	wav
}type_file_t;


/*TYPEDEF********************************************************************************/
struct file_linked_list{
#ifndef MAX_LENGH_NAME_FILE
#define MAX_LENGH_NAME_FILE 256
#endif
	char name[MAX_LENGH_NAME_FILE];
	type_file_t type;
	struct file_linked_list *next;
	struct file_linked_list *prev;
};
typedef struct file_linked_list file_linked_list_t;


/*PROTOTYPES*****************************************************************************/
uint8_t init_sd(char *base_dir);
int scan_directory(char * dir_name, file_linked_list_t ** file_list);
void delete_file_list(file_linked_list_t **file_list, uint8_t size);


#endif /* INC_SD_DEVICE_H_ */
