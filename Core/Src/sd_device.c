/*
 * sd_device.c
 *
 *  Created on: 3 nov. 2021
 *      Author: JoseCarlos
 */

/*INCLUDES******************************************************************************/
#include "sd_device.h"
#include "fatfs.h"

uint8_t init_sd(char *base_dir)
{
	  FATFS fs;
	  if(f_mount(&fs, base_dir, 1) != FR_OK)
	  	  return 0;

	  return 1;
}

int scan_directory(char * dir_name, file_linked_list_t ** file_list)
{
	file_linked_list_t *aux = (*file_list);
	file_linked_list_t *aux_prev;

	DIR dir;
	FILINFO fno;
	int size_dir=0;
	char buffer[3];
	uint8_t i;

	if(f_opendir(&dir, dir_name) != FR_OK)
		return -1;


	while(aux != NULL)
	{
		(*file_list) = aux->next;
		vPortFree(aux);
		aux = (*file_list);
	}

	while ((f_readdir(&dir, &fno)) == FR_OK)
	{
		if(fno.fname[0] == '\0')
			break;

		if((*file_list) == NULL)
		{
			(*file_list) = aux = (file_linked_list_t *)pvPortMalloc(sizeof(file_linked_list_t));
			aux_prev = NULL;
		}
		else
		{
			aux->next = (file_linked_list_t *)pvPortMalloc(sizeof(file_linked_list_t));
			aux_prev= aux;
			aux = aux->next;
		}
		strcpy(aux->name, fno.fname);
		aux->next = NULL;
		aux->prev = aux_prev;
		size_dir++;

		for(i=0; i<MAX_LENGH_NAME_FILE; i++)
		{
			if(aux->name[i] == '.')
				break;
		}

		if(i<MAX_LENGH_NAME_FILE)
		{
			buffer[0] = aux->name[i+1];
			buffer[1] = aux->name[i+2];
			buffer[2] = aux->name[i+3];

			if(strcmp("txt", buffer) == 0)
				aux->type = txt;
			else if(strcmp("bmp", buffer) == 0)
				aux->type = bmp;
			else if(strcmp("wav", buffer) == 0)
				aux->type = wav;
			else
				aux->type = unkown;
		}
		else
		{
			strcat(aux->name, "/");
			aux->type = folder;
		}

	}

	f_closedir(&dir);


	aux->next = (*file_list);
	aux_prev = aux;
	aux = aux->next;
	aux->prev = aux_prev;

	return size_dir;
}

void delete_file_list(file_linked_list_t **file_list, uint8_t size)
{

	file_linked_list_t *aux = (*file_list);
	for(uint8_t i=0; i<size; i++)
	{
		(*file_list) = aux->next;
		vPortFree(aux);
		aux = (*file_list);
	}
}

