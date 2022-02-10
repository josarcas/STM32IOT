/*
 * wifi_driver.c
 *
 *  Created on: Oct 31, 2021
 *      Author: JoseCarlos
 */

#ifndef SRC_WIFI_DRIVER_C_
#define SRC_WIFI_DRIVER_C_

/*INCLUDES*******************************************************************/
#include "stdarg.h"

#include "wifi_driver.h"

//FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

/*GLOBAL VARIABLES***********************************************************/
static char encoding_table[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
                                'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
                                'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
                                'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
                                'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
                                'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
                                'w', 'x', 'y', 'z', '0', '1', '2', '3',
                                '4', '5', '6', '7', '8', '9', '+', '/'};

static int mod_table[] = {0, 2, 1};
//EXTERN VARIABLES-----------------------------------------------------
extern UART_HandleTypeDef huart1;

/*PROTOTYPES*****************************************************************/
wifi_status write_cmd_wifi(char *cmd, char *data, uint32_t timeout);
wifi_status write_data_wifi(char *data, uint32_t timeout);
wifi_status write_data_size_wifi(char *data, uint16_t size, uint32_t timeout);
wifi_status write_direct_wifi(char *data, uint16_t size, uint32_t timeout);
wifi_status read_wifi(uint32_t timeout);
wifi_status wait_ready_wifi(uint32_t timeout);
wifi_status error_wifi(wifi_status error_code);
wifi_status error_check_wifi(wifi_status status, char *response);
wifi_status response_check_wifi(wifi_status status);
//wifi_status write_read_transport_data(char *data, uint32_t timeout);
wifi_status write_transport_packet_size_wifi(uint16_t size, uint32_t timeout);
wifi_status write_transport_packet_wifi(uint32_t timeout, const char *format, ...);

wifi_scan_ap_t * create_scan_node(wifi_scan_ap_t **list);

char *base64_encode(const unsigned char *data,size_t input_length,
		size_t *output_length);

void str_add(char *str1, char *str2, uint16_t size);
void str_copy(char *str1, char str2);
int str_find(char *str1, char *str2, int index);
void str_cat(char *str1, char *str2, uint16_t index);
//void str_get_time(data_time_t *time, char *str);

/*FUNCTIONS*******************************************************************/
/*
 * configura los puertos y el spi por defecto integrado en el kit de desarrollo
 */
extern SPI_HandleTypeDef hspi3;
void set_default_wifi()
{
	wifi._hspi = &hspi3;

	wifi.RST_PORT = GPIOE;
	wifi.CS_PORT = GPIOE;
	wifi.WKP_PORT = GPIOB;
	wifi.RDY_PORT = GPIOE;

	wifi.RST_PIN = GPIO_PIN_8;
	wifi.CS_PIN = GPIO_PIN_0;
	wifi.WKP_PIN = GPIO_PIN_13;
	wifi.RDY_PIN = GPIO_PIN_1;

}

/*
 * Inicializa el modulo esWi-Fi
 * timeout : tiempo maximo para la operacion
 */
wifi_status init_wifi(uint32_t timeout)
{
	wakeup_wifi();
	reset_wifi();
	vTaskDelay(50);

	return error_check_wifi(read_wifi(timeout), "\025\025\r\n> ");
}

/*
 * Reinicia el modulo WiFi
 */
void reset_wifi()
{
	vTaskDelay(1000);
	HAL_GPIO_WritePin(wifi.RST_PORT, wifi.RST_PIN, 0);
	vTaskDelay(1000);
	HAL_GPIO_WritePin(wifi.RST_PORT, wifi.RST_PIN, 1);
}

/*
 * Despierta el modulo WiFi
 */
void wakeup_wifi()
{
	HAL_GPIO_WritePin(wifi.WKP_PORT, wifi.WKP_PIN, 1);
}

/*
 * Detiene el modulo WiFi
 */
void stop_module_wifi()
{
	HAL_GPIO_WritePin(wifi.WKP_PORT, wifi.WKP_PIN, 0);
}

/*
 * Escribe al modulo Wi-Fi y espera la respuesta
 *     cmd : comando para el modulo
 *    data : datos del comando usado(si se envia \0 solo manda el comando)
 * timeout : tiempo maximo de la operacion
 */
wifi_status write_cmd_wifi(char *cmd, char *data, uint32_t timeout)
{

	if(data[0] == '\0')
		sprintf(wifi.buffer_tx,"%s\r", cmd);
	else
		sprintf(wifi.buffer_tx,"%s=%s\r", cmd, data);

	uint16_t len = strlen(wifi.buffer_tx);

	if(len&0x01)
	{
		wifi.buffer_tx[len] = '\n';
		len++;
	}

	if(wait_ready_wifi(timeout) != Timeout_OK)
		return Timeout_ERROR;

	HAL_GPIO_WritePin(wifi.CS_PORT, wifi.CS_PIN, 0);
	vTaskDelay(1);

	if(HAL_SPI_Transmit(wifi._hspi, (uint8_t *)wifi.buffer_tx,
			len/2, timeout) != HAL_OK)
		return error_wifi(SPI_ERROR);

	HAL_GPIO_WritePin(wifi.CS_PORT, wifi.CS_PIN, 1);


	return read_wifi(timeout);
}

/*
 * Escribe solo datos al modulo
 *    data : datos a escribir
 * timeout : tiempo maximo de operacion
 */
wifi_status write_data_wifi(char *data, uint32_t timeout)
{
	sprintf(wifi.buffer_tx, "%s", data);

	uint16_t len = strlen(wifi.buffer_tx);

	if(len&0x01)
	{
		wifi.buffer_tx[len] = 0x15;
		len++;
	}

	if(wait_ready_wifi(timeout) != Timeout_OK)
		return Timeout_ERROR;

	HAL_GPIO_WritePin(wifi.CS_PORT, wifi.CS_PIN, 0);
	vTaskDelay(1);

	if(HAL_SPI_Transmit(wifi._hspi, (uint8_t *)wifi.buffer_tx,
			len/2, timeout) != HAL_OK)
		return error_wifi(SPI_ERROR);

	HAL_GPIO_WritePin(wifi.CS_PORT, wifi.CS_PIN, 1);


	return read_wifi(timeout);
}

wifi_status write_data_size_wifi(char *data, uint16_t size, uint32_t timeout)
{
	//sprintf(wifi.buffer_tx, "%s", data);

	//uint16_t len = strlen(wifi.buffer_tx);

	for(uint16_t i=0; i<size; i++)
		wifi.buffer_tx[i] = data[i];


	if(size&0x01)
	{
		wifi.buffer_tx[size] = 0x15;
		size++;
	}

	if(wait_ready_wifi(timeout) != Timeout_OK)
		return Timeout_ERROR;

	HAL_GPIO_WritePin(wifi.CS_PORT, wifi.CS_PIN, 0);
	vTaskDelay(1);

	if(HAL_SPI_Transmit(wifi._hspi, (uint8_t *)wifi.buffer_tx,
			size/2, timeout) != HAL_OK)
		return error_wifi(SPI_ERROR);

	HAL_GPIO_WritePin(wifi.CS_PORT, wifi.CS_PIN, 1);


	return read_wifi(timeout);
}

wifi_status write_direct_wifi(char *buffer, uint16_t size, uint32_t timeout)
{

	if(wait_ready_wifi(timeout) != Timeout_OK)
		return Timeout_ERROR;

	HAL_GPIO_WritePin(wifi.CS_PORT, wifi.CS_PIN, 0);
	vTaskDelay(1);

	if(HAL_SPI_Transmit(wifi._hspi, (uint8_t *)buffer,
			size/2, timeout) != HAL_OK)
		return error_wifi(SPI_ERROR);

	HAL_GPIO_WritePin(wifi.CS_PORT, wifi.CS_PIN, 1);

	return read_wifi(timeout);
}

/*
 * Lee los datos enviados por el modulo y los guarda en WiFi.buffer_rx
 * timeout : tiempo maximo de la operacion
 */
wifi_status read_wifi(uint32_t timeout)
{
	uint8_t buffer[2];
	uint16_t i = 0;

	memset(wifi.buffer_rx, '\0', MAX_RX_BUFFER_SIZE);

	if(wait_ready_wifi(timeout) != Timeout_OK)
		return Timeout_ERROR;

	HAL_GPIO_WritePin(wifi.CS_PORT, wifi.CS_PIN, 0);
	vTaskDelay(1);

	while(HAL_GPIO_ReadPin(wifi.RDY_PORT, wifi.RDY_PIN) == GPIO_PIN_SET)
	{
		if(HAL_SPI_Receive(wifi._hspi, buffer, 1, timeout) != HAL_OK)
			return SPI_ERROR;
		if(i<MAX_RX_BUFFER_SIZE)
		{
			wifi.buffer_rx[i] = buffer[0];
			i++;
			wifi.buffer_rx[i] = buffer[1];
			i++;
		}
		else
		{
			return OVER_FLOW_BUFFER;

		}
	}
	HAL_GPIO_WritePin(wifi.CS_PORT, wifi.CS_PIN, 1);
	if(i == 0)
		return WiFi_ERROR;
	return WiFi_OK;

}

/*
 * Espera a que el modulo este listo para recibir o transmitir datos
 * timeout : tiempo maximo de operacion
 */
wifi_status wait_ready_wifi(uint32_t timeout)
{
	while (HAL_GPIO_ReadPin(wifi.RDY_PORT, wifi.RDY_PIN) == GPIO_PIN_RESET)
	{
        if((timeout--)==0)
		{
		  return Timeout_ERROR;
		}
        vTaskDelay(1);
	}

	return Timeout_OK;

}

/*
 * Gestiona los errores
 */
wifi_status error_wifi(wifi_status error_code)
{
	HAL_GPIO_WritePin(wifi.CS_PORT, wifi.CS_PIN, 1);
	return error_code;
}

wifi_status error_check_wifi(wifi_status status, char *response)
{
	if(status == WiFi_OK)
	{
		for(uint16_t i=0; i<strlen(response); i++)
		{
			if(wifi.buffer_rx[i] != response[i])
			{
				return Operation_ERROR;
				//break;
			}
		}
	}

	return status;
}

wifi_status response_check_wifi(wifi_status status)
{
	return error_check_wifi(status, "\r\n\r\nOK\r\n> ");
}

/*
wifi_status write_read_transport_data(char *data, uint32_t timeout)
{
	uint16_t size = strlen(data);
	char buffer[5];

	sprintf("")

}
*/

wifi_status write_transport_packet_size_wifi(uint16_t size, uint32_t timeout)
{
	char buffer[6];

	sprintf(buffer, "%d", size);
	return response_check_wifi(write_cmd_wifi(SET_WRITE_TRANSPORT_PACKET_SIZE, buffer, timeout));
}

#define MAX_ARGS_SIZE		15
wifi_status write_transport_packet_wifi(uint32_t timeout, const char *format, ...)
{
	va_list args;

	char buffer[MAX_ARGS_SIZE];
	uint16_t len=3;

	memset(wifi.buffer_tx, '\0', MAX_TX_BUFFER_SIZE);
	memset(buffer, '\0', MAX_ARGS_SIZE);

	strcat(wifi.buffer_tx, "S0\r");

	va_start(args, format);

	while(*format != '\0')
	{
		if(*format == '%')
		{
			format++;
            switch (*format)
            {
                case 'd':
                {
                    int i = va_arg(args, int);
                    sprintf(buffer, "%d", i);
                    strcat(wifi.buffer_tx, buffer);
                    len += strlen(buffer);
                }
                break;

                case 'c':
                {
                    int i = va_arg(args, int);
                    wifi.buffer_tx[len] = i;
                    len++;
                }
                break;

                /*
                case 'f':
                {
                    double d = va_arg(args, double);
                    sprintf(buffer, "%f", d);
                    strcat(wifi.buffer_tx, buffer);
                    aux += strlen(buffer);
                }
                break;
                */

                case 's':
                {
                	char *ch = va_arg(args, char *);
                	strcat(wifi.buffer_tx, ch);
                	len += strlen(ch);
				}
                break;
                default:
                break;
            }
		}

		else
		{
        	wifi.buffer_tx[len] = *format;
        	len++;
		}

		format++;
	}

	va_end(args);

	wifi_status status;
	//uint16_t len = strlen(wifi.buffer_tx);

	//status = write_transport_packet_size_wifi(len-3, timeout);
	//if(status != WiFi_OK) return status;

	sprintf(buffer,"%s=%d\r", SET_WRITE_TRANSPORT_PACKET_SIZE, len-3);
	if(strlen(buffer)&0x01)
		buffer[strlen(buffer)] = '\n';

	status = write_direct_wifi(buffer, strlen(buffer), timeout);
	if(status != WiFi_OK) return status;


	if(len&0x01)
	{
		wifi.buffer_tx[len] = 0x15;
		len++;
	}

	if(wait_ready_wifi(timeout) != Timeout_OK)
		return Timeout_ERROR;

	HAL_GPIO_WritePin(wifi.CS_PORT, wifi.CS_PIN, 0);
	vTaskDelay(1);

	if(HAL_SPI_Transmit(wifi._hspi, (uint8_t *)wifi.buffer_tx,
			len/2, timeout) != HAL_OK)
		return error_wifi(SPI_ERROR);

	HAL_GPIO_WritePin(wifi.CS_PORT, wifi.CS_PIN, 1);


	return read_wifi(timeout);

}


/*
 * Codifica datos tipo char a base 64
 * 	   	    data : datos para codificar
 *  input_length : numero de datos para codificar
 * output_length : numero de datos codificados
 *
 * 		  return : datos codificados en base 64
 */
char *base64_encode(const unsigned char *data, size_t input_length,
		size_t *output_length)
{

    *output_length = 4 * ((input_length + 2) / 3);

    char *encoded_data = pvPortMalloc(*output_length);
    if (encoded_data == NULL) return NULL;

    for (int i = 0, j = 0; i < input_length;)
    {

        uint32_t octet_a = i < input_length ? (unsigned char)data[i++] : 0;
        uint32_t octet_b = i < input_length ? (unsigned char)data[i++] : 0;
        uint32_t octet_c = i < input_length ? (unsigned char)data[i++] : 0;

        uint32_t triple = (octet_a << 0x10) + (octet_b << 0x08) + octet_c;

        encoded_data[j++] = encoding_table[(triple >> 3 * 6) & 0x3F];
        encoded_data[j++] = encoding_table[(triple >> 2 * 6) & 0x3F];
        encoded_data[j++] = encoding_table[(triple >> 1 * 6) & 0x3F];
        encoded_data[j++] = encoding_table[(triple >> 0 * 6) & 0x3F];
    }

    for (int i = 0; i < mod_table[input_length % 3]; i++)
        encoded_data[*output_length - 1 - i] = '=';

    return encoded_data;
}

/*
 * Copia una cadena a otra
 * str1 : Cadena a la que se va a copiar
 * str2 : Cadena copiada
 * size : datos para copiar
 */
void str_add(char *str1, char *str2, uint16_t size)
{
	uint16_t i=0;

	while(str1[i] != '\0')
		i++;
	for(uint16_t j=0; j<size; j++)
	{
		str1[i]=str2[j];
		i++;
	}

	str1[i] = '\0';
}

void str_copy(char *str1, char str2)
{
	uint16_t i=0;
	while(str1[i] != '\0')
		i++;
	str1[i] = str2;
	str1[i+1] = '\0';
}

int str_find(char *str1, char *str2, int index)
{
	int i, j, aux;

	for(i=index; i<strlen(str1); i++)
	{
		aux = i;
		for(j=0; j<strlen(str2); j++)
		{
			if(str1[i] == str2[j])
			{
				i++;
			}
			else{
				i = aux;
				break;
			}
		}

		if(j == strlen(str2))
			return aux;
	}

	return -1;
}

void str_cat(char *str1, char *str2, uint16_t index)
{
	//char aux;
	uint16_t size = strlen(str1)+ strlen(str2);
	uint16_t aux_s = strlen(str1);
	uint16_t i;

	for(i=size; i>index; i--)
	{
		str1[i] = str1[aux_s];
		aux_s--;
	}

	for(i=0; i<strlen(str2); i++)
	{
		str1[index+i] = str2[i];
	}

	str1[size+1] = '\0';
}

/*
void str_get_time(data_time_t *time, char *str)
{
	char buffer[5];
}
*/

//SOFT AP---------------------------------------------------------------------
/*
 * Conexion a una red wifi
 *  _ssid : nombre de la red
 *  _pass : contraseña de la red
 *   mode : modo de seguridad de conexion
 *timeout : tiempo maximo de la operacion
 */
wifi_status connect_wifi(char *_ssid, char *_pass, security_mode mode,
		uint32_t timeout)
{
	wifi_status status;
	char buffer;

	status = response_check_wifi(write_cmd_wifi(SET_NETWORK_SSID, _ssid, timeout));
	if(status != WiFi_OK) return status;
	status = response_check_wifi(write_cmd_wifi(SET_NETWORK_PASSPHRASE, _pass, timeout));
	if(status != WiFi_OK) return status;

	switch(mode)
	{
	case Open:
		buffer = '0';
		break;
	case WEP:
		buffer = '1';
		break;
	case WPA:
		buffer = '2';
		break;
	case WPA2:
		buffer = '3';
		break;
	case WPAandWPA2:
		buffer = '4';
		break;
	case WPA2_TKIP:
		buffer = '5';
		break;
	}

	status = response_check_wifi(write_cmd_wifi(SET_NETWORK_SECURITY_TYPE, &buffer , timeout));
	if(status != WiFi_OK) return status;
	return error_check_wifi(write_cmd_wifi(JOIN_A_NETWORK, '\0', timeout), "\r\n[JOIN   ]");
}

/*
 * Se conecta a un socket
 * timeout : tiempo maximo de la operacion
 */
wifi_status set_socket_wifi(uint32_t timeout)
{
	return write_cmd_wifi(SET_DISPLAY_COMMUNICATION_SOCKET, "0" , timeout);
}

/*
 * Añade un elemento a la lista de redes escaneadas
 *   list : lista de redes
 *
 * return : ultimo miembro agragado
 */
wifi_scan_ap_t * create_scan_node(wifi_scan_ap_t **list)
{
	wifi_scan_ap_t *aux = (*list);

	if(aux == NULL)
		(*list) = aux = (wifi_scan_ap_t*)pvPortMalloc(sizeof(wifi_scan_ap_t));
	else
	{
		while(aux->next != NULL)
			aux = aux->next;

		aux->next = (wifi_scan_ap_t*)pvPortMalloc(sizeof(wifi_scan_ap_t));
		aux = aux->next;
	}

	aux->next = NULL;
	return aux;
}

/*
 * Escanea las redes wifi disponibles
 * 	  list : lista para agregar redes
 * timeout : tiempo maximo de la operacion
 */
wifi_status scan_wifi_soft_ap(wifi_scan_ap_t **list, uint32_t timeout)
{
	wifi_status status;

	status = error_check_wifi(write_cmd_wifi(SCAN_FOR_NETWORK_ACCESS_POINTS, '\0', timeout), "\r\n#001,");
	if(status != WiFi_OK) return status;

	uint16_t i=0;
	uint16_t j;
	wifi_scan_ap_t * aux;
	char buffer_aux[4];
	memset(buffer_aux, '\0', 4);


	while(i<MAX_RX_BUFFER_SIZE)
	{
		if(wifi.buffer_rx[i] == '"')
		{
			i++;
			aux = create_scan_node(list);
			j=0;

			while(wifi.buffer_rx[i] != '"')
			{
				aux->SSID[j] = wifi.buffer_rx[i];
				j++;
				i++;
			}
			i+=2;

			for(j=0; j<6; j++)
			{
				buffer_aux[0] = wifi.buffer_rx[i];
				buffer_aux[1] = wifi.buffer_rx[i+1];
				i+=3;
				aux->MAC[j] = atoi(buffer_aux);
			}

			i++;
			j=0;
			memset(buffer_aux, '\0', 4);

			while(wifi.buffer_rx[i] != ',')
			{
				buffer_aux[j] = wifi.buffer_rx[i];
				i++;
				j++;
			}

			aux->RSSI = atoi(buffer_aux);

		}
		else
			i++;
	}

	return WiFi_OK;
}

/*
 * Elimina la lista de redes escaneadas
 * list : lista de redes
 */
void delete_scan_list(wifi_scan_ap_t **list)
{
	wifi_scan_ap_t *aux = (*list);

	while(aux != NULL)
	{
		(*list) = aux->next;
		vPortFree(aux);
		aux = (*list);
	}
}

/*
 * Intensidad de la señal de wifi conectada
 * timeout : tiempo maximo de la operacion
 *
 *  return : intensidad de la señal, si no esta conectado retorna 0
 */
int8_t rssi_soft_ap(uint32_t timeout)
{
	wifi_status status;

	status = write_cmd_wifi(GET_RSSS_OF_ASSOCIATED_NETWORK_ACCESS_POINT, '\0', timeout);
	if(status != WiFi_OK) return status;
	char buffer_aux[4];
	memset(buffer_aux, '\0', 4);

	for(uint16_t i=2; i<MAX_RX_BUFFER_SIZE; i++)
	{
		if(wifi.buffer_rx[i]=='\r')
			break;
		buffer_aux[i-2] = wifi.buffer_rx[i];
	}
	return atoi(buffer_aux);
}

//MQTT-----------------------------------------------------------------------
/*
 * Configura un broker MQTT
 *   _host : Direccion del host
 *   _port : puerto del host
 *     sec : tipo de seguridad
 * timeout : tiempo maximo de operacion
 */
wifi_status set_broker_mqtt(char *_host, char *_port, mqtt_security sec,
		uint32_t timeout)
{
	wifi_status status;
	char type_sec[3] = {'2',',', '\0'};

	status = response_check_wifi(write_cmd_wifi(SET_TRANSPORT_PROTOCOL, "4", timeout));
	if(status != WiFi_OK) return status;
	status = write_cmd_wifi(DNS_LOOKUP, _host, timeout);
	if(status != WiFi_OK) return status;
	status = response_check_wifi(write_cmd_wifi(SET_TRANSPORT_REMOTE_PORT_NUMBER, _port, timeout));
	if(status != WiFi_OK) return status;

	switch (sec)
	{
	case none:
		type_sec[2] = '0';
		break;
	case user_name_password:
		type_sec[2] = '1';
		break;
	case ca_cert_key:
		type_sec[2] = '2';
		break;
	}

	return response_check_wifi(write_cmd_wifi(SHOW_MQTT_ATTRIBUTES, type_sec, timeout));
}

/*
 * Configura las credenciales para conectarse al broker MQTT (User_Name_Password)
 *      _user : usuario para el host
 *      _pass : contraseÃ±a del usuario
 * _client_id : identificador del dispositivo
 *    timeout : tiempo maximo de la operacion
 */
wifi_status set_credentials_mqtt(char *_user, char *_pass, char *_client_id,
		uint32_t timeout)
{
	char buffer[MAX_SIZE_USER];
	wifi_status status;

	sprintf(buffer, "3,%s", _user);
	status = write_cmd_wifi(SHOW_MQTT_ATTRIBUTES, buffer, timeout);
	if(status != WiFi_OK) return status;

	sprintf(buffer, "4,%s", _pass);
	status = write_cmd_wifi(SHOW_MQTT_ATTRIBUTES, buffer, timeout);
	if(status != WiFi_OK) return status;

	sprintf(buffer, "5,%s", _client_id);
	status = write_cmd_wifi(SHOW_MQTT_ATTRIBUTES, buffer, timeout);
	return status;
}

/*
 * Configura el topico para publicar mensajes
 *  _topic : topico
 * timeout : tiempo maximo de la operacion
 */
wifi_status set_topic_publish(char *_topic, uint32_t timeout)
{
	char buffer[MAX_SIZE_TOPIC];

	sprintf(buffer, "0,%s", _topic);
	return write_cmd_wifi(SHOW_MQTT_ATTRIBUTES, buffer, timeout);
}

/*
 * Configura el topico para recibir mensajes
 *  _topic : topico
 * timeout : tiempo maximo de la operacion
 */
wifi_status set_topic_susbcribe(char *_topic, uint32_t timeout)
{
	char buffer[MAX_SIZE_TOPIC];

	sprintf(buffer, "1,%s", _topic);
	return write_cmd_wifi(SHOW_MQTT_ATTRIBUTES, buffer, timeout);
}

/*
 * Se conecta a un broker MQTT
 * timeout : tiempo maximo de la operacion
 */
wifi_status connect_broker_mqtt(uint32_t timeout)
{
	return write_cmd_wifi(STOP_START_TRANSPORT_CLIENT, "1", timeout);
}

/*
 * Termina la conexion con el broker
 * timeout : tiempo maximo de la operacion
 */
wifi_status disconnect_broker_mqtt(uint32_t timeout)
{
	return response_check_wifi(write_cmd_wifi(STOP_START_TRANSPORT_CLIENT, "0", timeout));
}

/*
 * Envia un mensaje al broker
 * message : mensaje para enviar
 * timeout : tiempo maximo de la operacion
 */
wifi_status send_message_mqtt(char *message, uint32_t timeout)
{
	wifi_status status;
	char buffer[MAX_SIZE_MESSAGE];

	sprintf(buffer, "%d", strlen(message));
	status = response_check_wifi(write_cmd_wifi(SET_WRITE_TRANSPORT_PACKET_SIZE, buffer, timeout));
	if(status != WiFi_OK) return status;

	sprintf(buffer, "S0\r%s", message);
	return write_data_wifi(buffer, timeout);
}

/*
 * Recibe un mensaje del broker
 * message : guarda el mensaje recibido
 * timeout : tiempo maximo de la operacion
 */
wifi_status receive_message_mqtt(char *message, uint32_t timeout)
{
	/*

	wifi_status status = write_data_WiFi(READ_TRANSPORT_DATA, timeout);

	memset(message, '\0', size(message));
	copy(message, wifi.buffer_rx);

	return status;
	*/
}


//SMTP------------------------------------------------------------------------
/*
 * Inicia una conexion con un servidor de correo (saliente)
 *   _host : dominio del servidor
 *   _port : puerto de conexion
 * timeout : tiempo maximo para la operacion
 */
wifi_status connect_smtp(char *_host, char *_port, uint32_t timeout)
{
	wifi_status status;
	//char buffer[MAX_TX_BUFFER_SIZE];

	status = response_check_wifi(write_cmd_wifi(SET_TRANSPORT_PROTOCOL, "3", timeout));
	if(status != WiFi_OK) return status;
	status = response_check_wifi(write_cmd_wifi(SET_TRANSPORT_REMOTE_PORT_NUMBER, _port, timeout));
	if(status != WiFi_OK) return status;
	status = write_cmd_wifi(DNS_LOOKUP, _host, timeout);
	if(status != WiFi_OK) return status;
	status = write_cmd_wifi(STOP_START_TRANSPORT_CLIENT, "1", timeout);
	if(status != WiFi_OK) return status;
	vTaskDelay(10);
	status = write_cmd_wifi(READ_TRANSPORT_DATA, '\0', timeout);
	if(status != WiFi_OK) return status;

	/*
	//sprintf(buffer, "%d", strlen(_host) + strlen("HELO \n\r"));
	//status = response_check_wifi(write_cmd_wifi(SET_WRITE_TRANSPORT_PACKET_SIZE, buffer, timeout));
	//if(status != WiFi_OK) return status;

	sprintf(buffer, "S0\rHELO %s\r\n", _host);
	status = write_transport_packet_size_wifi(strlen(buffer)-3, timeout);
	if(status != WiFi_OK) return status;

	status = write_data_wifi(buffer, timeout);
	if(status != WiFi_OK) return status;
	vTaskDelay(10);
	*/

	status = write_transport_packet_wifi(timeout, "HELO %s\r\n", _host);
	if(status != WiFi_OK) return status;

	status = write_cmd_wifi(READ_TRANSPORT_DATA, '\0', timeout);
	return status;
}

/*
 * Abre una sesion en el servidor de correo (saliente)
 *   _user : nombre de usuario
 *   _pass : constraseña
 * timeout : tiempo maximo para la operacion
 */
wifi_status open_smtp(char *_user, char *_pass, uint32_t timeout)
{
	wifi_status status;
	//char buffer[MAX_TX_BUFFER_SIZE];

	/*
	sprintf(buffer, "%d", strlen("auth login\r\n"));
	status = write_cmd_wifi(SET_WRITE_TRANSPORT_PACKET_SIZE, buffer, timeout);
	if(status != WiFi_OK) return status;
	sprintf(buffer, "S0\rauth login\r\n");
	status = write_data_wifi(buffer, timeout);
	if(status != WiFi_OK) return status;
	vTaskDelay(10);
	*/
	status = write_transport_packet_wifi(timeout, "auth login\r\n");
	if(status != WiFi_OK) return status;
	status = write_cmd_wifi(READ_TRANSPORT_DATA, '\0', timeout);
	if(status != WiFi_OK) return status;


	uint16_t base64_len;
	char *base64_user = base64_encode(_user, strlen(_user), &base64_len);
	/*
	sprintf(buffer, "%d", base64_len+2);
	status = response_check_wifi(write_cmd_wifi(SET_WRITE_TRANSPORT_PACKET_SIZE, buffer, timeout));
	if(status != WiFi_OK) return status;


	sprintf(buffer, "S0\r");
	str_add(buffer, base64_user, base64_len);
	str_add(buffer, "\r\n", 2);
	status = write_data_wifi(buffer, timeout);
	if(status != WiFi_OK) return status;
	vTaskDelay(10);
	*/
	status = write_transport_packet_wifi(timeout, "%s\r\n", base64_user);
	if(status != WiFi_OK) return status;

	status = write_cmd_wifi(READ_TRANSPORT_DATA, "\0", timeout);
	vPortFree(base64_user);
	if(status != WiFi_OK) return status;


	char *base64_pass = base64_encode(_pass, strlen(_pass), &base64_len);
	/*
	sprintf(buffer, "%d", base64_len+2);
	status = response_check_wifi(write_cmd_wifi(SET_WRITE_TRANSPORT_PACKET_SIZE, buffer, timeout));
	if(status != WiFi_OK) return status;


	sprintf(buffer, "S0\r");
	str_add(buffer, base64_pass, base64_len);
	str_add(buffer, "\r\n", 2);
	status = write_data_wifi(buffer, timeout);
	if(status != WiFi_OK) return status;
	vTaskDelay(10);
	*/
	status = write_transport_packet_wifi(timeout, "%s\r\n", base64_pass);
	if(status != WiFi_OK) return status;

	status = write_cmd_wifi(READ_TRANSPORT_DATA, "\0", timeout);
	vPortFree(base64_pass);
	if(status != WiFi_OK) return status;

	return status;

}

/*
 * Envia un e-mail a traves del cliente de correo
 *    sender : direccion de quien envia
 * recipient : destinatario
 *   subject : asunto
 *      data : mensaje
 *   timeout : tiempo maximo para la operacion
 */
wifi_status send_email_smtp(char *sender, char *recipient, char *subject,
		char *data, uint32_t timeout)
{
	wifi_status status;
	//char buffer[MAX_TX_BUFFER_SIZE];

	/*
	sprintf(buffer, "%d", strlen(sender)+strlen("mail FROM:<>\r\n"));
	status = response_check_wifi(write_cmd_wifi(SET_WRITE_TRANSPORT_PACKET_SIZE, buffer, timeout));
	if(status != WiFi_OK) return status;
	sprintf(buffer, "S0\rmail FROM:<%s>\r\n", sender);
	status = write_data_wifi(buffer, timeout);
	if(status != WiFi_OK) return status;
	vTaskDelay(10);
	*/
	status = write_transport_packet_wifi(timeout, "mail FROM:<%s>\r\n", sender);
	if(status != WiFi_OK) return status;

	status = write_cmd_wifi(READ_TRANSPORT_DATA, '\0', timeout);
	if(status != WiFi_OK) return status;

	/*
	sprintf(buffer, "%d", strlen(recipient)+strlen("rcpt TO:<>\r\n"));
	status = response_check_wifi(write_cmd_wifi(SET_WRITE_TRANSPORT_PACKET_SIZE, buffer, timeout));
	if(status != WiFi_OK) return status;
	sprintf(buffer, "S0\rrcpt TO:<%s>\r\n", recipient);
	status = write_data_wifi(buffer, timeout);
	if(status != WiFi_OK) return status;
	vTaskDelay(10);
	*/
	status = write_transport_packet_wifi(timeout, "rcpt TO:<%s>\r\n", recipient);
	if(status != WiFi_OK) return status;

	status = write_cmd_wifi(READ_TRANSPORT_DATA, '\0', timeout);
	if(status != WiFi_OK) return status;

	/*
	sprintf(buffer, "%d", strlen("data\r\n"));
	status = response_check_wifi(write_cmd_wifi(SET_WRITE_TRANSPORT_PACKET_SIZE, buffer, timeout));
	if(status != WiFi_OK) return status;
	sprintf(buffer, "S0\rdata\r\n");
	status = write_data_wifi(buffer, timeout);
	if(status != WiFi_OK) return status;
	vTaskDelay(10);
	*/
	status = write_transport_packet_wifi(timeout, "data\r\n");
	if(status != WiFi_OK) return status;
	status = write_cmd_wifi(READ_TRANSPORT_DATA, '\0', timeout);
	if(status != WiFi_OK) return status;

	/*
	sprintf(buffer, "%d", strlen(subject)+strlen("Subject: \r\n"));
	status = response_check_wifi(write_cmd_wifi(SET_WRITE_TRANSPORT_PACKET_SIZE, buffer, timeout));
	if(status != WiFi_OK) return status;
	sprintf(buffer, "S0\rSubject: %s\r\n", subject);
	status = write_data_wifi(buffer, timeout);
	if(status != WiFi_OK) return status;
	vTaskDelay(10);
	*/
	status = write_transport_packet_wifi(timeout, "Subject: %s\r\n", subject);
	if(status != WiFi_OK) return status;
	status = write_cmd_wifi(READ_TRANSPORT_DATA, '\0', timeout);
	if(status != WiFi_OK) return status;

	/*
	sprintf(buffer, "%d", strlen(data)+2);
	status = response_check_wifi(write_cmd_wifi(SET_WRITE_TRANSPORT_PACKET_SIZE, buffer, timeout));
	if(status != WiFi_OK) return status;
	sprintf(buffer, "S0\r%s\r\n", data);
	status = write_data_wifi(buffer, timeout);
	if(status != WiFi_OK) return status;
	vTaskDelay(10);
	*/
	status = write_transport_packet_wifi(timeout, "%s\r\n", data);
	if(status != WiFi_OK) return status;
	status = write_cmd_wifi(READ_TRANSPORT_DATA, '\0', timeout);
	if(status != WiFi_OK) return status;

	/*
	status = response_check_wifi(write_cmd_wifi(SET_WRITE_TRANSPORT_PACKET_SIZE, "3", timeout));
	if(status != WiFi_OK) return status;
	status = write_data_wifi("S0\r.\r\n", timeout);
	if(status != WiFi_OK) return status;
	vTaskDelay(10);
	*/
	status = write_transport_packet_wifi(timeout, ".\r\n");
	if(status != WiFi_OK) return status;
	status = write_cmd_wifi(READ_TRANSPORT_DATA, '\0', timeout);
	if(status != WiFi_OK) return status;

	return status;
}

/*
 * Termina la conexion con el cliente de correo (saliente)
 */
wifi_status close_smtp(uint32_t timeout)
{
	wifi_status status;

	/*
	status = response_check_wifi(write_cmd_wifi(SET_WRITE_TRANSPORT_PACKET_SIZE, "6", timeout));
	if(status != WiFi_OK) return status;
	status = write_data_wifi("S0\rquit\r\n", timeout);
	if(status != WiFi_OK) return status;
	vTaskDelay(10);
	*/

	status = write_transport_packet_wifi(timeout, "quit\r\n");
	if(status != WiFi_OK) return status;
	status = write_cmd_wifi(READ_TRANSPORT_DATA, '\0', timeout);
	if(status != WiFi_OK) return status;

	return response_check_wifi(write_cmd_wifi(STOP_START_TRANSPORT_CLIENT, "0", timeout));
}

//POP-------------------------------------------------------------------------
/*
 * Inicia una conexion con un servidor de correo (entrante)
 *   _host : dominio del servidor
 *   _port : puerto de conexion
 * timeout : tiempo maximo para la operacion
 */
wifi_status connect_pop(char *_host, char *_port, uint32_t timeout)
{
	wifi_status status;
	//char buffer[MAX_TX_BUFFER_SIZE];

	status = response_check_wifi(write_cmd_wifi(SET_TRANSPORT_PROTOCOL, "3", timeout));
	if(status != WiFi_OK) return status;
	status = response_check_wifi(write_cmd_wifi(SET_TRANSPORT_REMOTE_PORT_NUMBER, _port, timeout));
	if(status != WiFi_OK) return status;
	status = write_cmd_wifi(DNS_LOOKUP, _host, timeout);
	if(status != WiFi_OK) return status;
	status = write_cmd_wifi(STOP_START_TRANSPORT_CLIENT, "1", timeout);
	if(status != WiFi_OK) return status;
	vTaskDelay(10);
	status = write_cmd_wifi(READ_TRANSPORT_DATA, '\0', timeout);
	if(status != WiFi_OK) return status;

	return status;
}

/*
 * Abre una sesion en el servidor de correo (entrante)
 *   _user : nombre de usuario
 *   _pass : constraseña
 * timeout : tiempo maximo para la operacion
 */
wifi_status open_pop(char *_user, char *_pass, uint32_t timeout)
{
	wifi_status status;
	char buffer[MAX_TX_BUFFER_SIZE];

	sprintf(buffer, "%d", strlen(_user)+strlen("user \r\n"));
	status = response_check_wifi(write_cmd_wifi(SET_WRITE_TRANSPORT_PACKET_SIZE, buffer, timeout));
	if(status != WiFi_OK) return status;
	sprintf(buffer, "S0\ruser %s\r\n", _user);
	status = write_data_wifi(buffer, timeout);
	if(status != WiFi_OK) return status;
	vTaskDelay(10);
	status = write_cmd_wifi(READ_TRANSPORT_DATA, '\0', timeout);
	if(status != WiFi_OK) return status;


	sprintf(buffer, "%d", strlen(_pass)+strlen("pass \r\n"));
	status = response_check_wifi(write_cmd_wifi(SET_WRITE_TRANSPORT_PACKET_SIZE, buffer, timeout));
	if(status != WiFi_OK) return status;
	sprintf(buffer, "S0\rpass %s\r\n", _pass);
	status = write_data_wifi(buffer, timeout);
	if(status != WiFi_OK) return status;
	vTaskDelay(10);
	status = write_cmd_wifi(READ_TRANSPORT_DATA, '\0', timeout);
	if(status != WiFi_OK) return status;

	return status;
}

/*
wifi_status check_inbox_pop(uint32_t timeout)
{
	wifi_status status;

	status = response_check_wifi(write_cmd_wifi(SET_WRITE_TRANSPORT_PACKET_SIZE, "6", timeout));
	if(status != WiFi_OK) return status;
	status = write_data_wifi("S0\rlist\r\n", timeout);
	if(status != WiFi_OK) return status;
	vTaskDelay(10);
	status = write_cmd_wifi(READ_TRANSPORT_DATA, '\0', timeout);
	if(status != WiFi_OK) return status;

	return status;
}
*/

wifi_status status_pop(uint16_t *n_messages, uint16_t *size)
{

}

/*
 * Lee el correo en la caja de entrada
 *      id : numero de correo
 *    mail : estructura para guardar los datos obtenidos
 * timeout : tiempo maximo para la operacion
 */
wifi_status read_email_pop(uint8_t id, pop_mail_t *mail, uint32_t timeout)
{
	wifi_status status;
	char buffer[MAX_TX_BUFFER_SIZE];
	int size_number;
	int index;
	int i;

	uint8_t sender_locate = 0;
	uint8_t date_locate = 0;
	uint8_t subject_locate = 0;
	uint8_t message_locate = 0;

	if(id<10)
		size_number = 1;
	else if(id<100)
		size_number = 2;
	else
		size_number = 3;

	memset(mail->sender, '\0', MAX_SENDER_SIZE_POP);
	memset(mail->date, '\0', MAX_DATE_SIZE_POP);
	memset(mail->subject, '\0', MAX_SUBJECT_SIZE_POP);
	memset(mail->message, '\0', MAX_MESSAGE_SIZE_POP);


	sprintf(buffer, "%d", strlen("retr \r\n")+size_number);
	status = write_cmd_wifi(SET_WRITE_TRANSPORT_PACKET_SIZE, buffer, timeout);
	if(status != WiFi_OK) return status;
	sprintf(buffer, "S0\rretr %d\r\n", id);
	status = write_data_wifi(buffer, timeout);
	if(status != WiFi_OK) return status;
	vTaskDelay(10);

	while(1)
	{
		status = write_cmd_wifi(READ_TRANSPORT_DATA, '\0', timeout);

		if(status != WiFi_OK) return status;

		if(!sender_locate)
		{
			if((index = str_find(wifi.buffer_rx, "From: ", 0)) != -1)
			{
				index+=5;

				for(i=0; i<MAX_SENDER_SIZE_POP; i++)
				{
					index++;
					if(wifi.buffer_rx[index] == '\r')
						break;
					mail->sender[i] = wifi.buffer_rx[index];
				}

				sender_locate = 1;
			}
		}

		if(!date_locate)
		{
			if((index = str_find(wifi.buffer_rx, "Date: ", 0)) != -1)
			{
				index+=5;

				for(i=0; i<MAX_DATE_SIZE_POP; i++)
				{
					index++;
					if(wifi.buffer_rx[index] == '\r')
						break;
					mail->date[i] = wifi.buffer_rx[index];
				}
				date_locate = 1;
			}
		}

		if(!subject_locate)
		{
			if((index = str_find(wifi.buffer_rx, "Subject: ", 0)) != -1)
			{
				index+=8;

				for(i=0; i<MAX_SUBJECT_SIZE_POP; i++)
				{
					index++;
					if(wifi.buffer_rx[index] == '\r')
						break;
					mail->subject[i] = wifi.buffer_rx[index];
				}
				subject_locate = 1;
			}
		}

		if(!message_locate)
		{
			if((index = str_find(wifi.buffer_rx, "boundary=\"", 0)) != -1)
			{
				index+=9;
				memset(buffer, '\0', MAX_TX_BUFFER_SIZE);
				for(i=0; i<MAX_TX_BUFFER_SIZE; i++)
				{
					index++;
					if(wifi.buffer_rx[index] == '"')
						break;
					buffer[i] = wifi.buffer_rx[index];
				}

				str_cat(buffer, "--", 0);

				if((index = str_find(wifi.buffer_rx, buffer, index)) != -1)
				{
					index +=(strlen(buffer)+1);
					int aux = str_find(wifi.buffer_rx, buffer, index);
					size_number = (aux-index)-1;
					if(aux != -1)
					{
						for(i=0; i< size_number; i++)
						{
							index++;
							mail->message[i] =wifi.buffer_rx[index];
						}

						message_locate =1;
					}
				}
			}
		}

		if(sender_locate & date_locate & subject_locate & message_locate)
			break;

	}


	return status;
}

/*
 * Elimina un correo de la caja de entrada
 *      id : numero del correo
 * timeout : tiempo maximo para la operacion
 */
wifi_status delete_message_pop(uint8_t id, uint32_t timeout)
{
	wifi_status status;
	char buffer[MAX_TX_BUFFER_SIZE];
	uint8_t size_number;

	if(id<10)
		size_number = 1;
	else if(id<100)
		size_number = 2;
	else
		size_number = 3;

	sprintf(buffer, "%d", strlen("dele \r\n")+size_number);
	status = write_cmd_wifi(SET_WRITE_TRANSPORT_PACKET_SIZE, buffer, timeout);
	if(status != WiFi_OK) return status;
	sprintf(buffer, "S0\rdele %d\r\n", id);
	status = write_data_wifi(buffer, timeout);
	if(status != WiFi_OK) return status;
	vTaskDelay(10);
	return write_cmd_wifi(READ_TRANSPORT_DATA, '\0', timeout);
}

/*
 * Termina la conexion con el cliente de correo (entrante)
 */
wifi_status close_pop(uint32_t timeout)
{
	wifi_status status;

	status = response_check_wifi(write_cmd_wifi(SET_WRITE_TRANSPORT_PACKET_SIZE, "6", timeout));
	if(status != WiFi_OK) return status;
	status = write_data_wifi("S0\rquit\r\n", timeout);
	if(status != WiFi_OK) return status;
	vTaskDelay(10);
	status = write_cmd_wifi(READ_TRANSPORT_DATA, '\0', timeout);
	if(status != WiFi_OK) return status;

	return response_check_wifi(write_cmd_wifi(STOP_START_TRANSPORT_CLIENT, "0", timeout));
}

//SNTP------------------------------------------------------------------------
/*
 * Inicia una conexion con un servidor de tiempo de red
 *   _host : dominio del servidor
 *   _port : puerto de conexion
 * timeout : tiempo maximo para la operacion
 */
wifi_status connect_sntp(char *_host, char *_port, uint32_t timeout)
{
	wifi_status status;

	status = response_check_wifi(write_cmd_wifi(SET_TRANSPORT_PROTOCOL, "1", timeout));
	if(status != WiFi_OK) return status;
	status = response_check_wifi(write_cmd_wifi(SET_TRANSPORT_REMOTE_PORT_NUMBER, _port, timeout));
	if(status != WiFi_OK) return status;
	status = write_cmd_wifi(DNS_LOOKUP, _host, timeout);
	if(status != WiFi_OK) return status;
	return response_check_wifi(write_cmd_wifi(STOP_START_TRANSPORT_CLIENT, "1", timeout));
}

/*
 * Obtiene el tiempo en formato unix de la red
 * unix_time : tiempo en segundos de la red en formato unix
 *   timeout : tiempo maximo para la operacion
 */
wifi_status get_time_sntp(time_t *unix_time, uint32_t timeout)
{
	wifi_status status;
	char buffer[PACKET_SIZE_NTP+3];

	sprintf(buffer, "%d", PACKET_SIZE_NTP);
	status = response_check_wifi(write_cmd_wifi(SET_WRITE_TRANSPORT_PACKET_SIZE, buffer, timeout));
	if(status != WiFi_OK) return status;

	memset(buffer, 0, PACKET_SIZE_NTP);

	buffer[0] = 'S';
	buffer[1] = '0';
	buffer[2] = '\r';
	buffer[3] = 0b11100011;
	buffer[4] = 0;
	buffer[5] = 6;
	buffer[6] = 0xEC;

	buffer[15]  = 49;
	buffer[16]  = 0x4E;
	buffer[17]  = 49;
	buffer[18]  = 52;

	status = write_data_size_wifi(buffer, PACKET_SIZE_NTP+3, timeout);
	if(status != WiFi_OK) return status;
	status = write_cmd_wifi(READ_TRANSPORT_DATA, '\0', timeout);
	if(status != WiFi_OK) return status;

	*unix_time = ((uint8_t)wifi.buffer_rx[42]<<24|(uint8_t)wifi.buffer_rx[43]<<16
			|(uint8_t)wifi.buffer_rx[44]<<8|(uint8_t)wifi.buffer_rx[45])-2208988800UL;

	return status;

}

/*
 * Termina la conexion con el cliente de correo (entrante)
 */
wifi_status disconnect_sntp(uint32_t timeout)
{
	return response_check_wifi(write_cmd_wifi(STOP_START_TRANSPORT_CLIENT, "0", timeout));
}

//UTC-------------------------------------------------------------------------
wifi_status get_utc(time_t *unix_time, uint32_t timeout)
{
	wifi_status status = write_cmd_wifi(GET_UTC_TIME, '\0', timeout);
	if(status != WiFi_OK) return status;

	for(uint16_t i=0; i<MAX_RX_BUFFER_SIZE; i++)
	{
		if(wifi.buffer_rx[i] == 'O')
		{
			wifi.buffer_rx[i] = '\0';
			break;
		}
	}

	unix_time = aotll(wifi.buffer_rx);
	return status;
}

//API´S-----------------------------------------------------------------------
/* Obtiene datos del tiempo atmosferico
 *     info : estructura para guardar los datos obtenidos
 * location : coordenadas del lugar para obtener el clima
 *  timeout : tiempo maximo de la operacion
 *
 *
 *    Notas : El desarrollador no tiene nigun convenio con la pagina para
 *    		  obtener el tiempo atmosferico, solo usa su servicio
 *    		  Para obtener una clave de consulta y mas informacion dirigete
 *    		  a : http://www.weatherunlocked.com/
 *    		  Una vez obtenidos el id y api key colocalos en sus respectivos
 *    		  espacios
 */
#ifndef WHEATER_UNLOKED_AUTH
#define WHEATER_UNLOKED_AUTH
#define WHEATER_HOST						"api.weatherunlocked.com"
#define WHEATER_PORT						"80"
#define WHEATHER_ID							"f18ad0e6"
#define WEATHER_API_KEY						"16fb92203de55d70a4f397668c117d21"
#endif
wifi_status http_perform_as_stream_wheather(weather_info_t *info, char *location, uint32_t timeout)//wifi_status check_wheater(char *location, uint32_t timeout)
{
	wifi_status status;
	char buffer[MAX_TX_BUFFER_SIZE];

	status = write_cmd_wifi(SET_TRANSPORT_PROTOCOL, "0", timeout);
	if(status != WiFi_OK) return status;
	status = write_cmd_wifi(SET_TRANSPORT_REMOTE_PORT_NUMBER, WHEATER_PORT, timeout);
	if(status != WiFi_OK) return status;
	status = write_cmd_wifi(DNS_LOOKUP, WHEATER_HOST, timeout);
	if(status != WiFi_OK) return status;
	status = write_cmd_wifi(STOP_START_TRANSPORT_CLIENT, "1", timeout);
	if(status != WiFi_OK) return status;

	sprintf(buffer, "%d", strlen(location)+strlen(WHEATHER_ID)+strlen(WEATHER_API_KEY)
			+strlen("GET /api/current/?app_id=&app_key= HTTP/1.1\r\nHost: \r\n\r\n")+strlen(WHEATER_HOST));
	status = write_cmd_wifi(SET_WRITE_TRANSPORT_PACKET_SIZE, buffer, timeout);
	if(status != WiFi_OK) return status;

	sprintf(buffer, "S0\rGET /api/current/%s?app_id=%s&app_key=%s HTTP/1.1\r\nHost: %s\r\n\r\n",
			location, WHEATHER_ID, WEATHER_API_KEY, WHEATER_HOST);
	status = write_data_wifi(buffer, timeout);
	if(status != WiFi_OK) return status;
	vTaskDelay(10);
	status = write_cmd_wifi(READ_TRANSPORT_DATA, '\0', timeout);
	if(status != WiFi_OK) return status;

    uint16_t i=0;
    char buffer_aux[25];

    while(i<strlen(wifi.buffer_rx))
    {
    	if(wifi.buffer_rx[i] == '"')
    	{
    		i++;
    		memset(buffer_aux, '\0', 25);
    		while(wifi.buffer_rx[i] != '"')
    		{
    			str_copy(buffer_aux, wifi.buffer_rx[i]);
    			i++;
    		}

    		i+=2;

    		if(strcmp("temp_c", buffer_aux) == 0)
    		{

    			memset(buffer_aux, '\0', 25);
    			while(wifi.buffer_rx[i] != ',')
    			{
    				str_copy(buffer_aux, wifi.buffer_rx[i]);
    				i++;
    			}

    			info->temp = atof(buffer_aux);
    		}

    		else if(strcmp("wx_code", buffer_aux) == 0)
    		{

    			memset(buffer_aux, '\0', 25);
    			while(wifi.buffer_rx[i] != ',')
    			{
    				str_copy(buffer_aux, wifi.buffer_rx[i]);
    				i++;
    			}

    			info->weather_code = atoi(buffer_aux);
    		}
    		else if(strcmp("humid_pct", buffer_aux) == 0)
    		{
    			memset(buffer_aux, '\0', 25);
    			while(wifi.buffer_rx[i] != ',')
    			{
    				str_copy(buffer_aux, wifi.buffer_rx[i]);
    				i++;
    			}

    			info->humidity = atof(buffer_aux);
    			break;
    		}

    	}
    	else
    		i++;
    }

	return write_cmd_wifi(STOP_START_TRANSPORT_CLIENT, "0", timeout);

}

/* Obtiene el tiempo de la red segun la ubicacion
 *     time : estructura para guardar los datos del tiempo
 * timezone : region (ej. America)
 *     city : lugar
 *  timeout : tiempo maximo para la operacion
 *
 *    Notas : no se recomienda el uso de esta api si se requiere una gran
 *    		  exactitud, pues la extracción de la información requiere un tiempo
 *    		  considerable. Se recomienda ampliamente el uso de la conexion sntp
 *
 */
/*
#define WORLD_TIME_API
#define WORLD_TIME_HOST			"worldtimeapi.org"
#define WORLD_TIME_PORT			"80"
wifi_status http_perform_as_stream_time(date_time_t *time, char *timezone,
		char *city, uint32_t timeout)
{
	wifi_status status;
	char buffer[MAX_TX_BUFFER_SIZE];

	status = write_cmd_wifi(SET_TRANSPORT_PROTOCOL, "0", timeout);
	if(status != WiFi_OK) return status;
	status = write_cmd_wifi(SET_TRANSPORT_REMOTE_PORT_NUMBER, WORLD_TIME_PORT, timeout);
	if(status != WiFi_OK) return status;
	status = write_cmd_wifi(DNS_LOOKUP, WORLD_TIME_HOST, timeout);
	if(status != WiFi_OK) return status;
	status = write_cmd_wifi(STOP_START_TRANSPORT_CLIENT, "1", timeout);
	if(status != WiFi_OK) return status;

	sprintf(buffer, "%d", strlen(timezone)+strlen(city)
			+strlen("GET /api/timezone// HTTP/1.1\r\nHost: \r\n\r\n")+strlen(WORLD_TIME_HOST));
	status = write_cmd_wifi(SET_WRITE_TRANSPORT_PACKET_SIZE, buffer, timeout);
	if(status != WiFi_OK) return status;

	sprintf(buffer, "S0\rGET /api/timezone/%s/%s HTTP/1.1\r\nHost: %s\r\n\r\n",
			timezone, city, WORLD_TIME_HOST);
	status = write_data_wifi(buffer, timeout);
	if(status != WiFi_OK) return status;
	vTaskDelay(10);
	status = write_cmd_wifi(READ_TRANSPORT_DATA, '\0', timeout);
	if(status != WiFi_OK) return status;

    uint16_t i=0;

    while(i<strlen(wifi.buffer_rx))
    {
    	if(wifi.buffer_rx[i] == '"')
    	{
    		i++;
    		memset(buffer, '\0', MAX_TX_BUFFER_SIZE);
    		while(wifi.buffer_rx[i] != '"')
    		{
    			str_copy(buffer, wifi.buffer_rx[i]);
    			i++;
    		}

    		//i+=3;
    		i++;

    		if(strcmp("datetime", buffer) == 0)
    		{
    			i+=2;
    			buffer[0] = wifi.buffer_rx[i];
    			buffer[1] = wifi.buffer_rx[i+1];
    			buffer[2] = wifi.buffer_rx[i+2];
    			buffer[3] = wifi.buffer_rx[i+3];
    			buffer[4] = '\0';
    			time->year = atoi(buffer);

    			buffer[0] = wifi.buffer_rx[i+5];
    			buffer[1] = wifi.buffer_rx[i+6];
    			buffer[2] = '\0';
    			time->month = atoi(buffer);

    			buffer[0] = wifi.buffer_rx[i+8];
    			buffer[1] = wifi.buffer_rx[i+9];
    			time->day = atoi(buffer);

    			buffer[0] = wifi.buffer_rx[i+11];
    			buffer[1] = wifi.buffer_rx[i+12];
    			time->hour = atoi(buffer);

    			buffer[0] = wifi.buffer_rx[i+14];
    			buffer[1] = wifi.buffer_rx[i+15];
    			time->minute = atoi(buffer);

    			buffer[0] = wifi.buffer_rx[i+17];
    			buffer[1] = wifi.buffer_rx[i+18];
    			time->second = atoi(buffer);

    			break;

    		}

    	}
    	else
    		i++;
    }

	return write_cmd_wifi(STOP_START_TRANSPORT_CLIENT, "0", timeout);

}
*/



#endif /* SRC_WIFI_DRIVER_C_ */
