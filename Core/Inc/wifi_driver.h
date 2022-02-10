/*
 * wifi_driver.h
 *
 *  Created on: Oct 31, 2021
 *      Author: JoseCarlos
 *     Version: 2.1 (Alfa)
 * Descripcion: Libreria para el manejo del modulo ISM43362-M3G-L44 (WiFi) en
 * 				el kit de desarrollo B-L47 IOT STM32 con version de comandos
 * 				2.0 y comunicacion SPI.
 *	Importante: Todas las funciones son incompatibles con comunicaciones que
 *				requieran certificados TLS o root CA.
 *
 *   Versiones:	2.0: Uso de freertos
 *     			     El pin de RDY se cambio de interrupcion a gpio input
 *     			     Se agrego la funcion de escaneo de redes wifi
 *     			     Se agrego la funcion de intensidad de senial rssi
 *     			     La biblioteca ahora incluye el servicio MQTT dentro de las
 * 				     mismas funciones.
 * 				     Se agrego el servico de SMTP
 * 				     Se agrego el servicio de POP
 *
 *
 *
 *	     Notas: Si se usa el kit de desarrollo, la configuracion se hace por
 * 		 		medio de la funcion set_default_WiFi(), en caso de usar un
 * 		 		modulo por separado iniciar las configuraciones de los puertos
 * 		 		y pines, y la comunicacion SPI.
 * 		 		Se trata de una biblioteca basica de comunicacion con el modulo
 * 		 		es necesario desarrollar blibliotecas para expandir sus funiones.
 * 		 		Apartir de la version 2.0 es necesario el uso de bibliotecas freertos.
 *
 *Configuracion: SPI->data_size=16 bits ->Clock=10 MHz(recomendado) max 20 MHz
 *				 CS(NSS)->GPIO_OUTPUT ->SPEED=MEDIUM
 *				 RST(RESET)-> GPIO_OUTPUT ->SPEED=LOW
 *				 WKP(WAKE UP)-> GPIO_OUTPUT ->SPEED=LOW
 *				 RDY(CMD/DATA READY)-> GPIO_INPUT
 */

#ifndef INC_WIFI_DRIVER_H_
#define INC_WIFI_DRIVER_H_


/*INCLUDES*******************************************************************/
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "time.h"
#include "main.h"

/*DEFINES********************************************************************/
//WIFI MODULE-----------------------------------------------------------------
#define VERSION 			2

#ifdef VERSION
#if VERSION == 2
#define PRINT_HELP_MESSAGE  										'?'
#define ENTER_COMMAND_MODE										    "$$$"
#define EXIT_COMMAND_MODE											"---"
#define ACTIVE_ACCESS_POINT											"A0"
#define ACCESS_POINT_SECURITY_MODE									"A1"
#define SET_ACCESS_POINT_SECURITY_KEY                               "A2"
#define GET_AP_DHCP_CACHE_ADDRESS                                   "AA"
#define SET_ACCESS_POINT_CHANNEL                                    "AC"
#define ACTIVATE_ACCESS_POINT_DIRECT_CONNECT_MODE                   "AD"
#define EXIT_ACCESS_POINT_DIRECT_CONNECT_MODE                       "AE"
#define SET_ACCESS_POINT_LEASE_TIME                                 "AL"
#define GET_CLIENT_RSSI                                             "AR"
#define SET_ACCESS_POINT_SSID                                       "AS"
#define SET_MAXIMUM_NUMBER_OF_AP_CLIENTS                            "AT"
#define SHOW_ACCESS_POINT_SETTINGS                                  "A?"
#define SET_SPI_MODE                                                "B2"
#define SET_SPI_READY_PIN                                           "B3"
#define SHOW_COMMUNICATION_INTERFACE_SETTINGS                       "B?"
#define JOIN_A_NETWORK                                              "C0"
#define SET_NETWORK_SSID                                            "C1"
#define SET_NETWORK_PASSPHRASE                                      "C2"
#define SET_NETWORK_SECURITY_TYPE                                   "C3"
#define SET_NETWORK_DHCP_MODE                                       "C4"
#define SET_NETWORK_IP_VERSION                                      "C5"
#define SET_NETWORK_IP_ADDRESS                                      "C6"
#define SET_NETWORK_IP_MASK                                         "C7"
#define SET_NETWORK_GATEWAY                                         "C8"
#define SET_NETWORK_PRIMARY_DNS                                     "C9"
#define SET_NETWORK_SECONDARY_DNS                                   "CA"
#define SET_NETWORK_JOIN_RETRY_COUNT                                "CB"
#define NETWORK_AUTO_CONNECT                                        "CC"
#define DISCONNECT_FROM_NETWORK                                     "CD"
#define SET_AUTHORIZATION_TYPE                                      "CE"
#define SET_CLEAR_PACKET_FILTERS                                    "CF"
#define JOIN_LEAVE_IGMP_GROUP                                       "CJ"
#define ADD_REMOVE_MAC_TO_FROM_MCAST_ALLOW_LIST                     "CM"
#define SET_COUNTRY_CODE                                            "CN"
#define GET_RSSS_OF_ASSOCIATED_NETWORK_ACCESS_POINT                 "CR"
#define GET_CONNECTION_STATUS                                       "CS"
#define SET_WPS_PB_PIN                                              "CT"
#define GET_CONNECTED_BIT_RATE                                      "CV"
#define CONNECT_USING_WPS_PIN_OR_PBC                                "CW"
#define SHOW_NETWORK_SETTINGS                                       "C?"
#define DNS_LOOKUP                                                  "D0"
#define ENABLE_MDNS                                          		"D1"
#define ENABLE_MDNS_SERVICE                                    		"D2"
#define SCAN_FOR_NETWORK_ACCESS_POINTS                              "F0"
#define SET_SCAN_REPEAT_COUNT                                    	"F1"
#define SET_SCAN_DELAY                                    			"F2"
#define SET_SCAN_CHANNEL                                    		"F3"
#define SET_SCAN_BBSID                                    			"F4"
#define SET_SCAN_SSID                                    			"F5"
#define SHOW_SCAN_SETTINGS                                    		"F?"
#define READ_GPIO_ADC                                    			"G2"
#define WRITE_GPIO                                    				"G3"
#define GPIO_SETUP                                    				"G4"
#define GET_UTC_TIME                                    			"GT"
#define SHOW_GPIO_SETTINGS                                    		"G?"
#define SHOW_APPLICATION_INFORMATION                                "I?"
#define TEST_EXTERNAL_SERIAL_FLASH                                  "MF"
#define MANUFACTURING_TEST                                    		"M3"
#define MESSAGE_READ                                    			"MR"
#define SUPPRESS_ASYNC_MESSAGE_DHCP                                 "MS"
#define SET_MESSAGE_TYPE                                    		"MT"
#define SET_DISPLAY_COMMUNICATION_SOCKET                            "P0"
#define SET_TRANSPORT_PROTOCOL                                    	"P1"
#define SET_TRANSPORT_LOCAL_PORT_NUMBER                             "P2"
#define SET_TRANSPORT_REMOTE_HOST_IP_ADDRESS                        "P3"
#define SET_TRANSPORT_REMOTE_PORT_NUMBER                            "P4"
#define STOP_START_TRANSPORT_SERVER                                 "P5"
#define STOP_START_TRANSPORT_CLIENT                                 "P6"
#define START_STOP_REQUEST_TCP_LOOP                                 "P7"
#define SET_LISTEN_BACKLOGS                                    		"P8"
#define SSL_CERTIFICATE_AUTHENTICATION                              "P9"
#define SET_CUSTOM_CERTIFICATE_AUTHORITY                            "PA"
#define SET_ROOT_CA_VERIFICATION_RESULTS                            "PB"
#define SECURITY_CERTIFICATES                                    	"PC"
#define SECURITY_KEYS                                    			"PD"
#define CERTIFICATE_SET_AVAILABILITY                                "PE"
#define SELECT_ACTIVE_CERTIFICATE_SET                               "PF"
#define PROGRAM_CA_CERTIFICATE_OR_KEY                               "PG"
#define TCP_KEEP_ALIVE                                    			"PK"
#define SHOW_MQTT_ATTRIBUTES                                   		"PM"
#define ENABLE_UART_STREAMING_MODE                                  "PX"
#define SET_TCP_API_MESSAGE_TIMEOUT                                 "PY"
#define SHOW_TRANSPORT_SETTINGS                                    	"P?"
#define READ_TRANSPORT_DATA                                    		"R0"
#define SET_READ_TRANSPORT_PACKET_SIZE                              "R1"
#define SET_READ_TRANSPORT_TIMEOUT                                  "R2"
#define RECEIVE_MODE                                    			"R3"
#define SHOW_READ_TRANSPORT_SETTINGS                                "R?"
#define WRITE_TRANSPORT_DATA                                    	"S0"
#define SET_WRITE_TRANSPORT_PACKET_SIZE                             "S1"
#define SET_WRITE_TRANSPORT_TIMEOUT                                 "S2"
//#define SET_WRITE_TRANSPORT_TIMEOUT                               "S3"
#define WRITE_TRANSPORT_DATA_W_PACKET_SIZE                          "S?"
#define PING_TARGET_ADDRESS                                  	    "T0"
#define SET_PING_TARGET_ADDRESS                                    	"T1"
#define SET_PING_REPEAT_COUNT                                    	"T2"
#define SET_PING_DELAY                                    			"T3"
#define SHOW_PING_SETTINGS                                    		"T?"
#define ACTIVE_UART_SETTINGS                                    	"U0"
#define SET_UART_BAUD_RATE                                    		"U2"
#define SHOW_UART_SETTINGS                                    		"U?"
#define RESET_TO_FACTORY_DEFAULTS                                   "Z0"
#define SAVE_CURRENT_SETTINGS                                    	"Z1"
#define CLEAR_CURRENT_SETTINGS                                    	"Z2"
#define SET_FACTORY_USER_SPACE                                    	"Z3"
#define SET_MAC_ADDRESS                                    			"Z4"
#define GET_MAC_ADDRESS                                    			"Z5"
#define SET_ACCESS_POINT_IP_ADDRESS                                 "Z6"
#define SET_WPS_PIN_NUMBER                                    		"Z7"
#define GET_WPS_PIN_NUMBER                                    		"Z8"
#define SET_USB_VID_PID                                    			"Z9"
#define CLEAR_FACTOR_LOCK_SWITCH                                    "ZC"
#define FLASH_DUMP                                    				"ZD"
#define SET_FACTORY_LOCK_SWITCH                                    	"ZF"
#define SET_PRODUCT_NAME                                    		"ZN"
#define OTA_FIRMWARE_UPDATE                                    		"ZO"
#define POWER_MANAGEMENT                                    		"ZP"
#define RESET_MODULE                                    			"ZR"
#define GET_SERIAL_NUMBER                                    		"ZS"
#define SET_SERIAL_NUMBER                                    		"ZT"
#define FIRMWARE_UPGRADE                                    		"ZU"
#define SET_OTA_METHOD                                    			"ZV"
#define SHOW_SYSTEM_SETTINGS                                    	"Z?"
#endif
#endif

//WIFI MAX-----------------------------------------------------------------
#define MAX_SSID_SIZE			30
#define MAX_PASS_SIZE			30
#define MAX_TX_BUFFER_SIZE		512
#define MAX_RX_BUFFER_SIZE		1024*2

//MQTT-----------------------------------------------------------------------
#define MAX_SIZE_ID				50				//maximo del identificador
#define MAX_SIZE_TOPIC			100				//maximo del tipico
#define MAX_SIZE_USER			50				//maximo del usuario
#define MAX_SIZE_PASS			50				//maximo de la contraseÃ±a
#define MAX_SIZE_HOST			50				//maximo de la direccion
#define MAX_SIZE_PORT			4				//maximo del puerto
#define MAX_SIZE_MESSAGE		80				//maximo del mensaje

//POP------------------------------------------------------------------------
#define MAX_SENDER_SIZE_POP		50
#define MAX_DATE_SIZE_POP		25
#define MAX_SUBJECT_SIZE_POP	50
#define MAX_MESSAGE_SIZE_POP	200

//SNTP-----------------------------------------------------------------------
#define PACKET_SIZE_NTP			48

/*TYPEDEFS*******************************************************************/
//WIFI SOFT AP---------------------------------------------------------------
typedef enum{
	WiFi_ERROR			= 0x00,				//Error en la conexion del modulo
	WiFi_OK				= 0x01,				//operacion exitosa
	SPI_ERROR			= 0x02,				//error en la com SPI
	Timeout_ERROR		= 0x03,				//Paso mas tiempo del esperado
	Timeout_OK			= 0x04,				//dentro del margen de tiempo
	OVER_FLOW_BUFFER 	= 0x05,				//se recibio mas info de la esperada
	Operation_ERROR		= 0x06
}wifi_status;

typedef enum{
	Open 			= 0, 		//No WiFi Security
	WEP 			= 1, 		//Wired Equivalent Privacy
	WPA 			= 2,      	//WiFi Protected Access TKIP
	WPA2 			= 3,		//WiFi Protected Access 2 AES
	WPAandWPA2 		= 4,		//WiFi Protected Access and WiFi Protected Access 2
	WPA2_TKIP		= 5			//WiFi Protected Access 2 TKIP
}security_mode;

struct wifi_scan_ap{						//Lista para el escaneo de redes wifi
	char SSID[MAX_SSID_SIZE];				//Nombre de la red
	int8_t RSSI;							//Fuerza de la señal
	security_mode sec;						//Seguridad
	uint8_t MAC[6];							//Direccion
	struct wifi_scan_ap *next;				//Mas miembros de la lista
};
typedef struct wifi_scan_ap		wifi_scan_ap_t;

//MQTT-----------------------------------------------------------------------
typedef enum{
	none					= 0,		//sin seguridad
	user_name_password		= 1,		//seguridad con usuario y contraseÃ±a
	ca_cert_key				= 2			//seguridad con certificados
}mqtt_security;

//POP------------------------------------------------------------------------
typedef struct{
	char sender[MAX_SENDER_SIZE_POP];
	char date[MAX_DATE_SIZE_POP];
	char subject[MAX_SUBJECT_SIZE_POP];
	char message[MAX_MESSAGE_SIZE_POP];

}pop_mail_t;

//API´S-----------------------------------------------------------------------
//WEATHER STREAM--------------------------------------------------------------
typedef struct
{
	float temp;
	float humidity;
	uint8_t weather_code;
	//char wheather_date[15];

}weather_info_t;

/*
//TIME STREAM-----------------------------------------------------------------
typedef struct{
	uint16_t year;
	uint8_t month;
	uint8_t day;

	uint8_t hour;
	uint8_t minute;
	uint8_t second;
}date_time_t;
*/

/*GLOBAL VARIABLES***********************************************************/
struct{
	SPI_HandleTypeDef *_hspi;

	GPIO_TypeDef *CS_PORT;
	GPIO_TypeDef *RST_PORT;
	GPIO_TypeDef *WKP_PORT;
	GPIO_TypeDef *RDY_PORT;

	uint16_t CS_PIN;
	uint16_t RST_PIN;
	uint16_t WKP_PIN;
	uint16_t RDY_PIN;

	char buffer_rx[MAX_RX_BUFFER_SIZE];
	char buffer_tx[MAX_TX_BUFFER_SIZE];
}wifi;

/*PROTOTYPES*****************************************************************/
void set_default_wifi();
wifi_status init_wifi(uint32_t timeout);
void reset_wifi();
void wakeup_wifi();
void stop_module_wifi();

//SOFT AP---------------------------------------------------------------------
wifi_status connect_wifi(char *_ssid, char *_pass, security_mode mode,
		uint32_t timeout);
wifi_status set_socket_wifi(uint32_t timeout);
wifi_status scan_wifi_soft_ap(wifi_scan_ap_t **list, uint32_t timeout);
void delete_scan_list(wifi_scan_ap_t **list);
int8_t rssi_soft_ap(uint32_t timeout);


//MQTT-----------------------------------------------------------------------
wifi_status set_broker_mqtt(char *_host, char *_port, mqtt_security sec,
		uint32_t timeout);
wifi_status set_credentials_mqtt(char *_user, char *_pass, char *_client_id,
		uint32_t timeout);
wifi_status set_topic_publish(char *_topic, uint32_t timeout);
wifi_status set_topic_susbcribe(char *_topic, uint32_t timeout);
wifi_status connect_broker_mqtt(uint32_t timeout);
wifi_status disconnect_broker_mqtt(uint32_t timeout);
wifi_status send_message_mqtt(char *message, uint32_t timeout);
wifi_status receive_message_mqtt(char *message, uint32_t timeout);


//SMTP------------------------------------------------------------------------
wifi_status connect_smtp(char *_host, char *_port, uint32_t timeout);
wifi_status open_smtp(char *_user, char *_pass, uint32_t timeout);
wifi_status send_email_smtp(char *sender, char *recipient, char *subject,
		char *data, uint32_t timeout);
wifi_status close_smtp(uint32_t timeout);

//POP-------------------------------------------------------------------------
wifi_status connect_pop(char *_host, char *_port, uint32_t timeout);
wifi_status open_pop(char *_user, char *_pass, uint32_t timeout);
//wifi_status check_inbox_pop(uint32_t timeout);
wifi_status status_pop(uint16_t *n_messages, uint16_t *size);
wifi_status read_email_pop(uint8_t id, pop_mail_t *mail, uint32_t timeout);
wifi_status delete_message_pop(uint8_t id, uint32_t timeout);
wifi_status close_pop(uint32_t timeout);

//SNTP------------------------------------------------------------------------
wifi_status connect_sntp(char *_host, char *_port, uint32_t timeout);
wifi_status get_time_sntp(time_t *unix_time, uint32_t timeout);
wifi_status disconnect_sntp(uint32_t timeout);

//UTC-------------------------------------------------------------------------
wifi_status get_utc(time_t *unix_time, uint32_t timeout);

//API´S-----------------------------------------------------------------------
wifi_status http_perform_as_stream_wheather(weather_info_t *info, char *location, uint32_t timeout);//wifi_status check_wheater(char *location, uint32_t timeout);
//wifi_status http_perform_as_stream_time(date_time_t *time, char *timezone,
	//	char *city, uint32_t timeout);


#endif /* INC_WIFI_DRIVER_H_ */
