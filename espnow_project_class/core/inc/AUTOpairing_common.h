#ifndef AUTOpairing_common_H
#define AUTOpairing_common_H

#include "string.h"

#define ERROR_NOT_PAIRED    1
#define ERROR_MSG_TOO_LARGE 2
#define ERROR_SIN_RESPUESTA 3
#define ERROR_ENVIO_ESPNOW  4
#define ENVIO_OK            0


// message types
#define DATA      0b00000001
#define PAIRING   0b00000000
#define NODATA    0b00000011
#define PAN_DATA  0b00000010
#define MASK_MSG_TYPE 0b00000011


#define CANAL 6

// PAN address
#define MASK_PAN 0b00111100
#define PAN_OFFSET 2

// message flags
#define CHECK   0b10000000
#define RESERVED 0b01000000


// network node types
#define GATEWAY 0
#define ESPNOW_DEVICE 1

#define MAX_BUFF 100

#define MAGIC_CODE1 0xA5A5
#define MAGIC_CODE2 0xC7C7
#define INVALID_CODE 0

#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   WIFI_IF_STA

#define ESPNOW_MAXDELAY 512
#define ESPNOW_QUEUE_SIZE           10

// Para los mensajes PAN necesitamos también la MAC de origen uint8_t[6] y la edad del mensaje (uint32_t)
// Van a estar así:
//      tipo de mensaje (1byte)  |  MAC origen (6 bytes) | antiguedad ms (4 bytes) | cuerpo del mensaje (hasta completar 250 bytes) 
#define PAN_type_offset  0
#define PAN_MAC_offset   1
#define PAN_MAC_size     6
#define PAN_MSold_offset 7 //(1+6)
#define PAN_MSold_size   4
#define PAN_payload_offset 11 //(1+6+4)


typedef enum
{
    NO_UPDATE_FOUND,
    THERE_IS_AN_UPDATE_AVAILABLE,
} UpdateStatus;

typedef struct{      // new structure for pairing
    uint8_t msgType;
    uint8_t id;
    uint8_t macAddr[6];
    uint8_t channel;
    uint8_t padding[3];
}struct_pairing;



typedef struct {
	uint8_t mac_addr[ESP_NOW_ETH_ALEN];
	esp_now_send_status_t status;
} espnow_send_cb_t;

struct struct_espnow {      // esp-now message structure
    uint8_t msgType;
    uint8_t payload[249];
};

typedef struct{
	char *topic;
	char *payload;
	uint8_t macAddr[6];
  unsigned long ms_old;
}struct_espnow_rcv_msg;

char * messType2String(uint8_t type)
{

  const char *text;
  char *buffer = (char*)malloc(MAX_BUFF);
  switch (type & MASK_MSG_TYPE)
  {
    case PAN_DATA :
    text = " DATOS PAN";
    break;
    case DATA :
    text = " DATOS";
    break;
    case PAIRING :
    text = " EMPAREJAMIENTO";
    break;
    case NODATA :
    text = " SIN_DATOS";
    break;
    default: text = " Desconocido";
  }
  sprintf(buffer,"%s PAN: %d",text,(type&MASK_PAN)>>PAN_OFFSET);
  if(type & CHECK)
  {
	  strcat(buffer," & SOLICITA MENSAJES");
  }
  return buffer;
}

// this method will print all the 32 bits of a number
long long dec2bin(int n) {
  long long bin = 0;
  int rem, i = 1;

  while (n!=0) {
    rem = n % 2;
    n /= 2;
    bin += rem * i;
    i *= 10;
  }
  return bin;
}

bool to_hex(uint8_t* dest, size_t dest_len, const uint8_t* values, size_t val_len) {
    static const char hex_table[] = "0123456789ABCDEF";
    if(dest_len < (val_len*2+1)) /* check that dest is large enough */
        return false;
    while(val_len--) {
        /* shift down the top nibble and pick a char from the hex_table */
        *dest++ = hex_table[*values >> 4];
        /* extract the bottom nibble and pick a char from the hex_table */
        *dest++ = hex_table[*values++ & 0xF];
    }
    *dest = 0;
    return true;
}



// network node types
#define GATEWAY 0
#define ESPNOW_DEVICE 1


#endif
