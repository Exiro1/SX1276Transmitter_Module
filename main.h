/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    ( C )2014 Semtech

Description: Contains the callbacks for the IRQs and any application related details

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __MAIN_H__
#define __MAIN_H__

#include <cstdint>


/* Set this flag to '1' to display pcCOM.printf messages on the console */
#define DEBUG_MESSAGE   1

/* Set this flag to '1' to use the LoRa modulation or to '0' to use FSK modulation */
#define USE_MODEM_LORA  1
#define USE_MODEM_FSK   !USE_MODEM_LORA

#define RF_FREQUENCY                                    434400000 // Hz
#define TX_OUTPUT_POWER                                 9        // 14 dBm


#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
//  1: 250 kHz,
//  2: 500 kHz,
//  3: Reserved]
#define LORA_SPREADING_FACTOR                       9         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
//  2: 4/6,
//  3: 4/7,
//  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_FHSS_ENABLED                           false
#define LORA_NB_SYMB_HOP                            4
#define LORA_IQ_INVERSION_ON                        false
#define LORA_CRC_ENABLED                            true


#define FSK_FDEV                                    25000     // Hz
#define FSK_DATARATE                                19200     // bps
#define FSK_BANDWIDTH                               50000     // Hz
#define FSK_AFC_BANDWIDTH                           83333     // Hz
#define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false
#define FSK_CRC_ENABLED                             true


#define RX_TIMEOUT_VALUE                                3500      // in ms
#define BUFFER_SIZE                                     64        // Define the payload size here

#define CONFIGNBR 4
#define WAIT_LOOP                                     333 //ms 

#define FILEPATH "logdata1.bin"


typedef enum {
    LOWPOWER = 0,
    IDLE,

    RX,
    RX_TIMEOUT,
    RX_ERROR,

    TX,
    TX_TIMEOUT,

    CAD,
    CAD_DONE
} AppStates_t;

typedef enum {
    HS,
    TIME,
    CONFIGTYPE,
    CONFIGBAND,
    CONFIGFDEV,
    CONFIGDR,
    PATTERN,
    END
} UpdateState_t;


void init_SD();
void init_serial();
void init_lora();

void serialRX_ISR(void);

//get configuration from receiver

void OTA_update();
void setPattern();
void setConfig();
void setTime(float t);
void setTime();
void changeConfig(uint8_t id);

//main loop
void processingLoop();

void sendFrame(uint8_t type);

//creating frame
uint16_t createFrame(uint8_t *frames, uint8_t type);

void add_high_res_time(uint8_t *frames, uint16_t *frameIndex, uint16_t *totalSize);
void add_full_time(uint8_t *frames, uint16_t *frameIndex, uint16_t *totalSize);

void add_serial_data(uint8_t *frames, uint16_t *frameIndex, uint16_t *totalSize);

//sd data save
void write_to_SD_card(uint8_t *data,uint8_t length, FILE* file);

//Sending data
void sendData(uint8_t* bytes,uint8_t len);
void sendData(const uint8_t* bytes,uint8_t len);
void shift(uint8_t *arr,uint8_t sh,uint8_t size);


//RADIO EVENT

void initRadioEvent();
void OnTxDone( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void OnTxTimeout( void );
void OnRxTimeout( void );
void OnRxError( void );








#endif // __MAIN_H__

