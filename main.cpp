#include "mbed.h"
#include "main.h"
#include "sx1276-hal.h"
#include "modulation.h"

#include "SDFileSystem.h"
#include <cstdint>


//pin serial utilisé
#define MCU_TX PC_10 
#define MCU_RX PC_11 

//taille du buffer Serial
#define SERIAL_BUFFER_SIZE 255

//taille des données reçu (Serial)
#define SERIAL_DATA_SIZE 255


UpdateState_t updState = HS;

volatile AppStates_t State = LOWPOWER;

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*
 *  Global variables declarations
 */
SX1276MB1xAS Radio( NULL );

const uint8_t ACK[] = "ACK";
const uint8_t TIMESET[] = "TIMESET";
const uint8_t TYPESET[] = "TYPESET";
const uint8_t BANDSET[] = "BANDSET";
const uint8_t FDEVSET[] = "FDEVSET";
const uint8_t DRSET[] =   "DATASET";
const uint8_t PATSET[] =  "PATSET";

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

uint8_t frame[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

int16_t RssiValue = 0.0;
int8_t SnrValue = 0.0;

uint8_t currentFrameType = 0b1011;


RadioConfig conf;


uint32_t freqTL[4] = {434400000,434400000,434400000,434400000};
uint32_t fDevTL[4] = {0,0,0,0};
uint32_t bandTL[4] = {0,0,0,0};
uint32_t drTL[4] = {12,11,10,7};
ModemType typeTL[4] = {MODEM_LORA,MODEM_LORA,MODEM_LORA,MODEM_LORA};
uint8_t crTL[4] = {1,1,1,1};
uint8_t ptxTL[4] = {14,14,14,14}; 

uint32_t freqTemp;
uint32_t fDevTemp;
uint32_t bandTemp;
uint32_t drTemp;

ModemType typeT;
uint8_t bandT;
uint8_t sfT;
uint8_t crT;
uint8_t ptxT;

uint16_t cycleCounter = 0;


uint8_t pattern[16];
uint8_t patternSize=0;

uint32_t hrtime=0;
Timer timerLoop;
uint8_t ID_config = 0;
uint8_t timeBeforeFlush = 0;

Serial pcCOM(USBTX,USBRX);


Serial mcuCOM(MCU_TX,MCU_RX);

//SD
//SDFileSystem sd(PC_3, PC_2, PB_10, PB_9, "sd");
FILE *file;



Timer gTimer;
uint32_t timeToSend = 0;
uint8_t indexType = 0;

uint8_t lastMinute = 0;

struct tm * timeInfosP;
time_t timestampP;

int main( void )
{
    pcCOM.baud (115200);
    pcCOM.printf( "\n\n\r     Transmitter Started \n\n\r" );

    // Initialize Radio driver
    
    init_lora();
    init_serial();
    //init_SD();
    
    pcCOM.printf("Starting Update process...\r\n" );

    
    //OTA_update();   
    changeConfig(0);

    pcCOM.printf("CONFIG SET %u %u %u %u %u %u %u\n",conf.freq,conf.datarate,conf.coderate,conf.power,conf.bandwidth,conf.dev, conf.type);
    pcCOM.printf("Starting sending process...\r\n" );
    

    processingLoop();

}

/*
#######################  INITIALIZATION  #######################
*/
#pragma region INITIALIZATION

void init_serial()
{
    pcCOM.printf("Init MCU Serial\n\r");
    mcuCOM.baud (115200);
    mcuCOM.attach(&serialRX_ISR,Serial::RxIrq);
}
void init_lora()
{
    initRadioEvent();

    while( Radio.Read( REG_VERSION ) == 0x00  ) {
        pcCOM.printf( "Radio could not be detected!\n\r", NULL );
        wait( 1 );
    }
    conf = (RadioConfig) {
        MODEM_LORA, RF_FREQUENCY, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,LORA_CODINGRATE, 0,
                    LORA_PREAMBLE_LENGTH, LORA_SYMBOL_TIMEOUT,LORA_FIX_LENGTH_PAYLOAD_ON,0,
                    LORA_CRC_ENABLED,LORA_FHSS_ENABLED,LORA_NB_SYMB_HOP,LORA_IQ_INVERSION_ON,
                    true,TX_OUTPUT_POWER,FSK_FDEV
    };
}

void init_SD()
{
    
    mkdir("/sd/saves", 0777);
    file = fopen("/sd/saves/logdataLTEST.bin", "a");
    if(file == NULL) {
        error("Could not open file for write\n");
        while(1);
    }
    fprintf(file, "%c",'\0');
    
}
#pragma endregion
/*
#######################  PARAMETRIZATION  #######################
*/
#pragma region PARAMETRIZATION

void OTA_update()
{
    setRF(conf,&Radio,false);
    wait_ms(500);
    bool end = false;
    Radio.Rx( 4000 );
    while( !end ) {
        switch( State ) {
            case RX:
                State = LOWPOWER;
                if( BufferSize > 0 ) {


                    if(strcmp((char*)Buffer,"HSINIT") == 0) {
                        updState = TIME;
                        pcCOM.printf("-----HandShake received--------\n");
                        sendData(ACK,4);
                    } else if(updState == TIME) {
                        updState = CONFIGTYPE;
                        setTime();
                        pcCOM.printf("Time set \n");
                        sendData(TIMESET,8);
                    } else if(updState == CONFIGTYPE) {
                        updState = CONFIGBAND;
                        setConfig();
                        pcCOM.printf("type + power set \n");
                        sendData(TYPESET,10);
                    } else if(updState == CONFIGBAND) {
                        updState = CONFIGFDEV;
                        if(typeT == MODEM_FSK)
                            bandTemp = atoi((char*)Buffer);
                        if(typeT == MODEM_LORA) {
                            bandT = Buffer[0];
                        }
                        pcCOM.printf("band set \n");
                        sendData(BANDSET,10);
                    } else if(updState == CONFIGFDEV) {
                        updState = CONFIGDR;
                        if(typeT == MODEM_FSK)
                            fDevTemp = atoi((char*)Buffer);
                        if(typeT == MODEM_LORA) {
                            crT = Buffer[0];
                        }
                        pcCOM.printf("freq dev / CR set \n");
                        sendData(FDEVSET,10);
                    } else if(updState == CONFIGDR) {
                        updState = PATTERN;
                        if(typeT == MODEM_FSK)
                            drTemp = atoi((char*)Buffer);
                        if(typeT == MODEM_LORA) {
                            sfT = Buffer[0];
                        }
                        pcCOM.printf("datarate set \n");
                        sendData(DRSET,10);
                    } else if(updState == PATTERN) {
                        updState = END;
                        setPattern();
                        pcCOM.printf("pattern set \n");
                        sendData(PATSET,11);
                    } else if(updState == END) {
                        freqTemp = atoi((char*)Buffer);
                        pcCOM.printf("OTA CONFIG END");
                        end = true;
                    } else {
                        //error, restarting process
                        updState = HS;
                        Radio.Rx( 4000 );
                    }

                }
                memset(Buffer, 0, 32);
                break;
            case TX:
                Radio.Rx( 3000 );
                State = LOWPOWER;
                break;
            //si rien reçu, on renvoi le handshake
            case RX_TIMEOUT:
                updState = HS;
                Radio.Rx( 4000 );
                State = LOWPOWER;
                break;
            case RX_ERROR:
                updState = HS;
                Radio.Rx( 4000 );
                State = LOWPOWER;
                break;
            case LOWPOWER:
                break;
            default:
                State = LOWPOWER;
                break;
        }
    }
    conf.freq = freqTemp;
    conf.power = ptxT;
    conf.type= typeT;
    if(conf.type == MODEM_LORA) {
        conf.datarate = sfT;
        conf.coderate = crT;
        conf.bandwidth = bandT;
    } else {
        conf.datarate = drTemp;
        conf.dev = fDevTemp;
        conf.bandwidth = bandTemp;
    }
    conf.bandwidthAfc = 83333;
    
    pcCOM.printf("UPDATE END %u %u %u %u %u %u %u\n",conf.freq,conf.datarate,conf.coderate,conf.power,conf.bandwidth,conf.dev, conf.type);
}

void changeConfig(uint8_t id)
{
    conf.freq = freqTL[id];
    conf.power = ptxTL[id];
    conf.type= typeTL[id];
    if(conf.type == MODEM_LORA) {
        conf.datarate = drTL[id];
        conf.coderate = crTL[id];
        conf.bandwidth = bandTL[id];
    } else {
        conf.datarate = drTL[id];
        conf.dev = fDevTL[id];
        conf.bandwidth = bandTL[id];
    }   
    setRF(conf,&Radio,true);
    wait_ms(3000);
}

void setPattern()
{
    uint8_t jj = 0;
    while(Buffer[jj]!=64 && Buffer[jj]!=0 ) {
        pcCOM.printf(" type : %u %u \n",jj, Buffer[jj]-48);
        pattern[jj] = Buffer[jj]-48;
        jj++;
    }
    pcCOM.printf(" size %u \n",jj);
    patternSize = jj;
}

void setConfig()
{
    typeT = MODEM_LORA;
    if (Buffer[0] == 0)
        typeT = MODEM_FSK;
    ptxT = Buffer[1];
}

//unused
void setTime(float t){
    //hhmmss
    int f = (int) t;
    uint8_t h = (uint8_t) (f/10000.0);
    uint16_t temp =   f - h*10000; //mmss
    uint8_t m = (uint8_t) (temp/100.0);
    uint8_t s = temp - m*100; //ss
    
    time_t timestamp = time( NULL );
    
    struct tm * timeInfos = localtime( & timestamp );
    if(timeInfos->tm_hour + 1 < h || timeInfos->tm_hour - 1 > h || h==0)//ça veut dire que le gps n'as pas la bonne heure (difficile d'avoir plus d'une heure de décallage sur un vol de 5h)
    {
        delete timeInfos;
        return;
    }
    
    
    timeInfos->tm_hour = h;
    timeInfos->tm_min = m;
    timeInfos->tm_sec= s;
    pcCOM.printf("TIME : %u %u %u\n\r",h,m,s);
    time_t newTimestamp = mktime(timeInfos);
    set_time(newTimestamp);
    delete timeInfos;
}
//get info from buffer
void setTime()
{
    struct tm myDate;
    myDate.tm_mday = Buffer[0];
    myDate.tm_mon = Buffer[1];
    myDate.tm_year = Buffer[2];
    myDate.tm_hour = Buffer[3];
    myDate.tm_min = Buffer[4];
    myDate.tm_sec = Buffer[5];
    time_t timestamp = mktime( & myDate );
    set_time(timestamp);
    
    lastMinute = myDate.tm_min;
    
    time_t seconds = time(NULL);
    char timet[32];
    strftime(timet, 32, "%c \n\r", localtime(&seconds));
    pcCOM.printf(" TIME : ");
    pcCOM.printf(timet);
    pcCOM.printf("\n\r");
}

#pragma endregion
/*
#######################  MAIN LOOP  #######################
*/
#pragma region MAIN LOOP

volatile char SerialBuffer[SERIAL_BUFFER_SIZE];
volatile char SerialSecondBuffer[SERIAL_BUFFER_SIZE];
volatile uint16_t SerialIndex = 0;
volatile uint8_t new_data = 0;
volatile uint8_t serial_data_size = 0;
volatile uint8_t reading = 0;

void serialRX_ISR(void)
{
    while(mcuCOM.readable())
    {    
        char c = mcuCOM.getc();

        if(reading==0){
            serial_data_size = (uint8_t) c;
            reading = 1;
            continue;
        }
        SerialBuffer[SerialIndex] = c;
        //pcCOM.printf("%c",c);
        if(SerialIndex==serial_data_size-1) {
            memcpy((void*)SerialSecondBuffer, (void*) SerialBuffer, serial_data_size);
            SerialIndex = 0;
            new_data = 1;
            reading = 0;
            //pcCOM.printf("\n\r");
            return;
        }
        SerialIndex++;
    }
}

void updateLoop()
{

}

void processingLoop()
{
    
    bool ready_to_TX = false;
    wait_ms(500);
    setRF(conf,&Radio,true);
    wait_ms(500);
    sendData(frame, 16);
    hrtime = 0;
    timeToSend=0;
    gTimer.start();
    while( 1 ) {
        updateLoop();
        timeToSend = (uint32_t) gTimer.read_ms();
        if(timeToSend > 1<<16) {
            gTimer.stop();
            gTimer.reset();
            gTimer.start();
            hrtime = (uint32_t) (hrtime + timeToSend);
            timeToSend = 0;
        }
    
        switch( State ) {

            case TX: {
                State = LOWPOWER;
                ready_to_TX = true;
                break;
            }
            case TX_TIMEOUT:
                pcCOM.printf("TX TIMEOUT");
                State = LOWPOWER;
                break;
            case LOWPOWER:
                break;
            default:
                State = LOWPOWER;
                break;
        }
        //if we have new data ready to be sent and transmitter is ready to transmit
        if(ready_to_TX && new_data == 1){
            ready_to_TX = false;
            pcCOM.printf("NEW FRAME READY");
            currentFrameType = pattern[indexType];
            sendFrame(currentFrameType);
            indexType++;
            if(indexType >= patternSize)
                indexType = 0;
        }
    }
}

void sendFrame(uint8_t type)
{
    uint8_t cframe[255];
    uint16_t tsize = createFrame(cframe,type);
    sendData(cframe,(uint8_t)ceil(tsize/8.0f));
    
    /*
    writeSD(cframe,(uint8_t)ceil(tsize/8.0f),file);
    timeBeforeFlush++;
    
    if(timeBeforeFlush > 10) {
        pcCOM.printf("saving!\n");
        fflush(file);
        timeBeforeFlush = 0;
    }
    */
}

void writeSD(uint8_t *data,uint8_t length, FILE* file)
{
    if (file != NULL) {
        for (uint8_t count = 0; count < length; count++) {
            fputc( data[count], file);
        }
    } else {
        printf("failed to write in file!\n");
    }
}

#pragma endregion
/*
#######################  FRAME GENERATION  #######################
*/
#pragma region FRAME GENERATION
uint16_t createFrame(uint8_t *frames, uint8_t type)
{
    uint16_t frameIndex = 0;
    uint16_t totalSize = 0;

    //HIGH RES TIME
    if(( (type & 0b0010)>>1 ) == 1) {
        add_high_res_time(frames, &frameIndex, &totalSize); 
    }
    //FULL TIME
    if(( (type & 0b0100)>>2 ) == 1) {
        add_full_time(frames, &frameIndex, &totalSize); 
    }

    add_serial_data(frames, &frameIndex, &totalSize);

    //Ajout de l'indicateur de Trame
    shift(frames,4,frameIndex);
    frames[0] |= currentFrameType<<4;
    totalSize += 4;
    return totalSize;
}


void add_high_res_time(uint8_t *frames, uint16_t *frameIndex, uint16_t *totalSize)
{
    frames[*frameIndex] = ((timeToSend + hrtime) & 0xFF0000)>> 16;
    frames[*frameIndex+1] = ((timeToSend + hrtime) & 0x00FF00) >> 8;
    frames[*frameIndex+2] = (timeToSend + hrtime) & 0x0000FF;
    *frameIndex+=3;
    *totalSize += 24;
}
void add_full_time(uint8_t *frames, uint16_t *frameIndex, uint16_t *totalSize)
{
        time_t seconds = time(NULL);//fait rentrer le temps unix dans 24bit
        seconds = seconds - ((int) (seconds/10000000.0))*10000000;
        frames[*frameIndex] = (seconds & 0xFF0000)>>16;
        frames[*frameIndex+1] = (seconds & 0x00FF00)>>8;
        frames[*frameIndex+2] = seconds & 0x0000FF;

        *frameIndex += 3;
        *totalSize += 24;
}

void add_serial_data(uint8_t *frames, uint16_t *frameIndex, uint16_t *totalSize)
{
        uint8_t dataSerial[serial_data_size];
        memcpy(dataSerial, (void*) SerialSecondBuffer, serial_data_size);
        //mcuCOM.printf("OK");
        new_data = 0;
        for(uint8_t i=0; i<serial_data_size; i++) {
            frames[*frameIndex] = dataSerial[i];
            *frameIndex +=1;
            *totalSize += 8;
        }
}

#pragma endregion
/*
#######################  RADIO  #######################
*/
#pragma region RADIO

void sendData(uint8_t* bytes,uint8_t len)
{
    pcCOM.printf("send %u bytes\n",len);
    Radio.Send(bytes, len);
}

void sendData(const uint8_t* bytes,uint8_t len)
{
    char text[255];
    strcpy(text, (char*)bytes);
    pcCOM.printf("%s %u\r\n",text, len);
    Radio.Send((uint8_t*)text, len);
}
void shift(uint8_t *arr,uint8_t sh,uint8_t size)
{
    for(uint8_t i=size; i>0; i--) {
        arr[i] = (arr[i-1]<<(8-sh)) | arr[i]>>sh;
    }
    arr[0] = arr[0]>>sh;
}

void initRadioEvent()
{
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxError = OnRxError;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    Radio.Init(&RadioEvents);
}
volatile int past=0;

void OnTxDone( void )
{
    Radio.Sleep( );
    State = TX;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    State = TX_TIMEOUT;
}

void OnRxTimeout( void )
{
    Radio.Sleep( );
    Buffer[BufferSize] = 0;
    State = RX_TIMEOUT;
}

void OnRxError( void )
{
    Radio.Sleep( );
    State = RX_ERROR;
    pcCOM.printf( "> OnRxError\n\r" );
}

#pragma endregion
/*
#######################  OTHER  #######################
*/

void getRandom(uint8_t *ran,uint8_t size)
{
    srand(time(NULL));
    for(uint8_t i = 0; i<size; i++) {
        *(ran+i) = rand()%(255);
    }
}



