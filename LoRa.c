#include "LoRa.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include <applibs/log.h>
#include <applibs/uart.h>
#include <applibs/gpio.h>

#include "peripheral_utilities.h"
#include "string_utilities.h"

#include "LoRa_ChipConfig.h"
#include "LoRa_Hal.h"

#define LORA_MAC_TX    "mac tx "
#define LORA_JOIN      "mac join "
#define LORA_RADIO_TX  "radio tx "
#define LORA_RADIO_RX  "radio rx "

/**
 * Timer Limit ( ms ) */
#define LORA_TIMER_EXPIRED 50000

/**
 * Command String Max Size */
#define  LORA_MAX_CMD_SIZE 64

/**
 * Response String Max Size */
#define LORA_MAX_RSP_SIZE 20

/**
 * Data String Max Size */
#define LORA_MAX_DATA_SIZE 256
#define LORA_MAX_TRANSFER_SIZE 384

/* Buffers */
static char            _tx_buffer[ LORA_MAX_TRANSFER_SIZE ];
static char            _rx_buffer[ LORA_MAX_TRANSFER_SIZE ];
static uint16_t        _rx_buffer_len;

/* Timer Flags and Counter */
static bool            _timer_f;
static bool            _timeout_f;
static bool            _timer_use_f;
static uint32_t        _ticker;
static uint32_t        _timer_max;

/* Process Flags */
static bool            _rsp_rdy_f;
static bool            _lora_rdy_f;

/* Response vars */
static bool                     _rsp_f;
static char*                    _rsp_buffer;
static bool                     _callback_default;
static struct timespec delay100ms = {.tv_sec = 0, .tv_nsec = 1000 * 1000 * 100};
static struct timespec delay1sec = {.tv_sec = 1, .tv_nsec = 0};


static void _delay_100ms(void) {
    nanosleep(&delay100ms, NULL);
}

static void _delay_1sec(void) {
    nanosleep(&delay1sec, NULL);
}

static void _lora_resp(void)
{
    _rx_buffer_len  = 0;
    _lora_rdy_f     = false;
    _rsp_rdy_f      = false;
    _rsp_f          = true;
}

static uint8_t _lora_par(void)
{
    Log_Debug("[DEBUG] _lora_par : %s\n", _rx_buffer);

    if( !strcmp( _rx_buffer, "invalid_param" ) )
        return 1;
    if( !strcmp( _rx_buffer, "not_joined" ) )
        return 2;
    if( !strcmp( _rx_buffer, "no_free_ch" ) )
        return 3;
    if( !strcmp( _rx_buffer, "silent" ) )
        return 4;
    if( !strcmp( _rx_buffer, "frame_counter_err_rejoin_needed" ) )
        return 5;
    if( !strcmp( _rx_buffer, "busy" ) )
        return 6;
    if( !strcmp( _rx_buffer, "mac_paused" ) )
        return 7;
    if( !strcmp( _rx_buffer, "invalid_data_len" ) )
        return 8;
    if( !strcmp( _rx_buffer, "keys_not_init" ) )
        return 9;
    return 0;
}
static uint8_t _lora_repar(void)
{
    Log_Debug("[DEBUG] _lora_repar : %s\n", _rx_buffer);

    if( !strcmp( _rx_buffer, "mac_err" ) )
        return 10;
    if( !strcmp( _rx_buffer, "mac_tx_ok" ) )
        return 0;
    if( !strcmp( _rx_buffer, "mac_rx" ) )
        return 12;
    if( !strcmp( _rx_buffer, "invalid_data_len" ) )
        return 13;
    if( !strcmp( _rx_buffer, "radio_err" ) )
        return 14;
    if( !strcmp( _rx_buffer, "radio_tx_ok" ) )
        return 0;
    if( !strcmp( _rx_buffer, "radio_rx" ) )
        return 0;
    if( !strcmp( _rx_buffer, "accepted" ) )
        return 0;
    if( !strcmp( _rx_buffer, "denied" ) )
        return 18;
    return 0;
}

static void _lora_write(void)
{
    char *ptr = _tx_buffer;

    Log_Debug("[DEBUG] UART > %s\n", _tx_buffer);

    while( *ptr )
        LoRa_hal_uartWrite( *ptr++ );

    LoRa_hal_uartWrite( '\r' );
    LoRa_hal_uartWrite( '\n' );

    _rx_buffer_len  = 0;
    _lora_rdy_f     = false;
    _rsp_rdy_f      = false;
    _timer_f        = true;
    _rsp_f          = true;
}

static void _lora_read(void)
{
    if( !_rsp_f )
    {
        LoRa_hal_gpio_csSet( true );
        LoRa_hal_gpio_csSet( false );

    } 
    else if( _rsp_f )
    {
        LoRa_hal_gpio_csSet( true );
        strcpy( _rsp_buffer, _rx_buffer );
        LoRa_hal_gpio_csSet( false );
    }

    _lora_rdy_f     = true;
    _rsp_rdy_f      = false;
    _timer_f        = false;
    _rsp_f          = true;
}


/* --------------------------------------------------------- PUBLIC FUNCTIONS */
void lora_uartDriverInit(void)
{
    if (!LoRa_hal_uartMap()) {
        return;
    }
    
    if (!LoRa_hal_gpio_gpioMap()) {
        return;
    }

    Log_Debug("LoRa/UART Driver Initialized...\n");
}

/* ----------------------------------------------------------- IMPLEMENTATION */
/******************************************************************************
*  LoRa INIT
*******************************************************************************/
void lora_init()
{
    lora_uartDriverInit();

    LoRa_hal_gpio_rstSet( 1 );
    _delay_100ms();
    LoRa_hal_gpio_rstSet( 0 );
    _delay_100ms();
    _delay_100ms();
    _delay_100ms();
    LoRa_hal_gpio_rstSet( 1 );
    _delay_100ms();
    LoRa_hal_gpio_csSet( 1 );
    
    memset( _tx_buffer, 0, LORA_MAX_CMD_SIZE + LORA_MAX_DATA_SIZE );
    memset( _rx_buffer, 0, LORA_MAX_RSP_SIZE + LORA_MAX_DATA_SIZE );
    
    _timer_max          = LORA_TIMER_EXPIRED;
    _rx_buffer_len      = 0;
    _ticker             = 0;
    _timer_f            = false;
    _timeout_f          = false;
    _timer_use_f        = false;
    _rsp_f              = false;
    _rsp_rdy_f          = false;
    _lora_rdy_f         = true;
    
    _delay_1sec();
}
/******************************************************************************
*  LoRa CMD
*******************************************************************************/
void lora_cmd(char *cmd,  char *response)
{
    while( !_lora_rdy_f )
        lora_process();

    strcpy( _tx_buffer, cmd );

    _rsp_buffer = response;
    _lora_write();

    while( !_lora_rdy_f )
        lora_process();

    Log_Debug( "[DEBUG] UART < %s", response);
}
/******************************************************************************
* LoRa MAC TX
*******************************************************************************/
uint8_t lora_mac_tx(char* payload, char* port_no, char *buffer, char *response)
{
    uint8_t res   = 0;

    while( !_lora_rdy_f )
        lora_process();

    strcpy( _tx_buffer, ( char* )LORA_MAC_TX );
    strcat( _tx_buffer, payload);
    strcat( _tx_buffer, " " );
    strcat( _tx_buffer, port_no );
    strcat( _tx_buffer, " " );
    strcat( _tx_buffer, buffer );
    _rsp_buffer = response;
    _lora_write();

    while( !_lora_rdy_f )
        lora_process();

    if( ( res = _lora_par() ) )
        return res;

    _lora_resp();

    do 
    {
        while( !_lora_rdy_f )
            lora_process();

    } while( ( res = _lora_repar() ) == 12 );

    return res;
}
/******************************************************************************
*  LoRa JOIN
*******************************************************************************/
uint8_t lora_join(char* join_mode, char *response)
{
    uint8_t res = 0;

    while( !_lora_rdy_f )
        lora_process();

    strcpy( _tx_buffer, ( char* )LORA_JOIN );
    strcat( _tx_buffer, join_mode );
    _rsp_buffer = response;
    _lora_write();

    while( !_lora_rdy_f )
        lora_process();

    if( ( res = _lora_par() ) )
        return res;

    _lora_resp();

    while( !_lora_rdy_f )
        lora_process();

    return _lora_repar();
}
/******************************************************************************
* LORA RX
*******************************************************************************/
uint8_t lora_rx(char* window_size, char *response)
{
    uint8_t res = 0;

    while( !_lora_rdy_f )
        lora_process();

    strcpy( _tx_buffer, "radio rx " );
    strcat( _tx_buffer, window_size );
    _rsp_buffer = response;
    _lora_write();

   while( !_lora_rdy_f )
        lora_process();

    if(res == _lora_par())
        return res;

    _lora_resp();

    while( !_lora_rdy_f )
        lora_process();

    return _lora_repar();
}
/******************************************************************************
* LORA TX
*******************************************************************************/
uint8_t lora_tx( char *buffer )
{
    uint8_t res = 0;
    
    lora_process();
    strcpy( _tx_buffer, "radio tx ");
    strcat( _tx_buffer, buffer );

    _lora_write();

    lora_process();

    if( ( res = _lora_par() ) )
        return res;

    _lora_resp();
    lora_process();

    return _lora_repar();
}
/******************************************************************************
* LORA RX ISR
*******************************************************************************/
void lora_rx_isr( char rx_input )
{
    _rx_buffer[ _rx_buffer_len++ ] = rx_input;
     if ( rx_input == '\r' )
    {
        _rx_buffer[ _rx_buffer_len++ ] = '\0';
        _rsp_rdy_f = true;  
    } 
}
/******************************************************************************
* LORA TICK ISR
*******************************************************************************/
void lora_tick_isr()
{
    if( _timer_use_f )
        if( _timer_f && ( _ticker++ > _timer_max ) )
            _timeout_f = true;
}
/******************************************************************************
* LoRa TICK CONF
*******************************************************************************/
void lora_tick_conf( uint32_t timer_limit )
{
    if ( timer_limit )
    {
        _timer_max = timer_limit;
        _timer_use_f = true;
    } 
    else 
    {
        _timer_max = LORA_TIMER_EXPIRED;
        _timer_use_f = false;
    }
}
/******************************************************************************
*  LoRa PROCESS
*******************************************************************************/
void lora_process()
{
    uint8_t tmp;

    while (LoRa_hal_uartRead(&tmp) > 0)
    {
        lora_rx_isr( tmp );
    }

    if ( _rsp_rdy_f )
    {        
        _lora_read();
    }

    if ( _timeout_f )
    {
        _lora_read();
    }
}