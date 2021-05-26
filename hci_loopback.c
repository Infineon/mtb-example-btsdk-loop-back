/*
 * Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 *  hci_loopback.c
 *
 */

/******************************************************************************
 *                                Includes
 ******************************************************************************/
#include "sparcommon.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_transport.h"
#include "wiced_platform.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"
#include "wiced_timer.h"
#include "hci_control_api.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/
typedef enum
{
    HCI_PKT_32,
    HCI_PKT_128,
    HCI_PKT_250,
    HCI_MAX_PKT_SIZE = 250,
} HCI_TEST_PACKET_SIZE_t;

typedef enum
{
    FILL_TYPE_COUNT_UP,
    FILL_TYPE_COUNT_DOWN,
 //   FILL_TYPE_ALL_ONES,
 //   FILL_TYPE_ALL_ZEROS,
 //   FILL_TYPE_PSEUDORANDOM,
} HCI_FILL_TYPE_t;

typedef enum
{
    HCI_PKT_LOOP_1,
    HCI_PKT_LOOP_2,
} HCI_PACKETS_IN_LOOP_t;

typedef enum
{
    HCI_ON_ERROR_STOP,
    HCI_ON_ERROR_CONTINUE,
} HCI_ON_ERROR_t;

typedef struct
{
    HCI_TEST_PACKET_SIZE_t   packet_size;
    HCI_FILL_TYPE_t          packet_pattern;
    HCI_PACKETS_IN_LOOP_t    packets_in_loop;
    HCI_ON_ERROR_t           on_error;
} HCI_LOOPBACK_CONFIG_t;

/******************************************************************************
 *                                Structures
 ******************************************************************************/
extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[];

/******************************************************************************
 *                          Function Declarations
 ******************************************************************************/
static void     hci_test_init(void);
wiced_result_t  hci_test_app_management_cback(wiced_bt_management_evt_t event,
                                wiced_bt_management_evt_data_t *p_event_data);
static void     hci_test_timeout(uint32_t count);
uint32_t        hci_test_handle_command( uint8_t *p_data, uint32_t length );
static void     hci_test_control_transport_status( wiced_transport_type_t type );
static void     hci_control_misc_handle_get_version( void );
static void     hci_test_start(void);
static void     hci_test_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data);
void            hci_test_status_callback(uint8_t status);

void hci_test_spi_dbg_Init(void);
void hci_test_spi_dbg_Write(UINT8 *pData, UINT32 uDataSize);


/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
wiced_timer_t test_timer;                        /* Seconds timer instance */
#define TRANS_UART_BUFFER_SIZE          1030
int test_active = FALSE;
uint32_t rx_packet_count = 0;
uint32_t tx_packet_count = 0;
uint32_t rx_packet_error_count = 0;
uint8_t packet_buffer[2][TRANS_UART_BUFFER_SIZE] = {0};
uint8_t current_packet_buffer = 0;


HCI_LOOPBACK_CONFIG_t hci_loopback_cfg = {0};

const wiced_transport_cfg_t  transport_cfg =
{
    .type = WICED_TRANSPORT_UART,
    .cfg =
    {
        .uart_cfg =
        {
            .mode = WICED_TRANSPORT_UART_HCI_MODE,
#ifdef HCI_115200
            .baud_rate =  115200,
#else
            .baud_rate =  HCI_UART_DEFAULT_BAUD,
#endif
        },
    },
    .rx_buff_pool_cfg =
    {
        .buffer_size  = TRANS_UART_BUFFER_SIZE,
        .buffer_count = 4
    },
    .p_status_handler   = hci_test_control_transport_status,
    .p_data_handler     = hci_test_handle_command,
    .p_tx_complete_cback= NULL,
};
wiced_transport_buffer_pool_t*  host_trans_pool;

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/*
 Function name:
 application_start

 Function Description:
 @brief    Starting point of your application

 @param void

 @return void
 */
APPLICATION_START()
{

    wiced_transport_init( &transport_cfg );
    // create special pool for sending data to the MCU
    host_trans_pool = wiced_transport_create_buffer_pool(TRANS_UART_BUFFER_SIZE, 2);
#ifdef WICED_BT_TRACE_ENABLE
     wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif
    WICED_BT_TRACE("HCI Test Application\r\n");

    wiced_bt_stack_init(hci_test_app_management_cback,NULL,NULL);
}


/*
 Function name:
 hci_test_app_management_cback

 Function Description:
 @brief    Callback function that will be invoked by application_start()

 @param  event           Bluetooth management event type
 @param  p_event_data    Pointer to the the bluetooth management event data

 @return timer success/failure status of the callback function
 */
wiced_result_t
hci_test_app_management_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_SUCCESS;

    WICED_BT_TRACE("Received Event : %d\n", event);

    switch(event)
    {
    /* Bluetooth  stack enabled */
    case BTM_ENABLED_EVT:
        hci_test_init();
        break;

    default:
        WICED_BT_TRACE("Unknown Event\n");
        break;
    }

    return result;
}

/*
 * Application initialization occurring after the stack is enabled.
 */
void hci_test_init()
{
#ifdef WICED_BT_TRACE_ENABLE
    wiced_result_t status;

    WICED_BT_TRACE("Init test\n");
    /* Starting the app timer */
    wiced_init_timer(&test_timer, hci_test_timeout, 0, WICED_SECONDS_PERIODIC_TIMER);
    status = wiced_start_timer(&test_timer, 1);
    if (status != WICED_SUCCESS)
    {
        WICED_BT_TRACE("%s: wiced_start_timer failed, status:%d \n", __func__, status);
    }

    /* Register callback for receiving hci traces */
    wiced_bt_dev_register_hci_trace(hci_test_trace_callback);
#endif
}

void fill_buffer(uint8_t *p, uint16_t len, HCI_FILL_TYPE_t type)
{
    uint8_t filler;
    switch(type)
    {
        case FILL_TYPE_COUNT_UP:
            filler = 0;
            while(len--)
            {
                *p++ = filler++;
            }
            break;
        case FILL_TYPE_COUNT_DOWN:
            while(len--)
            {
                *p++ = (uint8_t)len;
            }
            break;
        default:
            WICED_BT_TRACE("unknown buffer fill type\n");
            break;
    }
}

/*
 * The function invoked on timeout of app seconds timer.
 */
void hci_test_timeout(uint32_t count)
{
    static uint32_t timer_count = 0;
    WICED_BT_TRACE("test tick: %d\n", timer_count++);
}

void hci_test_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data)
{
    wiced_transport_send_hci_trace(host_trans_pool, type, length, p_data);
}

void hci_test_send_packet(uint16_t len, HCI_FILL_TYPE_t type)
{
    fill_buffer(&packet_buffer[current_packet_buffer][0], len, type);
    if(WICED_SUCCESS == wiced_transport_send_data( HCI_CONTROL_HCITEST_EVENT_PACKET, &packet_buffer[current_packet_buffer][0], len ))
    {
        tx_packet_count++;
        current_packet_buffer ^= 1;
    }
    else
    {
        WICED_BT_TRACE("error in wiced_transport_send_data\n");
        test_active = FALSE;
    }
}

int handle_packet_error(uint32_t i, uint8_t *p, uint8_t byte)
{
    WICED_BT_TRACE("error in packet at index %d data %02x != pattern %02x\n", i, p[i], byte);
    rx_packet_error_count++;
    if(hci_loopback_cfg.on_error == HCI_ON_ERROR_STOP)
    {
        test_active = FALSE;
    }
    return (hci_loopback_cfg.on_error == HCI_ON_ERROR_CONTINUE);
}

void hci_test_check_packet(uint8_t *p, uint16_t len, HCI_FILL_TYPE_t type)
{
    uint8_t filler = 0;
    uint32_t i;
    switch(type)
    {
        case FILL_TYPE_COUNT_UP:
            filler = 0;
            for(i=0; i < len; i++)
            {
                if(p[i] != filler)
                {
                    if(!handle_packet_error(i, p, filler))
                        break;
                }
                filler++;
            }
            break;
        case FILL_TYPE_COUNT_DOWN:
            filler = len;
            for(i=0; i < len; i++)
            {
                filler--;
                if(p[i] != filler)
                {
                    if(!handle_packet_error(i, p, filler))
                        break;
                }
            }
            break;
        default:
            WICED_BT_TRACE("unknown buffer fill type\n");
            break;
    }
}

uint16_t hci_test_packet_size()
{
    uint16_t len;
    switch(hci_loopback_cfg.packet_size)
    {
        case HCI_PKT_32: len = 32; break;
        case HCI_PKT_128: len = 128; break;
        case HCI_PKT_250: len = 250; break;
        default: len = 32; break;
    }
    return len;
}

void hci_test_start(void)
{
    WICED_BT_TRACE("start test\n");
    test_active = TRUE;
    rx_packet_count = 0;
    tx_packet_count = 0;
    rx_packet_error_count = 0;
    // start sending packets
    hci_test_send_packet(hci_test_packet_size(), hci_loopback_cfg.packet_pattern);
    if(hci_loopback_cfg.packets_in_loop == HCI_PKT_LOOP_2)
    {
        hci_test_send_packet(hci_test_packet_size(), hci_loopback_cfg.packet_pattern);
    }
}

void hci_test_stop()
{
    WICED_BT_TRACE("Stop test tx %d packets rx %d packets, %d errors\n",
                    tx_packet_count, rx_packet_count, rx_packet_error_count);
    test_active = FALSE;
}

void handle_rx_test_packet(uint8_t *packet, uint16_t length)
{
    rx_packet_count++;
    if(test_active)
        hci_test_check_packet(packet, length, hci_loopback_cfg.packet_pattern);
    if(test_active)
    {
        // keep sending
        hci_test_send_packet(hci_test_packet_size(), hci_loopback_cfg.packet_pattern);
    }
    else
    {
        hci_test_stop();
    }
}

/*
 * hci_control_transport_status
 * This callback function is called when the MCU opens the Wiced UART
 */
static void hci_test_control_transport_status( wiced_transport_type_t type )
{
    WICED_BT_TRACE( "hci_test_control_transport_status %x \n", type );

    // Tell Host that App is started
    wiced_transport_send_data( HCI_CONTROL_EVENT_DEVICE_STARTED, NULL, 0 );
}
/*
 * Handle command received over the UART.  First buffer of the command is the opcode
 * of the operation.  Rest are parameters specific for particular command.
 *
 */
uint32_t hci_test_handle_command( uint8_t *p_data, uint32_t length )
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t* p_rx_buf = p_data;
    uint32_t rc = 0;

    if ( !p_rx_buf )
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }
    //Expected minimum 4 byte as the wiced header
    if( length < 4 )
    {
        WICED_BT_TRACE("invalid params\n");
        wiced_transport_free_buffer( p_rx_buf );
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    (void) payload_len; // makes compiler happy
    opcode = p_data[0] + ( p_data[1] << 8 );     // Get opcode
    payload_len = p_data[2] + ( p_data[3] << 8 );     // Get len
    p_data += 4;
    length -= 4;

    //WICED_BT_TRACE("Cmd:%04X\n", opcode);

    switch ( opcode )
    {
    case HCI_CONTROL_HCITEST_COMMAND:
        switch(p_data[0])
        {
            case 1:
                hci_test_start();
                break;
            case 2:
                hci_test_stop();
                break;
            case 3:
            default:
                if(test_active)
                {
              //    WICED_BT_TRACE("received %04x %02x %02x %02x\n", opcode, p_data[0], p_data[1], p_data[2]);
                    handle_rx_test_packet(&p_data[1], length-1);
                }
                break;
        }
        break;

    case HCI_CONTROL_HCITEST_CONFIGURE:
        if(length == 4 && !test_active)
        {
            hci_loopback_cfg.packet_size = p_data[0];
            hci_loopback_cfg.packet_pattern = p_data[1];
            hci_loopback_cfg.packets_in_loop = p_data[2];
            hci_loopback_cfg.on_error = p_data[3];
            WICED_BT_TRACE( "Configure: %d %d %d %d\n", p_data[0], p_data[1], p_data[2], p_data[3]);
        }
        break;

    case HCI_CONTROL_MISC_COMMAND_GET_VERSION:
        WICED_BT_TRACE( "Get Version:\n");
        hci_control_misc_handle_get_version();
        break;

    default:
        WICED_BT_TRACE( "Ignored\n");
        break;
    }

    //Freeing the buffer in which data is received
    wiced_transport_free_buffer( p_rx_buf );
    return rc;
}

/* Handle get version command */
void hci_control_misc_handle_get_version( void )
{
    uint8_t   tx_buf[15];
    uint8_t   cmd = 0;

// If this is 20819 or 20820, we do detect the device from hardware
#define RADIO_ID    0x006007c0
#define RADIO_20820 0x80
#define CHIP_20820  20820
#define CHIP_20819  20819
#if (CHIP==CHIP_20819) || (CHIP==CHIP_20820)
    uint32_t chip = CHIP_20819;
    if (*(UINT32*) RADIO_ID & RADIO_20820)
    {
        chip = CHIP_20820;
    }
#else
    uint32_t  chip = CHIP;
#endif

    tx_buf[cmd++] = WICED_SDK_MAJOR_VER;
    tx_buf[cmd++] = WICED_SDK_MINOR_VER;
    tx_buf[cmd++] = WICED_SDK_REV_NUMBER;
    tx_buf[cmd++] = WICED_SDK_BUILD_NUMBER & 0xFF;
    tx_buf[cmd++] = (WICED_SDK_BUILD_NUMBER>>8) & 0xFF;
    tx_buf[cmd++] = chip & 0xFF;
    tx_buf[cmd++] = (chip>>8) & 0xFF;
    tx_buf[cmd++] = (chip>>24) & 0xFF;
    tx_buf[cmd++] = 0; // not used

    /* Send MCU app the supported features */
    tx_buf[cmd++] = HCI_CONTROL_GROUP_HCITEST;

    wiced_transport_send_data( HCI_CONTROL_MISC_EVENT_VERSION, tx_buf, cmd );
}
