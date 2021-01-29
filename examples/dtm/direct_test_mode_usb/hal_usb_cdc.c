#include "hal_usb_cdc.h"

#include <stdint.h>
#include <stddef.h>

#include "nrf.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_power.h"

#include "app_error.h"
#include "app_util.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"
#include "app_fifo.h"

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst, app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1

/** @brief CDC_ACM class instance */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250);

#define RAW_BUFFER_SIZE 512
#define READ_SIZE (NRFX_USBD_EPSIZE * 2)
#define SEND_SIZE (NRFX_USBD_EPSIZE)
static char m_rx_buffer[READ_SIZE];
static char m_tx_buffer[SEND_SIZE];
static volatile bool m_port_opened = false;
static volatile bool m_transmitting = false;

static app_fifo_t m_rx_fifo;
static app_fifo_t m_tx_fifo;
static uint8_t tx_buffer[RAW_BUFFER_SIZE];
static uint8_t rx_buffer[RAW_BUFFER_SIZE];

#define FIFO_LENGTH(p_fifo)     fifo_length(p_fifo)  /**< Macro for calculating the FIFO length. */
#define IS_FIFO_FULL(p_fifo)    fifo_full(p_fifo)
static __INLINE uint32_t fifo_length(app_fifo_t * p_fifo) {
    uint32_t tmp = p_fifo->read_pos;
    return p_fifo->write_pos - tmp;
}
static __INLINE bool fifo_full(app_fifo_t * p_fifo) {
    return (FIFO_LENGTH(p_fifo) > p_fifo->buf_size_mask);
}

// Don't call this function in the main thread, app_usbd_cdc_acm_write() causes interrupt
// before return, which will mess up m_transmitting state
static void usb_cdc_schedule_tx() {
    if (!FIFO_LENGTH(&m_tx_fifo) || m_transmitting) {
        return;
    }

    size_t to_send = 0;
    uint8_t data;
    while ((to_send < SEND_SIZE) && app_fifo_get(&m_tx_fifo, &data) == NRF_SUCCESS) {
        m_tx_buffer[to_send++] = data;
    }

    if (to_send) {
        // app_usbd_cdc_acm_write() causes interrupt before return,
        // which will mess up m_transmitting state
        m_transmitting = true;
        if (app_usbd_cdc_acm_write(&m_app_cdc_acm, m_tx_buffer, to_send) != NRF_SUCCESS) {
            m_transmitting = false;
        }
    }
}

/**
 * @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t (headphones)
 * */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
            m_port_opened = true;
            m_transmitting = false;
            app_fifo_flush(&m_rx_fifo);
            app_fifo_flush(&m_tx_fifo);
            /*Setup first transfer*/
            ret_code_t ret = app_usbd_cdc_acm_read_any(&m_app_cdc_acm, m_rx_buffer, READ_SIZE);
            UNUSED_VARIABLE(ret);
            break;
        }
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            m_port_opened = false;
            break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
        {
            m_transmitting = false;
            usb_cdc_schedule_tx();
            break;
        }
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
            ret_code_t ret;
            // NRF_LOG_INFO("Bytes waiting: %d", app_usbd_cdc_acm_bytes_stored(p_cdc_acm));
            do
            {
                /*Get amount of data transfered*/
                size_t size = app_usbd_cdc_acm_rx_size(p_cdc_acm);
                if (size > 0) {
                    // FIXME: handler buffer overflow state
                    for (size_t i = 0; i < size; i++) {
                        SPARK_ASSERT(app_fifo_put(&m_rx_fifo, m_rx_buffer[i]) == NRF_SUCCESS);
                    }
                }

                /* Fetch data until internal buffer is empty */
                ret = app_usbd_cdc_acm_read_any(&m_app_cdc_acm, m_rx_buffer, READ_SIZE);
            } while (ret == NRF_SUCCESS);

            // bsp_board_led_invert(LED_CDC_ACM_RX);
            break;
        }
        default:
            break;
    }
}

int hal_usb_cdc_init() {
    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret_code_t ret = app_usbd_class_append(class_cdc_acm);
    SPARK_ASSERT(ret == NRF_SUCCESS);

    SPARK_ASSERT(app_fifo_init(&m_tx_fifo, tx_buffer, RAW_BUFFER_SIZE) == NRF_SUCCESS);
    SPARK_ASSERT(app_fifo_init(&m_rx_fifo, rx_buffer, RAW_BUFFER_SIZE) == NRF_SUCCESS);

    return 0;
}

int hal_usb_cdc_available_for_read() {
    return (int)FIFO_LENGTH(&m_rx_fifo);
}

int hal_usb_cdc_available_for_write() {
    return RAW_BUFFER_SIZE - (int)FIFO_LENGTH(&m_tx_fifo);
}

int hal_usb_cdc_read(uint8_t* buf, size_t size) {
    if (!m_port_opened) {
        return SYSTEM_ERROR_INVALID_STATE;
    }

    int read_size = 0;
    for (size_t i = 0; i < size; i++) {
        if (app_fifo_get(&m_rx_fifo, &buf[i]) == NRF_SUCCESS) {
            read_size++;
        } else {
            break;
        }
    }

    return read_size;
}

int hal_usb_cdc_write(uint8_t* buf, size_t size) {
    if (!m_port_opened) {
        return SYSTEM_ERROR_INVALID_STATE;
    }

    // Wait until fifo is available
    while (IS_FIFO_FULL(&m_tx_fifo)) {
        ;
    }

    int ret = SYSTEM_ERROR_NONE;
    for (size_t i = 0; i < size; i++) {
        if (app_fifo_put(&m_tx_fifo, buf[i]) != NRF_SUCCESS) {
            ret = SYSTEM_ERROR_NO_MEMORY;
            break;
        }
    }
    usb_cdc_schedule_tx();
    return ret;
}
