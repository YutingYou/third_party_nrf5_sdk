#include "hal_usb.h"
#include "hal_usb_cdc.h"
// #include "service_debug.h"

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

/**
 * @brief Enable power USB detection
 *
 * Configure if example supports USB port connection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif


static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SOF:
            break;
        case APP_USBD_EVT_DRV_SUSPEND:
            // bsp_board_led_off(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_DRV_RESUME:
            // bsp_board_led_on(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_STARTED:
            break;
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            // bsp_board_leds_off();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
            // NRF_LOG_INFO("USB power detected");
            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            // NRF_LOG_INFO("USB power removed");
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
            // NRF_LOG_INFO("USB ready");
            app_usbd_start();
            break;
        default:
            break;
    }
}

// static void usbd_user_ev_handler(app_usbd_event_type_t event)
// {
//     switch (event) {
//         case APP_USBD_EVT_DRV_SOF: {
//             // usb_cdc_schedule_rx();
//             // usb_cdc_schedule_tx();
//             break;
//         }
//         case APP_USBD_EVT_DRV_SUSPEND: {
//             // set_usb_state(HAL_USB_STATE_SUSPENDED);
//             // usb_cdc_change_port_state(false);
//             // nrf_drv_usbd_ep_abort(CDC_ACM_DATA_EPIN);
//             // nrf_drv_usbd_ep_abort(CDC_ACM_DATA_EPOUT);
//             // m_usb_instance.transmitting = false;
//             // m_usb_instance.rx_state = false;
//             // // Required so that we can restart rx transfers after waking up
//             // m_app_cdc_acm.specific.p_data->ctx.rx_transfer[0].p_buf = NULL;
//             // m_app_cdc_acm.specific.p_data->ctx.rx_transfer[1].p_buf = NULL;
//             // m_app_cdc_acm.specific.p_data->ctx.bytes_left = 0;
//             // m_app_cdc_acm.specific.p_data->ctx.bytes_read = 0;
//             // m_app_cdc_acm.specific.p_data->ctx.last_read = 0;
//             // m_app_cdc_acm.specific.p_data->ctx.cur_read = 0;
//             // m_app_cdc_acm.specific.p_data->ctx.p_copy_pos = m_app_cdc_acm.specific.p_data->ctx.internal_rx_buf;
//             // // IMPORTANT! Otherwise the state machine will fail
//             // // to come out of the suspended state
//             // app_usbd_suspend_req();
//             break;
//         }
//         case APP_USBD_EVT_DRV_RESUME: {
//             // set_usb_state(nrf_usb_state_to_hal_usb_state(app_usbd_core_state_get()));
//             // if (m_app_cdc_acm.specific.p_data->ctx.line_state & APP_USBD_CDC_ACM_LINE_STATE_DTR) {
//             //     usb_cdc_change_port_state(true);
//             // }
//             break;
//         }
//         case APP_USBD_EVT_START_REQ: {
//             // set_usb_state(HAL_USB_STATE_DETACHED);
//             break;
//         }
//         case APP_USBD_EVT_STARTED: {
//             // // triggered by app_usbd_start()
//             // reset_rx_tx_state();
//             // set_usb_state(nrf_usb_state_to_hal_usb_state(app_usbd_core_state_get()));
//             break;
//         }
//         case APP_USBD_EVT_STOPPED: {
//             // // triggered by app_usbd_stop()
//             // app_usbd_disable();
//             // if (m_usb_instance.enabled) {
//             //     set_usb_state(HAL_USB_STATE_DETACHED);
//             // } else {
//             //     set_usb_state(HAL_USB_STATE_DISABLED);
//             // }
//             break;
//         }
//         case APP_USBD_EVT_POWER_DETECTED: {
//             // if (!nrf_drv_usbd_is_enabled()) {
//             //     app_usbd_enable();
//             // }
//             break;
//         }
//         case APP_USBD_EVT_POWER_REMOVED: {
//             // // We need to check for nrfx_usbd_is_enabled() in order not to accidentally
//             // // trigger an assertion
//             // if (nrfx_usbd_is_enabled()) {
//             //     app_usbd_stop();
//             // } else if (m_usb_instance.enabled) {
//             //     // Just in case go into detached state
//             //     set_usb_state(HAL_USB_STATE_DETACHED);
//             // }
//             // m_usb_instance.rx_state = false;
//             // usb_cdc_change_port_state(false);
//             break;
//         }
//         case APP_USBD_EVT_DRV_RESET: {
//             // // Nordic CDC driver willl silently abort transfers
//             // m_usb_instance.rx_state = false;
//             // usb_cdc_change_port_state(false);
//             break;
//         }
//         case APP_USBD_EVT_POWER_READY: {
//             // if (m_usb_instance.enabled) {
//             //     app_usbd_start();
//             // }
//             break;
//         }
//         case APP_USBD_EVT_STATE_CHANGED: {
//             // set_usb_state(nrf_usb_state_to_hal_usb_state(app_usbd_core_state_get()));
//             break;
//         }
//         default:
//             break;
//     }
// }

int hal_usb_init(void) {
    ret_code_t ret = nrf_drv_clock_init();
    if (ret != NRF_ERROR_MODULE_ALREADY_INITIALIZED) {
        SPARK_ASSERT(ret == NRF_SUCCESS);
    }

    nrf_drv_clock_lfclk_request(NULL);
    while(!nrf_drv_clock_lfclk_is_running()) {
        /* Just waiting */
    }

    // Inistialize USB dirver
    app_usbd_config_t usbd_config = {};
    usbd_config.ev_state_proc = usbd_user_ev_handler;
    ret = app_usbd_init(&usbd_config);
    SPARK_ASSERT(ret == NRF_SUCCESS);

    // Add USB CDC configuration
    hal_usb_cdc_init();

    // Start USB
    if (USBD_POWER_DETECTION) {
        ret = app_usbd_power_events_enable();
        APP_ERROR_CHECK(ret);
    } else {
        // NRF_LOG_INFO("No USB power detection enabled\r\nStarting USB now");
        app_usbd_enable();
        app_usbd_start();
    }

    return SYSTEM_ERROR_NONE;
}

int hal_usb_attach(void) {
    if (!nrf_drv_usbd_is_enabled()) {
        app_usbd_enable();
    }
    app_usbd_start();
    return SYSTEM_ERROR_NONE;
}

int hal_usb_deattach(void) {
    // We need to check for nrfx_usbd_is_enabled() in order not to accidentally
    // trigger an assertion
    if (nrfx_usbd_is_enabled()) {
        app_usbd_stop();
    }
    if (nrf_drv_usbd_is_enabled()) {
        app_usbd_disable();
    }
    return SYSTEM_ERROR_NONE;
}
