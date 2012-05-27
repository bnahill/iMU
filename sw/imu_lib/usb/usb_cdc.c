#include "usb_cdc.h"

#include "stm32f4xx_conf.h"

#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"

__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END;

static uint16_t cdc_init(void);
static uint16_t cdc_deinit(void);
static uint16_t cdc_ctrl(uint32_t cmd, uint8_t* buf, uint32_t len);
static uint16_t cdc_tx(uint8_t* buf, uint32_t len);
static uint16_t cdc_rx(uint8_t* buf, uint32_t len);

CDC_IF_Prop_TypeDef cdc_fops = {
	cdc_init,
	cdc_deinit,
	cdc_ctrl,
	cdc_tx,
	cdc_rx
};

void usb_cdc_init(void){
	USBD_Init(&USB_OTG_dev,
	          USB_OTG_HS_CORE_ID,
	          &USR_desc,
	          &USBD_CDC_cb, 
	          &USR_cb);
}


static uint16_t cdc_ctrl(uint32_t cmd, uint8_t* buf, uint32_t len){
	switch (cmd){
	case SEND_ENCAPSULATED_COMMAND:
		break;
	case GET_ENCAPSULATED_RESPONSE:
		break;
	case SET_COMM_FEATURE:
		break;
	case CLEAR_COMM_FEATURE:
		break;
	case SET_LINE_CODING:
		break;
	case GET_LINE_CODING:
		break;
	case SET_CONTROL_LINE_STATE:
		break;
	case SEND_BREAK:
		break;    
	default:
		break;
	}

	return USBD_OK;
}

static uint16_t cdc_init(void){
	return USBD_OK;
}

static uint16_t cdc_deinit(void){
	return USBD_OK;
}


static uint16_t cdc_tx(uint8_t* buf, uint32_t len){
	return USBD_OK;
}

static uint16_t cdc_rx(uint8_t* buf, uint32_t len){
	return USBD_OK;
}

