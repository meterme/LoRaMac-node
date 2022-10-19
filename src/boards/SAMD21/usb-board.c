/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file or main.c
 * to avoid loosing it when reconfiguring.
 */
// #include "atmel_start.h"
#include "usb-board.h"

#if CONF_USBD_HS_SP
static uint8_t single_desc_bytes[] = {
    /* Device descriptors and Configuration descriptors list. */
    CDCD_ACM_HS_DESCES_LS_FS};
static uint8_t single_desc_bytes_hs[] = {
    /* Device descriptors and Configuration descriptors list. */
    CDCD_ACM_HS_DESCES_HS};
#define CDCD_ECHO_BUF_SIZ CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ_HS
#else
static uint8_t single_desc_bytes[] = {
    /* Device descriptors and Configuration descriptors list. */
    CDCD_ACM_DESCES_LS_FS};
#define CDCD_ECHO_BUF_SIZ CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ
#endif

static struct usbd_descriptors single_desc[]
    = {{single_desc_bytes, single_desc_bytes + sizeof(single_desc_bytes)}
#if CONF_USBD_HS_SP
       ,
       {single_desc_bytes_hs, single_desc_bytes_hs + sizeof(single_desc_bytes_hs)}
#endif
};



/** Ctrl endpoint buffer */
static uint8_t ctrl_buffer[64];
volatile uint32_t count_out, count_in;

#include <board.h>


/*
 * Ringbuffer
 */

#define	RING_BUFFER_SIZE	1024

typedef struct {
	uint8_t buffer[RING_BUFFER_SIZE];
	uint8_t *in;
	uint8_t *out;
	uint16_t count;
} ringBuffer_t;

static ringBuffer_t consoleRing;

static void
ringInit(ringBuffer_t *r)
{
	r->count = 0;
	r->in = r->out = r->buffer;
}

static uint8_t
ringPull(ringBuffer_t *r)
{
	uint8_t rv = 0;

	if (r->count > 0) {
		r->count -= 1;
		rv = *r->out++;
		if (r->out >= (r->buffer + RING_BUFFER_SIZE)) {
			r->out = r->buffer;
		}
	}

	return (rv);
}

static void
ringPush(ringBuffer_t *r, uint8_t d)
{

	if (r->count < RING_BUFFER_SIZE) {
		r->count += 1;
		*r->in++ = d;
		if (r->in >= (r->buffer + RING_BUFFER_SIZE)) {
			r->in = r->buffer;
		}		
	}
}

static uint16_t
ringCount(ringBuffer_t *r)
{
	return (r->count);
}


/**
 * \brief Callback invoked when bulk OUT data received
 */
static bool usb_device_cb_bulk_out(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{

	count_out += count;
	// XXX: copy incoming characters to queue

	// re-start the read
	// cdcdf_acm_read((uint8_t *)usbd_cdc_buffer, sizeof(usbd_cdc_buffer));

	/* No error. */
	return false;
}


/*
 * Console write / read functions
 *
 * Read:
 *   Start: start a read into a buffer
 *   Out callback: copy received characters into ring, restart read
 * 
 * Write:
 *    Copy to next available output buffer(s), start write if not active/set active
 *    In callback: free buffer, start next one if available, otherwise, inactive
 */


/**
 * \brief Callback invoked when bulk IN data received (result of a write())
 */
static bool usb_device_cb_bulk_in(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
	uint32_t x;
	uint16_t c, i;
	static uint32_t in_buffer[CDCD_ECHO_BUF_SIZ / 4];
	uint8_t *p = (uint8_t *) &in_buffer;

	BoardCriticalSectionBegin(&x);

	c = ringCount(&consoleRing);
	if (c > 0) {
		if (c > CDCD_ECHO_BUF_SIZ) {
			c = CDCD_ECHO_BUF_SIZ;
		}
		i = c;
		while (i--) {
			*p++ = ringPull(&consoleRing);
		}
		cdcdf_acm_write((uint8_t *)in_buffer, c);
	}

	BoardCriticalSectionEnd(&x);

	/* No error. */
	return false;
}

/*
 *
 */
void usb_write(uint8_t *buf, uint32_t count)
{
	uint32_t x, y;
	
	// XXX: not the way to do this; need to break into chunks
	// XXX: avoid overflow
	// XXX: chain based on completion

	/* CRITICAL SECTION BEGIN */

	/* CRITICAL SECTION END */
	BoardCriticalSectionBegin(&x);

	while (count-- > 0) {
		ringPush(&consoleRing, *buf++);
	}
	
	// cdcdf_acm_write((uint8_t *) usbd_chunks[first_ndx].buf, usbd_chunks[first_ndx].count);
	cdcdf_acm_write((uint8_t *) &y, 0);

	BoardCriticalSectionEnd(&x);
}



/**
 * \brief Callback invoked when Line State Change
 */
static bool usb_device_cb_state_c(usb_cdc_control_signal_t state)
{
	if (state.rs232.DTR) {
		/* Callbacks must be registered after endpoint allocation */
		cdcdf_acm_register_callback(CDCDF_ACM_CB_READ, (FUNC_PTR)usb_device_cb_bulk_out);
		cdcdf_acm_register_callback(CDCDF_ACM_CB_WRITE, (FUNC_PTR)usb_device_cb_bulk_in);
		/* Start Rx */
	//	cdcdf_acm_read((uint8_t *)usbd_cdc_buffer, sizeof(usbd_cdc_buffer));
	}

	/* No error. */
	return false;
}

/**
 * \brief CDC ACM Init
 */
void cdc_device_acm_init(void)
{
	/* usb stack init */
	usbdc_init(ctrl_buffer);

	/* usbdc_register_funcion inside */
	cdcdf_acm_init();

	usbdc_start(single_desc);
	usbdc_attach();
}

/**
 * Example of using CDC ACM Function.
 * \note
 * In this example, we will use a PC as a USB host:
 * - Connect the DEBUG USB on XPLAINED board to PC for program download.
 * - Connect the TARGET USB on XPLAINED board to PC for running program.
 * The application will behave as a virtual COM.
 * - Open a HyperTerminal or other COM tools in PC side.
 * - Send out a character or string and it will echo the content received.
 */
void cdcd_acm_example(void)
{
	while (!cdcdf_acm_is_enabled()) {
		// wait cdc acm to be installed
	};

	cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, (FUNC_PTR)usb_device_cb_state_c);

	while (1) {
	}
}

void usb_init(void)
{

	ringInit(&consoleRing);

	cdc_device_acm_init();
	cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, (FUNC_PTR)usb_device_cb_state_c);

}
