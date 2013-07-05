/*
 * File      : cdc_vcom.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2012, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-10-02     Yi Qiu       first version
 * 2012-12-12     heyuanjie87  change endpoints and class handler
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_mem.h"
#include "hw_config.h"
#include "usb_istr.h"
#include "usb_pwr.h"

/* Interval between sending IN packets in frame number (1 frame = 1ms) */
#define VCOMPORT_IN_FRAME_INTERVAL             5
rt_uint8_t  USB_Tx_State = 0;


#define CDC_RX_BUFSIZE          512
#define CDC_TX_BUFSIZE          512
static rt_uint8_t rx_rbp[CDC_RX_BUFSIZE];
static rt_uint8_t tx_rbp[CDC_TX_BUFSIZE];
static struct rt_ringbuffer rx_ringbuffer;
static struct rt_ringbuffer tx_ringbuffer;
static struct serial_ringbuffer vcom_int_rx;

static struct rt_serial_device vcom_serial;

#define CDC_MaxPacketSize 64
ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t rx_buf[CDC_RX_BUFSIZE];
ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t tx_buf[CDC_TX_BUFSIZE];

volatile static rt_bool_t vcom_connected = RT_FALSE;
volatile static rt_bool_t vcom_in_sending = RT_FALSE;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : EP1_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback (void)
{
  rt_uint32_t level;
  rt_uint32_t remain;

  if (USB_Tx_State == 1)
  {
	  level = rt_hw_interrupt_disable();
	  remain = RT_RINGBUFFER_SIZE(&tx_ringbuffer);
	  if (remain == 0) 
	    {
	      USB_Tx_State = 0;
		  rt_hw_interrupt_enable(level);
		  return;
	    }
	    else
	    {
	    	if (remain > VIRTUAL_COM_PORT_DATA_SIZE)
	    	{
				remain = VIRTUAL_COM_PORT_DATA_SIZE;
			}
	        /* although vcom_in_sending is set in SOF handler in the very
	         * beginning, we have to guarantee the state is right when start
	         * sending. There is at least one extreme case where we have finished the
	         * last IN transaction but the vcom_in_sending is RT_FALSE.
	         *
	         * Ok, what the extreme case is: pour data into vcom in loop. Open
	         * terminal on the PC, you will see the data. Then close it. So the
	         * data will be sent to the PC in the back. When the buffer of the PC
	         * driver is full. It will not send IN packet to the board and you will
	         * have no chance to clear vcom_in_sending in this function. The data
	         * will fill into the ringbuffer until it is full, and we will reset
	         * the state machine and clear vcom_in_sending. When you open the
	         * terminal on the PC again. The IN packet will appear on the line and
	         * we will, eventually, reach here with vcom_in_sending is clear.
	         */
	        vcom_in_sending = RT_TRUE;
	        rt_ringbuffer_get(&tx_ringbuffer, tx_buf, remain);
	        rt_hw_interrupt_enable(level);

	        /* send data to host */
			UserToPMABufferCopy(tx_buf, ENDP1_TXADDR, remain);
	        SetEPTxCount(ENDP1, remain);
	        SetEPTxValid(ENDP1); 

	        //return RT_EOK;
	    }
  	}

}

/*******************************************************************************
* Function Name  : EP3_OUT_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP3_OUT_Callback(void)
{
  rt_uint32_t level;
  rt_uint16_t size;

  /* receive data from USB VCOM */
    level = rt_hw_interrupt_disable();

	size = USB_SIL_Read(EP3_OUT, rx_buf);
    rt_ringbuffer_put(&rx_ringbuffer, rx_buf, size);
    rt_hw_interrupt_enable(level);
	/* Enable the receive of data on EP3 */
  	SetEPRxValid(ENDP3);

    /* notify receive data */
    rt_hw_serial_isr(&vcom_serial);

    //return RT_EOK;

}


/*******************************************************************************
* Function Name  : Handle_USBAsynchXfer.
* Description    : send data to USB.
* Input          : None.
* Return         : none.
*******************************************************************************/
void Handle_USBAsynchXfer (void)
{
  rt_uint32_t level;
  rt_uint32_t remain;
  
  if(USB_Tx_State != 1)
  {
  	level = rt_hw_interrupt_disable();
	  remain = RT_RINGBUFFER_SIZE(&tx_ringbuffer);
    
    if(remain == 0) 
    {
      USB_Tx_State = 0;
	  rt_hw_interrupt_enable(level);
      return;
    }
    
    if (remain > VIRTUAL_COM_PORT_DATA_SIZE)
    {
      remain = VIRTUAL_COM_PORT_DATA_SIZE;	
    }
	rt_ringbuffer_get(&tx_ringbuffer, tx_buf, remain);
	rt_hw_interrupt_enable(level);

    USB_Tx_State = 1; 
    UserToPMABufferCopy(tx_buf, ENDP1_TXADDR, remain);
    SetEPTxCount(ENDP1, remain);
    SetEPTxValid(ENDP1); 
  }  
}



/*******************************************************************************
* Function Name  : SOF_Callback / INTR_SOFINTR_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SOF_Callback(void)
{
  static uint32_t FrameCount = 0;
  
  if(bDeviceState == CONFIGURED)
  {
    if (FrameCount++ == VCOMPORT_IN_FRAME_INTERVAL)
    {
      /* Reset the frame counter */
      FrameCount = 0;
      
      /* Check the data to be sent through IN pipe */
      Handle_USBAsynchXfer();
    }
  }  
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



static void _vcom_reset_state(void)
{
    int lvl = rt_hw_interrupt_disable();
    tx_ringbuffer.read_mirror  = tx_ringbuffer.read_index = 0;
    tx_ringbuffer.write_mirror = tx_ringbuffer.write_index = 0;
    vcom_connected = RT_FALSE;
    vcom_in_sending = RT_FALSE;
    /*rt_kprintf("reset USB serial\n", cnt);*/
    rt_hw_interrupt_enable(lvl);
}


/**
* UART device in RT-Thread
*/
static rt_err_t _vcom_configure(struct rt_serial_device *serial,
                                struct serial_configure *cfg)
{
    return RT_EOK;
}

static rt_err_t _vcom_control(struct rt_serial_device *serial,
                              int cmd, void *arg)
{
    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        break;
    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        break;
    }

    return RT_EOK;
}

static int _vcom_putc(struct rt_serial_device *serial, char c)
{
    rt_uint32_t level;
    int cnt;

    /*if (vcom_connected != RT_TRUE)
    {
        return 0;
    }*/

    /* if the buffer is full, there is a chance that the host would pull some
     * data out soon. But we cannot rely on that and if we wait to long, just
     * return. */
    for (cnt = 500;
         RT_RINGBUFFER_EMPTY(&tx_ringbuffer) == 0 && cnt;
         cnt--)
    {
        /*rt_kprintf("wait for %d\n", cnt);*/
        /*if (vcom_connected != RT_TRUE)
            return 0;*/
    }
    if (cnt == 0)
    {
        /* OK, we believe that the connection is lost. So don't send any more
         * data and act as the USB cable is not plugged in. Reset the VCOM
         * state machine */
        _vcom_reset_state();
        return 0;
    }

    level = rt_hw_interrupt_disable();
    if (RT_RINGBUFFER_EMPTY(&tx_ringbuffer))
    {
        rt_ringbuffer_putchar(&tx_ringbuffer, c);
    }
    rt_hw_interrupt_enable(level);

    return 1;
}

static int _vcom_getc(struct rt_serial_device *serial)
{
    int result;
    rt_uint8_t ch;
    rt_uint32_t level;

    result = -1;

    level = rt_hw_interrupt_disable();
    if (RT_RINGBUFFER_SIZE(&rx_ringbuffer))
    {
        rt_ringbuffer_getchar(&rx_ringbuffer, &ch);
        result = ch;
    }
    rt_hw_interrupt_enable(level);

    return result;
}

static const struct rt_uart_ops usb_vcom_ops =
{
    _vcom_configure,
    _vcom_control,
    _vcom_putc,
    _vcom_getc,
};

void rt_usb_serial_init(void)
{
    struct serial_configure config;

    /* initialize ring buffer */
    rt_ringbuffer_init(&rx_ringbuffer, rx_rbp, CDC_RX_BUFSIZE);
    rt_ringbuffer_init(&tx_ringbuffer, tx_rbp, CDC_TX_BUFSIZE);

    config.baud_rate = BAUD_RATE_115200;
    config.bit_order = BIT_ORDER_LSB;
    config.data_bits = DATA_BITS_8;
    config.parity = PARITY_NONE;
    config.stop_bits = STOP_BITS_1;
    config.invert = NRZ_NORMAL;

    vcom_serial.ops = &usb_vcom_ops;
    vcom_serial.int_rx = &vcom_int_rx;
    vcom_serial.config = config;

    /* register vcom device */
    rt_hw_serial_register(&vcom_serial, "vcom",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          RT_NULL);
}

extern void finsh_set_device(const char* device);
void usb_init()
{
#ifdef RT_USING_STD_USB
    /* usb device controller driver initilize */
	Set_System();
	Set_USBClock();
	USB_Interrupts_Config();
	rt_usb_serial_init();
	USB_Init();
#ifdef RT_USING_CONSOLE
		//rt_console_set_device("vcom");
		//finsh_set_device( "vcom" );
#endif
#endif
}

void set_finsh_dev(char *device)
{
#ifdef RT_USING_CONSOLE
		rt_console_set_device(device);
		finsh_set_device( device );
#endif
}

#include "finsh.h"
FINSH_FUNCTION_EXPORT(usb_init, init usb device);
FINSH_FUNCTION_EXPORT(set_finsh_dev, set finsh device);


