/** \copyright
 * Copyright (c) 2014, Stuart W Baker
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 * 
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file TivaDev.hxx
 * This file implements the device class prototypes for TivaWare.
 *
 * @author Stuart W. Baker
 * @date 6 May 2014
 */

#ifndef _FREERTOS_DRIVERS_TI_TIVADEV_HXX_
#define _FREERTOS_DRIVERS_TI_TIVADEV_HXX_

#ifndef gcc
#define gcc
#endif

#include <cstdint>

#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/timer.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"

#include "Serial.hxx"
#include "Can.hxx"

/* This is fixed and equals the USB packet size that the CDC device will
 * advertise to be able to receive. This is a performance parameter, 64 is the
 * largest packet size permitted by USB for virtual serial ports. */
#define USB_CDC_TX_DATA_SIZE 64
#define USB_CDC_RX_DATA_SIZE 64

/** Private data for this implementation of serial.
 */
class TivaCdc : public Serial
{
public:
    /** Constructor.
     * @param name name of this device instance in the file system
     * @param interrupt interrupt number used by the device
     */
    TivaCdc(const char *name, uint32_t interrupt);
    
    /** Destructor.
     */
    ~TivaCdc()
    {
    }

    /** @todo (Stuart Baker) this should be made private */
    /** handle an interrupt.
     */
    void interrupt_handler();

private:
    void enable(); /**< function to enable device */
    void disable(); /**< function to disable device */
    void tx_char(); /**< function to try and transmit a character */

    static unsigned long control_callback(void *data, unsigned long event, unsigned long msg_param, void *msg_data);

    static unsigned long rx_callback(void *data, unsigned long event, unsigned long msg_param, void *msg_data);

    static unsigned long tx_callback(void *data, unsigned long event, unsigned long msg_param, void *msg_data);

    tUSBDCDCDevice usbdcdcDevice; /**< CDC serial device instance */
    /** buffer for pending tx data */
    unsigned char txData[USB_CDC_TX_DATA_SIZE];
    /** buffer for pending rx data */
    unsigned char rxData[USB_CDC_RX_DATA_SIZE];
    uint32_t interrupt; /**< interrupt number for device */
    bool connected; /**< connection status */
    bool enabled; /**< enabled status */
    int woken; /**< task woken metadata for ISR */
    
    /** Default constructor.
     */
    TivaCdc();
    
    DISALLOW_COPY_AND_ASSIGN(TivaCdc);
};

/** Specialization of Serial driver for Tiva UART.
 */
class TivaUart : public Serial
{
public:
    /** Constructor.
     * @param name name of this device instance in the file system
     * @param base base address of this device
     * @param interrupt interrupt number of this device
     */
    TivaUart(const char *name, unsigned long base, uint32_t interrupt);
    
    /** Destructor.
     */
    ~TivaUart()
    {
    }

    /** @todo (Stuart Baker) this should be made private */
    /** handle an interrupt.
     */
    void interrupt_handler();

private:
    void enable(); /**< function to enable device */
    void disable(); /**< function to disable device */
    void tx_char(); /**< function to try and transmit a character */

    unsigned long base; /**< base address of this device */
    unsigned long interrupt; /**< interrupt of this device */
    bool txPending; /**< transmission currently pending */

    /** Default constructor.
     */
    TivaUart();
    
    DISALLOW_COPY_AND_ASSIGN(TivaUart);
};

/** Specialization of CAN driver for Tiva CAN.
 */
class TivaCan : public Can
{
public:
    /** Constructor.
     * @param name name of this device instance in the file system
     * @param base base address of this device
     * @param interrupt interrupt number of this device
     */
    TivaCan(const char *name, unsigned long base, uint32_t interrupt);
    
    /** Destructor.
     */
    ~TivaCan()
    {
    }

    /** @todo (Stuart Baker) this should be made private */
    /** handle an interrupt.
     */
    void interrupt_handler();

private:
    void enable(); /**< function to enable device */
    void disable(); /**< function to disable device */
    void tx_msg(); /**< function to try and transmit a message */

    unsigned long base; /**< base address of this device */
    unsigned long interrupt; /**< interrupt of this device */
    uint8_t data[8]; /**< transmit data */
    bool txPending; /**< transmission currently pending */

    /** Default constructor.
     */
    TivaCan();
    
    DISALLOW_COPY_AND_ASSIGN(TivaCan);
};

/** A device driver for sending DCC packets.  If the packet queue is empty,
 *  then the device driver automatically sends out idle DCC packets.  The
 *  device driver uses two instances of the 16/32-bit timer pairs.  The user
 *  is responsible for providing interrupt entry point for the interval timer
 *  and calling the inline method @ref interrupt_handler on behalf of this
 *  device driver.
 *
 *  Write calls work by sending the whole DCC packet excluding the X-OR linkage
 *  byte (which will be calculated automatically by the driver).  Only one DCC
 *  packet may be written per call to the write method.  If there is no space
 *  currently available in the write queue, the write method will return -1
 *  with errno set to ENOSPC.
 */
class TivaDCC : public Node
{
public:
    /** Constructor.
     * @param name name of this device instance in the file system
     * @param ccp_base base address of a capture compare pwm timer pair
     * @param interval_base base address of an interval timer 
     * @param interrupt interrupt number of interval timer
     * @param preamble_count number of preamble bits to send exclusive of 
     *        end of packet '1' bit
     * @param one_bit_period number of system clock cycles for a one bit
     * @param zero_bit_period number of system clock cycles for a one bit
     * @param startup_delay offset for introducing some deadband at startup
     * @param deadband_adjust ajustment factor for adding deadband
     * @param railcom_cuttout true to produce the RailCom cuttout, else false
     */
    TivaDCC(const char *name,
            unsigned long ccp_base,
            unsigned long interval_base,
            uint32_t interrupt,
            int preamble_count,
            int one_bit_period,
            int zero_bit_period,
            int startup_delay,
            int deadband_adjust,
            bool railcom_cuttout = false);
    
    /** Destructor.
     */
    ~TivaDCC()
    {
    }

    /** Handle an interrupt.
     */
    inline void interrupt_handler();

private:
    /** Read from a file or device.
     * @param file file reference for this device
     * @param buf location to place read data
     * @param count number of bytes to read
     * @return number of bytes read upon success, -1 upon failure with errno containing the cause
     */
    ssize_t read(File *file, void *buf, size_t count) OVERRIDE;

    /** Write to a file or device.
     * @param file file reference for this device
     * @param buf location to find write data
     * @param count number of bytes to write
     * @return number of bytes written upon success, -1 upon failure with errno containing the cause
     */
    ssize_t write(File *file, const void *buf, size_t count) OVERRIDE;

    /** Request an ioctl transaction
     * @param file file reference for this device
     * @param node node reference for this device
     * @param key ioctl key
     * @param data key data
     */
    int ioctl(File *file, unsigned long int key, unsigned long data) OVERRIDE;

    void enable(){} /**< function to enable device */
    void disable(){} /**< function to disable device */

    /** Discards all pending buffers.  Called after disable().
     */
    void flush_buffers(){};

    /** number of outgoing messages we can queue */
    static const size_t Q_SIZE = 16;

    /** maximum packet size we can support */
    static const size_t MAX_PKT_SIZE = 6;

    /** Queue structure for holding outgoing DCC packets.
     */
    struct Q
    {
        int count; /**< number of items in the queue */
        int rdIndex; /**< current read index */
        int wrIndex; /**< current write index */
        uint8_t data[Q_SIZE][MAX_PKT_SIZE + 1]; /**< queue data */
    };

    unsigned long ccpBase; /**< capture compare pwm base address */
    unsigned long intervalBase; /**< interval timer base address */
    uint32_t interrupt; /**< interrupt number of interval timer */
    int preambleCount; /**< number of preamble bits to send */
    int oneBitPeriod; /**< period of one bit */
    int zeroBitPeriod; /**< period of zero bit */
    int startupDelay; /**< startup offset */
    int deadbandAdjust; /**< deadband adjustment */
    bool railcomCuttout; /**< true if we should produce the RailCom cuttout */

    /** idle packet */
    static const uint8_t IDLE_PKT[3];

    Q q; /**< DCC packet queue */

    /** Default constructor.
     */
    TivaDCC();
    
    DISALLOW_COPY_AND_ASSIGN(TivaDCC);
};

/** Handle an interrupt.
 */
inline void TivaDCC::interrupt_handler()
{
    enum State
    {
        PREAMBLE,
        START,
        DATA_0,
        DATA_1,
        DATA_2,
        DATA_3,
        DATA_4,
        DATA_5,
        DATA_6,
        DATA_7,
        FRAME,
    };

    static State state = PREAMBLE;
    static int preamble_count = 0;
    static int last_bit = 1;
    static int count = 0;
    static uint8_t xor_byte;
    static const uint8_t *packet = IDLE_PKT;
    int current_bit;

    MAP_TimerIntClear(intervalBase, TIMER_TIMA_TIMEOUT);

    switch (state)
    {
        default:
        case PREAMBLE:
            current_bit = 1;
            if (++preamble_count == preambleCount)
            {
                state = START;
                preamble_count = 0;
            }
            break;
        case START:
            current_bit = 0;
            count = 0;
            xor_byte = 0;
            state = DATA_0;
            break;
        case DATA_0:
        case DATA_1:
        case DATA_2:
        case DATA_3:
        case DATA_4:
        case DATA_5:
        case DATA_6:
        case DATA_7:
            if (count < packet[0])
            {
                current_bit = (packet[count + 1] >> (state - DATA_0)) & 0x01;
            }
            else
            {
                current_bit = (xor_byte >> (state - DATA_0)) & 0x01;
            }
            state = static_cast<State>(static_cast<int>(state) + 1);
            break;
        case FRAME:
            if (++count > packet[0])
            {
                if (packet != IDLE_PKT)
                {
                    --q.count;
                    ++q.rdIndex;
                    if (q.rdIndex == Q_SIZE)
                    {
                        q.rdIndex = 0;
                    }
                }
                if (q.count)
                {
                    packet = &q.data[q.rdIndex][0];
                }
                else
                {
                    packet = IDLE_PKT;
                }
                current_bit = 1;
                state = PREAMBLE;
            }
            else
            {
                xor_byte ^= packet[count];
                current_bit = 0;
                state = DATA_0;
            }
            break;
    }

    if (last_bit != current_bit)
    {
        if (current_bit)
        {
            MAP_TimerLoadSet(intervalBase, TIMER_A, oneBitPeriod);
            MAP_TimerLoadSet(ccpBase, TIMER_A, oneBitPeriod);
            MAP_TimerLoadSet(ccpBase, TIMER_B, oneBitPeriod);
            MAP_TimerMatchSet(ccpBase, TIMER_A, (oneBitPeriod >> 1) - deadbandAdjust);
            MAP_TimerMatchSet(ccpBase, TIMER_B, (oneBitPeriod >> 1) + deadbandAdjust);
        }
        else
        {
            MAP_TimerLoadSet(intervalBase, TIMER_A, zeroBitPeriod);
            MAP_TimerLoadSet(ccpBase, TIMER_A, zeroBitPeriod);
            MAP_TimerLoadSet(ccpBase, TIMER_B, zeroBitPeriod);
            MAP_TimerMatchSet(ccpBase, TIMER_A, (zeroBitPeriod >> 1) - deadbandAdjust);
            MAP_TimerMatchSet(ccpBase, TIMER_B, (zeroBitPeriod >> 1) + deadbandAdjust);
        }
        last_bit = current_bit;
    }
}

#endif /* _FREERTOS_DRIVERS_TI_TIVADEV_HXX_ */

