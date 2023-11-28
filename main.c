/*
 * This file is part of the Arducam SPI Camera project.
 *
 * Copyright 2021 Arducam Technology co., Ltd. All Rights Reserved.
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 */
#include "Arducam.h"
#include "spi.h"
#include <msp430.h>
#include "uart.h"

uint8_t temp = 0xff;
uint8_t commandBuff[20] = {0};
uint8_t commandLength = 0;
uint8_t sendFlag = TRUE;
uint32_t readImageLength = 0;
uint8_t jpegHeadFlag = 0;
ArducamCamera myCAM;

void initClockTo16MHz(void)
{
//    if (CALBC1_16MHZ==0xFF)                  // If calibration constant erased
//    {
//        while(1);                               // do not load, trap CPU!!
//    }
//    DCOCTL = 0;                               // Select lowest DCOx and MODx settings
//    BCSCTL1 = CALBC1_16MHZ;                    // Set DCO
//    DCOCTL = CALDCO_16MHZ;

    // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRCTL0 = FRCTLPW | NWAITS_1;

    // Clock System Setup
    CSCTL0_H = CSKEY_H;                     // Unlock CS registers
    CSCTL1 = DCOFSEL_0;                     // Set DCO to 1MHz

    // Set SMCLK = MCLK = DCO, ACLK = VLOCLK
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;

    // Per Device Errata set divider to 4 before changing frequency to
    // prevent out of spec operation from overshoot transient
    CSCTL3 = DIVA__4 | DIVS__4 | DIVM__4;   // Set all corresponding clk sources to divide by 4 for errata
    CSCTL1 = DCOFSEL_4 | DCORSEL;           // Set DCO to 16MHz

    // Delay by ~10us to let DCO settle. 60 cycles = 20 cycles buffer + (10us / (1/4MHz))
    __delay_cycles(60);
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;   // Set all dividers to 1 for 16MHz operation
    CSCTL0_H = 0;                           // Lock CS registers
    //==========================================================================================================

}

uint8_t ReadBuffer(uint8_t* imagebuf,uint8_t length)
{

//  if(arducamUartAvailable())
//  {
//    temp = arducamUartRead();
//    arducamDelayMs(1);
//    if (temp == 0x55)
//    {
//      while (arducamUartAvailable())
//      {
//        commandBuff[commandLength] = arducamUartRead();
//        if (commandBuff[commandLength] == 0xAA)
//        {
//          break;
//        }
//        commandLength++;
//      }
//      if(commandBuff[0]==0x21)
//      {
//        readImageLength = 0;
//        jpegHeadFlag = 0;
//        arducamUartWrite(0x55);
//        arducamUartWrite(0xBB);
//        return FALSE;
//      }
//      commandLength=0;
//    }
//  }

  //below checked-----------------------------------------------------------------
  if (imagebuf[0] == 0xff && imagebuf[1] == 0xd8)
  {
    jpegHeadFlag = 1;
    readImageLength = 0;
    arducamUartWrite(0xff);
    arducamUartWrite(0xAA);
    arducamUartWrite(0x01);

    arducamUartWrite((uint8_t)(myCAM.totalLength&0xff));
    arducamUartWrite((uint8_t)((myCAM.totalLength>>8)&0xff));
    arducamUartWrite((uint8_t)((myCAM.totalLength>>16)&0xff));
    arducamUartWrite((uint8_t)((myCAM.receivedLength>>24)&0xff));
    arducamUartWrite(((CAM_IMAGE_PIX_FMT_JPG & 0x0f) << 4) | 0x01);
  }

  //below checked-----------------------------------------------------------------
  if (jpegHeadFlag == 1)
  {
    readImageLength += length;
    for (uint8_t i = 0; i < length; i++)
    {
        arducamUartWrite(imagebuf[i]);
    }
  }

  //below checked-----------------------------------------------------------------
  if (readImageLength == myCAM.totalLength)
  {
    jpegHeadFlag = 0;
    arducamUartWrite(0xff);
    arducamUartWrite(0xBB);
  }
  return sendFlag;

}

//below checked-----------------------------------------------------------------
void stop_preivew()
{
    readImageLength = 0;
    jpegHeadFlag    = 0;
    uint32_t len    = 9;

    arducamUartWrite(0xff);
    arducamUartWrite(0xBB);
    arducamUartWrite(0xff);
    arducamUartWrite(0xAA);
    arducamUartWrite(0x06);
    uartWriteBuffer((uint8_t*)&len, 4);
    //printf("streamoff");
    arducamUartWrite(0xff);
    arducamUartWrite(0xBB);
}

void initGPIO(void)
{
    // Configure SPI GPIO
    P5SEL1 &= ~(BIT0 | BIT1 | BIT2);        // USCI_B1 SCLK, MOSI,
    P5SEL0 |= (BIT0 | BIT1 | BIT2);         // and MISO pin

    // CS pins
    CS_DIR |= CS_PIN;
    P5REN  |= CS_PIN; // SPI at port 5
    CS_OUT |= CS_PIN;

    // Configure UART GPIO
    P6SEL1 &= ~(BIT0 | BIT1);
    P6SEL0 |= (BIT0 | BIT1);                // USCI_A3 UART operation

    P1DIR |= BIT0;
    P1OUT &= ~BIT0;


    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;
}
void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;       // stop watchdog timer

    initGPIO();
    initClockTo16MHz();
    uartInit();

    myCAM = createArducamCamera(CS_PIN);
    begin(&myCAM);
    //registerCallback(&myCAM,ReadBuffer,50);
    registerCallback(&myCAM, ReadBuffer, 50, stop_preivew);

    __enable_interrupt();

    P1OUT ^= BIT0;    // red LED turn on show that initialization is success


    while(1)
    {

        if (arducamUartAvailable())
        {
            arducamDelayMs(5);
            temp = arducamUartRead();
            if (temp == 0x55)
            {
                while (arducamUartAvailable())
                {
                    commandBuff[commandLength] = arducamUartRead();
                    if (commandBuff[commandLength] == 0xAA)
                    {
                        break;
                    }
                    commandLength++;
                }
                arducamFlush();
                commandProcessing(&myCAM,commandBuff,commandLength);
                commandLength = 0;
            }
        }
    }
}
