/*
 * This file is part of the Arducam SPI Camera project.
 *
 * Copyright 2021 Arducam Technology co., Ltd. All Rights Reserved.
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 */
#include "uart.h"
#include <msp430.h>

uint8_t uartCommBuff[20] = {0};
uint8_t uartCommLength = 0;
uint8_t readBuffLength = 0;

unsigned char rxData = 0;

int fputc(int ch,FILE *f)
{
//    UCA0TXBUF = ch&0xff;
//    while(!(IFG2 & UCA0TXIFG));
//      return ch;
    while(!(UCA3IFG&UCTXIFG));
    UCA3TXBUF = ch&0xff;
     return ch;
}

int fputs(const char *_ptr, register FILE *_fp)
{
  unsigned int i, len;

  len = strlen(_ptr);

  for(i=0 ; i<len ; i++)
  {
//      UCA0TXBUF = _ptr[i]&0xff;
//      while(!(IFG2 & UCA0TXIFG));
      while(!(UCA3IFG&UCTXIFG));
      UCA3TXBUF = _ptr[i]&0xff;
  }

  return len;
}

void uartInit(void)
{
//    P1SEL |= BIT1 + BIT2;
//    P1SEL2 |= BIT1 + BIT2;
//    UCA0CTL1 |= UCSSEL_2;                     // SMCLK
//    UCA0BR0 = 0x11;                            // 16MHz 921600
//    UCA0BR1 = 0;                              // 16MHz 921600
//    UCA0MCTL =UCBRS_0;//UCBRS0;               // Modulation UCBRSx = 1
//    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
//    IE2 |= UCA0RXIE;
//    _enable_interrupts();

// Configure USCI_A3 for UART mode
    UCA3CTLW0 = UCSWRST;                    // Put eUSCI in reset
    UCA3CTLW0 |= UCSSEL__SMCLK;             // CLK = SMCLK

// Baud Rate calculation
// https://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
/*
    clockPrescalar = UCA3BRW

    firstModReg = UCBRF_x

    secondModReg = UCBRS_x = 0x??00 (HEX)

    overSampling = UCOS16 = 1
 */
// 9600 Baud rate

    UCA3BRW = 104;
    UCA3MCTLW |=  UCOS16 | UCBRF_2 | 0xB600;
    UCA3CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
    UCA3IE |= UCRXIE;                       // Enable USCI_A3 RX interrupt
    //__enable_interrupt();
}

//***********************CUSTOM*******************************************************
void reportVerInfo(ArducamCamera* camera)
{
    uint8_t headAndtail[] = {0xff, 0xaa, 0x03, 0xff, 0xbb};

    uint32_t len = 6;
    uartWriteBuffer(&headAndtail[0], 3);
    uartWriteBuffer((uint8_t*)&len, 4);
    uartWriteBuffer(camera->verDateAndNumber, 4);
    printf("\r\n");
    uartWriteBuffer(&headAndtail[3], 2);
}

void reportSdkVerInfo(ArducamCamera* camera)
{
    uint8_t headAndtail[] = {0xff, 0xaa, 0x05, 0xff, 0xbb};

    uint32_t len = 6;
    uartWriteBuffer(&headAndtail[0], 3);
    uartWriteBuffer((uint8_t*)&len, 4);
    uartWriteBuffer((uint8_t*)&camera->currentSDK->sdkVersion, 4);
    printf("\r\n");
    uartWriteBuffer(&headAndtail[3], 2);
}

void reportCameraInfo(ArducamCamera* camera)
{
    uint8_t headAndtail[] = {0xff, 0xaa, 0x02, 0xff, 0xbb};

    uint32_t len = 0;
    char buff[400];
    uartWriteBuffer(&headAndtail[0], 3);
    sprintf(buff,
            "ReportCameraInfo\r\nCamera Type:%s\r\nCamera Support Resolution:%d\r\nCamera Support "
            "specialeffects:%d\r\nCamera Support Focus:%d\r\nCamera Exposure Value Max:%d\r\nCamera Exposure Value " //Camera Exposure Value Max:%ld\r\n
            "Min:%d\r\nCamera Gain Value Max:%d\r\nCamera Gain Value Min:%d\r\nCamera Support Sharpness:%d\r\n",
            camera->myCameraInfo.cameraId, camera->myCameraInfo.supportResolution,
            camera->myCameraInfo.supportSpecialEffects, camera->myCameraInfo.supportFocus,
            camera->myCameraInfo.exposureValueMax, camera->myCameraInfo.exposureValueMin,
            camera->myCameraInfo.gainValueMax, camera->myCameraInfo.gainValueMin,
            camera->myCameraInfo.supportSharpness);
    len = strlen(buff);
    uartWriteBuffer((uint8_t*)&len, 4);
    printf(buff);
    uartWriteBuffer(&headAndtail[3], 2);
}

void cameraGetPicture(ArducamCamera* camera)
{
    uint8_t headAndtail[] = {0xff, 0xaa, 0x01, 0xff, 0xbb};
    uint8_t buff[READ_IMAGE_LENGTH] = {0};

    uint8_t rtLength = 0;
    uint32_t len = camera->totalLength;
    uartWriteBuffer(&headAndtail[0], 3);
    uartWriteBuffer((uint8_t*)(&len), 4);
    arducamUartWrite(((camera->currentPictureMode & 0x0f) << 4) | 0x01);
    while (camera->receivedLength) {
        rtLength = readBuff(camera, buff, READ_IMAGE_LENGTH);
        uartWriteBuffer(buff, rtLength);
    }
    uartWriteBuffer(&headAndtail[3], 2);
}

//void send_data_pack(char cmd_type, char* msg)
//{
//    uint8_t headAndtail[] = {0xff, 0xaa, 0x07, 0xff, 0xbb};
//    headAndtail[2] = cmd_type;
//    uint32_t len = strlen(msg) + 2;
//    uartWriteBuffer(&headAndtail[0], 3);
//    uartWriteBuffer((uint8_t*)&len, 4);
//    printf(msg);
//    printf("\r\n");
//    uartWriteBuffer(&headAndtail[3], 2);
//}

//***********************CUSTOM*******************************************************

uint8_t commandProcessing(ArducamCamera*camera,uint8_t* buff,uint8_t length)
{
    CamStatus state;
    uint16_t GainValue = 0;
    uint32_t ExposureValue = 0;
    uint32_t ExposureLen1 = 0;
    uint32_t ExposureLen2 = 0;
    uint32_t ExposureLen3 = 0;
    uint8_t CameraResolution = camera->currentPictureMode;
    uint8_t CameraFarmat = camera->currentPixelFormat;
    switch (buff[0])
    {
        case SET_PICTURE_RESOLUTION:                                                                //Set Camera Resolution
                CameraResolution = buff[1] & 0x0f;
                CameraFarmat = (buff[1] & 0x70) >> 4;
                takePicture(camera,(CAM_IMAGE_MODE)CameraResolution, (CAM_IMAGE_PIX_FMT)CameraFarmat);
            break;
        case SET_VIDEO_RESOLUTION:                                                              //Set Video Resolution
                camera->previewMode=TRUE;
                CameraResolution = buff[1] & 0x0f;
                state = startPreview(camera,(CAM_VIDEO_MODE)CameraResolution);
                if (state == CAM_ERR_NO_CALLBACK)
                {
//                  printf("callback function is not registered");
                }
            break;
        case SET_BRIGHTNESS:                                                                //Set brightness
                setBrightness(camera,(CAM_BRIGHTNESS_LEVEL)buff[1]);
            break;
        case SET_CONTRAST:                                                              //Set Contrast
                setContrast(camera,(CAM_CONTRAST_LEVEL)buff[1]);
            break;
        case SET_SATURATION:                                                                //Set saturation
                setSaturation(camera,(CAM_STAURATION_LEVEL)buff[1]);
            break;
        case SET_EV:                                                                //Set EV
                setEV(camera,(CAM_EV_LEVEL)buff[1]);
            break;
        case SET_WHITEBALANCE:                                                              //Set White balance
                setAutoWhiteBalanceMode(camera,(CAM_WHITE_BALANCE)buff[1]);
            break;
        case SET_SPECIAL_EFFECTS:                                                               //Set Special effects
                setColorEffect(camera,(CAM_COLOR_FX)buff[1]);
            break;
        case SET_FOCUS_CONTROL:                                                             //Focus Control
                setAutoFocus(camera,buff[1]);
                if (buff[1] == 0)
                {
                    setAutoFocus(camera,0x02);
                }
            break;
        case SET_EXPOSUREANDGAIN_CONTROL:                                                //exposure and gain control
                setAutoExposure(camera,buff[1]&0x01);
                setAutoISOSensitive(camera,buff[1]&0x01);
            break;
        case SET_WHILEBALANCE_CONTROL:                                                              //while balance control
                setAutoWhiteBalance(camera,buff[1]&0x01);
            break;
        case SET_SHARPNESS:
                setSharpness(camera,(CAM_SHARPNESS_LEVEL)buff[1]);
        break;
        case SET_MANUAL_GAIN:                                                           //manual gain control
                GainValue = (buff[1]<<8) | buff[2];
                setISOSensitivity(camera,GainValue);
            break;
        case SET_MANUAL_EXPOSURE:                                                           //manual exposure control
                ExposureLen1 = buff[1];
                ExposureLen2 = buff[2];
                ExposureLen3 = buff[3];
                ExposureValue = (ExposureLen1<<16) | (ExposureLen2<<8) | ExposureLen3;
                setAbsoluteExposure(camera,ExposureValue);
            break;
        case GET_CAMERA_INFO:                                                           //Get Camera info
                reportCameraInfo(camera);
            break;
        case TAKE_PICTURE:
                takePicture(camera,(CAM_IMAGE_MODE)CameraResolution,(CAM_IMAGE_PIX_FMT)CameraFarmat);
                cameraGetPicture(camera);
            break;
        case DEBUG_WRITE_REGISTER:
                debugWriteRegister(camera,buff+1);
            break;
        case STOP_STREAM:
                stopPreview(camera);
                break;
        case GET_FRM_VER_INFO: // Get Firmware version info
            reportVerInfo(camera);
            break;
        case GET_SDK_VER_INFO: // Get sdk version info
            reportSdkVerInfo(camera);
            break;
        case RESET_CAMERA:
            reset(camera);
        case SET_IMAGE_QUALITY:
            setImageQuality(camera, (IMAGE_QUALITY)buff[1]);
        default:
            break;
    }
    return buff[0];
}

//void cameraGetPicture(ArducamCamera*camera)
//{
//    camera->burstFirstFlag = 0;
//    uint8_t buff[READ_IMAGE_LENGTH] = {0};
//    uint8_t rt_length = 0;
//    arducamUartWrite(0x55);
//    arducamUartWrite(0xAA);
//    arducamUartWrite(camera->cameraDataFormat);
//    arducamUartWrite((uint8_t)(camera->totalLength&0xff));
//    arducamUartWrite((uint8_t)((camera->totalLength>>8)&0xff));
//    arducamUartWrite((uint8_t)((camera->totalLength>>16)&0xff));
//    arducamUartWrite((uint8_t)((camera->totalLength>>24)&0xff));
//    while (camera->receivedLength)
//    {
//        rt_length=readBuff(camera,buff,READ_IMAGE_LENGTH);
//        for (uint8_t i = 0; i < rt_length; i++)
//        {
//            arducamUartWrite(buff[i]);
//
//        }
//    }
//    arducamUartWrite(0x55);
//    arducamUartWrite(0xBB);
//}


//void reportCameraInfo(ArducamCamera*camera)
//{
//    printf("ReportCameraInfo\r\n");
//    printf("Camera Type:");
//    printf("%s\r\n",camera->myCameraInfo.cameraId);
//    printf("Camera Support Resolution:");
//    printf("%d\r\n",camera->myCameraInfo.supportResolution);
//    printf("Camera Support specialeffects:");
//    printf("%d\r\n",camera->myCameraInfo.supportSpecialEffects);
//    printf("Camera Support Focus:");
//    printf("%d\r\n",camera->myCameraInfo.supportFocus);
//    printf("Camera Exposure Value Max:");
//    printf("%d\r\n",camera->myCameraInfo.exposureValueMax);
//    printf("Camera Exposure Value Min:");
//    printf("%d\r\n",camera->myCameraInfo.exposureValueMin);
//    printf("Camera Gain Value Max:");
//    printf("%d\r\n",camera->myCameraInfo.gainValueMax);
//    printf("Camera Gain Value Min:");
//    printf("%d\r\n",camera->myCameraInfo.gainValueMin);
//    printf("Camera Support Sharpness:");
//    printf("%d\r\n",camera->myCameraInfo.supportSharpness);
//}

void arducamUartWrite(uint8_t data)
{
    // while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
    while(!(UCA3IFG&UCTXIFG));                  // eUSCI_A3 TX buffer ready?
    UCA3TXBUF = data;
}

void uartWriteBuffer(uint8_t* buff,uint32_t length)
{
    uint16_t num = 0;
    for(num = 0; num < length; num++)
    {
        arducamUartWrite(buff[num]);
    }
}

uint8_t arducamUartRead(void)
{
    uint8_t rt = 0;
    rt = uartCommBuff[readBuffLength];
    readBuffLength++;
    if(readBuffLength == uartCommLength)
    {
        readBuffLength = 0;
        uartCommLength = 0;
    }
    return rt;
}


uint32_t arducamUartAvailable(void)
{
    return uartCommLength;
}

void arducamFlush(void)
{
    while (arducamUartAvailable()) {
        arducamUartRead();
    }
}

#pragma vector=EUSCI_A3_VECTOR
__interrupt void USCI_A3_ISR(void)
{
//    uartCommBuff[uartCommLength] = UCA3RXBUF;
//    uartCommLength++;

    switch(__even_in_range(UCA3IV, USCI_UART_UCTXCPTIFG))
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
                while(!(UCA3IFG&UCTXIFG));
                uartCommBuff[uartCommLength] = UCA3RXBUF;
                uartCommLength++;
            break;
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
}

