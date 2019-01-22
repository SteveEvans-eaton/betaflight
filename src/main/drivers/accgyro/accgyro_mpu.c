/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#include "build/atomic.h"
#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/dma.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu3050.h"
#include "drivers/accgyro/accgyro_mpu6050.h"
#include "drivers/accgyro/accgyro_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_bmi160.h"
#include "drivers/accgyro/accgyro_spi_icm20649.h"
#include "drivers/accgyro/accgyro_spi_icm20689.h"
#include "drivers/accgyro/accgyro_spi_mpu6000.h"
#include "drivers/accgyro/accgyro_spi_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_mpu9250.h"
#include "drivers/accgyro/accgyro_spi_l3gd20.h"
#include "drivers/accgyro/accgyro_mpu.h"

#include "pg/pg.h"
#include "pg/gyrodev.h"
#include "pg/pinio.h" // SCEDEBUG

#ifndef MPU_ADDRESS
#define MPU_ADDRESS             0x68
#endif

#define MPU_INQUIRY_MASK   0x7E

#ifdef USE_I2C_GYRO
static void mpu6050FindRevision(gyroDev_t *gyro)
{
    // There is a map of revision contained in the android source tree which is quite comprehensive and may help to understand this code
    // See https://android.googlesource.com/kernel/msm.git/+/eaf36994a3992b8f918c18e4f7411e8b2320a35f/drivers/misc/mpu6050/mldl_cfg.c

    // determine product ID and revision
    uint8_t readBuffer[6];
    bool ack = busReadRegisterBuffer(&gyro->bus, MPU_RA_XA_OFFS_H, readBuffer, 6);
    uint8_t revision = ((readBuffer[5] & 0x01) << 2) | ((readBuffer[3] & 0x01) << 1) | (readBuffer[1] & 0x01);
    if (ack && revision) {
        // Congrats, these parts are better
        if (revision == 1) {
            gyro->mpuDetectionResult.resolution = MPU_HALF_RESOLUTION;
        } else if (revision == 2) {
            gyro->mpuDetectionResult.resolution = MPU_FULL_RESOLUTION;
        } else if ((revision == 3) || (revision == 7)) {
            gyro->mpuDetectionResult.resolution = MPU_FULL_RESOLUTION;
        } else {
            failureMode(FAILURE_ACC_INCOMPATIBLE);
        }
    } else {
        uint8_t productId;
        ack = busReadRegisterBuffer(&gyro->bus, MPU_RA_PRODUCT_ID, &productId, 1);
        revision = productId & 0x0F;
        if (!ack || revision == 0) {
            failureMode(FAILURE_ACC_INCOMPATIBLE);
        } else if (revision == 4) {
            gyro->mpuDetectionResult.resolution = MPU_HALF_RESOLUTION;
        } else {
            gyro->mpuDetectionResult.resolution = MPU_FULL_RESOLUTION;
        }
    }
}
#endif

/*
 * Gyro interrupt service routine
 */
#ifdef USE_GYRO_EXTI
static void mpuIntExtiHandler(extiCallbackRec_t *cb)
{
    pinioSet(3, 0); // SCEDEBUG
#ifdef DEBUG_MPU_DATA_READY_INTERRUPT
    static uint32_t lastCalledAtUs = 0;
    const uint32_t nowUs = micros();
    debug[0] = (uint16_t)(nowUs - lastCalledAtUs);
    lastCalledAtUs = nowUs;
#endif
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    gyro->dataReady = true;
#ifdef DEBUG_MPU_DATA_READY_INTERRUPT
    const uint32_t now2Us = micros();
    debug[1] = (uint16_t)(now2Us - nowUs);
#endif
    pinioSet(3, 1); // SCEDEBUG
}

static void mpuIntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    const IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

#ifdef ENSURE_MPU_DATA_READY_IS_LOW
    uint8_t status = IORead(mpuIntIO);
    if (status) {
        return;
    }
#endif

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, mpuIntExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO, true);
}
#endif // USE_GYRO_EXTI

bool mpuAccRead(accDev_t *acc)
{
    uint8_t data[6];

    const bool ack = busReadRegisterBuffer(&acc->bus, MPU_RA_ACCEL_XOUT_H, data, 6);
    if (!ack) {
        return false;
    }

    acc->ADCRaw[X] = (int16_t)((data[0] << 8) | data[1]);
    acc->ADCRaw[Y] = (int16_t)((data[2] << 8) | data[3]);
    acc->ADCRaw[Z] = (int16_t)((data[4] << 8) | data[5]);

    return true;
}

bool mpuGyroRead(gyroDev_t *gyro)
{
    uint8_t data[6];

    const bool ack = busReadRegisterBuffer(&gyro->bus, MPU_RA_GYRO_XOUT_H, data, 6);
    if (!ack) {
        return false;
    }

    gyro->gyroADCRaw[X] = (int16_t)((data[0] << 8) | data[1]);
    gyro->gyroADCRaw[Y] = (int16_t)((data[2] << 8) | data[3]);
    gyro->gyroADCRaw[Z] = (int16_t)((data[4] << 8) | data[5]);

    return true;
}

#ifdef USE_SPI_GYRO
/* So, this stinks for now, but until there's a per target mechanism to define DMA channel usage...
 *
 * On the Revo Nano, the MPU9250 is on SPI bus 2 using DMA1 thus:
 * RX: Channel 0, Stream 3
 * TX: Channel 0, Stream 4
 */
#define MPU_DMA_CHANNEL_RX DMA1_Stream3
#define MPU_DMA_CHANNEL_TX DMA1_Stream4
//#define MPU_DMA_IRQ_HANDLER_ID DMA1_ST3_HANDLER

#ifdef MPU_DMA_IRQ_HANDLER_ID
volatile bool dmaTransactionInProgress = false;
busDevice_t *gyro_busdev;

static bool mpuSpiBusTransfer(busDevice_t *busdev, const uint8_t* tx_buffer, uint8_t* rx_buffer, uint16_t buffer_size)
{
    DMA_InitTypeDef DMA_InitStructure;
    static uint16_t dummy[] = {0xffff};

    while (dmaTransactionInProgress); // Wait for prev DMA transaction

    DMA_DeInit(MPU_DMA_CHANNEL_TX);
    DMA_DeInit(MPU_DMA_CHANNEL_RX);

    // Common to both channels
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(busdev->busdev_u.spi.instance->DR));
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_BufferSize = buffer_size;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;

    // Rx Channel

#ifdef STM32F4
    DMA_InitStructure.DMA_Memory0BaseAddr = rx_buffer ? (uint32_t)rx_buffer : (uint32_t)(dummy);
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
#else
    DMA_InitStructure.DMA_MemoryBaseAddr = rx_buffer ? (uint32_t)rx_buffer : (uint32_t)(dummy);
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
#endif
    DMA_InitStructure.DMA_MemoryInc = rx_buffer ? DMA_MemoryInc_Enable : DMA_MemoryInc_Disable;

    DMA_Init(MPU_DMA_CHANNEL_RX, &DMA_InitStructure);
    DMA_Cmd(MPU_DMA_CHANNEL_RX, ENABLE);

    // Tx channel

#ifdef STM32F4
    DMA_InitStructure.DMA_Memory0BaseAddr = tx_buffer ? (uint32_t)tx_buffer : (uint32_t)(dummy);
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
#else
    DMA_InitStructure.DMA_MemoryBaseAddr = tx_buffer ? (uint32_t)tx_buffer : (uint32_t)(dummy);
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
#endif
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;

    DMA_Init(MPU_DMA_CHANNEL_TX, &DMA_InitStructure);
    DMA_Cmd(MPU_DMA_CHANNEL_TX, ENABLE);

    DMA_ITConfig(MPU_DMA_CHANNEL_RX, DMA_IT_TC, ENABLE);
    DMA_ITConfig(MPU_DMA_CHANNEL_TX, DMA_IT_TC, ENABLE);

    // Enable SPI TX/RX request

    spiBusTransactionBegin(busdev);
    gyro_busdev = busdev;
    IOLo(busdev->busdev_u.spi.csnPin);
    dmaTransactionInProgress = true;
    pinioSet(2, 0); // SCEDEBUG

    SPI_I2S_DMACmd(busdev->busdev_u.spi.instance,
            SPI_I2S_DMAReq_Rx |
            SPI_I2S_DMAReq_Tx, ENABLE);

    while (dmaTransactionInProgress);

    return true;
}

static void mpu_dma_irq_handler(dmaChannelDescriptor_t* descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        DMA_Cmd(MPU_DMA_CHANNEL_RX, DISABLE);
        // Make sure SPI DMA transfer is complete

        while (SPI_I2S_GetFlagStatus (gyro_busdev->busdev_u.spi.instance, SPI_I2S_FLAG_TXE) == RESET) {};
        while (SPI_I2S_GetFlagStatus (gyro_busdev->busdev_u.spi.instance, SPI_I2S_FLAG_BSY) == SET) {};

        // Empty RX buffer. RX DMA takes care of it if enabled.
        // This should be done after transmission finish!!!

        while (SPI_I2S_GetFlagStatus(gyro_busdev->busdev_u.spi.instance, SPI_I2S_FLAG_RXNE) == SET) {
            gyro_busdev->busdev_u.spi.instance->DR;
        }

        DMA_Cmd(MPU_DMA_CHANNEL_TX, DISABLE);

        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);

        SPI_I2S_DMACmd(gyro_busdev->busdev_u.spi.instance,
                SPI_I2S_DMAReq_Rx |
                SPI_I2S_DMAReq_Tx, DISABLE);

        spiBusTransactionEnd(gyro_busdev);
        IOHi(gyro_busdev->busdev_u.spi.csnPin);
        dmaTransactionInProgress = false;
        pinioSet(2, 1); // SCEDEBUG
    }

    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_HTIF)) {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_HTIF);
    }
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TEIF)) {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TEIF);
    }
}
#endif // MPU_DMA_IRQ_HANDLER_ID

bool mpuGyroReadSPI(gyroDev_t *gyro)
{
    static const uint8_t dataToSend[7] = {MPU_RA_GYRO_XOUT_H | 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t data[7];

    pinioSet(1, 0); // SCEDEBUG
#ifdef MPU_DMA_IRQ_HANDLER_ID
    bool ack = mpuSpiBusTransfer(&gyro->bus, dataToSend, data, 7);
#else // MPU_DMA_IRQ_HANDLER_ID
    bool ack = spiBusTransfer(&gyro->bus, dataToSend, data, 7);
#endif // MPU_DMA_IRQ_HANDLER_ID
    pinioSet(1, 1); // SCEDEBUG

    if (!ack) {
        return false;
    }

    gyro->gyroADCRaw[X] = (int16_t)((data[1] << 8) | data[2]);
    gyro->gyroADCRaw[Y] = (int16_t)((data[3] << 8) | data[4]);
    gyro->gyroADCRaw[Z] = (int16_t)((data[5] << 8) | data[6]);

    return true;
}

typedef uint8_t (*gyroSpiDetectFn_t)(const busDevice_t *bus);

static gyroSpiDetectFn_t gyroSpiDetectFnTable[] = {
#ifdef USE_GYRO_SPI_MPU6000
    mpu6000SpiDetect,
#endif
#ifdef USE_GYRO_SPI_MPU6500
    mpu6500SpiDetect,   // some targets using MPU_9250_SPI, ICM_20608_SPI or ICM_20602_SPI state sensor is MPU_65xx_SPI
#endif
#ifdef  USE_GYRO_SPI_MPU9250
    mpu9250SpiDetect,
#endif
#ifdef USE_GYRO_SPI_ICM20649
    icm20649SpiDetect,
#endif
#ifdef USE_GYRO_SPI_ICM20689
    icm20689SpiDetect,  // icm20689SpiDetect detects ICM20602 and ICM20689
#endif
#ifdef USE_ACCGYRO_BMI160
    bmi160Detect,
#endif
#ifdef USE_GYRO_L3GD20
    l3gd20Detect,
#endif
    NULL // Avoid an empty array
};

static bool detectSPISensorsAndUpdateDetectionResult(gyroDev_t *gyro, const gyroDeviceConfig_t *config)
{
    SPI_TypeDef *instance = spiInstanceByDevice(SPI_CFG_TO_DEV(config->spiBus));
    if (!instance) {
        return false;
    }
    spiBusSetInstance(&gyro->bus, instance);

    gyro->bus.busdev_u.spi.csnPin = IOGetByTag(config->csnTag);
    IOInit(gyro->bus.busdev_u.spi.csnPin, OWNER_GYRO_CS, RESOURCE_INDEX(config->index));
    IOConfigGPIO(gyro->bus.busdev_u.spi.csnPin, SPI_IO_CS_CFG);
    IOHi(gyro->bus.busdev_u.spi.csnPin); // Ensure device is disabled, important when two devices are on the same bus.

    uint8_t sensor = MPU_NONE;

    // It is hard to use hardware to optimize the detection loop here,
    // as hardware type and detection function name doesn't match.
    // May need a bitmap of hardware to detection function to do it right?

    for (size_t index = 0 ; gyroSpiDetectFnTable[index] ; index++) {
        sensor = (gyroSpiDetectFnTable[index])(&gyro->bus);
        if (sensor != MPU_NONE) {
            gyro->mpuDetectionResult.sensor = sensor;
            return true;
        }
    }

    spiPreinitByTag(config->csnTag);

    return false;
}
#endif

void mpuPreInit(const struct gyroDeviceConfig_s *config)
{
#ifdef USE_SPI_GYRO
    spiPreinitRegister(config->csnTag, IOCFG_IPU, 1);
#ifdef MPU_DMA_IRQ_HANDLER_ID
    dmaSetHandler(MPU_DMA_IRQ_HANDLER_ID, mpu_dma_irq_handler, NVIC_PRIO_MPU_DMA, 0);
#endif
#else
    UNUSED(config);
#endif
}

void mpuDetect(gyroDev_t *gyro, const gyroDeviceConfig_t *config)
{
    // MPU datasheet specifies 30ms.
    delay(35);

    if (config->bustype == BUSTYPE_NONE) {
        return;
    }

    if (config->bustype == BUSTYPE_GYRO_AUTO) {
        gyro->bus.bustype = BUSTYPE_I2C;
    } else {
        gyro->bus.bustype = config->bustype;
    }

#ifdef USE_I2C_GYRO
    if (gyro->bus.bustype == BUSTYPE_I2C) {
        gyro->bus.busdev_u.i2c.address = config->i2cAddress ? config->i2cAddress : MPU_ADDRESS;

        uint8_t sig = 0;
        bool ack = busReadRegisterBuffer(&gyro->bus, MPU_RA_WHO_AM_I, &sig, 1);

        if (ack) {
            // If an MPU3050 is connected sig will contain 0.
            uint8_t inquiryResult;
            ack = busReadRegisterBuffer(&gyro->bus, MPU_RA_WHO_AM_I_LEGACY, &inquiryResult, 1);
            inquiryResult &= MPU_INQUIRY_MASK;
            if (ack && inquiryResult == MPUx0x0_WHO_AM_I_CONST) {
                gyro->mpuDetectionResult.sensor = MPU_3050;
                return;
            }

            sig &= MPU_INQUIRY_MASK;
            if (sig == MPUx0x0_WHO_AM_I_CONST) {
                gyro->mpuDetectionResult.sensor = MPU_60x0;
                mpu6050FindRevision(gyro);
            } else if (sig == MPU6500_WHO_AM_I_CONST) {
                gyro->mpuDetectionResult.sensor = MPU_65xx_I2C;
            }
            return;
        }
    }
#endif

#ifdef USE_SPI_GYRO
    gyro->bus.bustype = BUSTYPE_SPI;
    detectSPISensorsAndUpdateDetectionResult(gyro, config);
#endif
}

void mpuGyroInit(gyroDev_t *gyro)
{
#ifdef USE_GYRO_EXTI
    mpuIntExtiInit(gyro);
#else
    UNUSED(gyro);
#endif
}

uint8_t mpuGyroDLPF(gyroDev_t *gyro)
{
    uint8_t ret = 0;
    
    // If gyro is in 32KHz mode then the DLPF bits aren't used
    if (gyro->gyroRateKHz <= GYRO_RATE_8_kHz) {
        switch (gyro->hardware_lpf) {
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
            case GYRO_HARDWARE_LPF_EXPERIMENTAL:
                // experimental mode not supported for MPU60x0 family
                if ((gyro->gyroHardware != GYRO_MPU6050) && (gyro->gyroHardware != GYRO_MPU6000)) {
                    ret = 7;
                } else {
                    ret = 0;
                }
                break;
#endif

            case GYRO_HARDWARE_LPF_1KHZ_SAMPLE:
                ret = 1;
                break;
                
            case GYRO_HARDWARE_LPF_NORMAL:
            default:
                ret = 0;
                break;
        }
    }
    return ret;
}

uint8_t mpuGyroFCHOICE(gyroDev_t *gyro)
{
    if (gyro->gyroRateKHz > GYRO_RATE_8_kHz) {
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
        if (gyro->hardware_32khz_lpf == GYRO_32KHZ_HARDWARE_LPF_EXPERIMENTAL) {
            return FCB_8800_32;
        } else {
            return FCB_3600_32;
        }
#else
        return FCB_3600_32;
#endif
    } else {
        return FCB_DISABLED;  // Not in 32KHz mode, set FCHOICE to select 8KHz sampling
    }
}

#ifdef USE_GYRO_REGISTER_DUMP
uint8_t mpuGyroReadRegister(const busDevice_t *bus, uint8_t reg)
{
    uint8_t data;
    const bool ack = busReadRegisterBuffer(bus, reg, &data, 1);
    if (ack) {
        return data;
    } else {
        return 0;
    }

}
#endif
