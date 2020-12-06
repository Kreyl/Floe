#pragma once

#include "kl_libG070.h"
#include "ch.h"

struct i2cParams_t {
    I2C_TypeDef *pi2c;
    GPIO_TypeDef *PSclGpio;
    GPIO_TypeDef *PSdaGpio;
    uint16_t SclPin;
    uint16_t SdaPin;
    AlterFunc_t PinAF;
    // DMA
#if I2C_USE_DMA
    uint32_t DmaTxID, DmaRxID;
    uint32_t DmaModeTx, DmaModeRx;
#endif
    // IRQ
    IRQn_Type IrqNumber;
};

#define I2C_TIMEOUT_MS      999


class i2c_t {
private:
    enum i2cState_t {istIdle, istWriteRead, istWriteWrite, istRead, istWrite, istFailure};
    const i2cParams_t *PParams;
#if I2C_USE_DMA
    const stm32_dma_stream_t *PDmaTx = nullptr, *PDmaRx = nullptr;
#endif
    uint8_t IBusyWait();
    void IReset();
    thread_reference_t PThd = nullptr;
    i2cState_t IState = istIdle;
    void IWakeup();
    uint8_t *IPtr = nullptr;
    uint8_t *IPtr2 = nullptr;
    uint32_t ILen2 = 0;
public:
    i2c_t(const i2cParams_t *APParams) : PParams(APParams) {}
    void Init();
    void ScanBus();
    void Standby();
    void PutBusLow();
    void Resume();
    uint8_t CheckBusAndResume();
    uint8_t CheckAddress(uint32_t Addr);
    uint8_t Write     (uint32_t Addr, uint8_t *WPtr,  uint32_t WLength);
    uint8_t WriteRead (uint32_t Addr, uint8_t *WPtr,  uint32_t WLength, uint8_t *RPtr, uint32_t RLength);
    uint8_t WriteWrite(uint32_t Addr, uint8_t *WPtr1, uint32_t WLength1, uint8_t *WPtr2, uint32_t WLength2);
    uint8_t WriteReadNoDMA(uint32_t Addr, uint8_t *WPtr,  uint32_t WLength, uint8_t *RPtr, uint32_t RLength);
    // Inner use
    void IServeIRQ(uint32_t isr);
};

#if I2C1_ENABLED
extern i2c_t i2c1
#endif

#if I2C2_ENABLED
extern i2c_t i2c2;
#endif
