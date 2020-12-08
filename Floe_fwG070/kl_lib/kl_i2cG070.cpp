#include "uartG070.h"
#include "kl_i2cG070.h"
#include "ch.h"
#include "hal.h"

#if I2C_USE_DMA
#define I2C_DMATX_MODE(Chnl) \
                        STM32_DMA_CR_CHSEL(Chnl) |   \
                        DMA_PRIORITY_LOW | \
                        STM32_DMA_CR_MSIZE_BYTE | \
                        STM32_DMA_CR_PSIZE_BYTE | \
                        STM32_DMA_CR_MINC |     /* Memory pointer increase */ \
                        STM32_DMA_CR_DIR_M2P    /* Direction is memory to peripheral */

#define I2C_DMARX_MODE(Chnl) \
                        STM32_DMA_CR_CHSEL(Chnl) |   \
                        DMA_PRIORITY_LOW | \
                        STM32_DMA_CR_MSIZE_BYTE | \
                        STM32_DMA_CR_PSIZE_BYTE | \
                        STM32_DMA_CR_MINC |         /* Memory pointer increase */ \
                        STM32_DMA_CR_DIR_P2M        /* Direction is peripheral to memory */
#endif

#if 1 // ==== Inner defines ====
#define I2C_INT_MASK    ((uint32_t)(I2C_ISR_TCR | I2C_ISR_TC | I2C_ISR_STOPF | I2C_ISR_NACKF | I2C_ISR_ADDR | I2C_ISR_RXNE | I2C_ISR_TXIS))
#define I2C_ERROR_MASK  ((uint32_t)(I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR | I2C_ISR_PECERR | I2C_ISR_TIMEOUT | I2C_ISR_ALERT))

#define I2C_NO_ERROR               0x00    // No error
#define I2C_ACK_FAILURE            0x04    // Acknowledge Failure
#define I2C_SMB_ALERT              0x40    // SMBus Alert
#endif

#if I2C1_ENABLED
static const i2cParams_t I2C1Params = {
        I2C1,
        I2C1_GPIO, I2C1_SCL, I2C1_SDA, I2C_AF,
#if I2C_USE_DMA
        I2C1_DMA_TX,
        I2C1_DMA_RX,
        I2C_DMATX_MODE(I2C1_DMA_CHNL),
        I2C_DMARX_MODE(I2C1_DMA_CHNL),
#endif
#if defined STM32L4XX
        STM32_I2C1_EVENT_NUMBER,
        STM32_I2C1_ERROR_NUMBER,
        I2C_CLK_SRC
#else
        STM32_I2C1_GLOBAL_NUMBER,
        STM32_I2C1_GLOBAL_NUMBER,
#endif
};
i2c_t i2c1 {&I2C1Params};
#endif

#if I2C2_ENABLED
static const i2cParams_t I2C2Params = {
        I2C2,
        SCL2_GPIO, SDA2_GPIO,
        SCL2_PIN, SDA2_PIN, I2C2_AF,
#if I2C_USE_DMA
        I2C2_DMA_TX,
        I2C2_DMA_RX,
        I2C_DMATX_MODE(I2C2_DMA_CHNL),
        I2C_DMARX_MODE(I2C2_DMA_CHNL),
#endif
        I2C2_IRQn
};
i2c_t i2c2 {&I2C2Params};
#endif

void i2c_t::Init() {
    // GPIO
    PinSetupAlterFunc(PParams->PSclGpio, PParams->SclPin, omOpenDrain, pudNone, PParams->PinAF);
    PinSetupAlterFunc(PParams->PSdaGpio, PParams->SdaPin, omOpenDrain, pudNone, PParams->PinAF);
    // I2C
    I2C_TypeDef *pi2c = PParams->pi2c;  // To make things shorter
    if(pi2c == I2C1) {
        Rcc::ResetAPB1(RCC_APBRSTR1_I2C1RST);
        Rcc::EnableAPB1Clk(RCC_APBENR1_I2C1EN);
    }
    else if(pi2c == I2C2) {
        Rcc::ResetAPB1(RCC_APBRSTR1_I2C2RST);
        Rcc::EnableAPB1Clk(RCC_APBENR1_I2C2EN);
    }
    pi2c->CR1 = 0;  // Clear PE bit => disable and reset i2c
    // ==== Setup timings ====
    // Get input clock
//    uint32_t ClkHz = Rcc::GetApbClkHz();
//    // Calc prescaler
//    uint32_t Prescaler = ClkHz / 16000000;
//    if(Prescaler > 0) Prescaler--;
//    // Calc Scl & Sda len
//    uint32_t SclLen = ((ClkHz / 2) / I2C_BAUDRATE_HZ) - 1;
//    if(SclLen >= 4) SclLen -= 4;
//    pi2c->TIMINGR = (Prescaler << 28) | 0x00100000 | (SclLen << 8) | SclLen;
    pi2c->TIMINGR = 0x1082102F; // 99 99
//    pi2c->TIMINGR = 0x00C12166; // 99 99
//    pi2c->TIMINGR = 0x0090216D; // 45 45
//    pi2c->TIMINGR = 0x00602173;

    // Analog filter enabled, digital disabled, clk stretch enabled, DMA enabled
#if I2C_USE_DMA
    pi2c->CR1 = I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN;
    // ==== DMA ====
    PDmaTx = dmaStreamAlloc(PParams->DmaTxID, IRQ_PRIO_MEDIUM, nullptr, nullptr);
    PDmaRx = dmaStreamAlloc(PParams->DmaRxID, IRQ_PRIO_MEDIUM, nullptr, nullptr);
    dmaStreamSetPeripheral(PDmaTx, &pi2c->TXDR);
    dmaStreamSetPeripheral(PDmaRx, &pi2c->RXDR);
#else
    pi2c->CR1 = 0;
#endif
    // ==== IRQ ====
    NVIC_SetPriority(PParams->IrqNumber, IRQ_PRIO_LOW);
    NVIC_EnableIRQ(PParams->IrqNumber);
}

void i2c_t::ScanBus() {
    Printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");
    uint32_t AddrHi, Addr;
    I2C_TypeDef *pi2c = PParams->pi2c;  // To make things shorter
    for(AddrHi = 0; AddrHi < 0x80; AddrHi += 0x10) {
        Printf("\r%02X: ", AddrHi);
        for(uint32_t n=0; n < 0x10; n++) {
            Addr = AddrHi + n;
            if(Addr <= 0x01 or Addr > 0x77) Printf("   ");
            else {
                IReset(); // Reset I2C
                // Set addr and autoend; NBYTES = 0
                pi2c->CR2 = (Addr << 1) | I2C_CR2_AUTOEND;
                pi2c->CR2 |= I2C_CR2_START;     // Start
                while(!(pi2c->ISR & I2C_ISR_STOPF));
                if(pi2c->ISR & I2C_ISR_NACKF) Printf("__ ");
                else Printf("%02X ", Addr);
            }
        } // for lo
    } // for hi
    // Disable I2C
    pi2c->CR1 &= ~I2C_CR1_PE;
    Printf("\r\n");
}

uint8_t i2c_t::CheckAddress(uint32_t Addr) {
    IReset(); // Reset I2C
    uint8_t Rslt;
    int32_t Retries = 9999;
    I2C_TypeDef *pi2c = PParams->pi2c;  // To make things shorter
    if(IBusyWait() != retvOk) {
        Rslt = retvBusy;
        Printf("i2cC Busy\r");
        goto ChckEnd;
    }
    IReset(); // Reset I2C
    pi2c->CR2 = (Addr << 1) | I2C_CR2_AUTOEND;
    pi2c->CR2 |= I2C_CR2_START;     // Start
    while(!(pi2c->ISR & I2C_ISR_STOPF)) {
        if(!Retries--) {
            Rslt = retvTimeout;
            Printf("i2cC TO\r");
            goto ChckEnd;
        }
    }
    if(pi2c->ISR & I2C_ISR_NACKF) Rslt = retvNotFound;
    else Rslt = retvOk;

    ChckEnd:
    return Rslt;
}

uint8_t i2c_t::Write(uint32_t Addr, uint8_t *WPtr, uint32_t WLength) {
    uint8_t Rslt;
    msg_t r;
    I2C_TypeDef *pi2c = PParams->pi2c;  // To make things shorter
    if(WLength == 0 or WPtr == nullptr) { Rslt = retvCmdError; goto WriteEnd; }
    if(IBusyWait() != retvOk) {
        Rslt = retvBusy;
        Printf("i2cW Busy\r");
        goto WriteEnd;
    }
    IReset(); // Reset I2C
#if I2C_USE_DMA    // Prepare TX DMA
    dmaStreamSetMode(PDmaTx, PParams->DmaModeTx);
    dmaStreamSetMemory0(PDmaTx, WPtr);
    dmaStreamSetTransactionSize(PDmaTx, WLength);
#endif
    // Prepare tx
    IState = istWrite;  // Nothing to read
    pi2c->CR2 = (Addr << 1) | (WLength << 16); // Put address and number of bytes to send
    chSysLock();
#if I2C_USE_DMA
    dmaStreamEnable(PDmaTx);   // Enable TX DMA
#else
    // Do not use AUTOEND as TC IRQ will not be generated.
    IPtr = WPtr;
    pi2c->CR1 |= I2C_CR1_TXIE;    // En IRQ on TX Data empty
#endif
    // Enable IRQs: TX completed, error, NAck
    pi2c->CR1 |= (I2C_CR1_TCIE | I2C_CR1_ERRIE | I2C_CR1_NACKIE);
    pi2c->CR2 |= I2C_CR2_START; // Start transmission

    // Wait completion
    r = chThdSuspendTimeoutS(&PThd, TIME_MS2I(I2C_TIMEOUT_MS));
    chSysUnlock();
    // Disable IRQs
    pi2c->CR1 &= ~(I2C_CR1_TCIE | I2C_CR1_ERRIE | I2C_CR1_NACKIE);
    if(r == MSG_TIMEOUT) {
        pi2c->CR2 |= I2C_CR2_STOP;
        Rslt = retvTimeout;
    }
    else Rslt = (IState == istFailure)? retvFail : retvOk;
    WriteEnd:
    return Rslt;
}

uint8_t i2c_t::WriteRead(uint32_t Addr, uint8_t *WPtr, uint32_t WLength, uint8_t *RPtr, uint32_t RLength) {
    uint8_t Rslt;
    msg_t r;
    I2C_TypeDef *pi2c = PParams->pi2c;  // To make things shorter
    if(WLength == 0 or WPtr == nullptr) { Rslt = retvCmdError; goto WriteReadEnd; }
    if(IBusyWait() != retvOk) {
        Rslt = retvBusy;
        Printf("i2cWR Busy\r");
        goto WriteReadEnd;
    }
    IReset(); // Reset I2C
    pi2c->CR2 = (Addr << 1) | (WLength << 16);
#if I2C_USE_DMA    // Prepare TX DMA
    dmaStreamSetMode(PDmaTx, PParams->DmaModeTx);
    dmaStreamSetMemory0(PDmaTx, WPtr);
    dmaStreamSetTransactionSize(PDmaTx, WLength);
    if(RLength != 0 and RPtr != nullptr) {
        // Prepare RX DMA
        dmaStreamSetMode(PDmaRx, PParams->DmaModeRx);
        dmaStreamSetMemory0(PDmaRx, RPtr);
        dmaStreamSetTransactionSize(PDmaRx, RLength);
        ILen2 = RLength;
        IState = istWriteRead;
    }
    else IState = istWrite;  // Nothing to read
    dmaStreamEnable(PDmaTx);   // Enable TX DMA
#else
    IPtr = WPtr;
    if(RLength != 0 and RPtr != nullptr) {
        IPtr2 = RPtr;
        ILen2 = RLength;
        IState = istWriteRead;
    }
    else IState = istWrite;  // Nothing to read
    pi2c->CR1 |= I2C_CR1_TXIE | I2C_CR1_RXIE;
#endif
    chSysLock();
    // Enable IRQs: TX completed, error, NAck
    pi2c->CR1 |= (I2C_CR1_TCIE | I2C_CR1_ERRIE | I2C_CR1_NACKIE);
    pi2c->CR2 |= I2C_CR2_START;         // Start transmission
    // Wait completion
    r = chThdSuspendTimeoutS(&PThd, TIME_MS2I(I2C_TIMEOUT_MS));
    chSysUnlock();
    // Disable IRQs
    pi2c->CR1 &= ~(I2C_CR1_TCIE | I2C_CR1_ERRIE | I2C_CR1_NACKIE);
    if(r == MSG_TIMEOUT) {
        pi2c->CR2 |= I2C_CR2_STOP;
        Rslt = retvTimeout;
    }
    else Rslt = (IState == istFailure)? retvFail : retvOk;
    WriteReadEnd:
    return Rslt;
}

uint8_t i2c_t::WriteReadNoDMA(uint32_t Addr, uint8_t *WPtr,  uint32_t WLength, uint8_t *RPtr, uint32_t RLength) {
    return 0;
}

uint8_t i2c_t::WriteWrite(uint32_t Addr, uint8_t *WPtr1, uint32_t WLength1, uint8_t *WPtr2, uint32_t WLength2) {
    uint8_t Rslt;
    msg_t r;
    I2C_TypeDef *pi2c = PParams->pi2c;  // To make things shorter
    if(WLength1 == 0 or WPtr1 == nullptr) { Rslt = retvCmdError; goto WriteWriteEnd; }
    if(IBusyWait() != retvOk) { Rslt = retvBusy; goto WriteWriteEnd; }
    IReset(); // Reset I2C
#if I2C_USE_DMA    // Prepare TX DMA
    dmaStreamSetMode(PDmaTx, PParams->DmaModeTx);
    dmaStreamSetMemory0(PDmaTx, WPtr1);
    dmaStreamSetTransactionSize(PDmaTx, WLength1);
#else
    IPtr = WPtr1;
    pi2c->CR1 |= I2C_CR1_TXIE;
#endif
    // Prepare transmission
    if(WLength2 != 0 and WPtr2 != nullptr) {
        IState = istWriteWrite;
        IPtr2 = WPtr2;
        ILen2 = WLength2;
        pi2c->CR2 = (Addr << 1) | (WLength1 << 16) | I2C_CR2_RELOAD; // Generate TCR IRQ on first batch done
    }
    else { // No second write
        IState = istWrite;
        pi2c->CR2 = (Addr << 1) | (WLength1 << 16);
    }
    chSysLock();
    // Enable IRQs: TX completed, error, NAck
    pi2c->CR1 |= (I2C_CR1_TCIE | I2C_CR1_ERRIE | I2C_CR1_NACKIE);
    pi2c->CR2 |= I2C_CR2_START;         // Start transmission
#if I2C_USE_DMA
    dmaStreamEnable(PDmaTx);   // Enable TX DMA
#endif
    // Wait completion
    r = chThdSuspendTimeoutS(&PThd, TIME_MS2I(I2C_TIMEOUT_MS));
    chSysUnlock();
    // Disable IRQs
    pi2c->CR1 &= ~(I2C_CR1_TCIE | I2C_CR1_ERRIE | I2C_CR1_NACKIE);
    if(r == MSG_TIMEOUT) {
        pi2c->CR2 |= I2C_CR2_STOP;
        Rslt = retvTimeout;
    }
    else Rslt = (IState == istFailure)? retvFail : retvOk;
    WriteWriteEnd:
    return Rslt;
}

void i2c_t::IReset() {
    PParams->pi2c->CR1 &= ~I2C_CR1_PE;
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); // Wait 9 cycles
    PParams->pi2c->CR1 |= I2C_CR1_PE;
    IState = istIdle;
}

void i2c_t::Standby() {
    PParams->pi2c->CR1 &= ~I2C_CR1_PE;
    I2C_TypeDef *pi2c = PParams->pi2c;  // To make things shorter
    if(pi2c == I2C1) {
        rccResetI2C1();
        rccDisableI2C1();
    }
    else if(pi2c == I2C2) {
        rccResetI2C2();
        rccDisableI2C2();
    }
    PinSetupAnalog(PParams->PSclGpio, PParams->SclPin);
    PinSetupAnalog(PParams->PSdaGpio, PParams->SdaPin);
#if I2C_USE_DMA
    dmaStreamFree(PDmaTx);
    dmaStreamFree(PDmaRx);
#endif
    __NOP(); __NOP(); __NOP();  // Wait 3 cycles
}

void i2c_t::PutBusLow() {
    PParams->pi2c->CR1 &= ~I2C_CR1_PE;
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    PinSetupOut(PParams->PSclGpio, PParams->SclPin, omOpenDrain);
    PinSetupOut(PParams->PSdaGpio, PParams->SdaPin, omOpenDrain);
    PinSetLo(PParams->PSclGpio, PParams->SclPin);
    PinSetLo(PParams->PSdaGpio, PParams->SdaPin);
}

void i2c_t::Resume() {
    PParams->pi2c->CR1 |= I2C_CR1_PE;
    PinSetupAlterFunc(PParams->PSclGpio, PParams->SclPin, omOpenDrain, pudNone, PParams->PinAF);
    PinSetupAlterFunc(PParams->PSdaGpio, PParams->SdaPin, omOpenDrain, pudNone, PParams->PinAF);
}

// Will disconnect bus and will not reconnect it
uint8_t i2c_t::CheckBusAndResume() {
    PParams->pi2c->CR1 &= ~I2C_CR1_PE;
    PinSetupInput(PParams->PSclGpio, PParams->SclPin, pudPullUp);
    PinSetupInput(PParams->PSdaGpio, PParams->SdaPin, pudPullUp);
    chThdSleepMilliseconds(2);
    uint8_t Rslt = (PinIsHi(PParams->PSclGpio, PParams->SclPin) and PinIsHi(PParams->PSdaGpio, PParams->SdaPin))? retvOk : retvFail;
    Resume();
    return Rslt;
}

uint8_t i2c_t::IBusyWait() {
    uint8_t RetryCnt = 4;
    while(RetryCnt--) {
        if(!(PParams->pi2c->ISR & I2C_ISR_BUSY)) return retvOk;
        chThdSleepMilliseconds(1);
    }
    return retvTimeout;
}


void i2c_t::IServeIRQ(uint32_t isr) {
    I2C_TypeDef *pi2c = PParams->pi2c;  // To make things shorter
//    PrintfI("ISR: 0x%X\r\n", isr);
#if 1 // ==== NACK ====
    if(isr & I2C_ISR_NACKF) {
        PrintfI("i2c 0x%X NACK\r\n", (pi2c->CR2 >> 1) & 0xFF);
#if I2C_USE_DMA // Stop DMA
        dmaStreamDisable(PDmaTx);
        dmaStreamDisable(PDmaRx);
#endif
        // Stop transaction
        pi2c->CR2 |= I2C_CR2_STOP;
        // Disable IRQs
        pi2c->CR1 &= ~(I2C_CR1_TCIE | I2C_CR1_TXIE | I2C_CR1_RXIE);
        IState = istFailure;
        IWakeup();
        return;
    }
#endif

#if !I2C_USE_DMA
    if(isr & I2C_ISR_TXIS) {
//        PrintfI("v: 0x%X\r\n", *IPtr);
        pi2c->TXDR = *IPtr++;
    }
    if(isr & I2C_ISR_RXNE) {
        *IPtr2++ = pi2c->RXDR;
    }
#endif

#if 1 // ==== TX partly completed ====
    if((isr & I2C_ISR_TCR) != 0) {
#if I2C_USE_DMA // Stop DMA
        dmaStreamDisable(PDmaTx);
#endif
        if(IState == istWriteWrite) {
            // Send next ILen bytes
            pi2c->CR2 = (pi2c->CR2 & ~(I2C_CR2_NBYTES | I2C_CR2_RELOAD)) | (ILen2 << 16);
#if I2C_USE_DMA // Prepare and enable TX DMA for second write
            dmaStreamSetMode(PDmaTx, PParams->DmaModeTx);
            dmaStreamSetMemory0(PDmaTx, IPtr2);
            dmaStreamSetTransactionSize(PDmaTx, ILen2);
            dmaStreamEnable(PDmaTx);
#else
            IPtr = IPtr2;
#endif
            IState = istWrite;
        }
    }
#endif
#if 1 // ==== TX completed ====
    if((isr & I2C_ISR_TC) != 0) {
#if I2C_USE_DMA // Stop DMA
        dmaStreamDisable(PDmaTx);  // }
        dmaStreamDisable(PDmaRx);  // } Both sorts of transaction may be completed
#endif
        if(IState == istWriteRead) {  // Write phase completed
            // Receive ILen bytes
            pi2c->CR2 = (pi2c->CR2 & ~I2C_CR2_NBYTES) | I2C_CR2_RD_WRN | (ILen2 << 16);
#if I2C_USE_DMA
            dmaStreamEnable(PDmaRx);
#endif
            pi2c->CR2 |= I2C_CR2_START; // Send repeated start
            IState = istRead;
        } // if WriteRead
        else { // istWrite, istRead
            IState = istIdle;
            pi2c->CR2 |= I2C_CR2_STOP;
            pi2c->CR1 &= ~I2C_CR1_TCIE; // Disable TransferComplete IRQ
            IWakeup();
        }
    }
#endif
    // ==== Errors ====
    isr &= (I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR | I2C_ISR_TIMEOUT);
    // If some error has been identified then wake the waiting thread
    if(isr != I2C_NO_ERROR) {
        PrintfI("i2c err: %X\r", isr);
        IWakeup();
    }
}

void i2c_t::IWakeup() {
    chSysLockFromISR();
    chThdResumeI(&PThd, MSG_OK);
    chSysUnlockFromISR();
}

#if 1 // =============================== IRQs ==================================
extern "C" {
#if I2C1_ENABLED // ==== I2C1 ====
#if defined STM32L4XX
OSAL_IRQ_HANDLER(STM32_I2C1_EVENT_HANDLER) {
//    Uart.PrintfI("i2c1 irq\r");
    uint32_t isr = I2C1->ISR;
    OSAL_IRQ_PROLOGUE();
    I2C1->ICR = isr & I2C_INT_MASK; // Clear IRQ bits
    i2c1.IServeIRQ(isr);
    OSAL_IRQ_EPILOGUE();
}

OSAL_IRQ_HANDLER(STM32_I2C1_ERROR_HANDLER) {
    uint32_t isr = I2C1->ISR;
    OSAL_IRQ_PROLOGUE();
    I2C1->ICR = isr & I2C_ERROR_MASK; // Clear IRQ bits
    i2c1.IServeErrIRQ(isr);
    OSAL_IRQ_EPILOGUE();
}
#else
OSAL_IRQ_HANDLER(STM32_I2C1_GLOBAL_HANDLER) {
//    Uart.PrintfI("i2c1 irq\r");
    uint32_t isr = I2C1->ISR;
    uint32_t isrEvt = isr & I2C_INT_MASK;
    uint32_t isrErr = isr & I2C_ERROR_MASK;
    OSAL_IRQ_PROLOGUE();
    I2C1->ICR = isr; // Clear IRQ bits
    if(isrEvt != 0) i2c1.IServeIRQ(isrEvt);
    if(isrErr != 0) i2c1.IServeErrIRQ(isrErr);
    OSAL_IRQ_EPILOGUE();
}
#endif // MCU type
#endif
#if I2C2_ENABLED // ==== I2C2 ====
void STM32_I2C2_GLOBAL_HANDLER() {
    OSAL_IRQ_PROLOGUE();
    uint32_t isr = I2C2->ISR;
    I2C2->ICR = isr & I2C_INT_MASK; // Clear IRQ bits
    i2c2.IServeIRQ(isr);
    OSAL_IRQ_EPILOGUE();
}

#endif
} // extern C
#endif
