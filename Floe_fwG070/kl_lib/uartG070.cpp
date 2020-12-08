/*
 * cmd_uart.cpp
 *
 *  Created on: 15.04.2013
 *      Author: kreyl
 */

#include "uartG070.h"
#include "kl_libG070.h"
#include "MsgQ.h"
#include "ch.h"
#include "hal.h"

#if 1 // ========================= Base UART ===================================
#if 1 // ==== TX ====

// Wrapper for TX IRQ
void UartDmaTxIrqHandler(void *p, uint32_t flags) { ((BaseUart_t*)p)->IRQDmaTxHandlerI(); }

// ==== UART IRQs ====
struct UartIrqHandler_t {
    ftVoidPVoidW32 Handler = nullptr;
    void *Param = nullptr;
};

static BaseUart_t *BaseUart1 = nullptr;
static BaseUart_t *BaseUart2 = nullptr;
static BaseUart_t *BaseUart3 = nullptr;
static BaseUart_t *BaseUart4 = nullptr;

extern "C" {
void STM32_USART1_HANDLER() {
    OSAL_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t flags = USART1->ISR;
    USART1->ICR = flags;
    if(flags and BaseUart1) BaseUart1->IRQUartHandlerI(flags);
    chSysUnlockFromISR();
    OSAL_IRQ_EPILOGUE();
}

void STM32_USART2_HANDLER() {
    OSAL_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t flags = USART2->ISR;
    USART2->ICR = flags;
    if(flags and BaseUart2) BaseUart2->IRQUartHandlerI(flags);
    chSysUnlockFromISR();
    OSAL_IRQ_EPILOGUE();
}

void STM32_USART3_4_LP1_HANDLER() {
    OSAL_IRQ_PROLOGUE();
    chSysLockFromISR();
    uint32_t flags = USART3->ISR;
    USART3->ICR = flags;
    if(flags and BaseUart3) BaseUart3->IRQUartHandlerI(flags);
    flags = USART4->ISR;
    USART4->ICR = flags;
    if(flags and BaseUart4) BaseUart4->IRQUartHandlerI(flags);
    chSysUnlockFromISR();
    OSAL_IRQ_EPILOGUE();
}
} // extern "C"

// ==== TX DMA IRQ ====
void BaseUart_t::IRQDmaTxHandlerI() {
    DmaTx.Disable(); // Registers may be changed only when stream is disabled
    IFullSlotsCount -= ITransSize;
    PRead += ITransSize;
    if(PRead >= (TXBuf + UART_TXBUF_SZ)) PRead = TXBuf; // Circulate pointer
    if(IFullSlotsCount == 0) {  // Nothing left to send
        ITxDmaIsIdle = true;
    }
    else ISendViaDMA();
}

void BaseUart_t::ISendViaDMA() {
    uint32_t PartSz = (TXBuf + UART_TXBUF_SZ) - PRead; // Cnt from PRead to end of buf
    ITransSize = MIN_(IFullSlotsCount, PartSz);
    if(ITransSize != 0) {
        ITxDmaIsIdle = false;
        DmaTx.SetMemoryAddr(PRead);
        DmaTx.SetTransferDataCnt(ITransSize);
        Params->Uart->ICR = USART_ICR_TCCF; // Clear TC flag in ISR reg
        DmaTx.Enable();
    }
}

uint8_t BaseUart_t::IPutByte(uint8_t b) {
    if(IFullSlotsCount >= UART_TXBUF_SZ) return retvOverflow;
    *PWrite++ = b;
    if(PWrite >= &TXBuf[UART_TXBUF_SZ]) PWrite = TXBuf;   // Circulate buffer
    IFullSlotsCount++;
    return retvOk;
}

void BaseUart_t::IStartTransmissionIfNotYet() {
    if(ITxDmaIsIdle) ISendViaDMA();
}


uint8_t BaseUart_t::PutByteNow(uint8_t b) {
    while(!(Params->Uart->ISR & USART_ISR_TXE_TXFNF));
    Params->Uart->TDR = b;
    while(!(Params->Uart->ISR & USART_ISR_TXE_TXFNF));
    return retvOk;
}
#endif // TX

#if 1 // ==== RX ====
uint8_t BaseUart_t::GetByte(uint8_t *b) {
    int32_t WIndx = UART_RXBUF_SZ - DmaRx.GetTransferDataCnt();
    int32_t BytesCnt = WIndx - RIndx;
    if(BytesCnt < 0) BytesCnt += UART_RXBUF_SZ;
    if(BytesCnt == 0) return retvEmpty;
    *b = IRxBuf[RIndx++];
    if(RIndx >= UART_RXBUF_SZ) RIndx = 0;
    return retvOk;
}
#endif // RX

#if 1 // ==== Init ====
void BaseUart_t::Init() {
#if 1 // ==== Tx and Rx pin ====
    AlterFunc_t PinAF = AF1;
    if(Params->PGpioTx == GPIOA) {
        if(Params->Uart == USART1 or Params->Uart == USART2) PinAF = AF1;
        else PinAF = AF4;
    }
    else if(Params->PGpioTx == GPIOB) {
        if(Params->Uart == USART1) PinAF = AF0;
        else PinAF = AF4;
    }
    else if(Params->PGpioTx == GPIOC) {
        if(Params->Uart == USART3) PinAF = AF0;
        else PinAF = AF1;
    }
    else if(Params->PGpioTx == GPIOD) PinAF = AF0;
    PinSetupAlterFunc(Params->PGpioTx, Params->PinTx, omPushPull, pudNone, PinAF);

    if(Params->PGpioRx == GPIOA) {
        if(Params->Uart == USART1 or Params->Uart == USART2) PinAF = AF1;
        else PinAF = AF4;
    }
    else if(Params->PGpioRx == GPIOB) {
        if(Params->Uart == USART1) PinAF = AF0;
        else PinAF = AF4;
    }
    else if(Params->PGpioRx == GPIOC) {
        if(Params->Uart == USART3) PinAF = AF0;
        else PinAF = AF1;
    }
    else if(Params->PGpioRx == GPIOD) PinAF = AF0;
    PinSetupAlterFunc(Params->PGpioRx, Params->PinRx, omPushPull, pudNone, PinAF);
#endif

#if 1 // Clock
    if(Params->Uart == USART1) {
        RCC->CCIPR &= ~RCC_CCIPR_USART1SEL;
        RCC->CCIPR |= ((uint32_t)Params->ClkSrc) << RCC_CCIPR_USART1SEL_Pos;
    }
    else if(Params->Uart == USART2) {
        RCC->CCIPR &= ~RCC_CCIPR_USART2SEL;
        RCC->CCIPR |= ((uint32_t)Params->ClkSrc) << RCC_CCIPR_USART2SEL_Pos;
    }
    if     (Params->Uart == USART1) { Rcc::EnableAPB2Clk(RCC_APBENR2_USART1EN); }
    else if(Params->Uart == USART2) { Rcc::EnableAPB1Clk(RCC_APBENR1_USART2EN); }
    else if(Params->Uart == USART3) { Rcc::EnableAPB1Clk(RCC_APBENR1_USART3EN); }
    else if(Params->Uart == USART4) { Rcc::EnableAPB1Clk(RCC_APBENR1_USART4EN); }
#endif // Clock

    OnClkChange();  // Setup baudrate

    Params->Uart->CR2 = 0;  // Nothing that interesting there
    // ==== DMA TX ====
    DmaTx.Init();
    DmaTx.SetPeriphAddr(&Params->Uart->TDR);
    DmaTx.SetMode(Params->DmaModeTx);
    ITxDmaIsIdle = true;
    // ==== DMA RX ====
    DmaRx.Init();
    DmaRx.SetPeriphAddr(&Params->Uart->RDR);
    DmaRx.SetMemoryAddr(IRxBuf);
    DmaRx.SetTransferDataCnt(UART_RXBUF_SZ);
    DmaRx.SetMode(Params->DmaModeRx);
    DmaRx.Enable();
    // Final setup
    Params->Uart->CR1 = USART_CR1_TE | USART_CR1_RE;        // TX & RX enable
    Params->Uart->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;    // Enable DMA at TX & RX
    // Do not enable UART as it may require additional tweaking later
}

void BaseUart_t::Shutdown() {
    Params->Uart->CR1 &= ~USART_CR1_UE; // UART Disable
//    if     (Params->Uart == USART1) { rccDisableUSART1(); }
//    else if(Params->Uart == USART2) { rccDisableUSART2(); }
}

void BaseUart_t::OnClkChange() {
    switch(Params->ClkSrc) {
        case uartclkPCLK:   Params->Uart->BRR = APBFreqHz / Params->Baudrate; break;
        case uartclkSYSCLK: Params->Uart->BRR = Rcc::GetSysClkHz() / Params->Baudrate; break;
        case uartclkHSI:    Params->Uart->BRR = HSI_FREQ_HZ / Params->Baudrate; break;
        case uartclkLSE:    Params->Uart->BRR = LSE_FREQ_HZ / Params->Baudrate; break;
    } // switch
}
#endif // Init

#endif // Base UART

#if 1 // ========================= Cmd UART ====================================
void CmdUart_t::Init() {
    BaseUart_t::Init();
    // Enable IRQ on \n reception
    uint32_t CharToIrq = UART_CHAR_TO_IRQ;
    uint32_t tmp = Params->Uart->CR2;
    tmp &= 0x00FFFFFFUL; // Clear
    tmp |= CharToIrq << 24;
    Params->Uart->CR2 = tmp;
    Params->Uart->CR1 |= USART_CR1_CMIE; // Enable IRQ
    Params->Uart->ICR = USART_ICR_CMCF;  // Clear IRQ flag1

    if(Params->Uart == USART1) {
        BaseUart1 = this;
        NVIC_SetPriority(USART1_IRQn, IRQ_PRIO_MEDIUM);
        NVIC_EnableIRQ(USART1_IRQn);
    }
    else if(Params->Uart == USART2) {
        BaseUart2 = this;
        NVIC_SetPriority(USART2_IRQn, IRQ_PRIO_MEDIUM);
        NVIC_EnableIRQ(USART2_IRQn);
    }
    else if(Params->Uart == USART3) {
        BaseUart3 = this;
        NVIC_SetPriority(USART3_4_IRQn, IRQ_PRIO_MEDIUM);
        NVIC_EnableIRQ(USART3_4_IRQn);
    }
    else {
        BaseUart4 = this;
        NVIC_SetPriority(USART3_4_IRQn, IRQ_PRIO_MEDIUM);
        NVIC_EnableIRQ(USART3_4_IRQn);
    }
    Enable();
}

uint8_t CmdUart_t::GetRcvdCmd() {
    uint8_t b;
    while(GetByte(&b) == retvOk) {
        if(Cmd.PutChar(b) == pdrNewCmd) return retvOk;
    } // while get byte
    return retvEmpty;
}

void CmdUart_t::IRQUartHandlerI(uint32_t flags) {
    if(flags & USART_ISR_CMF) { // Desired Char found
        EvtQMain.SendNowOrExitI(EvtMsg_t(evtIdShellCmd, (BaseUart_t*)this));
    }
}
#endif


#if BYTE_UART_EN // ========================= Byte UART ========================
static const UartParams_t ByteUartParams = {
        FT_UART,
        FT_GPIO, FT_TX,
        FT_GPIO, FT_RX,
        // DMA
        FT_UART_DMA_TX, FT_UART_DMA_RX,
        UART_DMA_TX_MODE(FT_UART_DMA_CHNL), UART_DMA_RX_MODE(FT_UART_DMA_CHNL),
#if defined STM32F072xB || defined STM32L4XX
        false    // Use independed clock
#endif
};

ByteUart_t ByteUart(&ByteUartParams);
thread_reference_t IByteRxThd = nullptr;

static THD_WORKING_AREA(waByteUartRxThread, 128);
__noreturn
static void ByteUartRxThread(void *arg) {
    chRegSetThreadName("ByteUartRx");
    while(true) {
        chThdSleepMilliseconds(UART_RX_POLLING_MS);
        ByteUart.IRxTask();
    }
}

void ByteUart_t::IRxTask() {
    if(CmdProcessInProgress) return;    // Busy processing cmd
    // Iterate received bytes
//    Printf("1\r");
    uint8_t b;
    while(GetByte(&b) == retvOk) {
        if(Cmd.PutChar(b) == pdrNewCmd) {
            EvtMsg_t Msg(evtIdByteCmd, (ByteShell_t*)this);
            CmdProcessInProgress = (EvtQMain.SendNowOrExit(Msg) == retvOk);
        }
    }
}

void ByteUart_t::Init(uint32_t ABaudrate) {
    BaseUart_t::Init(ABaudrate);
#if UART_RX_ENABLED
    // Create RX Thread if not created
    if(IByteRxThd == nullptr) {
        IByteRxThd = chThdCreateStatic(waByteUartRxThread, sizeof(waByteUartRxThread),
                NORMALPRIO, ByteUartRxThread, NULL);
    }
#endif
}
#endif
