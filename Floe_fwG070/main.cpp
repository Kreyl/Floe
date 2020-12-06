#include "board.h"
#include "kl_libG070.h"
#include "ch.h"
#include "hal.h"
#include "uartG070.h"
#include "LEDs.h"
#include "shell.h"
#include "Settings.h"
#include "Effects.h"
#include "kl_i2cG070.h"
#include "FloeMotion.h"
#include "FloeTypes.h"
#include "MsgQ.h"

#if 1 // ======================== Variables & prototypes =======================
EvtMsgQ_t<EvtMsg_t, MAIN_EVT_Q_LEN> EvtQMain;
void OnCmd(Shell_t *PShell);
static const UartParams_t CmdUartParams(115200, CMD_UART_PARAMS);
CmdUart_t Uart{&CmdUartParams};
void ClockInit();
void ITask();

//Settings_t Settings;
//Adc_t Adc;

enum State_t { stateIdle, stateWave, stateKnock, statePressed };
void SetState(State_t NewState);
TmrKL_t TmrStateEnd { evtIdStateEnd, tktOneShot };

// Button
void BtnIrqHandler() {
    chSysLockFromISR();
    EvtQMain.SendNowOrExitI(EvtMsg_t(evtIdPress));
    chSysUnlockFromISR();
}
static const PinIrq_t ButtonPin{BTN_PIN, pudPullDown, BtnIrqHandler};
#endif

int main(void) {
    ClockInit();
    // Start Watchdog. Will reset in main thread by periodic 1 sec events.
//    Iwdg::InitAndStart(4500);
//    Iwdg::DisableInDebug();

    // === Init OS ===
    halInit();
    chSysInit();
    EvtQMain.Init();

    // ==== Init hardware ====
    PinSetupOut(DBG_PIN, omPushPull);
    Uart.Init();
    Printf("\r%S %S\r", APP_NAME, XSTRINGIFY(BUILD_TIME));

    Leds::Init();
    i2c2.Init();
//    i2c2.ScanBus();
    FloeMotionInit();

    Effects::Init();
    Effects::Set(EffIdle);

    ButtonPin.Init(risefallRising);
    ButtonPin.EnableIrq(IRQ_PRIO_MEDIUM);

//    Settings.Load();

    // ADC for temperature measurement. Will trigger periodically by TIM6. IRQ-driven.
//    Adc.Init();
//    Adc.SetupOversampling(Adc_t::ratio4x, 2); // *4, /4
//    Adc.EnableChnl(ADC_CHNL_TSNS); // temperature
//    Adc.EnableChnl(ADC_CHNL_VREF); // VRef
//    Adc.EnableConvDoneIRQ();
//    Adc.SetupConvOnTimer(1); // 1 Hz
//    Adc.Start();

    // Main cycle
    ITask();
}

__attribute__((__noreturn__))
void ITask() {
    while(true) {
        EvtMsg_t Msg = EvtQMain.Fetch(TIME_INFINITE);
        switch(Msg.ID) {
            case evtIdPress:
                Printf("Btn\r");
                SetState(statePressed);
                break;

            case evtIdStateEnd:
                SetState(stateIdle);
                break;

            case evtIdShellCmd:
                while(((CmdUart_t*)Msg.Ptr)->GetRcvdCmd() == retvOk) OnCmd((Shell_t*)((CmdUart_t*)Msg.Ptr));
                break;

            default: Printf("Unhandled Msg %u\r", Msg.ID); break;
        } // Switch
    } // while true
}

void SetState(State_t NewState) {
    switch(NewState) {
        case stateIdle:
            Effects::Set(EffIdle);
            break;
        case stateWave:
            Effects::Set(EffWave);
            TmrStateEnd.StartOrRestart(TIME_MS2I(DURATION_OF_WAVE));
            break;
        case stateKnock:
            Effects::Set(EffKnock);
            TmrStateEnd.StartOrRestart(TIME_MS2I(DURATION_OF_KNOCK));
            break;
        case statePressed:
            Effects::Set(EffOnPress);
            TmrStateEnd.StartOrRestart(TIME_MS2I(DURATION_OF_PRESS));
            break;
    } // switch
}

#if 1 // ================= Command processing ====================
void OnCmd(Shell_t *PShell) {
    Cmd_t *PCmd = &PShell->Cmd;
    __attribute__((unused)) int32_t dw32 = 0;  // May be unused in some configurations
//    Printf("%S%S\r", PCmd->IString, PCmd->Remainer? PCmd->Remainer : " empty");
    // Handle command
    if(PCmd->NameIs("Ping")) PShell->Ok();
    else if(PCmd->NameIs("Version")) PShell->Print("%S %S\r", APP_NAME, XSTRINGIFY(BUILD_TIME));

    else if(PCmd->NameIs("Idle"))  Effects::Set(EffIdle);
    else if(PCmd->NameIs("Wave"))  Effects::Set(EffWave);
    else if(PCmd->NameIs("Knock")) Effects::Set(EffKnock);
    else if(PCmd->NameIs("Press")) Effects::Set(EffOnPress);

    else PShell->CmdUnknown();
}
#endif

void ClockInit() {
    uint32_t tmpreg;
    // Reset everything
    RCC->AHBRSTR = 0xFFFFFFFF;
    RCC->AHBRSTR = 0;
    (void)RCC->AHBRSTR;
    RCC->APBRSTR1 = 0xFFFFFFFF;
    RCC->APBRSTR1 = 0;
    (void)RCC->APBRSTR1;
    RCC->APBRSTR2 = 0xFFFFFFFF;
    RCC->APBRSTR2 = 0;
    (void)RCC->APBRSTR2;

    // Enable syscfg and pwr
    Rcc::EnableAPB2Clk(RCC_APBENR2_SYSCFGEN);
    Rcc::EnableAPB1Clk(RCC_APBENR1_PWREN);
    // Set Flash latency
    tmpreg = FLASH->ACR;
    tmpreg &= ~FLASH_ACR_LATENCY;
    tmpreg |= 0b010UL; // Flash latency: two wait states
    tmpreg |= FLASH_ACR_ICEN | FLASH_ACR_PRFTEN; // Enable instruction cache and prefetch
    FLASH->ACR = tmpreg;

    // === PLL ===
    // PLL dividers and src: R = /3, P = /2, N = 12, M = /1, PLLSRC = HSI = 16MHz
    RCC->PLLCFGR = (0b010UL << 29) | (0b00001UL << 17) | (12UL << 8) | (0b000UL << 4) | (0b10UL << 0);
    RCC->CR |= RCC_CR_PLLON; // Enable PLL
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN; // Enable R output
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLPEN; // Enable P output

    // I2S1 clock src is PLL P output
    RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_I2S1SEL) | (0b01UL << 14);

    // Wait until PLL started
    while(!(RCC->CR & RCC_CR_PLLRDY));
    // Switch to PLL
    tmpreg = RCC->CFGR & ~RCC_CFGR_SW;
    tmpreg |= 0b010UL; // PLLR is sys clk
    RCC->CFGR = tmpreg;
    // Wait until done
    while((RCC->CFGR & RCC_CFGR_SWS) != (0b010UL << 3));
    // Enable DMA clock
    Rcc::EnableAHBClk(RCC_AHBENR_DMA1EN);
    // ADC clock is SYSCLK: ADCSEL = 0
    RCC->CCIPR &= ~RCC_CCIPR_ADCSEL;
    AHBFreqHz = 64000000;
    APBFreqHz = 64000000;
}
