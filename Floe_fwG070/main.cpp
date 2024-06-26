#include "board.h"
#include "kl_libG070.h"
#include "ch.h"
#include "hal.h"
#include "uartG070.h"
#include "LEDs.h"
#include "shell.h"
#include "Effects.h"
#include "kl_i2cG070.h"
#include "FloeMotion.h"
#include "FloeTypes.h"
#include "MsgQ.h"

#if 1 // ======================== Variables & prototypes =======================
EvtMsgQ_t<EvtMsg_t, MAIN_EVT_Q_LEN> EvtQMain;
void OnCmd(Shell_t *PShell);
static const UartParams_t CmdUartParams(256000, CMD_UART_PARAMS);
CmdUart_t Uart{&CmdUartParams};
void ClockInit();
void ITask();

//Adc_t Adc;

uint32_t TypeID = 1;

// State
enum State_t { stateIdle, stateWave, stateKnock, statePressed };
void SetState(State_t NewState);
TmrKL_t TmrStateEnd { evtIdStateEnd, tktOneShot };
systime_t KnockStartTime = 0;
uint32_t KnockCounter = 0;
systime_t WaveStartTime = 0;
uint32_t WaveCounter = 0;
#define MOTION_EVT_DEAD_TIME_MS     720
#define KNOCK_CNT                   3
#define WAVE_CNT                    4
#define MOTION_EVT_PERIOD_MAX_MS    2007

// Button
TmrKL_t TmrBtn { TIME_MS2I(360), evtIdPress, tktOneShot };
void BtnIrqHandler(RiseFall_t RiseFall) {
    chSysLockFromISR();
    TmrBtn.StartOrRestartI();
    chSysUnlockFromISR();
}
static const PinIrq_t ButtonPin{BTN_PIN, pudPullDown, BtnIrqHandler};
#endif

void main(void) {
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
    Printf("\r%S %S; ID=%u, Type: %S\r", APP_NAME, XSTRINGIFY(BUILD_TIME), TypeID, FloeTypes[TypeID].Description);

    chThdSleepMilliseconds(9); // Let it rise
    Leds::Init();
    i2c2.Init();
//    i2c2.ScanBus();

    Effects::Init();
    if(FloeMotionInit() == retvOk) Effects::Set(EffPwrOn);
    else Effects::Set(EffBad);
    TmrStateEnd.StartOrRestart(TIME_S2I(4));

    ButtonPin.Init(risefallRising);
    ButtonPin.EnableIrq(IRQ_PRIO_MEDIUM);

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
                // Check if Btn still pressed
                if(ButtonPin.IsHi()) SetState(statePressed);
                break;

            case evtIdKnock:
                if(!ButtonPin.IsHi() and chVTTimeElapsedSinceX(WaveStartTime) > MOTION_EVT_DEAD_TIME_MS) {
                    Effects::Blink(hsvWhite);
                    // Count them
                    if(chVTTimeElapsedSinceX(KnockStartTime) > MOTION_EVT_PERIOD_MAX_MS) {
                        KnockCounter = 1;
                    }
                    else KnockCounter++;
                    Printf("Knock %u\r", KnockCounter);
                    KnockStartTime = chVTGetSystemTimeX();
                    if(KnockCounter >= KNOCK_CNT) SetState(stateKnock);
                }
                break;
            case evtIdWave0:
            case evtIdWave1:
                if(!ButtonPin.IsHi() and chVTTimeElapsedSinceX(WaveStartTime) > MOTION_EVT_DEAD_TIME_MS) {
                    Effects::Blink(hsvBlack);
                    // Count them
                    if(chVTTimeElapsedSinceX(WaveStartTime) > MOTION_EVT_PERIOD_MAX_MS) {
                        WaveCounter = 1;
                    }
                    else WaveCounter++;
                    Printf("Wave %u\r", WaveCounter);
                    WaveStartTime = chVTGetSystemTimeX();
                    if(WaveCounter >= WAVE_CNT) SetState(stateWave);
                }
                break;

            case evtIdStateEnd: SetState(stateIdle);  break;

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
            Printf("Set Idle\r");
            Effects::Set(EffIdle);
            break;
        case stateWave:
            Printf("Set Wave\r");
            Effects::Set(FloeTypes[TypeID].Wave);
            TmrStateEnd.StartOrRestart(TIME_S2I(DURATION_OF_WAVE_S));
            break;
        case stateKnock:
            Printf("Set Knock\r");
            Effects::Set(FloeTypes[TypeID].Knock);
            TmrStateEnd.StartOrRestart(TIME_S2I(DURATION_OF_KNOCK_S));
            break;
        case statePressed:
            Printf("Set Pressed\r");
            Effects::Set(FloeTypes[TypeID].Press);
            TmrStateEnd.StartOrRestart(TIME_S2I(DURATION_OF_PRESS_S));
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
    else if(PCmd->NameIs("Wave"))  Effects::Set(FloeTypes[TypeID].Wave);
    else if(PCmd->NameIs("Knock")) Effects::Set(FloeTypes[TypeID].Knock);
    else if(PCmd->NameIs("Press")) Effects::Set(FloeTypes[TypeID].Press);

//    else if(PCmd->NameIs("Set")) {
//        uint32_t NewType;
//        if(PCmd->GetNext<uint32_t>(&NewType) == retvOk) {
//            if(NewType < TYPE_CNT) {
//                TypeID = NewType;
//                Settings.Save();
//            }
//            else PShell->BadParam();
//        }
//        else PShell->CmdError();
//    }

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
