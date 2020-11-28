#include "board.h"
#include "kl_libG070.h"
#include "uartG070.h"
#include "LEDs.h"
#include "shell.h"
#include "Settings.h"
#include "Effects.h"

#if 1 // ======================== Variables & prototypes =======================
static const UartParams_t CmdUartParams(115200, CMD_UART_PARAMS);
CmdUart_t Uart{&CmdUartParams};
void ClockInit();
void ITask();

Settings_t Settings;
Adc_t Adc;

// Animation
Time_t Time{TIME_TIMER};
const uint8_t *AniPtr;
uint32_t TimePicStart;
uint32_t ShowDuration = 0;
//void DoAnimation();

uint32_t Start = 0;

Color_t Pic[7] = { clBlack };
#endif

int main(void) {
    __disable_irq();
    ClockInit();
    __enable_irq();
    // Start Watchdog. Will reset in main thread by periodic 1 sec events.
//    Iwdg::InitAndStart(4500);
//    Iwdg::DisableInDebug();

    // ==== Init hardware ====
    PinSetupOut(DBG_PIN, omPushPull);
    Uart.Init();
    Printf("\r%S %S\r", APP_NAME, XSTRINGIFY(BUILD_TIME));

    Time.Init();
    Leds::Init();
    Effects::Init();

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
//        Iwdg::Reload();
        Effects::Task();

//        if(Time.ElapsedSince(Start) > 306) {
//            Start = Time.GetCurrent();
////            Printf("Aga\r");
//
//            Pic[N] = clBlack;
//            N++;
//            if(N>6) N=0;
//            Pic[N] = clGreen;
//            Leds::ShowPic(Pic);
//        }


//        switch(Settings.Mode) {
//            case modeIdle:
//                if(IgnoreShowDelay or Time.ElapsedSince(TimePicStart) >= ShowDuration) {
//                    __disable_irq();
//                    SpiCmd_t *PCmd = Framebuf.GetAndLock();
//                    __enable_irq();
//                    if(PCmd) {
//                        // Get duration: how long to show it
//                        uint32_t MetaDataSz = (SpiReply.ProtocolVersion == VERSION_PROTOCOL_WITH_CRC) ? (SHOW_DURATION_SZ + CRC_SZ) : SHOW_DURATION_SZ;
//                        if(PCmd->DataSz > MetaDataSz) {
//                            PCmd->DataSz -= MetaDataSz;
//                            ShowDuration = Bytes2Word16(PCmd->Cmd.FrameData[PCmd->DataSz], PCmd->Cmd.FrameData[PCmd->DataSz+1]);
//                        }
//                        else ShowDuration = 0; // No such data
//                        // Show
//                        Leds::PutPicToBuf(PCmd->Cmd.FrameData, PCmd->DataSz); // DataSz may be bigger than picSz (due to Show Duration), it is ok.
//                        Leds::ShowBuf();
//                        // Setup delay
//                        IgnoreShowDelay = false;
//                        TimePicStart = Time.GetCurrent();
//                        // Unlock it
//                        __disable_irq();
//                        Framebuf.UnlockAndFree();
//                        __enable_irq();
//                    }
//                    else IgnoreShowDelay = true; // No pic to show now. Show next at once.
//                }
//                break;
//
//            case modeAnimation: DoAnimation();                break;
//            case modeGradient:  TestPatterns.ShowGradient();  break;
//            case modeClock:     TestPatterns.ShowClock();     break;
//            case modeSwitchBrt: TestPatterns.ShowSwitchBrt(); break;
//            case modeSteadyTest:
//                Leds::ShowBuf();
//                Settings.Mode = modeIdle;
//                break;
//        } // switch
    } // while true
}

//void DoAnimation() {
//    if(Time.ElapsedSince(TimePicStart) < ANI_DURATION_ms) return;
//    TimePicStart = Time.GetCurrent();
//    // Get frame
//    AniFrame_t *pFrame = (AniFrame_t*)AniPtr;
//    uint32_t Sz = pFrame->xsz * pFrame->ysz;
//    AniPtr += 4+Sz;
//    if(((uint32_t)AniPtr - (uint32_t)AniFile) >= ANI_SZ) AniPtr = AniFile;
//    // Prepare pic
//    uint8_t Brt;
//    for(int32_t y=0; y<ROW_CNT; y++) {
//        for(int32_t x=0; x<COL_CNT; x++) {
//            // Get pixel from frame
//            int32_t xi = x - (int32_t)pFrame->x0;
//            int32_t yi = y - (int32_t)pFrame->y0;
//            if(xi < 0 or yi < 0 or xi >= (int32_t)pFrame->xsz or yi >= (int32_t)pFrame->ysz) {
//                Brt = 0;
//            }
//            else {
//                Brt = pFrame->Data[xi + yi * pFrame->xsz];
//            }
//            Leds::PutPixelToBuf(x, y, Brt);
//        }
//    }
//    Leds::ShowBuf();
//}

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
    EnableAPB2Clk(RCC_APBENR2_SYSCFGEN);
    EnableAPB1Clk(RCC_APBENR1_PWREN);
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
    EnableAHBClk(RCC_AHBENR_DMA1EN);
    // ADC clock is SYSCLK: ADCSEL = 0
    RCC->CCIPR &= ~RCC_CCIPR_ADCSEL;
    AHBFreqHz = 64000000;
    APBFreqHz = 64000000;
}
