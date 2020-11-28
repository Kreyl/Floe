/*
 * SaveToFlash.cpp
 *
 *  Created on: 5 ����. 2017 �.
 *      Author: Kreyl
 */

#include "SaveToFlash.h"
#include "kl_libG070.h"
#include "uartG070.h"
#include "shell.h"
#include "board.h"

#define WORD64_CNT     (FLASH_PAGE_SZ/8)

namespace Flash {
static uint64_t IBuf[WORD64_CNT];

void Read(uint32_t Addr, void *ptr, uint32_t ByteSz) {
    uint8_t *p8 = (uint8_t*)ptr;
    while(ByteSz--) {
        *p8++ = *(uint8_t*)Addr;
        Addr++;
    }
}

// Data must be aligned to uint64_t
uint8_t Write(uint32_t Addr, void *ptr, uint32_t ByteSz) {
    // Disable flash and instruction cache
    uint32_t OldFlashReg = FLASH->ACR;
    FLASH->ACR &= ~(FLASH_ACR_ICEN | FLASH_ACR_PRFTEN);
    // Unlock Flash
    Flash::LockFlash(); // Otherwise HardFault occurs after flashing and without reset
    Flash::UnlockFlash();
    // Prepare variables
    uint8_t Rslt = retvOk;
    uint64_t *Buf = (uint64_t*)ptr;
    uint32_t DWordCnt = (ByteSz + 7) / 8;

    // Erase all
    uint32_t PageN = (Addr - 0x08000000) / FLASH_PAGE_SZ;
    uint32_t PageCnt = (ByteSz + FLASH_PAGE_SZ - 1) / FLASH_PAGE_SZ;
    while(PageCnt--) {
        if(Flash::ErasePage(PageN) != retvOk) {
            Printf("\rPage %u Erase fail\r", PageN);
            Rslt = retvFail;
            goto End;
        }
        PageN++;
    }

    // Write data
    while(DWordCnt--) {
        if(Flash::ProgramDWord(Addr, *Buf) != retvOk) {
            Printf("Write %X Fail\r", Addr);
            Rslt = retvFail;
            goto End;
        }
        Addr += 8;
        Buf++;
    }
    End:
    Flash::LockFlash();
    // Reset instruction cache and Restore Flash settings
    FLASH->ACR |= FLASH_ACR_ICRST;
    FLASH->ACR &= ~FLASH_ACR_ICRST;
    FLASH->ACR = OldFlashReg;
    return Rslt;
}

uint8_t WritePage(uint32_t Addr, uint64_t *ptr, uint32_t ByteSz) {
    uint32_t PageN = (Addr - 0x08000000) / FLASH_PAGE_SZ;
    if(Flash::ErasePage(PageN) != retvOk) {
        Printf("\rPage %u Erase fail\r", PageN);
        return retvFail;
    }
    // Write data
    uint32_t DWordCnt = (ByteSz + 7) / 8;
    while(DWordCnt--) {
        if(Flash::ProgramDWord(Addr, *ptr) != retvOk) {
            Printf("Write %X Fail\r", Addr);
            return retvFail;
        }
        Addr += 8;
        ptr++;
    }
    return retvOk;
}


uint8_t WriteUnaligned(uint32_t Addr, void *ptr, uint32_t Sz) {
    // Disable flash and instruction cache
    uint32_t OldFlashReg = FLASH->ACR;
    FLASH->ACR &= ~(FLASH_ACR_ICEN | FLASH_ACR_PRFTEN);
    // Unlock Flash
    Flash::LockFlash(); // Otherwise HardFault occurs after flashing and without reset
    Flash::UnlockFlash();
    uint8_t Rslt = retvOk;
    uint8_t *p8 = (uint8_t*)ptr;
    while(Sz) {
        uint32_t AlignedAddr = (Addr / FLASH_PAGE_SZ) * FLASH_PAGE_SZ;
        if(Addr != AlignedAddr) { // unaligned
            uint32_t SzOccupied = Addr - AlignedAddr;
            uint32_t SzFree = FLASH_PAGE_SZ - SzOccupied;
            // Copy old data from flash
            uint8_t *Dst = (uint8_t*)IBuf;
            uint8_t *Src = (uint8_t*)AlignedAddr;
            for(uint32_t i=0; i<SzOccupied; i++) *Dst++ = *Src++;
            // Copy new data
            uint32_t SzToUse = (Sz > SzFree)? SzFree : Sz;
            for(uint32_t i=0; i<SzToUse; i++) *Dst++ = *p8++;
            // Write buf to flash
            uint32_t SzToWrite = SzOccupied + SzToUse;
//            Printf("A: %X; Sz: %u; AA: %X; O: %u; F: %u; 2W: %u\r", Addr, Sz, AlignedAddr, SzOccupied, SzFree, SzToWrite);
            DelayLoop(99000);
            if(WritePage(AlignedAddr, IBuf, SzToWrite) == retvOk) {
                Sz -= SzToUse;
                Addr += SzToUse;
            }
            else {
                Rslt = retvFail;
                break;
            }
        } // if aligned
        else {
            uint32_t SzToWrite = (Sz > FLASH_PAGE_SZ)? FLASH_PAGE_SZ : Sz;
            // Copy data to IBuf to be aligned
            uint8_t *Dst = (uint8_t*)IBuf;
            for(uint32_t i=0; i<SzToWrite; i++) *Dst++ = *p8++;
//            Printf("A: %X; Sz: %u; 2W: %u\r", Addr, Sz, SzToWrite);
            DelayLoop(99000);
            if(WritePage(Addr, IBuf, SzToWrite) == retvOk) {
                Sz -= SzToWrite;
                Addr += SzToWrite;
            }
            else {
                Rslt = retvFail;
                break;
            }
        }
    } // while Sz

    Flash::LockFlash();
    // Reset instruction cache and Restore Flash settings
    FLASH->ACR |= FLASH_ACR_ICRST;
    FLASH->ACR &= ~FLASH_ACR_ICRST;
    FLASH->ACR = OldFlashReg;
    return Rslt;
}

} // namespace
