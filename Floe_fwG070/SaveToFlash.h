/*
 * SaveToFlash.h
 *
 *  Created on: 5 ����. 2017 �.
 *      Author: Kreyl
 */

#ifndef SAVETOFLASH_H__
#define SAVETOFLASH_H__

#include "inttypes.h"
#include "board.h"

// Time of saving: 32ms @ 8MHz

namespace Flash {

void Read(uint32_t Addr, void *ptr, uint32_t ByteSz);
uint8_t Write(uint32_t Addr, void *ptr, uint32_t ByteSz);

uint8_t WriteUnaligned(uint32_t Addr, void *ptr, uint32_t Sz);

}

#endif //SAVETOFLASH_H__