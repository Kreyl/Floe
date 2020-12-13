from enum import Enum
import serial
from typing import List
import serial.tools.list_ports


DELTA_IDLE = 100
KNOCK_LEVEL = 800
KNOCK_LENGTH = 20
DELTA_KNOCK = 200
MAX_KNOCK = 300
MIN_KNOCK = 50
MIN_WAVE_TIME = 50
MAX_WAVE_TIME = 500
MIN_WAVE_PAUSE = 10
MAX_WAVE_PAUSE = 500
WAVE_NUM = 4


class State (Enum):
    MOVING = 0
    WAIT_FOR_IDLE = 1
    IDLE = 2
    WAIT_FOR_KNOCK_END = 3


def diff_len(data1: List[int], data2: List[int]) -> int:

    return round(sum([(data1[i]-data2[i])**2 for i in range(len(data1))])**0.5)


def main():
    state: State = State.MOVING

    comports: List = [comport.device for comport in serial.tools.list_ports.comports()]
    comport = comports[-1]
    ser = serial.Serial(comport, timeout=0.1, baudrate=115200)

    data_prev = [1, 1, 4096]
    count = 0
    knock_count = 0
    last_knock = 0
    knock_number = 0
    last_wave = 0
    wave_number = 0
    a1 = 1
    a2 = 1
    a1_count = 0
    a2_count = 0

    while(True):
        # обработка входа
        answer = ser.readline()
        try:
            answer_decoded = answer.decode('ascii')
        except UnicodeDecodeError:
            answer_decoded = ""
            for byte in answer:
                try:
                    if byte < 128:
                        char = chr(byte)
                    else:
                        char = ''
                except ValueError:
                    char = ''
                answer_decoded += char
        try:
            data_new: List[int] = [int(a) for a in answer_decoded.split()]
        except ValueError:
            continue
        if len(data_new) != 3:
            print("Ktulhu!")
            continue
        # print(diff_len(data_new, data_prev))

        # начало алгоритма
        if knock_number != 0:
            last_knock += 1

        if wave_number != 0:
            last_wave += 1

        # обработка волн
        if data_prev[0]*data_new[0] < 0 :
            if MIN_WAVE_TIME < a1_count < MAX_WAVE_TIME:
                print("Waving!")
                print(a1_count)
                if MIN_WAVE_PAUSE < last_wave < MAX_WAVE_PAUSE:
                    wave_number += 1
                else:
                    wave_number = 1
                last_wave = 0
            a1 = (-1)*a1
            a1_count = 1

        else:
            #print(data_new[0])
            a1_count += 1

        if data_prev[1]*data_new[1] < 0:
            if MIN_WAVE_TIME < a2_count < MAX_WAVE_TIME:
                print("Waving!")
                print(a2_count)
                print(last_wave)
                if MIN_WAVE_PAUSE < last_wave < MAX_WAVE_PAUSE:
                    wave_number += 1
                else:
                    wave_number = 1
                last_wave = 0
            a2 = (-1)*a2
            a2_count = 1
        else:
            #print(a2_count)
            a2_count += 1
        if wave_number == WAVE_NUM:
            print("WAVE!!!")
            wave_number = 0

        # обработка стуков
        if state == State.MOVING:
            if diff_len(data_new, data_prev) < DELTA_IDLE:
                state = State.WAIT_FOR_IDLE
                # print("Entering waiting idle")
                count = 0
                continue

        if state == State.WAIT_FOR_IDLE:
            if diff_len(data_new, data_prev) < DELTA_IDLE:
                count += 1
                if count >= 10:
                    count = 0
                    state = State.IDLE
                    # print("Entering IDLE")
                    continue
            else:
                count = 0
                state = State.MOVING
                #print("Entering MOVING")
                continue

        if state == State.IDLE:
            if diff_len(data_new, data_prev) > KNOCK_LEVEL:
                #print("Entering wait for knock end")
                state = State.WAIT_FOR_KNOCK_END
                knock_count = 0
                continue
            if diff_len(data_new, data_prev) > DELTA_IDLE:
                #print("Entering moving")
                state = State.MOVING
                continue

        if state == State.WAIT_FOR_KNOCK_END:
            #print(diff_len(data_new, data_prev))
            knock_count += 1
            if knock_count > KNOCK_LENGTH:
                if diff_len(data_new, data_prev) < DELTA_KNOCK:
                    print("Knock knock knock")
                    knock_count = 0
                    if knock_number != 0:
                        # print(last_knock)
                        if MIN_KNOCK < last_knock < MAX_KNOCK:
                            knock_number += 1
                            if knock_number == 3:
                                knock_number = 0
                                print("KNOCK!!!!")
                        else:
                            knock_number = 1
                    else:
                        knock_number = 1
                    last_knock = 0
                    state = State.IDLE
                    continue
                else:
                    # print("Entering moving")
                    state = State.MOVING
                    continue
        data_prev = data_new


if __name__ == '__main__':
    main()
