
from machine import Pin,mem32,Timer
from machine import *
from rp2 import PIO, StateMachine, asm_pio
from array import array
from utime import sleep,sleep_us
from math import pi,sin,exp,sqrt,floor
from uctypes import *
from random import random
import sys
import uselect
from time import *


@asm_pio(set_init=PIO.OUT_LOW)
def clk_10MHz():
    set(pins,1)
    set(pins,0)
    

@asm_pio(out_init=PIO.OUT_LOW, sideset_init=PIO.OUT_LOW, out_shiftdir=PIO.SHIFT_LEFT, autopull=True, pull_thresh=32)
def data_out():
    wrap_target()
    out(pins, 1).side(0)
    mov(y, y).side(1) 
    wrap()


# LE pin manually controlled
le = Pin(10, Pin.OUT)
le.value(1)

# MOUXOUT pin 
Mux_out = Pin(9,Pin.IN)
print(Mux_out.value())

###INITIALIZING STATE MACHINES
#sm2 for data 
#sm3 for REFin 10 MHz
sm2 = StateMachine(1, data_out, freq=20000, out_base=Pin(11), sideset_base=Pin(12))
sm3 = StateMachine(2, clk_10MHz, freq=50000000, set_base=Pin(3))

#value for registers in ADF5355
word=[0x0001041C, 0x61300B, 0xC03EBA, 0x2A29FCC9, 0x102D0428, 0x12000067, 0x35036036, 0x00800025, 0x30008984, 0x00000003, 0x7AE3FFF2, 0x7AE141, 0x200240]

voltage=[]

conversion_factor=3.3/(65535)
adc_pin=ADC(28)

##ACTIVATION OF STATE MACHINES 
sm3.active(1)
sleep(0.1) #192.5 ms sleep(0.1)
sm2.active(1)

def parser_stdin(buffer):
    global sm2, word
    if (buffer[0]=='start'):
        return 1
    elif (buffer[0] == 'ack'):
        return 1
    else:
        return 0
        
print ("ciao")

i=0

#sleep(0.001) equivale a 1ms sleep(0.05) equivale a 1.75 ms
def spi_send_word(w):
    #le.value(1)
    #sleep(0.004)
    le.value(0)
    sleep(0.0000000001)
    sm2.put(w,0)
    sleep(0.004)
    le.value(1)
 

for i in range(13):
    spi_send_word(word[i])
    sleep(0.0002)
    #print(j)
    #j=j+1
sleep(0.002)
le.value(0)
#sleep(0.0000000001)
#le.value(1)

#LED
led = Pin(25, Pin.OUT)
while True:
    led.on()
    sleep(1)
    led.off()
    sleep(1)


#sm2.put(word[1],0)
                #sleep(0.001)
#sm2.put(word[2],0)
#sm2.put(word[3],0)
#sm2.put(word[4],0)
#sm2.put(word[5],0)
#sm2.put(word[6],0)
#sm2.put(word[7],0)
#sm2.put(word[8],0)
#sm2.put(word[9],0)
#sm2.put(word[10],0)
#sm2.put(word[11],0)                
#sleep(0.1)
#sm2.put(word[12],0)
                #sleep(0.001)
                #sm2.put(word[1],8)
                #sleep(0.001)
                #sm2.put(AB_reg[i],8)
    
                #CE.value(1)
                #print("scritto")
            #i = i+1
               # while (Mux_out.value() != 1):
                    #print (Mux_out.value())
                    #pass
               # samples = 50
                #tension = 0
                #for j in range(samples):
                  #  tension = adc_pin.read_u16()*conversion_factor + tension
                #tension=round(tension/samples,3)
                #stringazio = stringazio + str(freq[i]) + " " + str(tension)+'\n'
            #stringazio="{:<1}".format(stringazio)
            #command="{:<1}".format(str(len(stringazio)+140)+'\n')
            #sys.stdout.write (command.encode())
            #while (parser_stdin(readbuffer_from_USB())):
              #  pass
            #command=stringazio
            #sys.stdout.write (command.encode())

    
 

   
'''
if (len(voltage) == len(freq)):
    for i in range(len(voltage)):
        stringazio = str(freq[i]) + " " + str(voltage[i])
        print (stringazio)
        #command="{:<200}".format(stringazio)
        #sys.stdout.write (command.encode('UTF8'))
        #sleep(0.2)
        
else:
    print("length doesn't match")
    print (len(voltage))
'''

