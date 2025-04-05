from machine import Pin,mem32,Timer, SPI, ADC
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

def spi_write(w):
    """Use of SPI with enable of sync and ldac """
    w = w.to_bytes(3, 'big')
    sync.value(0)
    ldac.value(1)
    
    # Write data to the connected device
    spi.write(w)
    sync.value(1)
    ldac.value(0)
    sleep(0.0000001) # 100 ns
    ldac.value(1)


def DAC(Vc1, Vc2):
    """Send Datas to DAC"""
    # Vc1 = G(Vout_A_DAC - Vref_out ), with G = 1, Vc1 = 0V then Vout_A_DAC = 2.5 V then D1 = 2**15
    #D1 = floor((Vc1/(G*Vref_out)+1)*(2**16/Gain))
    #D2 = floor((Vc2/(G*Vref_out)+1)*(2**16/Gain))
    D1 = floor((Vc1/Vref_out)*((2**16 - 1)/Gain))
    D2 = D1

    # Data to send to DAC
    DATA_A = (CMD_SETA_UPDATEA << 16)|D1 # Ex: (0x18 << 16)|0x8000 = 0x188000
    DATA_B = (CMD_SETB_UPDATEB << 16)|D2

    spi_write(DATA_A)
    # print ("Data_A sent!")

    spi_write(DATA_B)
    # print ("Data_B sent!")

    # print("end")


def Check_V(Vc):
    if Vc > V_max:
        Vc = V_max
    elif Vc < V_min:
        Vc = V_min
    
    return Vc


########################################################################################################
print ("start")

# DAC
G        = 1   # G*R = 680 kOhm, R = 680 kOhm. The condition is G >= 0.8
Gain     = 2   # following Datasheet
Vref_out = 2.5 # 2.5 V

CMD_SETA_UPDATEA = 0x18       # Write to DAC-A input register and update DAC-A
CMD_SETB_UPDATEB = 0x19       # Write to DAC-B input register and update DAC-B
RESET_ALL_REG    = 0x280001   # Reset all registers and update all DACs (Power-on-reset update)
INTERNAL_REF_EN  = 0x380001   # Enable Internal Reference & reset DACs to gain = 2

# ADC
port_value = ADC(Pin(27)) # GP28

# Initialize SPI
spi = SPI(0,                  # peripheral 0
          baudrate = 100000,  # Frequency f = 100 kHz
          polarity = 0,
          phase = 1,          # Falling Edge Clcok
          firstbit = SPI.MSB,
          sck  = Pin(2),      # CLK pin
          mosi = Pin(3)       # Data pin
          )
sync = Pin(4, Pin.OUT, value = 1) # sync pin
ldac = Pin(5, Pin.OUT, value = 1) # ldac pin
clr = Pin(6, Pin.OUT, value = 1)
print ("SPI, sync, ldac initialized!")

# Initialize DAC
spi_write(RESET_ALL_REG)   # soft reset
# print ("DAC resetted!")

spi_write(INTERNAL_REF_EN) # int ref enabled and gain = 2
# print ("Vref_out on, Gain = 2")

# Initialize PI Controller
Kp_loop  = 1            # The loop starts to oscillate
#Kp       = 0.45*Kp_loop # 13.5
Kp = 15#300
T_loop   = 5.5E-03      # [s], period of the loop
Ti       = 0.8*T_loop   # 4.4E-03 s
# Ti = Kp/Ki -> Ki = Kp/Ti = 3068.1818 -> circa Ki = 3068
Ki       =  10          # Best solution for overshoot and for stability
dt       = 4.4E-03      # [s], Ti
P,I,B    = 0,0,0
e = 0

V_sp     = 0.7#1.09         # [V], set point
# V_sp     = 1.137      # [V], set point
# V_sp     = 1.20       # [V], set point    
V_min    = -2           # [V], minimum attenuation
V_max    = 0            # [V], maximum attenuation
V_mid    = 1.137        # [V], middle point
Vc1,Vc2  = -2,-2        # [V]

deadband = 1E-05        # [V], to avoid unnecessary adjustments

# print(Vc1, ",")
# print(Vc2, ",")
#DAC(Vc1, Vc2)

reset = True
V_read = 0

def timer_callback(timer):
    print(V_read)

#tim = Timer()
#tim.init(mode=Timer.PERIODIC, period=100, callback=timer_callback)

while True:
    for i in range(0, 2**16 - 1, 100):
        DAC_out =  i * (5.0/2**16)
        print(f'Testing {DAC_out}')
        DAC(DAC_out, 0)
        sleep(0.1)
    
while True:
    V_read = port_value.read_u16()*3.3/65535
    
    # PI
    e = V_sp - V_read # Error
    
    if True:
        if abs(e) > deadband:
            P = Kp * e
            I = I + Ki * e * dt
            V = P + I
            
            DAC(V, -2)
            sleep(dt)
            
    
    elif abs(e) > deadband:
        P = Kp*e          # Proportional
        I = I + Ki*e*dt   # Integral
        V = P + I
        
        if True:
            pass
        elif V_read <= V_mid:
            
            if not(reset):
                reset = True
                I = 0
                V = -abs(V)
            
            Vc1 = V
            Vc2 = V_min
            Vc1 = Check_V(Vc1)
        
        else:
                        
            if reset:
                reset = False
                I = 0
                V = -2 + abs(V)
            
            Vc1 = V_max
            Vc2 = V
            Vc2 = Check_V(Vc2)
      
        DAC(Vc1, Vc2)
#         print(V_read, ",")
#         print(e, ",")
#         print(V, ",")
#         print(Vc1, ",")
#         print(Vc2, ",")
