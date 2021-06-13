import machine, time
from machine import Pin, Timer
from time import sleep_ms
uart = machine.UART(1, 9600)                         # init with given baudrate
uart.init(9600, bits=8, parity=None, stop=1, tx = 19, rx = 18)

def esptogsm():
        mystr = ''
        time.sleep(5)
        #print('Ingreso al if')
        if(uart.any() > 0):
            while(uart.any() > 0):
                mystr = mystr + str(uart.read())
            #print("ANTES DEL IF:")
            print(mystr)
            if '&' in mystr:
                print('Mensaje recibido desde otra esp32: ', mystr)
                mystr = mystr.split("'")
                #print('esta es mystr nueva: ')
                #print(mystr)
                splitstr = mystr[1].split('&')
                ntosend = splitstr[0]
                dtosend = splitstr[1]
                mtosend = splitstr[2]
                print('Contenido: ' + mtosend + ' El remitente es ' + ntosend + ' y el mensaje fue enviado: ' + dtosend)
                #sendSms(ntosend, dtosend, mtosend)
                mystr = ''

while(True):
    #print('waiting for message... ')
    while(uart.any() > 0):
        uart.read()
    esptogsm()