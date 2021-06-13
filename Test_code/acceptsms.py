import machine, time
import _thread as th
uart = machine.UART(1, 9600)                         # init with given baudrate
uart.init(9600, bits=8, parity=None, stop=1, tx = 19, rx = 18)
gsm = machine.UART(2, 9600)
gsm.init(9600, bits = 8, parity=None, stop=1)

def updateGsm(toGsm):
    if(toGsm != ''):
        gsm.write(toGsm)
        print('Sending -> ')
        print(toGsm)
        toGsm = ''
        
    time.sleep(10)
    
    while(gsm.any() > 0):
        mystr = gsm.read()
        print(mystr)
        
def sendSms(ntosend, dtosend, mtosend):
    time.sleep(1)

    toGsm = 'AT\r'
    updateGsm(toGsm)
    time.sleep(1)

    toGsm = 'AT+CMGF=1\r'
    updateGsm(toGsm)
    time.sleep(1)

    toGsm = 'AT+CMGS=\"' + ntosend + '\"\r'
    updateGsm(toGsm)

    toGsm = mtosend
    updateGsm(toGsm)

    toGsm = chr(26)
    updateGsm(toGsm)
    print('SMS enviado')
    uart.write('mensaje enviado')
        
def shareSms(n, d, m):
    if((n != '') and (d != '') and (m != '')):
        toEsp = n + "/" + d + "/" + m
        uart.write(toEsp)
        print('Sending -> ')
        print(toEsp)
        n = ''
        d = ''
        m = ''
        
        

def gsmtoesp():
    mystr = ''
    if(gsm.any() > 0):
        while(gsm.any() > 0):
            mystr = mystr + str(gsm.read())
        newstr = mystr.split('\r\n')
    
        for i in newstr:
            if 'CMT' in i:
                print('entro al if')
                newstr2 = i.split('"')

                
                number = newstr2[1]
                date = newstr2[5]
                msj = newstr2[6]
                msj2 = msj.split("\\r\\n")
                print('separo componentes')
                print('Contenido: ' + msj2[1] + ' El remitente es ' + number + ' y el mensaje fue enviado: ' + date)
                shareSms(number, date, msj2[1])
    
def esptogsm():
    mystr = ''
    time.sleep(5)
    print('Ingreso al if')
    if(uart.any() > 0):
        while(uart.any() > 0):
            mystr = mystr + str(uart.read())
        if '/' in mystr:
            print('Mensaje recibido desde otra esp32: ', mystr)
            mystr = mystr.split("'")
            print('esta es mystr nueva: ')
            print(mystr)
            splitstr = mystr[1].split('/')
            ntosend = splitstr[0]
            dtosend = splitstr[1]
            mtosend = splitstr[2]
            print('Contenido: ' + mtosend + ' El remitente es ' + ntosend + ' y el mensaje fue enviado: ' + dtosend)
            sendSms(ntosend, dtosend, mtosend)
            mystr = ''


def gsmtoesp_th():
    while True:
        gsmtoesp()

time.sleep(5)

print('inicializando')
toGsm = 'AT+CMGF=1\r'
updateGsm(toGsm)

time.sleep(1)

toGsm = 'AT+CNMI=1,2,0,0,0\r'
updateGsm(toGsm)
time.sleep(1)

print('esperando sms')
th.start_new_thread(gsmtoesp_th, ())

while(True):
    #print('waiting for message... ')
    esptogsm()
    
    
