import machine, time
import _thread as th
import ubluetooth
from machine import Pin, Timer
from time import sleep_ms
uart = machine.UART(1, 9600)                         # init with given baudrate
uart.init(9600, bits=8, parity=None, stop=1, tx = 19, rx = 18)
gsm = machine.UART(2, 115200)
gsm.init(115200, bits = 8, parity=None, stop=1)

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
        toEsp = n + "&" + d + "&" + m
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
                #print('entro al if')
                newstr2 = i.split('"')
                #print("newstr2:")
                #print(newstr2)
                
                number = newstr2[1]
                #print("number:" + number)
                date = newstr2[5]
                #print("date:" + date)
                msj = newstr2[6]
                #print("msj:" + msj)
                msj2 = msj.split("\\r\\n")
                #print("msj2:")
                #print(msj2)
                #print('separo componentes')
                #print('Contenido: ' + msj2[1] + ' El remitente es ' + number + ' y el mensaje fue enviado: ' + date)
                shareSms(number, date, msj2[1])


def gsmtoesp_th():
    while True:
        gsmtoesp()

def esptogsm():
        mystr = ''
        time.sleep(5)
        #print('Ingreso al if')
        if(uart.any() > 0):
            while(uart.any() > 0):
                mystr = mystr + str(uart.read())
            #print("ANTES DEL IF:")
            print(mystr)
            if 'BLE' in mystr:
                print('Mensaje recibido desde Bluetooth: ', mystr)
                mystr = mystr.split("'")
                #print('esta es mystr nueva: ')
                #print(mystr)
                splitstr = mystr[1].split('|')
                ntosend = splitstr[0]
                mtosend = splitstr[1]
                print('Contenido: ' + mtosend + ' El remitente es ' + ntosend)
                mystr = ''
                ble.write(mtosend, True)
                print("msj de Ble enviado")
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
                sendSms(ntosend, dtosend, mtosend)
                mystr = ''
            if '/' in mystr:
                print('Mensaje recibido desde otra esp32: ', mystr)
                mystr = mystr.split("'")
                #print('esta es mystr nueva: ')
                #print(mystr)
                splitstr = mystr[1].split('/')
                ntosend = splitstr[0]
                mtosend = splitstr[1]
                dtosend = "nada"
                print('Contenido: ' + mtosend + ' El remitente es ' + ntosend)
                mystr = ''
                sendSms(ntosend, dtosend, mtosend)
                print("msj sms enviado, contestando al emisor de TELEGRAM")
        
class BLE():


    def __init__(self):
        
        self.name = 'ESP32_GATEWAY'
        self.ble = ubluetooth.BLE()
        self.ble.active(True)

        self.led = Pin(2, Pin.OUT)
        self.timer1 = Timer(0)
        self.timer2 = Timer(1)
        
        self.disconnected()
        self.ble.irq(self.ble_irq)
        self.register()
        self.advertiser()
        


    def connected(self):
        
        self.timer1.deinit()
        self.timer2.deinit()


    def disconnected(self):
        
        self.timer1.init(period=1000, mode=Timer.PERIODIC, callback=lambda t: self.led(1))
        sleep_ms(200)
        self.timer2.init(period=1000, mode=Timer.PERIODIC, callback=lambda t: self.led(0))
    

    def ble_irq(self, event, data):

        if event == 1:
            '''Central connected'''
            self.connected()
            self.led(1)
        
        elif event == 2:
            '''Central disconnected'''
            self.advertiser()
            self.disconnected()
        
        elif event == 3:
            '''New message received'''
            buffer = self.ble.gatts_read(self.trx)
            message = buffer.decode('UTF-8').strip()
            print('Smartphone:', message)
            shareSms('BLE', 'gateway', message)
        
            
    def register(self):
        
        # Nordic UART Service (NUS)
        NUS_UUID = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E'
        TRX_UUID = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'
        
            
        BLE_NUS = ubluetooth.UUID(NUS_UUID)
        BLE_TRX = (ubluetooth.UUID(TRX_UUID), ubluetooth.FLAG_READ | ubluetooth.FLAG_WRITE,)
        
            
        BLE_UART = (BLE_NUS, (BLE_TRX,))
        SERVICES = (BLE_UART, )
        ((self.trx,), ) = self.ble.gatts_register_services(SERVICES)
        
        
    def write(self, data, notify=False, indicate=False):
        #print("stage 1")
        self.ble.gatts_write(self.trx, data + '\n')
        #print("stage 2")

    def advertiser(self):
        name = bytes(self.name, 'UTF-8')
        self.ble.gap_advertise(100, bytearray('\x02\x01\x02') + bytearray((len(name) + 1, 0x09)) + name)


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

blue_led = Pin(2, Pin.OUT)
ble = BLE()
'''El siguiente mensaje enviado desde la ESP32 al periférico es simplemente para
setear el tamaño del buffer lo suficientemente grande como para poder enviar y
recibir textos largos. Aunque no se llega a leer el mensaje en el periférico ya que
inmediatamente se le envía el siguiente mensaje y lo pisa.'''
ble.write('123456789111315171921232527293133353796163656769717375777981838587899193959799102105108111114117120123126129132135138141144147150', True)

while(True):
    #print('waiting for message... ')
    while(uart.any() > 0):
        uart.read()
    esptogsm()
    
    
    
    

