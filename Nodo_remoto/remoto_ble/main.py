import machine, time
from machine import Pin, Timer
from time import sleep_ms
import ubluetooth
from esp32 import raw_temperature

uart = machine.UART(1, 9600)                         # init with given baudrate
uart.init(9600, bits=8, parity=None, stop=1, tx = 19, rx = 18)

def esptoble():
        mystr = ''
        time.sleep(5)
        print('Ingreso al if')
        if(uart.any() > 0):
            while(uart.any() > 0):
                mystr = mystr + str(uart.read())
            print("ANTES DEL IF:")
            print(mystr)
            if 'BLE' in mystr:
                print('Mensaje recibido desde Bluetooth: ', mystr)
                mystr = mystr.split("'")
                print('esta es mystr nueva: ')
                print(mystr)
                splitstr = mystr[1].split('|')
                ntosend = splitstr[0]
                mtosend = splitstr[1]
                print('Contenido: ' + mtosend + ' El remitente es ' + ntosend)
                mystr = ''
                ble.write(mtosend, True)
                print("msj de Ble enviado")
            if '&' in mystr:
                print('Mensaje recibido desde Gateway: ', mystr)
                mystr = mystr.split("'")
                print('esta es mystr nueva: ')
                print(mystr)
                splitstr = mystr[1].split('&')
                ntosend = splitstr[0]
                dtosend = splitstr[1]
                mtosend = splitstr[2]
                print('Contenido: ' + mtosend + ' El remitente es ' + ntosend + ' y el mensaje fue enviado: ' + dtosend)
                mystr = ''
                ble.write(mtosend, True)
                print("msj de Ble enviado")
            if '/' in mystr:
                print('Mensaje recibido desde Telegram: ', mystr)
                mystr = mystr.split("'")
                print('esta es mystr nueva: ')
                print(mystr)
                splitstr = mystr[1].split('/')
                ntosend = splitstr[0]
                dtosend = "nada"
                mtosend = splitstr[1]
                print('Contenido: ' + mtosend + ' y el remitente es ' + ntosend)
                mystr = ''
                ble.write(mtosend, True)
                print("msj de Ble enviado")

class BLE():


    def __init__(self, name):
        
        self.name = name
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
            uart.write(message)
        
            
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
        self.ble.gatts_write(self.trx, data + '\n')
        

    def advertiser(self):
        name = bytes(self.name, 'UTF-8')
        self.ble.gap_advertise(100, bytearray('\x02\x01\x02') + bytearray((len(name) + 1, 0x09)) + name)
        
# test
blue_led = Pin(2, Pin.OUT)
ble = BLE("ESP32_REMOTO",)
'''El siguiente mensaje enviado desde la ESP32 al periférico es simplemente para
setear el tamaño del buffer lo suficientemente grande como para poder enviar y
recibir textos largos. Aunque no se llega a leer el mensaje en el periférico ya que
inmediatamente se le envía el siguiente mensaje y lo pisa.'''
ble.write('123456789111315171921232527293133353739414345474951535557596163656769717375777981838587899193959799102105108111114117120123126129132135138141144147150', True)
while True:
    #print('waiting for message... ')
    while(uart.any() > 0):
        uart.read()
    esptoble()