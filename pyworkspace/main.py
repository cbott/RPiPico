from machine import UART, Pin
import time

BAUDRATE = 1000000


def test_uart():
    uart1 = UART(1, baudrate=BAUDRATE, tx=Pin(8), rx=Pin(9))

    uart0 = UART(0, baudrate=BAUDRATE, tx=Pin(0), rx=Pin(1))

    txData = b'hello world\n\r'
    print('Writing Data', txData)
    uart1.write(txData)
    time.sleep(0.1)
    print('Reading Data...')
    rxData = bytes()
    while uart0.any() > 0:
        rxData += uart0.read(1)

    print('Received', rxData.decode('utf-8'))


# main
test_uart()
