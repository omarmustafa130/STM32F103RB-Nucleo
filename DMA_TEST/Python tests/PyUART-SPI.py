import serial, logging, random
from time import sleep

# Configure logging to output to the console
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# Parameters
port = 'COM3'  # Replace with your port name
baudrate = 921600
timeout = 0.05   

SYNC_BYTE_READ = 0xAA
SYNC_BYTE_WRITE = 0xAB
WRITE_BYTE = 0x80
OLD_PROTOCOL = False

node_addresses = [1]
#node_addresses = [1, 10, 12]

# Dataset: register address, 32-bit data, comment
register_data = [[0x6C, 0x000100C5, 'CHOPCONF'],
                 [0x2D, 0x1234ABCD, 'XTARGET'],
                 [0x27, 0x10000000, 'VMAX, write only in TMC5160'],
                 [0x80, 0x1234ABCD, 'STM32 Digital Inputs, RO'],
                 [0x81, 0x1234ABCD, 'STM32 Digital Outputs RW'],
                 [0x90, 0x1234ABCD, 'STM32 AIN1'], 
                 [0x94, 0x1234ABCD, 'STM32 AIN4'],
                 [0xA0, 0x1234ABCD, 'DS18B20 #1'],
                 [0xF0, 0x1234ABCD, 'Stat reg #1']
                 ] 

def settings_generator():
    pattern = [(True, False), (False, True), (True, True)]
    index = 0
    
    while True:
        yield pattern[index]
        index = (index + 1) % len(pattern)

def crc8(datagram: bytearray, initial_value=0) -> int:
    crc = initial_value
    for byte in datagram:
        for _ in range(8):
            if (crc >> 7) ^ (byte & 0x01):
                crc = ((crc << 1) ^ 0x07) & 0xff
            else:
                crc = (crc << 1) & 0xff
            byte >>= 1
    return crc


def send_datagram(node_address, register_address, data, write, desc, wrong_crc, malformed):
    desc = " (" + desc + ")"
    if not write:
        desc = desc + " READ"
        datagram = bytearray([SYNC_BYTE_READ, node_address])
        datagram.append(register_address)
    else:
        desc = desc + " WRITE"

        if OLD_PROTOCOL:
            datagram = bytearray([SYNC_BYTE_READ, node_address])
            datagram.extend([WRITE_BYTE | register_address] + list(data.to_bytes(4, 'big'))) # Converts to 4 bytes in big-endian
        else:
            datagram = bytearray([SYNC_BYTE_WRITE, node_address])
            datagram.append(register_address)
            datagram.extend(list(data.to_bytes(4, 'big')))  # Converts to 4 bytes in big-endian

        
    if not wrong_crc:
        crc = crc8(datagram)
    else:
        crc = crc8(datagram)-1
        desc = desc + " -CRC INCORRECT-"

    if malformed:
        dropped = random.randint(1, 3)
        desc = desc + " -MALFORMED-, " + str(dropped) + " byte(s) dropped. Original datagram: " + " ".join(f"0x{byte:02X}" for byte in datagram)
        datagram = datagram[:-dropped]

    logging.info('Sent datagram: ' + " ".join(f"0x{byte:02X}" for byte in datagram) + " " + (f"0x{crc:02X}") + desc)
    datagram.append(crc)

    try:
        uart.write(datagram)
        sleep(0.05)  # Add delay after sending each datagram

    except serial.SerialException as e:
        logging.error(f"Failed to send datagram: {e}")


def receive_datagram():
    try:
        datagram = uart.read(8)  # Adjust buffer size if necessary
        if datagram:
            datagram = bytearray(datagram)
            datagram_without_crc = datagram[:-1]
            received_crc = datagram[-1]
            calculated_crc = crc8(datagram_without_crc)
            if calculated_crc == received_crc:
                logging.info('Received datagram: ' + " ".join(f"0x{byte:02X}" for byte in datagram) + ', CRC OK')
            else:
                logging.info('Received datagram: ' + " ".join(f"0x{byte:02X}" for byte in datagram) + ', CRC ERROR, calculated CRC: ' + "0x{:X}".format(calculated_crc))
        else:

            logging.warning("No response received.")
    except serial.SerialException as e:
        logging.error(f"Failed to read response: {e}")
    finally:
        print()


# Open the serial port
try:
    uart = serial.Serial(port, baudrate, timeout=timeout)
    logging.info(f"Serial port {port} opened successfully.")
except serial.SerialException as e:
    logging.error(f"Failed to open serial port {port}: {e}")
    uart = None

if uart:
    # Correct datagrams
    for node_address in node_addresses:
        # In your test script, send a dummy datagram first

        for register_address, data, desc in register_data:
            write = False
            wrong_crc = False
            malformed = False
            send_datagram(node_address, register_address, data, write, desc, wrong_crc, malformed)
            receive_datagram()
            sleep(0.01)
            write = True
            send_datagram(node_address, register_address, data, write, desc, wrong_crc, malformed)           
            receive_datagram()

    # Warped datagrams
    settings = settings_generator()
    for node_address in node_addresses:
        for register_address, data, desc in register_data:
            write = random.choice([True, False])
            wrong_crc, malformed = next(settings)
            send_datagram(node_address, register_address, data, write, desc, wrong_crc, malformed)
            receive_datagram()
            sleep(0.01)


    # Close the serial port
    try:
        uart.close()
        logging.info("Serial port closed successfully.")
    except serial.SerialException as e:
        logging.error(f"Failed to close serial port: {e}")
