import serial
import time

# Set up serial connection (replace with your actual COM port)
ser = serial.Serial(
    port='COM3',  # Replace 'COM3' with your port (e.g., /dev/ttyUSB0 on Linux)
    baudrate=921600,  # Must match the baud rate set in STM32CubeMX
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0.001  # Adjust timeout to wait for the entire response
)

def send_data(data):
    """ Send data to STM32 via UART """
    if ser.is_open:
        ser.write(data.encode())  # Encode string to bytes and send over UART
        print(f"Sent: {data}")

def receive_data():
    """ Receive data from STM32 via UART """
    if ser.is_open:
        data = ser.read(32).decode().strip()  # Read up to 32 bytes or less
        return data

try:
    # Test loop: send data and wait for response
    while True:
        # Send data
        send_data(data_to_send)  # No need to append '\n', just send raw data

        # Wait a bit to allow STM32 to respond
        response = receive_data()
        if response:
            print(f"Received: {response}")
        else:
            print("No response received")

except KeyboardInterrupt:
    print("Program interrupted")

finally:
    # Close the serial connection when done
    ser.close()
    print("Serial connection closed")
