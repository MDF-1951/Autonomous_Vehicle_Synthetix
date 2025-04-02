import serial
import time

# Set up the serial connection (adjust port as needed)
ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)  # Change to ttyAMA0 if using GPIO pins
time.sleep(2)  # Allow time for Arduino to reset

while True:
    try:
        # Define two float numbers to send
        num1 = 12.34 #servo
        num2 = 56.78 #dc motors

        # Format the string: "12.34,56.78\n"
        data_out = f"{num1},{num2}\n"

        # Send data
        ser.write(data_out.encode('utf-8'))
        print(f"Sent: {data_out.strip()}")

        # Wait for Arduino response
        data_in = ser.readline().decode('utf-8').strip()
        
        if data_in:
            received_numbers = data_in.split(',')
            if len(received_numbers) == 2:
                rx_num1 = float(received_numbers[0])
                rx_num2 = float(received_numbers[1])
                print(f"Received: {rx_num1}, {rx_num2}")

        time.sleep(1)  # Delay before sending again

    except KeyboardInterrupt:
        print("Exiting...")
        ser.close()
        break
