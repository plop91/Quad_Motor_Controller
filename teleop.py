"""
Temporary teleoperation script for the robot
"""

import keyboard
import serial
import socket
import argparse
import threading



def main(args):

    server_address = ('10.10.10.220', 9100)
    client_address = ('10.10.10.10', 9100)

    # Check if the user has selected a mode
    if not args.server and not args.client and not args.local:
        print('Please select a mode')
        exit()

    i = 0
    if args.server:
        print('Server mode')
        i += 1
    if args.client:
        print('Client mode')
        i += 1
    if args.local:
        print('Local mode')
        i += 1
    if i > 1:
        print('Please select only one mode')
        exit()

    if args.local or args.server:
        # Establish a serial connection
        if args.local:
            ser = serial.Serial('/dev/COM5', 9600)
        if args.server:
            ser = serial.Serial('/dev/serial/by-id/usb-Arduino_RaspberryPi_Pico_086461E632630743-if00', 9600)
            # Establish a server connection
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # Bind the server to the address
            server.bind(server_address)
            server.listen()
            print('Waiting for a connection...')
            # Accept the connection
            client, client_address = server.accept()
            print(f'Connected to {client_address}')

    elif args.client:
        # Establish a client connection
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Connect to the server
        client.connect(server_address)
        

    speed = 250
    # Main loop
    while True:
        # Read the key pressed
        motor_velocities = [0, 0, 0, 0]

        if args.local or args.client:
            try:
                if keyboard.is_pressed('shift'):
                    print('Speed up')
                    speed += 50
                    if speed > 255:
                        speed = 255

                elif keyboard.is_pressed('ctrl'):
                    print('Speed down')
                    speed -= 50
                    if speed < 0:
                        speed = 0

                if keyboard.is_pressed('w'):
                    print('Forward')
                    for i in range(4):
                        motor_velocities[i] += speed

                if keyboard.is_pressed('a'):
                    print('Left')
                    motor_velocities[0] -= speed
                    motor_velocities[1] += speed
                    motor_velocities[2] -= speed
                    motor_velocities[3] += speed

                if keyboard.is_pressed('s'):
                    print('Backward')
                    for i in range(4):
                        motor_velocities[i] -= speed

                if keyboard.is_pressed('d'):
                    print('Right')
                    motor_velocities[0] += speed
                    motor_velocities[1] -= speed
                    motor_velocities[2] += speed
                    motor_velocities[3] -= speed

                if keyboard.is_pressed('q'):
                    print('Rotate left')
                    motor_velocities[0] -= speed
                    motor_velocities[1] += speed
                    motor_velocities[2] += speed
                    motor_velocities[3] -= speed

                if keyboard.is_pressed('e'):
                    print('Rotate right')
                    motor_velocities[0] += speed
                    motor_velocities[1] -= speed
                    motor_velocities[2] -= speed
                    motor_velocities[3] += speed

                if keyboard.is_pressed('space'):
                    print('Stop')
                    motor_velocities = [0, 0, 0, 0]

                if keyboard.is_pressed('esc'):
                    print('Exit')
                    break

                for i in range(4):
                    if motor_velocities[i] > 255:
                        motor_velocities[i] = 255
                    elif motor_velocities[i] < -255:
                        motor_velocities[i] = -255
                
                command = f"s {motor_velocities[0]} {motor_velocities[1]} {motor_velocities[2]} {motor_velocities[3]}"

            except Exception as e:
                print(e)
        elif args.server:
            data = client.recv(1024)
            command = data.decode().strip()

        
        if args.local or args.server:
            print(command)
            ser.write(command.encode())
            res = ser.readline()
            print(res.decode().strip())
            if args.server:
                client.sendall(res)
        elif args.client:
            print(f"Sent: {command}")
            client.sendall(command.encode())
            res = client.recv(1024)
            print(res.decode().strip())
        

if __name__ == '__main__':
    # parser for command line arguments
    parser = argparse.ArgumentParser(description='Teleoperation script for the robot')
    # server
    parser.add_argument('--server', dest='server', action='store_true', help='Server mode')
    # client
    parser.add_argument('--client', dest='client', action='store_true', help='Client mode')
    # serial
    parser.add_argument('--local', dest='local', action='store_true', help='local mode')
    # parse the arguments
    args = parser.parse_args()
    
    main(args)