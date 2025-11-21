#!/usr/bin/env python3

import random
import socket
import json

while True:
    # Create a UDP socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Bind the socket to a specific address and port
    server_address = ('localhost', 4242)
    server_socket.bind(server_address)

    print('Server is listening on {}:{}'.format(*server_address))

    while True:
        try:
            # Receive data and the address of the sender
            data, address = server_socket.recvfrom(1024)
            message = data.decode()
            print('Received message from {}: {}'.format(address, message))

            # Send a response back to the sender
            reading = {
                    'min': 0.001,           # m
                    'max': 12.000,          # m
                    'angle': 0.3490659,     # radians
                    'value': float(random.randrange(1, 12000)) / 1000.0
                }

            response = json.dumps(reading)
            server_socket.sendto(response.encode(), address)
        except socket.error as e:
            print('Socket error:', e)
            server_socket.close()
            # break the inner loop to recreate the server socket
            break 

