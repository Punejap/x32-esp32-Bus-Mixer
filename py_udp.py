import socket

 

msgFromClient       = "/ch/01/config/name\0\0"


bytesToSend         = str.encode(msgFromClient)
print(str(bytesToSend))
serverAddressPort   = ("192.168.1.165", 10023)

bufferSize          = 1024

 

# Create a UDP socket at client side

UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

 

# Send to server using created UDP socket

UDPClientSocket.sendto(bytesToSend, serverAddressPort)

 

msgFromServer = UDPClientSocket.recvfrom(bufferSize)

 

msg = "Message from Server {}".format(msgFromServer[0])

print(msg)