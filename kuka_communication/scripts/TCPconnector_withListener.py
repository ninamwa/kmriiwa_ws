# load additional Python modules
import socket
import time
import _thread as thread


class TCPconnector:
    def __init__(self):

        # create TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Set buffer size
        self.BUFFER_SIZE = 1024

        # retrieve local hostname
        local_hostname = socket.gethostname()

        # get fully qualified hostname
        local_fqdn = socket.getfqdn()

        # get the according IP address
        ip_address = socket.gethostbyname(local_hostname)

        # bind the socket to the port 23456, and connect
        server_address = (ip_address, 23456)
        self.sock.connect(server_address)
        print("connecting to %s (%s) with %s" % (local_hostname, local_fqdn, ip_address))
        self.isconnected = True

        ## Make listener thread
        thread.start_new_thread(self.listener, ())


        ## Send data over socket
        self.talker()

    # Send data to initialize system
    def talker(self):
        # Define example data to be sent to the server:
            # The data here is required to initialize the system to enable communication
        Tool_Force = ">Tool_Force  13.4, 0.38, 5.9"
        Joint_pos = ">Joint_Pos 0, 0.17, 0, 1.92, 0, 0.35, 0"
        Tool_pos = ">Tool_Pos 433.59, 0.03, 601, 3.14, 1.1, 3.14"
        Tool_Torq = ">Tool_Torque 13.48, 0.378, 5.96"
        isComp = ">isCompliance false"
        isCollision = ">isCollision false"
        isReadyToMove = ">isReadyToMove true"
        isMastered = ">isMastered true"
        isOp = ">OperationMode T1" # Can be T1, T2 or AUT
        Change = [Joint_pos, Tool_pos, Tool_Force, Tool_Torq, isComp, isCollision, isReadyToMove, isMastered, isOp]
        i = 0
        # Keep sending in order for the system not to retire (5s)
        while i < 1000:
            for entry in Change:
                # SEND DATA OVER TCP:
                # new_data = entry.encode("utf-8")
                new_data = entry.encode()
                self.sock.send(new_data)
                # sock.sendall(new_data)
                # wait for two seconds
                time.sleep(0.1)
            i = i + 1
            time.sleep(1)

        ## close connection
        self.sock.close()


    def listener(self):
        while self.isconnected:
            data = self.sock.recv(self.BUFFER_SIZE)
            data = data.decode()
            print(data)




def main(args=None):
    con = TCPconnector()

if __name__ == '__main__':
    main()
