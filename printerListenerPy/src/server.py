#!/usr/bin/env python

import SocketServer
import subprocess
import os
import time

class MyUDPHandler(SocketServer.BaseRequestHandler):

    def handle(self):
        directory = os.environ['ROS_DIR'] + "printer/c++/"
        
        subprocess.call([directory + "async", "0", str(time.gmtime()), directory + "message.txt", "true", "3", "true"])

if __name__ == "__main__":
    HOST, PORT = "localhost", 9999
    server = SocketServer.UDPServer((HOST, PORT), MyUDPHandler)
    server.serve_forever()

