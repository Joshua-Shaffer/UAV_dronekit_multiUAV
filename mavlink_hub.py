from __future__ import print_function

from pymavlink import mavutil
import time
import threading
from queue import Queue

class Connection():
    def __init__(self, addr, addr_baud):
        self.addr = addr
        self.addr_baud = addr_baud
        self._active = False
        self.last_packet_received = 0
        self.last_connection_attempt = 0

    def open(self):
        try:
            print("Opening connection to %s" % (self.addr,))
            self.mav = mavutil.mavlink_connection(self.addr, baud=self.addr_baud)
            self._active = True
            self.last_packet_received = time.time() # lie

        except Exception as e:
            print("Connection to (%s) failed: %s" % (self.addr, str(e)))

    def close(self):
        self.mav.close()
        self._active = False

    def active(self):
        return self._active

class MAVLinkHub():
    def __init__(self, addrs, valid_fcn, addrs_baud):
        self.addrs = addrs
        self.addrs_baud = addrs_baud
        self.conns = []
        self.connection_maintenance_target_should_live = True
        self.inactivity_timeout = float('inf')
        self.reconnect_interval = 0
        self.valid_router = valid_fcn
        #self.mes_queue = Queue()

    def maintain_connections(self):
        now = time.time()
        for conn in self.conns:
            if not conn.active():
                continue
            if now - conn.last_packet_received > self.inactivity_timeout:
                print("Connection (%s) timed out" % (conn.addr,))
                conn.close()
        for conn in self.conns:
            if not conn.active():
                print("(%s) Innactive" % (conn.addr,))
                if now - conn.last_connection_attempt > self.reconnect_interval:
                    conn.last_connection_attempt = now
                    conn.open()
                    print("Reopened (%s)" % (conn.addr,))
        time.sleep(0.1)

    def create_connections(self):
        for idx, addr in enumerate(self.addrs):
            print("Creating connection (%s)" % addr)
            self.conns.append(Connection(addr, self.addrs_baud[idx]))

    def handle_messages(self):
        now = time.time()
        packet_received = False
        for conn in self.conns:
            if not conn.active():
                print('Inactive thread is ' + conn.addr)
                continue
            m = None
            try:
                m = conn.mav.recv_msg()
            except Exception as e:
                print("Exception receiving message on addr(%s): %s" % (str(conn.addr),str(e)))
                conn.close()

            if m is not None:
                conn.last_packet_received = now
                packet_received = True
                #print("Received message (%s) on connection %s from src=(%d/%d)" % (str(m), conn.addr, m.get_srcSystem(), m.get_srcComponent(),))
                for j in self.conns:

                    if not j.active():
                        #print('Innactive thread is ' + j.addr)
                        continue
                    if j.addr == conn.addr:
                        continue
                    if self.valid_router(m.get_srcSystem(), j):
                        # TODO: routing dependent on src System and ports!!!
                        #print("  Resending message on connection %s" % (j.addr,))
                        j.mav.write(m.get_msgbuf())
        if not packet_received:
            time.sleep(0.01)



    def init(self):
        self.create_connections()
        self.create_connection_maintenance_thread()

    def create_conn_threads(self, conn):
        def handle_message_loop(conn_item):
            #now = time.time()
            #packet_received = False
            while True:
                now = time.time()
                if not conn_item.active():
                    print('Inactive thread is ' + conn_item.addr)
                    continue
                m = None
                try:
                    m = conn_item.mav.recv_msg()
                except Exception as e:
                    print("Exception receiving message on addr(%s): %s" % (str(conn_item.addr),str(e)))
                    conn_item.close()

                if m is not None:
                    conn_item.last_packet_received = now
                    #packet_received = True
                    #print("Received message (%s) on connection %s from src=(%d/%d)" % (str(m), conn.addr, m.get_srcSystem(), m.get_srcComponent(),))
                    for j in self.conns:

                        if not j.active():
                            #print('Innactive thread is ' + j.addr)
                            continue
                        if j.addr == conn_item.addr:
                            continue
                        if self.valid_router(m.get_srcSystem(), j):
                            # TODO: routing dependent on src System and ports!!!
                            #print("  Resending message on connection %s" % (j.addr,))
                            j.mav.write(m.get_msgbuf())
            #if not packet_received:
            #    time.sleep(0.01)
        conn_track_thread = threading.Thread(target=handle_message_loop, args=(conn,))
        conn_track_thread.start()

    def loop(self):
        self.handle_messages()

    def create_connection_maintenance_thread(self):
        '''create and start helper threads and the like'''
        def connection_maintenance_target():
            while self.connection_maintenance_target_should_live:
                self.maintain_connections()
                time.sleep(0.1)
        connection_maintenance_thread = threading.Thread(target=connection_maintenance_target)
        connection_maintenance_thread.start()

    def run(self):
        self.init()


        print("Creating port threads")
        conn_threads = [0]*len(self.conns)
        for idx, conn in enumerate(self.conns):
            print('Opening thread for ' + str(idx))
            self.create_conn_threads(conn)

        while True:
            i=1
            #self.loop()

if __name__ == '__main__':

    from optparse import OptionParser
    parser = OptionParser("mavlink_hub.py [options]")
    (opts, args) = parser.parse_args()

    hub = MAVLinkHub(args)
    if len(args) == 0:
        print("Insufficient arguments")
        sys.exit(1)
    hub.run()
