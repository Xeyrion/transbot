#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""UDP client for robot_side/arm_udp_agent.py.

Used by arm_gui.py so the simulation panel can command and display the real
arm without leaving the local Gazebo master.
"""
from __future__ import division

import json
import socket
import threading
import time

DEFAULT_PORT = 42100


class ArmUdpClient(object):
    def __init__(self, host, port=DEFAULT_PORT):
        self.host = host
        self.port = int(port)
        self.state = None
        self.last_rx = 0.0
        self.last_error = ''
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 0))
        self.sock.settimeout(0.3)
        self._stop = threading.Event()
        self._thread = None
        self._last_ping = 0.0

    # ------------------------------------------------------------- lifecycle
    def start(self):
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._loop)
        self._thread.daemon = True
        self._thread.start()
        self.ping()

    def stop(self):
        self._stop.set()
        try:
            self.sock.close()
        except Exception:
            pass
        self._thread = None

    # --------------------------------------------------------------- sending
    def _send(self, msg):
        try:
            data = json.dumps(msg).encode('utf-8')
            self.sock.sendto(data, (self.host, self.port))
            return True
        except Exception as e:
            self.last_error = '%s' % e
            return False

    def ping(self):
        return self._send({'cmd': 'ping'})

    def send_set(self, j7, j8, j9=None, run_time=800):
        msg = {'cmd': 'set', 'j7': float(j7), 'j8': float(j8),
               'run_time': int(run_time)}
        if j9 is not None:
            msg['j9'] = float(j9)
        return self._send(msg)

    # -------------------------------------------------------------- receiving
    def _loop(self):
        while not self._stop.is_set():
            try:
                data, _ = self.sock.recvfrom(4096)
            except socket.timeout:
                # Keep the agent pointed back at us: it only replies to the
                # last sender, so after an agent restart it needs a packet.
                now = time.time()
                if now - self.last_rx > 1.0 and now - self._last_ping > 1.0:
                    self._last_ping = now
                    self.ping()
                continue
            except Exception:
                break
            try:
                msg = json.loads(data.decode('utf-8'))
            except ValueError:
                continue
            if isinstance(msg, dict) and msg.get('type') == 'state':
                self.state = msg
                self.last_rx = time.time()

    # ----------------------------------------------------------------- status
    def age(self):
        return time.time() - self.last_rx if self.last_rx else 1e9

    def connected(self, timeout=1.5):
        return self.age() < timeout
