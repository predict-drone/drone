import sys
import queue
import serial
import struct
import argparse
import threading

from cmd2 import Cmd, with_argparser

class CommOpcode:
    GET_PID = 1
    SET_PID = 2
    ANGLES = 3

class PidId:
    PITCH = 1
    ROLL = 2
    YAW = 3
    PITCH_VEL = 4
    ROLL_VEL = 5
    YAW_VEL = 6

class App(Cmd):
    intro = "Drone project telemetry"
    prompt = ">>> "
    def __init__(self, port: str):
        self.s = serial.Serial(port, 115200)
        self.read_queue = queue.Queue()
        threading.Thread(target=self.read_msg, daemon=True).start()
        super().__init__()

    def read_msg(self):
        while True:
            msg_len = int.from_bytes(self.s.read(1), "little")
            msg = self.s.read(msg_len)
            if msg[0] == CommOpcode.ANGLES:
                op, x, y, z = struct.unpack(">Biii", msg)
                with open("/tmp/drone_data.log", "a") as f:
                    f.write(f"x: {x/100}, y:{y/100}, z: {z/100}\n")
            else:
                self.read_queue.put(msg)

    def recv_msg(self):
        return self.read_queue.get()

    def send_msg(self, msg: bytes):
        self.s.write(struct.pack(">B", len(msg)) + msg)

    def get_pid(self, pid_id: int):
        msg = struct.pack(">BB", CommOpcode.GET_PID, pid_id)
        self.send_msg(msg)

    def set_pid(self, pid_id: int, kp: float, ki: float, kd: float):
        kp = int(kp*100)
        ki = int(ki*100)
        kd = int(kd*100)
        msg = struct.pack(">BBIII", CommOpcode.SET_PID, pid_id, kp, ki, kd)
        self.send_msg(msg)

    get_pid_parser = argparse.ArgumentParser()
    get_pid_parser.add_argument("PID_ID", nargs="+", type=int)
    @with_argparser(get_pid_parser)
    def do_get_pid(self, opt):
        for pid_id in opt.PID_ID:
            self.get_pid(pid_id)
        for pid_id in opt.PID_ID:
            msg = self.recv_msg()
            op, pid_id, kd, ki, kd = struc.unpack(">BBIII", msg)
            print("GET:", op, pid_id, kd, ki, kd)
        return False

    set_pid_parser = argparse.ArgumentParser()
    set_pid_parser.add_argument("KP", nargs=1, type=float)
    set_pid_parser.add_argument("KI", nargs=1, type=float)
    set_pid_parser.add_argument("KD", nargs=1, type=float)
    set_pid_parser.add_argument("PID_ID", nargs="+", type=int)
    @with_argparser(set_pid_parser)
    def do_set_pid(self, opt):
        for pid_id in opt.PID_ID:
            self.set_pid(pid_id, opt.KP, opt.KI, opt.KD)
        for pid_id in opt.PID_ID:
            msg = self.recv_msg()
            op, pid_id = struc.unpack(">BB", msg)
            print("SET:", op, pid_id, kd, ki, kd)
        return False

    def do_quit(self, opt):
        return True

if __name__ == "__main__":
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = "/dev/ttyUSB0"

    app = App(port)
    app.cmdloop()
