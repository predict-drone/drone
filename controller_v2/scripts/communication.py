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
                op, x, y, z, m0, m1, m2, m3 = struct.unpack(">BiiiIIII", msg)
                with open("/tmp/drone_data.log", "a") as f:
                    f.write(f"{round(x/(2**16), 2)}, {round(y/(2**16), 2)} {round(z/(2**16), 2)} "
                        + f"{round(m0/(2**16), 2)}, {round(m1/(2**16), 2)}, {round(m2/(2**16), 2)}, {round(m3/(2**16), 2)}\n")
            else:
                self.read_queue.put(msg)

    def recv_msg(self):
        return self.read_queue.get()

    def send_msg(self, msg: bytes):
        self.s.write(struct.pack(">B", len(msg)) + msg)

    def get_pid(self, pid_id: int):
        msg = struct.pack(">BB", CommOpcode.GET_PID, pid_id)
        self.send_msg(msg)

    def set_pid(self, pid_id: int, kp: int, ki: int, kd: int):
        msg = struct.pack(">BBIII", CommOpcode.SET_PID, pid_id, kp, ki, kd)
        print(msg.hex())
        self.send_msg(msg)

    get_pid_parser = argparse.ArgumentParser()
    get_pid_parser.add_argument("PID_ID", nargs="+", type=int)
    @with_argparser(get_pid_parser)
    def do_get_pid(self, opt):
        for pid_id in opt.PID_ID:
            self.get_pid(pid_id)
        for pid_id in opt.PID_ID:
            msg = self.recv_msg()
            op, pid_id, kp, ki, kd = struct.unpack(">BBIII", msg)
            kp /= 2**16
            ki /= 2**16
            kd /= 2**16
            print("GET:", op, pid_id, round(kp, 2), round(ki, 2), round(kd, 2))
        return False

    set_pid_parser = argparse.ArgumentParser()
    set_pid_parser.add_argument("KP", nargs=1, type=float)
    set_pid_parser.add_argument("KI", nargs=1, type=float)
    set_pid_parser.add_argument("KD", nargs=1, type=float)
    set_pid_parser.add_argument("PID_ID", nargs="+", type=int)
    @with_argparser(set_pid_parser)
    def do_set_pid(self, opt):
        for pid_id in opt.PID_ID:
            kp = int(opt.KP[0]/100*(2**16))
            ki = int(opt.KI[0]/100*(2**16))
            kd = int(opt.KD[0]/100*(2**16))
            self.set_pid(pid_id, kp, ki, kd)
        for pid_id in opt.PID_ID:
            msg = self.recv_msg()
            op, pid_id = struct.unpack(">BB", msg)
            print("SET:", op, pid_id, kp, ki, kd)
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
