import sys
import queue
import serial
import struct
import tkinter
import threading
import tkinter.messagebox
import tkinter.font as tfont


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


class PidController:
    def __init__(self, pid_id, serial, parent, label):
        self.pid_id = pid_id
        self.serial = serial

        self.pane = tkinter.PanedWindow(parent, orient=tkinter.HORIZONTAL,
                height=45)
        self.label= tkinter.Label(self.pane, text=label, width=12,
                font=(None, 25))
        self.pane.add(self.label)

        self.entry_kp = tkinter.Entry(self.pane, width=5, font=(None, 25))
        self.pane.add(self.entry_kp)
        self.entry_ki = tkinter.Entry(self.pane, width=5, font=(None, 25))
        self.pane.add(self.entry_ki)
        self.entry_kd = tkinter.Entry(self.pane, width=5, font=(None, 25))
        self.pane.add(self.entry_kd)

        self.button = tkinter.Button(self.pane, text="Send", font=(None, 25),
            command=self.set_pid)
        self.pane.add(self.button)
        parent.add(self.pane)

    def set_pid(self):
        try:
            kp = int(float(self.entry_kp.get())*2**16)
            ki = int(float(self.entry_ki.get())*2**16)
            kd = int(float(self.entry_kd.get())*2**16)
        except ValueError:
            tkinter.messagebox.showerror("Send Error","Inavlid PID values")
            return
        msg = struct.pack(">BBIII", CommOpcode.SET_PID, self.pid_id, kp, ki, kd)
        print("Set pid msg:", msg.hex())
        self.serial.send_msg(msg)
        msg = self.serial.recv_msg()
        if msg is None:
            tkinter.messagebox.showerror("Send Error","Reply not received")
        else:
            op, pid_id = struct.unpack(">BB", msg)
            print("SET:", op, pid_id, kp, ki, kd)

    def get_pid(self):
        msg = struct.pack(">BB", CommOpcode.GET_PID, self.pid_id)
        self.serial.send_msg(msg)
        msg = self.serial.recv_msg()
        if msg is None:
            tkinter.messagebox.showerror("Send Error","Reply not received")
        else:
            print(msg, type(msg))
            op, pid_id, kp, ki, kd = struct.unpack(">BBIII", msg)
            self.entry_kp.set(str(round(float(kp)/(2**16), 2)))
            self.entry_ki.set(str(round(float(ki)/(2**16), 2)))
            self.entry_kd.set(str(round(float(kd)/(2**16), 2)))


class SerialController:
    def __init__(self, port):
        self.s = serial.Serial(port, 115200)
        self.read_queue = queue.Queue()

    def read(self):
        while True:
            msg_len = int.from_bytes(self.s.read(1), "little")
            msg = self.s.read(msg_len)
            self.read_queue.put(msg)

    def send_msg(self, msg):
        self.s.write(bytes([len(msg)]) + msg)

    def recv_msg(self):
        try:
            return self.read_queue.get(timeout=1.5)
        except queue.Empty:
            return None


def main(port):
    serial_controller = SerialController(port)
    threading.Thread(target=serial_controller.read, daemon=True).start()

    root = tkinter.Tk()
    root.attributes('-type', 'dialog')

    top_pane = tkinter.PanedWindow(root, orient=tkinter.VERTICAL)
    top_pane.pack()


    pitch_pos = PidController(1, serial_controller, top_pane, "Pitch position")
    roll_pos = PidController(2, serial_controller, top_pane, "Roll position")
    pitch_vel = PidController(4, serial_controller, top_pane, "Pitch vel")
    roll_vel = PidController(5, serial_controller, top_pane, "Roll vel")
    yaw_vel = PidController(6, serial_controller, top_pane, "Yaw vel")

    def refresh():
        pitch_pos.get_pid()
        roll_pos.get_pid()
        pitch_vel.get_pid()
        roll_vel.get_pid()
        yaw_vel.get_pid()

    refresh_button = tkinter.Button(top_pane, text="Refresh", font=(None, 25),
            command=refresh, width=10, height=1, padx = 30)
    top_pane.add(refresh_button)

    root.mainloop()


if __name__ == "__main__":
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = "/dev/ttyUSB0"
    main(port)
