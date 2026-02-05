import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time

class MotorControlApp:
    def __init__(self, master):
        self.master = master
        master.title("Motor Control GUI")
        self.serial_port = None

        # --- Top Controls ---
        top_frame = tk.Frame(master)
        top_frame.pack(pady=5, fill="x")

        tk.Label(top_frame, text="Select COM Port:").pack(side="left", padx=(10, 5))

        self.port_var = tk.StringVar()
        self.port_dropdown = ttk.Combobox(top_frame, textvariable=self.port_var, width=35, state="readonly")
        self.refresh_ports()
        self.port_dropdown.pack(side="left", padx=5)

        self.connect_button = tk.Button(top_frame, text="Connect", width=15, command=self.connect_port)
        self.connect_button.pack(side="left", padx=5)

        self.reset_com_button = tk.Button(top_frame, text="Reset COM Port", width=15, command=self.reset_com_port)
        self.reset_com_button.pack(side="left", padx=5)

        self.reset_button = tk.Button(top_frame, text="Reset Device", width=15, command=self.reset_port)
        self.reset_button.pack(side="left", padx=5)

        # --- Stage Selection (Linear/Rotary) ---
        stage_frame = tk.Frame(master)
        stage_frame.pack(pady=5)

        self.stage_var = tk.StringVar(value="Linear")
        tk.Radiobutton(stage_frame, text="Linear Stage", variable=self.stage_var, value="Linear", command=self.toggle_modes).pack(side="left", padx=20)
        tk.Radiobutton(stage_frame, text="Rotary Stage", variable=self.stage_var, value="Rotary", command=self.toggle_modes).pack(side="left", padx=20)

        # --- Motor Sections ---
        self.motor_frame = tk.Frame(master)
        self.motor_frame.pack(pady=10)

        # Frame to hold all mode boxes in a row
        self.modes_frame = tk.Frame(self.motor_frame)
        self.modes_frame.pack()

        # Single Jog Mode box
        self.jog_frame = tk.LabelFrame(self.modes_frame, text="Jog Mode", padx=10, pady=10)
        self.jog_frame.pack(side="left", padx=5)

        self.button_cw = tk.Button(self.jog_frame, text="Move Forward", width=20)  # Default for Linear Stage
        self.button_ccw = tk.Button(self.jog_frame, text="Move Reverse", width=20)  # Default for Linear Stage
        self.button_cw.pack(pady=2)
        self.button_ccw.pack(pady=2)

        self.bind_jog_buttons()

        # Incremental Mode (for both Linear and Rotary Stages)
        self.incremental_frame = tk.LabelFrame(self.modes_frame, text="Incremental Mode", padx=10, pady=10)
        self.incremental_frame.pack(side="left", padx=5)

        self.button_step_cw = tk.Button(self.incremental_frame, text="Step Forward", width=20)  # Default for Linear Stage
        self.button_step_ccw = tk.Button(self.incremental_frame, text="Step Reverse", width=20)  # Default for Linear Stage
        self.button_step_cw.pack(pady=2)
        self.button_step_ccw.pack(pady=2)

        self.bind_incremental_buttons()

        # Cyclical Mode (only for Rotary Stage)
        self.cyclical_frame = tk.LabelFrame(self.modes_frame, text="Cyclical Mode", padx=10, pady=10)
        self.cyclical_frame.pack(side="left", padx=5)

        # Frame to hold Arc Length and No. of Cycles in a row
        self.cyclical_inputs_frame = tk.Frame(self.cyclical_frame)
        self.cyclical_inputs_frame.pack()

        # Arc Length section (left)
        self.arc_frame = tk.Frame(self.cyclical_inputs_frame)
        self.arc_frame.pack(side="left", padx=5)
        tk.Label(self.arc_frame, text="Arc Length (in mm : 0-10):").pack()
        self.length_entry = tk.Entry(self.arc_frame, justify='center', width=10)
        self.length_entry.pack(pady=2)

        # No. of Cycles section (right of Arc Length)
        self.cycles_frame = tk.Frame(self.cyclical_inputs_frame)
        self.cycles_frame.pack(side="left", padx=5)
        tk.Label(self.cycles_frame, text="No. of Cycles (1-100):").pack()
        self.cycles_entry = tk.Entry(self.cycles_frame, justify='center', width=10)
        self.cycles_entry.pack(pady=2)

        # Run button below the inputs
        self.run_button = tk.Button(self.cyclical_frame, text="Run", width=15, command=self.send_cyclical_command)
        self.run_button.pack(pady=5)

        # Initially hide Cyclical mode (since Linear Stage is default)
        self.toggle_modes()

        # --- Terminal Header ---
        terminal_header = tk.Frame(master)
        terminal_header.pack(fill="x", padx=10)

        terminal_label = tk.Label(terminal_header, text="Serial Monitor:")
        terminal_label.pack(side="left")

        self.clear_terminal_button = tk.Button(terminal_header, text="Clear Terminal", command=self.clear_terminal)
        self.clear_terminal_button.pack(side="right")

        # --- Terminal Area ---
        self.terminal = tk.Text(master, height=10, state='disabled')
        self.terminal.pack(fill="both", expand=True, padx=10, pady=5)

        self.append_to_terminal("Welcome! Select a COM port to begin.")

    def refresh_ports(self):
        ports = list(serial.tools.list_ports.comports())
        display_ports = []
        for port in ports:
            name = f"{port.device}"
            if any(x in port.description for x in ["Arduino", "CH340", "Uno", "USB Serial Device"]):
                name += " (Motor Control Box)"
            display_ports.append(name)
        self.port_dropdown['values'] = display_ports

    def connect_port(self):
        selected = self.port_var.get()
        if not selected:
            messagebox.showwarning("Warning", "Please select a COM port.")
            return
        port = selected.split()[0]
        try:
            self.serial_port = serial.Serial(port, baudrate=9600, timeout=0.1, parity=serial.PARITY_EVEN)
            self.append_to_terminal("Connected successfully.")
            self.serial_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.serial_thread.start()
        except Exception as e:
            self.append_to_terminal("COM port not available, please reset.")

    def reset_port(self):
        if self.serial_port:
            self.send_serial("RZ")
            #self.append_to_terminal("Sent: RZ")

    def reset_com_port(self):
        if self.serial_port:
            try:
                self.serial_port.close()
            except:
                pass
        self.serial_port = None
        self.port_dropdown.set('')
        self.refresh_ports()
        self.append_to_terminal("COM port reset.")

    def read_serial(self):
        while self.serial_port and self.serial_port.is_open:
            try:
                line = self.serial_port.readline().decode().strip()
                if line:
                    self.master.after(0, lambda msg=line: self.append_to_terminal(f"Received: {msg}"))
            except:
                break

    def send_serial(self, data):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(data.encode())
                time.sleep(0.1)
            except:
                self.append_to_terminal("Failed to send data. Port may be closed.")

    def append_to_terminal(self, message):
        self.terminal.configure(state='normal')
        self.terminal.insert(tk.END, message + '\n')
        self.terminal.configure(state='disabled')
        self.terminal.see(tk.END)

    def clear_terminal(self):
        self.terminal.configure(state='normal')
        self.terminal.delete('1.0', tk.END)
        self.terminal.configure(state='disabled')

    def bind_jog_buttons(self):
        # Unbind previous bindings
        self.button_cw.bind("<ButtonPress>", lambda e: None)
        self.button_cw.bind("<ButtonRelease>", lambda e: None)
        self.button_ccw.bind("<ButtonPress>", lambda e: None)
        self.button_ccw.bind("<ButtonRelease>", lambda e: None)

        # Bind based on selected stage
        motor_id = "1" if self.stage_var.get() == "Linear" else "2"
        self.button_cw.bind("<ButtonPress>", lambda e: self.send_serial(f"C{motor_id}Z"))
        self.button_cw.bind("<ButtonRelease>", lambda e: self.send_serial(f"S{motor_id}Z"))
        self.button_ccw.bind("<ButtonPress>", lambda e: self.send_serial(f"D{motor_id}Z"))
        self.button_ccw.bind("<ButtonRelease>", lambda e: self.send_serial(f"S{motor_id}Z"))

    def bind_incremental_buttons(self):
        # Unbind previous bindings
        self.button_step_cw.config(command=lambda: None)
        self.button_step_ccw.config(command=lambda: None)

        # Bind based on selected stage
        motor_id = "1" if self.stage_var.get() == "Linear" else "2"
        self.button_step_cw.config(command=lambda: self.send_serial(f"F{motor_id}Z"))
        self.button_step_ccw.config(command=lambda: self.send_serial(f"G{motor_id}Z"))

    def toggle_modes(self):
        # Update Jog Mode bindings and text based on selected stage
        self.bind_jog_buttons()
        if self.stage_var.get() == "Linear":
            self.button_cw.config(text="Move Forward")
            self.button_ccw.config(text="Move Reverse")
            self.button_step_cw.config(text="Step Forward")
            self.button_step_ccw.config(text="Step Reverse")
        else:
            self.button_cw.config(text="Move Clockwise")
            self.button_ccw.config(text="Move Counter Clockwise")
            self.button_step_cw.config(text="Step Clockwise")
            self.button_step_ccw.config(text="Step Counter Clockwise")

        # Update Incremental Mode bindings
        self.bind_incremental_buttons()

        # Show/Hide Cyclical mode based on stage
        if self.stage_var.get() == "Rotary":
            self.cyclical_frame.pack(side="left", padx=5)
            self.run_button.config(state='normal')
        else:
            self.cyclical_frame.pack_forget()
            self.run_button.config(state='disabled')

    def send_cyclical_command(self):
        try:
            length = int(self.length_entry.get())
            cycles = int(self.cycles_entry.get())
            if not (0 <= length <= 10):
                raise ValueError("Length out of range")
            if not (1 <= cycles <= 100):
                raise ValueError("Cycles out of range")
            length_str = f"{length}"
            cycles_str = f"{cycles}"
            command = f"E2:{length_str}:{cycles_str}:Z"
            self.send_serial(command)
            #self.append_to_terminal(f"Sent: {command}")
        except ValueError as e:
            messagebox.showerror("Invalid Input", str(e))

if __name__ == "__main__":
    root = tk.Tk()
    app = MotorControlApp(root)
    root.mainloop()