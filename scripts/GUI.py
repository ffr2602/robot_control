import math
import transformations
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import roslibpy


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Robot Control Panel")
        self.is_connected = False
        self.create_widgets()

    def create_widgets(self):
        # IP Address
        tk.Label(self, text="IP Address:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.ip_address = tk.Entry(self)
        self.ip_address.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)

        # Port
        tk.Label(self, text="Port:").grid(row=0, column=2, pady=5, sticky=tk.W)
        self.port = tk.Entry(self, width=7)
        self.port.grid(row=0, column=3, padx=5, pady=5)
        self.port.insert(0, '9090')

        # Connection Indicator
        self.connection_indicator = tk.Label(self, text="Disconnected", bg="red", width=15, relief=tk.GROOVE)
        self.connection_indicator.grid(row=0, column=4, padx=5, pady=5, columnspan=1)

        frame_01 = ttk.Frame(self)
        frame_02 = ttk.Frame(self)
        frame_01.grid(column=0, row=3, sticky=tk.W, columnspan=5)
        frame_02.grid(column=2, row=3, sticky=tk.W, columnspan=5)

        tk.Label(frame_01, text='Monitor Position:', bg='Yellow', relief=tk.FLAT).grid(
            row=0, column=0, pady=5, padx=5, sticky=tk.W, columnspan=2)
        
        tk.Label(frame_01, text="Position X:").grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        tk.Label(frame_01, text="Position Y:").grid(row=2, column=0, padx=5, pady=5, sticky=tk.W)
        tk.Label(frame_01, text="Heading:").grid(row=3, column=0, padx=5, pady=5, sticky=tk.W)

        self.position_data_x = tk.Label(frame_01, text="0.0 m", width=20, relief=tk.SUNKEN)
        self.position_data_y = tk.Label(frame_01, text="0.0 m", width=20, relief=tk.SUNKEN)
        self.position_data_z = tk.Label(frame_01, text="0°", width=20, relief=tk.SUNKEN)
        self.position_data_x.grid(row=1, column=1, padx=5, pady=5, sticky=tk.W)
        self.position_data_y.grid(row=2, column=1, padx=5, pady=5, sticky=tk.W)
        self.position_data_z.grid(row=3, column=1, padx=5, pady=5, sticky=tk.W)

        tk.Label(frame_02, text='Monitor Velocity:', bg='Yellow', relief=tk.FLAT).grid(
            row=0, column=0, pady=5, padx=5, sticky=tk.W, columnspan=2)

        tk.Label(frame_02, text="Linier X:").grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        tk.Label(frame_02, text="Linier Y:").grid(row=2, column=0, padx=5, pady=5, sticky=tk.W)
        tk.Label(frame_02, text="Angular Z:").grid(row=3, column=0, padx=5, pady=5, sticky=tk.W)

        self.speed_data_x = tk.Label(frame_02, text="0.0 m/s", width=10, relief=tk.SUNKEN)
        self.speed_data_y = tk.Label(frame_02, text="0.0 m/s", width=10, relief=tk.SUNKEN)
        self.speed_data_z = tk.Label(frame_02, text="0.0 m/s", width=10, relief=tk.SUNKEN)
        self.speed_data_x.grid(row=1, column=1, padx=5, pady=5, sticky=tk.W)
        self.speed_data_y.grid(row=2, column=1, padx=5, pady=5, sticky=tk.W)
        self.speed_data_z.grid(row=3, column=1, padx=5, pady=5, sticky=tk.W)

        tk.Label(self).grid(row=4, column=0, sticky=tk.W, pady=5)
        tk.Label(self, text="Input Position", bg='Yellow', relief=tk.FLAT).grid(
            row=5, column=0, sticky=tk.W, padx=10, pady=5, columnspan=5)

        # Position X
        tk.Label(self, text="Position X:").grid(row=6, column=0, padx=10, pady=5, sticky=tk.W)
        tk.Label(self, text="Meter (M)").grid(row=6, column=2, sticky=tk.W)
        self.position_x = tk.Entry(self)
        self.position_x.grid(row=6, column=1, padx=10, pady=5)

        # Position Y
        tk.Label(self, text="Position Y:").grid(row=7, column=0, padx=10, pady=5, sticky=tk.W)
        tk.Label(self, text="Meter (M)").grid(row=7, column=2, sticky=tk.W)
        self.position_y = tk.Entry(self)
        self.position_y.grid(row=7, column=1, padx=10, pady=5)

        # Heading
        tk.Label(self, text="Heading:").grid(row=8, column=0, padx=10, pady=5, sticky=tk.W)
        tk.Label(self, text="Derajat (°)").grid(row=8, column=2, sticky=tk.W)
        self.heading = tk.Entry(self)
        self.heading.grid(row=8, column=1, padx=10, pady=5)

        # Buttons
        self.reset_button = ttk.Button(self, text="Reset", command=self.reset)
        self.reset_button.grid(row=9, column=0, padx=10, pady=10)

        self.publish_button = ttk.Button(self, text="Publish", command=self.publish, state=tk.DISABLED, width=20)
        self.publish_button.grid(row=9, column=1, padx=10, pady=10)

        self.connect_button = ttk.Button(self, text="Connect", command=self.toggle_connection, width=17)
        self.connect_button.grid(row=2, column=4, padx=10, pady=10)

        self.messages_text = tk.Text(self, height=10, width=55, state=tk.DISABLED)
        self.messages_text.grid(row=10, column=0, pady=5, padx=20, columnspan=5)

    def reset(self):
        def reset_thread():
            self.position_x.delete(0, tk.END)
            self.position_y.delete(0, tk.END)
            self.heading.delete(0, tk.END)
            self.reset_pub = roslibpy.Topic(
                self.client, '/rset', 'std_msgs/String', queue_size=5)
            if self.reset_pub and self.is_connected:
                message = roslibpy.Message({'data': 'reset'})
                self.reset_pub.publish(message)
                messagebox.showinfo("Publish Status", "RESET OK!!")
        threading.Thread(target=reset_thread).start()

    def publish(self):
        if not self.is_connected:
            messagebox.showerror("Connection Error", "Not connected to ROS.")
            return
        try:
            pos_x = float(self.position_x.get())
            pos_y = float(self.position_y.get())
            tf_quat = transformations.quaternion_from_euler(0.0, 0.0, float(self.heading.get()) * math.pi / 180)
            print(tf_quat)
        except ValueError:
            messagebox.showerror("Input Error", "Please enter valid numerical values.")
            return

        def publish_thread():
            self.position_pub = roslibpy.Topic(self.client, '/goal_pose', 'geometry_msgs/PoseStamped', queue_size=1)
            message = roslibpy.Message({
                'pose': {
                    'position': {'x': pos_x, 'y': pos_y, 'z': 0.0},
                    'orientation': {'w': tf_quat[0], 'x': tf_quat[1], 'y': tf_quat[2], 'z': tf_quat[3]}
                }
            })
            self.position_pub.publish(message)
            self.messages_text.config(state=tk.NORMAL)
            self.messages_text.insert(tk.END, f"sending: pos_x:{pos_x} pos_y:{pos_y} heading:{float(self.heading.get())}\n")
            self.messages_text.config(state=tk.DISABLED)
            self.messages_text.see(tk.END)
            messagebox.showinfo("Publish Status", "Message published")
        threading.Thread(target=publish_thread).start()

    def toggle_connection(self):
        if self.is_connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        def connect_thread():
            try:
                host = self.ip_address.get()
                port = int(self.port.get())
                self.client = roslibpy.Ros(host=host, port=port)
                self.client.run()
                if self.client.is_connected:
                    self.is_connected = True
                    self.connection_indicator.config(text="Connected", bg="green")
                    self.connect_button.config(text="Disconnect")
                    self.publish_button.config(state=tk.NORMAL)
                    roslibpy.Topic(self.client, '/cmd_vel', 'geometry_msgs/Twist', queue_size=10).subscribe(self.velocity_msg)
                    roslibpy.Topic(self.client, '/odom', 'nav_msgs/Odometry', queue_size=10).subscribe(self.position_msg)
                    messagebox.showinfo("Connection Status", "Connected to ROS successfully!")
                else:
                    messagebox.showerror(
                        "Connection Status", "Failed to connect to ROS.")
            except Exception as e:
                messagebox.showerror("Connection Error", str(e))
        threading.Thread(target=connect_thread).start()

    def disconnect(self):
        def disconnect_thread():
            if self.client.is_connected:
                self.client.close()
                self.is_connected = False
                self.connection_indicator.config(text="Disconnected", bg="red")
                self.connect_button.config(text="Connect")
                self.publish_button.config(state=tk.DISABLED)
                messagebox.showinfo("Connection Status", "Disconnected from ROS.")
        threading.Thread(target=disconnect_thread).start()

    def velocity_msg(self, msg):
        vel_x = round(msg['linear']['x'], 2)
        vel_y = round(msg['linear']['y'], 2)
        vel_z = round(msg['angular']['z'], 2)
        self.speed_data_x.config(text=f"{vel_x} m/s")
        self.speed_data_y.config(text=f"{vel_y} m/s")
        self.speed_data_z.config(text=f"{vel_z} m/s")

    def position_msg(self, msg):
        pos_x = round(msg['pose']['pose']['position']['x'], 2)
        pos_y = round(msg['pose']['pose']['position']['y'], 2)
        ori_w = msg['pose']['pose']['orientation']['w']
        ori_x = msg['pose']['pose']['orientation']['x']
        ori_y = msg['pose']['pose']['orientation']['y']
        ori_z = msg['pose']['pose']['orientation']['z']
        data_z = transformations.euler_from_quaternion([ori_w, ori_x, ori_y, ori_z])
        self.position_data_x.config(text=f"{pos_x} m")
        self.position_data_y.config(text=f"{pos_y} m")
        self.position_data_z.config(text=f"{int(data_z[2] * 180 / math.pi)}°")


if __name__ == "__main__":
    app = App()
    app.mainloop()