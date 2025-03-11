#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray,String
import ttkbootstrap as ttk
from ttkbootstrap.constants import *
from robot_motion_service.srv import SetPosition, SolveIK
import random
class RobotControlUI:
    def switch_controllers(self):
        import subprocess
        cmd = ["ros2", "control", "switch_controllers", "--activate", "joint_state_broadcaster", "--deactivate", "joint_trajectory_controller", "--activate", "velocity_controller"]
        subprocess.run(cmd)
    def __init__(self, root, ros_node):
        self.style = ttk.Style()
        self.root = root
        self.ros_node = ros_node
        self.velocity = 0.5  # Default velocity
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.current_joint = self.joint_names[0]
        self.control_mode = "jog"  # Modes: 'jog', 'velocity', or 'cartesian'
        self.client = self.ros_node.create_client(SetPosition, '/set_position')
        self.solve_ik_client = self.ros_node.create_client(SolveIK, '/solve_ik')
        
        # Initialize kinematics for cartesian control
        self.kinematics = None
        self.init_kinematics()
        
    def __init__(self, root, ros_node):
        self.style = ttk.Style()
        self.root = root
        self.ros_node = ros_node
        self.velocity = 0.5  # Default velocity
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.current_joint = self.joint_names[0]
        self.control_mode = "jog"  # Modes: 'jog', 'velocity', or 'cartesian'
        self.client = self.ros_node.create_client(SetPosition, '/set_position')
        self.solve_ik_client = self.ros_node.create_client(SolveIK, '/solve_ik')
        
        # Initialize kinematics for cartesian control
        self.kinematics = None
        
        # Default cartesian values - adjusted to be within workspace
        self.cartesian_values = {
            'x': 100.0,
            'y': 0.0,
            'z': 100.0,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0
        }

        self.publisher = self.ros_node.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.control_mode_publisher = self.ros_node.create_publisher(String, '/control_mode/state', 10)  # New publisher

        self.root.geometry("800x600")
        self.root.attributes('-fullscreen', True)

        # Initialize kinematics
        self.init_kinematics()
        
        # Create the UI
        self.create_ui()
        
    def init_kinematics(self):
        """Initialize the kinematics module"""
        try:
            # Get the absolute path to the kinematics.py file
            import os
            import sys
            
            # Print current directory for debugging
            self.ros_node.get_logger().info(f"Current directory: {os.getcwd()}")
            
            # Get the directory of the current script
            script_dir = os.path.dirname(os.path.abspath(__file__))
            self.ros_node.get_logger().info(f"Script directory: {script_dir}")
            
            # Add script directory to Python path
            if script_dir not in sys.path:
                sys.path.append(script_dir)
                
            # Try to import the kinematics module
            kinematics_path = os.path.join(script_dir, "kinematics.py")
            if os.path.exists(kinematics_path):
                self.ros_node.get_logger().info(f"Kinematics file exists at: {kinematics_path}")
                
                # Try direct import
                try:
                    from kinematics import FiboX_Borot
                    self.kinematics = FiboX_Borot()
                    self.ros_node.get_logger().info("Successfully imported kinematics module")
                    return
                except ImportError as e:
                    self.ros_node.get_logger().error(f"Direct import failed: {e}")
                
                # Try importing using importlib
                try:
                    import importlib.util
                    spec = importlib.util.spec_from_file_location("kinematics", kinematics_path)
                    kinematics_module = importlib.util.module_from_spec(spec)
                    spec.loader.exec_module(kinematics_module)
                    self.kinematics = kinematics_module.FiboX_Borot()
                    self.ros_node.get_logger().info("Successfully imported kinematics module using importlib")
                    return
                except Exception as e:
                    self.ros_node.get_logger().error(f"Importlib import failed: {e}")
            else:
                self.ros_node.get_logger().error(f"Kinematics file not found at: {kinematics_path}")
                
            self.ros_node.get_logger().error("Failed to initialize kinematics module")
        except Exception as e:
            self.ros_node.get_logger().error(f"Error initializing kinematics: {e}")
    
    def create_ui(self):
        self.switch_controllers()
        for widget in self.root.winfo_children():
            widget.destroy()
        
        # Create a frame for mode selection buttons
        mode_frame = ttk.Frame(self.root)
        mode_frame.pack(pady=20)
        
        # Create buttons for each mode
        self.jog_mode_button = ttk.Button(mode_frame, text="Jog Mode", 
                                     bootstyle="primary" if self.control_mode == "jog" else "secondary", 
                                     command=lambda: self.set_mode("jog"), 
                                     width=30, padding=30)
        self.jog_mode_button.pack(side="left", padx=10)
        
        self.velocity_mode_button = ttk.Button(mode_frame, text="Velocity Mode", 
                                          bootstyle="primary" if self.control_mode == "velocity" else "secondary", 
                                          command=lambda: self.set_mode("velocity"), 
                                          width=30, padding=30)
        self.velocity_mode_button.pack(side="left", padx=10)
        
        self.cartesian_mode_button = ttk.Button(mode_frame, text="Cartesian Mode", 
                                           bootstyle="primary" if self.control_mode == "cartesian" else "secondary", 
                                           command=lambda: self.set_mode("cartesian"), 
                                           width=30, padding=30)
        self.cartesian_mode_button.pack(side="left", padx=10)

        if self.control_mode == "jog":
            self.create_jog_ui()
        elif self.control_mode == "velocity":
            self.create_velocity_ui()
        else:
            self.create_cartesian_ui()
    
    def create_jog_ui(self):
        ttk.Label(self.root, text="Select Joint", font=("Arial", 36)).pack(pady=20)
        self.joint_selector = ttk.Combobox(self.root, values=self.joint_names, state="readonly", font=("Arial", 44), width=30, postcommand=lambda: self.joint_selector.configure(height=10))
        self.joint_selector.bind('<Up>', lambda e: self.navigate_joint_selection(-1))
        self.joint_selector.bind('<Down>', lambda e: self.navigate_joint_selection(1))
        self.style.configure('TCombobox', font=('Arial', 44))
        self.style.configure('TCombobox.Listbox', font=('Arial', 44))
        self.root.option_add('*TCombobox*Listbox*Font', ('Arial', 44))
        self.root.option_add('*TCombobox*Listbox*width', 30)
        self.root.option_add('*TCombobox*Listbox.font', ('Arial', 44))
        self.joint_selector.option_add('*TCombobox*Listbox*Font', ('Arial', 44))
        self.joint_selector.option_add('*TCombobox*Listbox.font', ('Arial', 44))
        self.joint_selector.pack(pady=20)
        self.joint_selector.set(self.current_joint)

        ttk.Label(self.root, text="Velocity", font=("Arial", 36)).pack(pady=20)
        
        # Create a frame for velocity controls
        velocity_frame = ttk.Frame(self.root)
        velocity_frame.pack(pady=20)
        
        # Create a frame for current velocity and adjustment buttons
        current_frame = ttk.Frame(velocity_frame)
        current_frame.pack(pady=10)
        
        # Add +/- 0.1 buttons next to current velocity display
        ttk.Button(current_frame, text="-0.1", bootstyle="warning", width=15, padding=10,
                  command=lambda: self.adjust_velocity(-0.1)).pack(side="left", padx=10)
        
        # Display current velocity value in the middle
        self.velocity_value_label = ttk.Label(current_frame, text=f"Current: {self.velocity:.2f}", font=("Arial", 36, "bold"))
        self.velocity_value_label.pack(side="left", padx=20)
        
        # Add +0.1 button on the right
        ttk.Button(current_frame, text="+0.1", bootstyle="warning", width=15, padding=10,
                  command=lambda: self.adjust_velocity(0.1)).pack(side="left", padx=10)
        
        # Add slider with large knob
        self.style = ttk.Style()
        self.style.configure('Large.Horizontal.TScale', 
                            sliderlength=3000,  # Very large slider knob
                            thickness=800,      # Very thick slider
                            borderwidth=30,     # Thick border
                            relief='solid', 
                            troughcolor='gray', 
                            gripcount=5)        # More grip lines
        
        # Create slider frame
        slider_frame = ttk.Frame(velocity_frame)
        slider_frame.pack(pady=20)
        
        # Add the slider
        self.speed_slider = ttk.Scale(slider_frame, from_=0.0, to=2.0, orient="horizontal", 
                                     length=900, style='Large.Horizontal.TScale',
                                     command=self.update_velocity_from_slider)
        self.speed_slider.set(self.velocity)
        self.speed_slider.pack(side="top", pady=10)

        jog_frame = ttk.Frame(self.root)
        jog_frame.pack(pady=50)

        self.btn_jog_minus = ttk.Button(jog_frame, text="◀ Jog -", bootstyle="danger", width=100, padding=50)
        self.btn_jog_minus.grid(row=0, column=0, padx=50)
        self.btn_jog_minus.bind("<ButtonPress>", lambda e: self.jog_negative())
        self.btn_jog_minus.bind("<ButtonRelease>", lambda e: self.stop_jog())

        self.btn_jog_plus = ttk.Button(jog_frame, text="Jog + ▶", bootstyle="success", width=100, padding=50)
        self.btn_jog_plus.grid(row=0, column=1, padx=50)
        self.btn_jog_plus.bind("<ButtonPress>", lambda e: self.jog_positive())
        self.btn_jog_plus.bind("<ButtonRelease>", lambda e: self.stop_jog())
    
    def create_velocity_ui(self):
        # Create a simple title
        ttk.Label(self.root, text="Set Joint Angles", font=("Arial", 36)).pack(pady=20)
        
        # Create a floating frame for special actions that won't affect layout
        # This frame will be positioned at the top-right corner
        self.special_frame = ttk.Frame(self.root)
        
        # Use place geometry manager to position the frame absolutely
        self.special_frame.place(relx=1.0, rely=0.0, anchor="ne", x=-20, y=20)
        
        # Toggle button for showing/hiding special actions
        self.show_special_actions = True
        self.toggle_actions_button = ttk.Button(
            self.special_frame, 
            text="Hide Special Actions", 
            bootstyle="secondary", 
            width=40, 
            padding=20, 
            command=self.toggle_special_actions
        )
        self.toggle_actions_button.pack(side="top", pady=5)
        
        # Special action buttons frame
        self.special_buttons_frame = ttk.Frame(self.special_frame)
        self.special_buttons_frame.pack(pady=5)
        
        # Add special action buttons
        self.btn_special_action = ttk.Button(
            self.special_buttons_frame, 
            text="Special Action", 
            bootstyle="warning", 
            width=40, 
            padding=20, 
            command=self.execute_special_action
        )
        self.btn_special_action.pack(side="top", pady=5)
        
        self.btn_special_action_2 = ttk.Button(
            self.special_buttons_frame, 
            text="Special Attack 2", 
            bootstyle="danger", 
            width=40, 
            padding=20, 
            command=self.execute_special_action_2
        )
        self.btn_special_action_2.pack(side="top", pady=5)
        
        self.btn_special_action_3 = ttk.Button(
            self.special_buttons_frame, 
            text="Special Action 3", 
            bootstyle="info", 
            width=40, 
            padding=20, 
            command=self.execute_special_action_3
        )
        self.btn_special_action_3.pack(side="top", pady=5)
        
        # Joint limits (degrees)
        self.joint_limits = {
            'joint1': (-134, 134),
            'joint2': (-90, 0),
            'joint3': (-90, 45),
            'joint4': (-180, 180),
            'joint5': (-180, 180),
            'joint6': (-180, 180)
        }
        
        # Initialize joint values dictionary to store entered values
        self.joint_values = {joint: 0.0 for joint in self.joint_names}
        self.current_joint_index = 0
        
        # Create main frame
        main_frame = ttk.Frame(self.root)
        main_frame.pack(pady=20, fill="both", expand=True)
        
        # Create frame for the large input
        input_frame = ttk.Frame(main_frame)
        input_frame.pack(pady=20)
        
        # Current joint label
        self.current_joint_label = ttk.Label(input_frame, text=f"Joint: {self.joint_names[self.current_joint_index]}", 
                                            font=("Arial", 36, "bold"))
        self.current_joint_label.pack(pady=10)
        
        # Large input field for the current joint with much larger font
        self.current_joint_entry = ttk.Entry(input_frame, width=15, font=("Arial", 48))
        self.current_joint_entry.pack(pady=20)
        
        # Preset buttons frame
        preset_frame = ttk.Frame(input_frame)
        preset_frame.pack(pady=10)
        
        # Create preset buttons for the current joint
        current_joint = self.joint_names[self.current_joint_index]
        joint_min, joint_max = self.joint_limits[current_joint]
        step = (joint_max - joint_min) / 4
        
        for i in range(5):
            angle = int(joint_min + i * step)
            btn = ttk.Button(
                preset_frame, 
                text=f"{angle}°", 
                bootstyle="info", 
                width=40, 
                padding=20,
                command=lambda a=angle: self.set_preset_angle(a)
            )
            btn.pack(side="left", padx=15)
        
        # Add increment/decrement buttons for angle adjustment
        adjust_frame = ttk.Frame(input_frame)
        adjust_frame.pack(pady=10)
        
        # Add +/- 1 degree buttons
        ttk.Button(adjust_frame, text="-1°", bootstyle="secondary", width=20, padding=20,
                  command=lambda: self.adjust_angle(-1)).pack(side="left", padx=10)
        ttk.Button(adjust_frame, text="+1°", bootstyle="secondary", width=20, padding=20,
                  command=lambda: self.adjust_angle(1)).pack(side="left", padx=10)
        
        # Add +/- 5 degree buttons
        ttk.Button(adjust_frame, text="-5°", bootstyle="info", width=20, padding=20,
                  command=lambda: self.adjust_angle(-5)).pack(side="left", padx=10)
        ttk.Button(adjust_frame, text="+5°", bootstyle="info", width=20, padding=20,
                  command=lambda: self.adjust_angle(5)).pack(side="left", padx=10)
        
        # Add +/- 10 degree buttons
        ttk.Button(adjust_frame, text="-10°", bootstyle="primary", width=20, padding=20,
                  command=lambda: self.adjust_angle(-10)).pack(side="left", padx=10)
        ttk.Button(adjust_frame, text="+10°", bootstyle="primary", width=20, padding=20,
                  command=lambda: self.adjust_angle(10)).pack(side="left", padx=10)
        
        # Navigation and action buttons in one frame
        nav_action_frame = ttk.Frame(main_frame)
        nav_action_frame.pack(pady=20)
        
        # Navigation buttons
        nav_frame = ttk.Frame(nav_action_frame)
        nav_frame.pack(side="left", padx=20)
        
        self.prev_button = ttk.Button(nav_frame, text="◀ Previous", bootstyle="secondary", 
                                     width=40, padding=30, command=self.previous_joint)
        self.prev_button.pack(side="left", padx=15)
        
        self.next_button = ttk.Button(nav_frame, text="Next ▶", bootstyle="primary", 
                                     width=40, padding=30, command=self.next_joint)
        self.next_button.pack(side="left", padx=15)
        
        # Action buttons next to navigation buttons
        action_frame = ttk.Frame(nav_action_frame)
        action_frame.pack(side="left", padx=20)
        
        # Main action buttons
        self.btn_send_angles = ttk.Button(
            action_frame, 
            text="Send Goal", 
            bootstyle="success", 
            width=40, 
            padding=30, 
            command=self.send_joint_angles
        )
        self.btn_send_angles.pack(side="left", padx=15)
        
        self.btn_home = ttk.Button(
            action_frame, 
            text="Home", 
            bootstyle="primary", 
            width=40, 
            padding=30, 
            command=self.execute_home
        )
        self.btn_home.pack(side="left", padx=15)
        
        
        # Display saved values in a more compact way
        self.values_display_frame = ttk.Frame(main_frame)
        self.values_display_frame.pack(pady=10, fill="x")
        
        # Create a label for each joint value
        self.joint_value_labels = {}
        joint_values_grid = ttk.Frame(self.values_display_frame)
        joint_values_grid.pack(padx=10, pady=5)
        
        # Create a grid of labels for joint values (3 columns)
        for i, joint in enumerate(self.joint_names):
            row = i // 3
            col = i % 3
            
            # Create a frame for each joint value pair
            joint_frame = ttk.Frame(joint_values_grid)
            joint_frame.grid(row=row, column=col, padx=10, pady=5)
            
            # Add joint name and value
            ttk.Label(joint_frame, text=f"{joint}:", 
                     font=("Arial", 16, "bold")).pack(side="left")
            
            value_label = ttk.Label(joint_frame, text="0.0°", 
                                   font=("Arial", 16))
            value_label.pack(side="left", padx=5)
            
            self.joint_value_labels[joint] = value_label
        
        self.update_values_display()

    def execute_special_action(self):
        """ Execute a predefined sequence of joint positions """
        sequence = [
            [90.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [90.0, -45.0, 20.0, 90.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, -45.0, 20.0, 90.0, 0.0, 0.0]
        ]

        self.execute_sequence(sequence)
    
    def execute_special_action_2(self):
        """ Execute another predefined sequence of joint positions """
        sequence = [
            [-100.0, -45.0, 0.0, 0.0, 0.0, 0.0],
            [-100.0, 0.0, -70.0, 0.0, 0.0, 0.0],
            [0.0, -45.0, 0.0, 0.0, 0.0, 0.0],
            [-90.0, -45.0, 0.0, 0.0, 0.0, 0.0]
        ]

        self.execute_sequence(sequence)
    def execute_special_action_3(self):
        """ Execute a randomized sequence of joint positions within 60% of their limits """
        sequence = []
        for _ in range(10):
            sequence.append([
                random.uniform(self.joint_limits[joint][0] * 0.6, self.joint_limits[joint][1] * 0.6)
                for joint in self.joint_names
            ])
        self.execute_sequence(sequence)
   
    def execute_home(self):
        """ Move the robot to home position """
        sequence = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
        self.execute_sequence(sequence)
    
    def execute_sequence(self, sequence):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.ros_node.get_logger().error("Service /set_position is not available.")
            return

        for angles in sequence:
            request = SetPosition.Request()
            request.target_positions = list(map(float, angles))  # Ensure all values are float
            future = self.client.call_async(request)
            future.add_done_callback(self.handle_service_response)
            self.ros_node.get_logger().info(f"Sent Sequence: {angles}")
            rclpy.spin_until_future_complete(self.ros_node, future)
            
            # Add delay between actions
            self.root.after(2000)
 
    def set_mode(self, mode):
        if mode == self.control_mode:
            return
            
        self.control_mode = mode
        self.publish_control_mode_state()
        self.create_ui()
        
    def toggle_mode(self):
        # For backward compatibility
        if self.control_mode == "jog":
            self.set_mode("velocity")
        elif self.control_mode == "velocity":
            self.set_mode("jog")
        else:
            self.set_mode("jog")
    def publish_control_mode_state(self):
        """ Publish the current control mode to /control_mode/state """
        msg = String()
        msg.data = self.control_mode
        self.control_mode_publisher.publish(msg)
        self.ros_node.get_logger().info(f"Published control mode: {self.control_mode}")
        
    def create_cartesian_ui(self):
        """Create UI for Cartesian coordinate control"""
        # Initialize joint values dictionary to store calculated joint angles
        self.joint_values = {joint: 0.0 for joint in self.joint_names}
        
        # Create a simple title
        ttk.Label(self.root, text="Cartesian Control", font=("Arial", 36)).pack(pady=20)
        
        # Create main frame
        main_frame = ttk.Frame(self.root)
        main_frame.pack(pady=20, fill="both", expand=True)
        
        # Create left frame for coordinate inputs
        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side="left", padx=20, fill="both", expand=True)
        
        # Create right frame for current joint values display
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side="right", padx=20, fill="both", expand=True)
        
        # Create coordinate input fields
        coord_frame = ttk.Frame(left_frame)
        coord_frame.pack(pady=20, fill="x")
        
        # Dictionary to store entry widgets
        self.cartesian_entries = {}
        
        # Create input fields for X, Y, Z, Roll, Pitch, Yaw
        cartesian_params = [
            ('x', 'X (mm)'),
            ('y', 'Y (mm)'),
            ('z', 'Z (mm)'),
            ('roll', 'Roll (deg)'),
            ('pitch', 'Pitch (deg)'),
            ('yaw', 'Yaw (deg)')
        ]
        
        for i, (param, label) in enumerate(cartesian_params):
            param_frame = ttk.Frame(coord_frame)
            param_frame.pack(pady=10, fill="x")
            
            ttk.Label(param_frame, text=label, font=("Arial", 24)).pack(side="left", padx=10)
            
            entry = ttk.Entry(param_frame, width=10, font=("Arial", 24))
            entry.pack(side="left", padx=10)
            entry.insert(0, str(self.cartesian_values[param]))
            
            # Add buttons for adjusting values
            btn_frame = ttk.Frame(param_frame)
            btn_frame.pack(side="left", padx=10)
            
            # Determine step size based on parameter
            step = 10.0 if param in ['x', 'y', 'z'] else 5.0
            
            ttk.Button(btn_frame, text=f"-{step}", bootstyle="warning", width=8, padding=10,
                      command=lambda p=param, s=-step: self.adjust_cartesian(p, s)).pack(side="left", padx=5)
            
            ttk.Button(btn_frame, text=f"+{step}", bootstyle="success", width=8, padding=10,
                      command=lambda p=param, s=step: self.adjust_cartesian(p, s)).pack(side="left", padx=5)
            
            self.cartesian_entries[param] = entry
        
        # Add buttons for sending commands
        btn_frame = ttk.Frame(left_frame)
        btn_frame.pack(pady=20)
        
        ttk.Button(btn_frame, text="Send to Robot", bootstyle="success", width=30, padding=30,
                  command=self.send_cartesian_command).pack(side="left", padx=20)
        
        ttk.Button(btn_frame, text="Reset Values", bootstyle="warning", width=30, padding=30,
                  command=self.reset_cartesian_values).pack(side="left", padx=20)
        
        # Display current joint values in right frame
        ttk.Label(right_frame, text="Current Joint Values", font=("Arial", 24, "bold")).pack(pady=10)
        
        self.joint_values_frame = ttk.Frame(right_frame)
        self.joint_values_frame.pack(pady=10, fill="x")
        
        # Create labels for joint values
        self.joint_value_labels = {}
        for i, joint in enumerate(self.joint_names):
            joint_frame = ttk.Frame(self.joint_values_frame)
            joint_frame.pack(pady=5, fill="x")
            
            ttk.Label(joint_frame, text=f"{joint}:", font=("Arial", 18, "bold")).pack(side="left", padx=10)
            
            value_label = ttk.Label(joint_frame, text="0.0°", font=("Arial", 18))
            value_label.pack(side="left", padx=5)
            
            self.joint_value_labels[joint] = value_label
 
    def send_velocity(self, vel):
        joint_index = self.joint_names.index(self.joint_selector.get())
        velocities = [0.0] * len(self.joint_names)
        velocities[joint_index] = vel
        msg = Float64MultiArray()
        msg.data = velocities
        self.publisher.publish(msg)
        self.ros_node.get_logger().info(f"Publishing: {velocities}")
    
    def jog_positive(self):
        velocity = self.speed_slider.get()
        self.send_velocity(velocity)
    
    def jog_negative(self):
        velocity = -self.speed_slider.get()
        self.send_velocity(velocity)
    
    def stop_jog(self):
        self.send_velocity(0.0)
    
    def set_preset_angle(self, angle):
        """Set a preset angle value in the current entry field"""
        self.current_joint_entry.delete(0, 'end')
        self.current_joint_entry.insert(0, str(angle))
        self.save_current_joint_value()
    
    def next_joint(self):
        """Move to the next joint"""
        self.save_current_joint_value()
        self.current_joint_index = (self.current_joint_index + 1) % len(self.joint_names)
        self.update_current_joint_display()
    
    def previous_joint(self):
        """Move to the previous joint"""
        self.save_current_joint_value()
        self.current_joint_index = (self.current_joint_index - 1) % len(self.joint_names)
        self.update_current_joint_display()
    
    def save_current_joint_value(self):
        """Save the current joint value"""
        try:
            value = float(self.current_joint_entry.get())
            self.joint_values[self.joint_names[self.current_joint_index]] = value
            self.update_values_display()
        except ValueError:
            # If the entry is empty or invalid, don't update
            pass
    
    def update_current_joint_display(self):
        """Update the display for the current joint"""
        current_joint = self.joint_names[self.current_joint_index]
        self.current_joint_label.config(text=f"Joint: {current_joint}")
        
        # Update entry with saved value
        self.current_joint_entry.delete(0, 'end')
        self.current_joint_entry.insert(0, str(self.joint_values[current_joint]))
        
        # Update preset buttons
        self.update_preset_buttons()
    
    def update_preset_buttons(self):
        """Update preset buttons for the current joint"""
        # Clear existing buttons in preset frame
        for widget in self.root.nametowidget(self.current_joint_entry.winfo_parent()).winfo_children()[2].winfo_children():
            widget.destroy()
        
        # Get current joint
        current_joint = self.joint_names[self.current_joint_index]
        joint_min, joint_max = self.joint_limits[current_joint]
        
        # Calculate step size for 5 buttons
        step = (joint_max - joint_min) / 4
        
        # Create preset buttons
        preset_frame = self.root.nametowidget(self.current_joint_entry.winfo_parent()).winfo_children()[2]
        for i in range(5):
            angle = int(joint_min + i * step)
            btn = ttk.Button(
                preset_frame, 
                text=f"{angle}°", 
                bootstyle="info", 
                width=40, 
                padding=20,
                command=lambda a=angle: self.set_preset_angle(a)
            )
            btn.pack(side="left", padx=15)
    
    def update_values_display(self):
        """Update the display of saved joint values"""
        for joint, value in self.joint_values.items():
            self.joint_value_labels[joint].config(text=f"{value}°")
    
    def adjust_angle(self, amount):
        """Adjust the current angle by the specified amount"""
        try:
            current_value = float(self.current_joint_entry.get())
            new_value = current_value + amount
            self.current_joint_entry.delete(0, 'end')
            self.current_joint_entry.insert(0, str(new_value))
            self.save_current_joint_value()
        except ValueError:
            # If the entry is empty or invalid, start from 0
            self.current_joint_entry.delete(0, 'end')
            self.current_joint_entry.insert(0, str(amount))
            self.save_current_joint_value()
    
    def toggle_special_actions(self):
        """Toggle visibility of special action buttons"""
        self.show_special_actions = not self.show_special_actions
        
        if self.show_special_actions:
            self.special_buttons_frame.pack(pady=5)
            self.toggle_actions_button.config(text="Hide Special Actions")
        else:
            self.special_buttons_frame.pack_forget()
            self.toggle_actions_button.config(text="Show Special Actions")
    
    def send_joint_angles(self):
        # Save the current joint value before sending (only in velocity mode)
        if self.control_mode == "velocity" and hasattr(self, 'current_joint_entry'):
            self.save_current_joint_value()
        
        # Get all joint values
        angles = [self.joint_values[joint] for joint in self.joint_names]
        
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.ros_node.get_logger().error("Service /set_position is not available.")
            return
        
        request = SetPosition.Request()
        request.target_positions = angles
        
        future = self.client.call_async(request)
        future.add_done_callback(self.handle_service_response)
    
    def handle_service_response(self, future):
        try:
            response = future.result()
            self.ros_node.get_logger().info(f"Service Response: {response}")
        except Exception as e:
            self.ros_node.get_logger().error(f"Service call failed: {str(e)}")
            
    def adjust_cartesian(self, param, step):
        """Adjust cartesian parameter by the specified step"""
        try:
            current_value = float(self.cartesian_entries[param].get())
            new_value = current_value + step
            
            self.cartesian_entries[param].delete(0, 'end')
            self.cartesian_entries[param].insert(0, str(new_value))
            
            self.cartesian_values[param] = new_value
        except ValueError:
            # If the entry is empty or invalid, reset to default
            self.cartesian_entries[param].delete(0, 'end')
            self.cartesian_entries[param].insert(0, str(self.cartesian_values[param]))
    
    def reset_cartesian_values(self):
        """Reset cartesian values to defaults"""
        default_values = {
            'x': 100.0,
            'y': 0.0,
            'z': 100.0,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0
        }
        
        for param, value in default_values.items():
            self.cartesian_entries[param].delete(0, 'end')
            self.cartesian_entries[param].insert(0, str(value))
            self.cartesian_values[param] = value
    
    def send_cartesian_command(self):
        """Convert cartesian coordinates to joint angles using the SolveIK service and send to robot"""
        # Get values from entry fields
        try:
            x = float(self.cartesian_entries['x'].get())
            y = float(self.cartesian_entries['y'].get())
            z = float(self.cartesian_entries['z'].get())
            roll = float(self.cartesian_entries['roll'].get())
            pitch = float(self.cartesian_entries['pitch'].get())
            yaw = float(self.cartesian_entries['yaw'].get())
            
            # Update stored values
            self.cartesian_values['x'] = x
            self.cartesian_values['y'] = y
            self.cartesian_values['z'] = z
            self.cartesian_values['roll'] = roll
            self.cartesian_values['pitch'] = pitch
            self.cartesian_values['yaw'] = yaw
            
            self.ros_node.get_logger().info(f"Calling SolveIK service for: x={x}, y={y}, z={z}, rx={roll}, ry={pitch}, rz={yaw}")
            
            # Check if service is available
            if not self.solve_ik_client.wait_for_service(timeout_sec=1.0):
                self.ros_node.get_logger().error("Service /solve_ik is not available.")
                return
            
            # Create and send request to SolveIK service
            request = SolveIK.Request()
            request.x = x
            request.y = y
            request.z = z
            request.rx = roll
            request.ry = pitch
            request.rz = yaw
            
            future = self.solve_ik_client.call_async(request)
            
            # Add callback to handle the response
            future.add_done_callback(self.handle_solve_ik_response)
            
            # Wait for the response
            rclpy.spin_until_future_complete(self.ros_node, future)
                
        except ValueError as e:
            self.ros_node.get_logger().error(f"Invalid input: {e}")
        except Exception as e:
            self.ros_node.get_logger().error(f"Error sending cartesian command: {str(e)}")
            import traceback
            self.ros_node.get_logger().error(traceback.format_exc())
    
    def handle_solve_ik_response(self, future):
        """Handle the response from the SolveIK service"""
        try:
            response = future.result()
            self.ros_node.get_logger().info(f"SolveIK Response: success={response.success}, message={response.message}")
            
            if response.success:
                # Convert radians to degrees if needed
                joint_angles = response.joint_angles
                
                # Check if angles are in radians (assuming values > 3.14 are degrees)
                if all(abs(angle) <= math.pi for angle in joint_angles):
                    joint_angles_deg = [math.degrees(angle) for angle in joint_angles]
                    self.ros_node.get_logger().info("Converting radians to degrees")
                else:
                    joint_angles_deg = joint_angles
                
                self.ros_node.get_logger().info(f"Joint angles (deg): {joint_angles_deg}")
                
                # Store the calculated joint angles in the joint_values dictionary
                for i, joint in enumerate(self.joint_names):
                    if i < len(joint_angles_deg):
                        self.joint_values[joint] = joint_angles_deg[i]
                
                # Update joint value display
                for i, joint in enumerate(self.joint_names):
                    if i < len(joint_angles_deg):
                        self.joint_value_labels[joint].config(text=f"{joint_angles_deg[i]:.2f}°")
                
                # Use the same method as in velocity mode to send the joint angles
                self.send_joint_angles()
            else:
                self.ros_node.get_logger().error(f"IK solution failed: {response.message}")
                
        except Exception as e:
            self.ros_node.get_logger().error(f"Error handling SolveIK response: {str(e)}")
            import traceback
            self.ros_node.get_logger().error(traceback.format_exc())
    
    def navigate_joint_selection(self, direction):
        """Navigate joint selection with keyboard"""
        current_index = self.joint_names.index(self.joint_selector.get())
        new_index = (current_index + direction) % len(self.joint_names)
        self.joint_selector.set(self.joint_names[new_index])
    
    def update_velocity_from_slider(self, value):
        """Update velocity value from slider"""
        try:
            self.velocity = float(value)
            self.velocity_value_label.config(text=f"Current: {self.velocity:.2f}")
        except ValueError:
            pass
    
    def adjust_velocity(self, amount):
        """Adjust velocity by the specified amount"""
        new_velocity = max(0.0, min(2.0, self.velocity + amount))
        self.velocity = new_velocity
        self.speed_slider.set(new_velocity)
        self.velocity_value_label.config(text=f"Current: {self.velocity:.2f}")


def main():
    rclpy.init()
    node = Node("robot_control_ui")
    root = ttk.Window(themename="superhero")
    root.title("Robot Control UI")
    app = RobotControlUI(root, node)

    def ros_spin():
        rclpy.spin_once(node, timeout_sec=0.1)
        root.after(100, ros_spin)
    
    root.after(100, ros_spin)
    root.mainloop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
