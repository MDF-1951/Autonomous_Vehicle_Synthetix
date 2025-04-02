import time
import math
import numpy as np
from mpu6050 import mpu6050
import serial

class PIDController:
    def __init__(self, Kp, Ki, Kd, max_output):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.max_output = max_output
        self.integral = 0
        self.prev_error = 0
        
    def compute(self, setpoint, current_value, dt):
        error = setpoint - current_value
        P = self.Kp * error
        self.integral += error * dt
        I = self.Ki * self.integral
        derivative = (error - self.prev_error) / (dt + 1e-6)
        D = self.Kd * derivative
        
        output = P + I + D
        output = np.clip(output, -self.max_output, self.max_output)
        self.prev_error = error
        return output

class StanleyController:
    def __init__(self, k=0.5, k_soft=1.0, wheelbase=0.1):
        self.k = k  
        self.k_soft = k_soft 
        self.L = wheelbase  
        
    def compute_steering(self, x, y, yaw, v, path):
        if len(path) < 2:
            return 0.0
            
        closest_point, idx = self.find_closest_point(x, y, path)
        dx = closest_point[0] - x
        dy = closest_point[1] - y
        cross_track_error = math.sqrt(dx**2 + dy**2)
        
        if idx < len(path) - 1:
            path_yaw = math.atan2(path[idx+1][1]-path[idx][1], path[idx+1][0]-path[idx][0])
        else:
            path_yaw = math.atan2(path[idx][1]-path[idx-1][1], path[idx][0]-path[idx-1][0]) if idx > 0 else yaw
        
        heading_error = self.normalize_angle(path_yaw - yaw)
        if abs(v) < 0.1:
            return 0.0
            
        steer = heading_error + math.atan2(self.k * cross_track_error, self.k_soft + v)
        return steer
    
    def find_closest_point(self, x, y, path):
        min_dist = float('inf')
        closest_point = path[0]
        closest_index = 0
        for i, (px, py) in enumerate(path):
            dist = (px - x)**2 + (py - y)**2
            if dist < min_dist:
                min_dist = dist
                closest_point = (px, py)
                closest_index = i
        return closest_point, closest_index
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

class MPU6050Interface:
    def __init__(self, address=0x68, accel_scale_factors=None, accel_offsets=None, gyro_offsets=None):
        self.mpu = mpu6050(address)
        self.accel_scale_factors = accel_scale_factors or [1.0, 1.0, 1.0]
        self.accel_offsets = accel_offsets or [0.0, 0.0, 0.0]
        self.gyro_offsets = gyro_offsets or [0.0, 0.0, 0.0]
    
    def get_acceleration(self):
        accel_data = self.mpu.get_accel_data()
        return [
            accel_data['x'] * self.accel_scale_factors[0] + self.accel_offsets[0],
            accel_data['y'] * self.accel_scale_factors[1] + self.accel_offsets[1],
            accel_data['z'] * self.accel_scale_factors[2] + self.accel_offsets[2]
        ]
    
    def get_gyro(self):
        gyro_data = self.mpu.get_gyro_data()
        return [
            gyro_data['x'] - self.gyro_offsets[0],
            gyro_data['y'] - self.gyro_offsets[1],
            gyro_data['z'] - self.gyro_offsets[2]
        ]

class RoverController:
    def __init__(self, imu_sensor):
        self.wheelbase = 0.172
        self.max_steer = math.radians(60)
        self.max_speed = 0.5
        self.waypoint_threshold = 0.1
        self.current_position = np.array([0.0, 0.0])
        self.current_yaw = 0.0
        self.current_speed = 0.0
        self.target_waypoint = None
        self.path = []
        self.current_waypoint_index = 0
        self.waypoints = []
        
        self.steering_controller = StanleyController(
            wheelbase=self.wheelbase,
            k=2.5,
            k_soft=0.2
        )
        
        self.speed_controller = PIDController(
            Kp=1.5,
            Ki=0.1,
            Kd=0.3,
            max_output=1.0
        )
        
        self.imu = imu_sensor
        self.last_update_time = time.time()
        
    def update_state(self):
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        accel = self.imu.get_acceleration()
        gyro = self.imu.get_gyro()
        
        self.current_yaw += gyro[2] * dt
        self.current_yaw = self.normalize_angle(self.current_yaw)
        
        ax_world = accel[0] * math.cos(self.current_yaw) - accel[1] * math.sin(self.current_yaw)
        self.current_speed += ax_world * dt
        
        self.current_position[0] += self.current_speed * math.cos(self.current_yaw) * dt
        self.current_position[1] += self.current_speed * math.sin(self.current_yaw) * dt
        
        return self.current_position.copy(), self.current_yaw, self.current_speed
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def set_waypoints(self, waypoints):
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        if self.waypoints:
            self.target_waypoint = np.array(self.waypoints[0])
            self.path = [self.current_position.tolist(), self.target_waypoint.tolist()]
    
    def update_target_waypoint(self):
        if self.target_waypoint is None and self.waypoints:
            self.target_waypoint = np.array(self.waypoints[0])
            return True
        
        distance = np.linalg.norm(self.target_waypoint - self.current_position)
        if distance < self.waypoint_threshold:
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                self.target_waypoint = np.array(self.waypoints[self.current_waypoint_index])
                self.path = [self.current_position.tolist(), self.target_waypoint.tolist()]
                return True
            else:
                self.target_waypoint = None
                return False
        return True
    
    def compute_controls(self):
        if self.target_waypoint is None:
            return 0.0, 0.0
            
        distance = np.linalg.norm(self.target_waypoint - self.current_position)
        target_speed = self.max_speed
        if distance < 1.0:
            target_speed = max(0.1, self.max_speed * distance)
        
        steer_angle = self.steering_controller.compute_steering(
            self.current_position[0],
            self.current_position[1],
            self.current_yaw,
            self.current_speed,
            self.path
        )
        
        throttle = self.speed_controller.compute(
            target_speed,
            self.current_speed,
            0.1
        )
        
        return np.clip(steer_angle, -self.max_steer, self.max_steer), np.clip(throttle, -1.0, 1.0)
    
    def get_controls(self):
        self.update_state()
        if not self.update_target_waypoint():
            return 0.0, 0.0
        return self.compute_controls()

if __name__ == "__main__":
    # Your calibration values
    CALIBRATION = {
        'accel_scale_factors': [1.09735396086182962, -1.0146302995226297, 0.845845554090055],
        'accel_offsets': [-1.5940553113939344, 0.11586722471198073, 1.154073526039583],
        'gyro_offsets': [-1.3650152671755675, 0.8258717557251948, 1.0169419847328252]
    }
    
    imu = MPU6050Interface(**CALIBRATION)
    controller = RoverController(imu)
    
    controller.set_waypoints([
    [0.25, 0.0], 
    [0.25, 0.25], 
    [0.0, 0.25], 
    [-0.25, 0.0], 
    [0.0, -0.25],
    [0.25, 0.0]  # Close the loop
])
    ])
    
    try:
        while True:
            steer, throttle = controller.get_controls()
            print(f"Pos: {controller.current_position.round(2)} | "
                f"Yaw: {math.degrees(controller.current_yaw):.1f}° | "
                f"Speed: {controller.current_speed:.2f}m/s | "
                f"Steer: {math.degrees(steer):.1f}° | "
                f"Throttle: {throttle:.2f}")
            time.sleep(0.1)
            num1 = steer #servo
            num2 = throttle #dc motors

            # Format the string: "12.34,56.78\n"
            data_out = f"{num1},{num2}\n"

            # Send data
            ser.write(data_out.encode('utf-8'))
            print(f"Sent: {data_out.strip()}")

            # Wait for Arduino response
            data_in = ser.readline().decode('utf-8').strip()
            
            if data_in:
                received_numbers = data_in.split(',')
                if len(received_numbers) == 2:
                    rx_num1 = float(received_numbers[0])
                    rx_num2 = float(received_numbers[1])
                    print(f"Received: {rx_num1}, {rx_num2}")

    except KeyboardInterrupt:
        print("Navigation stopped")
        ser.close()
