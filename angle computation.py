import time
from mpu6050 import mpu6050
import math as m
import paho.mqtt.client as mqtt
import json
import serial
sensor = mpu6050(0x68)
dt = 0.02
roll,pitch,yaw = 0,0,0
vx,vy,vz=0,0,0
dx,dy,dz=0,0,0
gyro_off_x=-1.3650152671755675
gyro_off_y=0.8258717557251948
gyro_off_z=1.0169419847328252

#accelX_b=-1.007868051256699 #v2, 1.09735396086182962
accelX_b=1.09735396086182962
#accelX_c=0.2912425529665293 #v2, -1.5940553113939344
accelX_c=-1.5940553113939344

#accelY_b=-0.9811819446475775 #v2, -1.0146302995226297
accelY_b=-1.0146302995226297
#accelY_c=0.6125881658240079 #v2, 0.11586722471198073
accelY_c=0.11586722471198073


#accelZ_b=0.8960764531948706 #v2, 0.845845554090055
accelZ_b=0.845845554090055
#accelZ_c=0.823849097173415 #v2, 1.154073526039583
accelZ_c=1.154073526039583
prev_time = time.time()
g=9.9
g=[]
print("hold stationary until mentioned")
for i in range(1000):
    
    a=sensor.get_accel_data()
    ax=a['x']*accelX_b+accelX_c
    ay=a['y']*accelY_b+accelY_c
    az=a['z']*accelZ_b+accelZ_c

    g.append(m.sqrt((ax**2+ay**2+az**2)))
g=sum(g)/1000 #acceleration due to gravity
print(g, "\n free to move")
input('enter to continue: ')
while True:
    gyro = sensor.get_gyro_data()
    accel=sensor.get_accel_data()
    gx=(gyro['x'] - gyro_off_x)
    gy=(gyro['y'] - gyro_off_y)
    gz = (gyro['z'] - gyro_off_z)  # Offset correction

    curr_time = time.time()
    dt = curr_time - prev_time  # Calculate actual time difference
    prev_time = curr_time  # Update previous time

    roll +=gx*dt
    pitch +=gy*dt
    yaw += gz * dt  # Integrate to get angle
    
    grx=g*m.sin(pitch)
    gry=g*m.sin(roll)
    grz=g*m.cos(roll)*m.cos(pitch)
    
    ax= accel['x']*accelX_b+accelX_c-grx
    ay=accel['y']*accelY_b+accelY_c-gry
    az=accel['z']*accelZ_b+accelZ_c-grz

    vx+=ax*dt
    vy+=ay*dt
    vz=az*dt

    dx+=vx*dt
    dy+=vy*dt
    dz+=vz*dt
    


    
    print(roll)
    print(pitch)
    print(yaw,'\n')
    
    def transmit(payload,THINGSBOARD_HOST = "mqtt.thingsboard.cloud", ACCESS_TOKEN = "oqkp22iwsieivtttk53o"):
    # Callback functions
        def on_connect(client, userdata, flags, rc):
            print(f"Connected with result code {rc}")
            if rc == 0:
                print("Successfully connected to ThingsBoard!")
            else:
                print(f"Connection failed with code {rc}")
    
        def on_publish(client, userdata, mid):
            print(f"Message {mid} published successfully")

    # Initialize MQTT client
    #client = mqtt.Client()
    #client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, protocol=mqtt.MQTTv311)

        client.username_pw_set(ACCESS_TOKEN)
        client.on_connect = on_connect
        client.on_publish = on_publish

        try:
        # Connect to ThingsBoard
            client.connect(THINGSBOARD_HOST, 1883, 60)
            client.loop_start()

        # Initialize sensor
        #sensor = mpu6050(0x68)

            #while True:
            '''
            init_ang_x,init_ang_y,init_ang_z=0,0,0
            init_vel_x,init_vel_y,init_vel_z=0,0,0
            init_dist_x,init_dist_y,init_dist_z=0,0,0
            
            accel_x_grav,accel_y_grav,accel_z_grav=calibrated_acceleration()
            accel_x,accel_y,accel_z=acceleration_wo_gravity()
            gyro_x,gyro_y,gyro_z = calibrated_gyroscope()


            payload = {
                #"temperature": sensor.get_temp(),
                "accel_x": accel_data["x"],
                "accel_y": accel_data["y"],
                "accel_z": accel_data["z"],
                "gyro_x": gyro_data["x"],
                "gyro_y": gyro_data["y"],
                "gyro_z": gyro_data["z"]
            }
            '''
        
            # Publish data
            client.publish('v1/devices/me/telemetry', json.dumps(payload), 1)
            #print(f"Published: {payload}")
            time.sleep(0.1)

        except Exception as e:
            print(f"Error: {e}")
        finally:
            client.loop_stop()
            client.disconnect()
    
    payload={"roll": roll, "pitch": pitch, "yaw": yaw, "velocity_x":vx,"velocity_y":vy,"velocity_z":vz,"acceleration_x":ax,"acceleration_y":ay,"acceleration_z":az, "distance_x":dx,"distance_y":dy,"distance_z":dz}
    transmit(payload)
    
    time.sleep(0.005)  # A small sleep to maintain smooth execution

