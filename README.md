# mechatronics_project
Working code is in folder hand_device_traffic_controller/. Sender Arduino Nano gets sensor data from MPU6050, interprets data and send commands via NRF module to Receiver Arduino Uno. Receiver manages between automatic and manual traffic signal modes. While in manual mode, executes commands trasmited from sender Arduino, and controls traffic lights. 
