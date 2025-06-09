# ESP32-to-ESP-NOW-Control
This project uses ESP-NOW communication between two ESP32 boards to control both a steering system and a motor. The sender reads input from a potentiometer (or joystick) and transmits the data wirelessly. The receiver processes the signals to adjust a servo motor for steering and control the speed and direction of a DC motor. The system ensures precise control by mapping the input values to appropriate ranges, with calibration adjustments for accurate steering and smooth motor operation.
If you want to learn more details, please watch the video.
![image](https://github.com/user-attachments/assets/0a1d6194-2771-4c25-b33d-186e1e498fee)
![image](https://github.com/user-attachments/assets/ea1e5d80-c57e-4362-bf68-ec82cac74917)


  NOTE: Because when WiFi is enabled, only ADC1 can be used, so I cut the Lx wire from the joystick and re-soldered it to the output pin of the potentiometer on pin 32.
![image](https://github.com/user-attachments/assets/0818fe40-f2fa-4d9c-9451-d4ba80c9d39a)

For the pre-built setups, I also integrated an autonomous vehicle with image processing using Python into the project.You can see the results in the video Autodriving-pythone.mp4, which also uses a Raspberry Pi.
