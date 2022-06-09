# Capstone
A collection of code related to the scale test bed vehicle dynamics capstone


Current Method:
  - Xbox controller connected to PC via usb wireless adapter
  - PC runs python script which grabs controller values and sends them to ESP_1, which is connected to the PC via usb, over serial
  - ESP_1 uses ESP_NOW to broadcast controller values to ESP_2 mounted on the TestBed. 
  - ESP_2 converts controller inputs to servo signals (electronic speed controller, steering servo)
  
  
ToDo:
- Make communication between ESP_1 and ESP_2 bilateral - potential to ditch onboard SD card, sending useful data back to the PC
- Develop method for defining maneuvers and initiating and cancelling them through the controller (eg. A to start, B to terminate)
- Develop method to calculate system state, deduce body slip angle, attempt control
  - Test throttle cut/steer adjustment control methods
  - Define limits, make on-off control. 
  - Develop emergency stop procedure
    - Controller disconnect -> loss of user control, set servos to neutral
    - More than X milliseconds since last command received over ESP-NOW -> loss of signal, set servos to neutral
    - Controller B-button -> user initiated emergency stop - set servos to neutral
    
    
