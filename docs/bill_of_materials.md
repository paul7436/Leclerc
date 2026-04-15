# Bill of materials

Indicative list of the parts this project was designed around. Any equivalent
part works; the firmware only needs three PWM outputs and one digital input.

| Qty | Part | Example | Notes |
| --- | --- | --- | --- |
| 1 | ESP32-S3 development board | ESP32-S3-DevKitC-1 | USB serial to the host, three servo PWM outputs, arming switch input |
| 1 | Pan servo | MG996R or similar standard servo | Base rotation; pair with a lazy-susan bearing |
| 1 | Tilt servo, high torque | MG996R | Carries the blaster and the camera |
| 1 | Trigger servo | MG90S metal-gear micro servo | Must pull the trigger reliably; use a standard servo for stiff triggers |
| 1 | USB UVC webcam | Any 720p UVC webcam | Mounted below the barrel, connected to the host |
| 1 | Foam dart blaster | Single-trigger, stock-power NERF-style blaster | Never modified for more power |
| 1 | Foam darts | Standard foam darts | No hard tips, no other projectiles |
| 1 | Arming switch, DPST | Key switch, 2 poles, 3 A or more | Pole 1 cuts trigger servo power, pole 2 senses arming |
| 1 | Emergency stop (recommended) | Latching mushroom button, 5 A or more | In series with the whole servo supply |
| 1 | Servo power supply | 5 to 6 V, 5 A or more (UBEC or bench supply) | Separate from the ESP32 USB power |
| 1 | Inline fuse and holder | 5 A blade fuse | On the servo supply positive |
| 1 | Bulk capacitor | 1000 uF, 10 V or more, electrolytic | Across the servo rail near the servos |
| 1 | Indicator LED and resistor (optional) | Red LED, 330 ohm | Lit when the firmware reads ARMED |
| 1 | Pan-tilt bracket | Aluminium bracket or 3D-printed mount | Must carry the blaster weight |
| 1 | Lazy-susan bearing | 100 to 150 mm | Takes the vertical load off the pan servo |
| 1 | Host computer | PC or Raspberry Pi 5 | Runs the Python host; a GPU speeds up YOLO |
| 1 | Eye protection per person | Safety glasses | Worn by everyone in the room |
| - | Wiring | Dupont leads, 18 to 22 AWG for servo power, screw terminals | |
| - | Targets | Cardboard shapes, balloons, small mobile robot | Inert targets only |
