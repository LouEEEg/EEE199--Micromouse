% Run code once to create Ras-pi camera and BLE objects
rpi = raspi();
cam = cameraboard(rpi, 'Resolution', '1280x720');
cam.ROI = [0.07 0.23 0.83 0.75];

esp = ble("MyESP32");
c_esp = characteristic(esp, "4FAFC201-1FB5-459E-8FCC-C5C9C331914B", "BEB5483E-36E1-4688-B7F5-EA07361B26A8");