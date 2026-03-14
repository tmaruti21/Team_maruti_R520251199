import serial.tools.list_ports

print("Searching for serial ports...")
ports = list(serial.tools.list_ports.comports())

if not ports:
    print("No serial ports found! Check your USB cable.")
else:
    print(f"Found {len(ports)} ports:")
    for p in ports:
        print(f"  - {p.device}: {p.description} [{p.hwid}]")
