# DronekitTestTask
Test Task Dronekit + Mission Planer

# Running the script:
pip install -r requirements.txt
# Termsnal1 (The root of the project):
Command: dronekit-sitl copter --home=50.450739,30.461242,0,0
# Termsnal2 (The Mavproxy directory):
Command: .\mavproxy.exe --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551
# Connection in Mission Planer (Port: UDP 14551)
# Termsnal3 (The root of the project):
Command: python main.py
