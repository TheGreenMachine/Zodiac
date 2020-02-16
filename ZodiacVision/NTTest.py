import networktables as nt
import time

def valueChanged(table, key, value, isNew):
    keys = ["HMIN, EXPOSURE"]
    for k in keys:
        print(value)
nt.NetworkTables.initialize("10.18.16.2")
table = nt.NetworkTables.getTable('SmartDashboard/Calibration')
table.putNumber("HMIN", 0)
table.putNumber("HMAX", 0)
table.putNumber("SMIN", 0)
table.putNumber("SMAX", 0)
table.putNumber("VMIN", 0)
table.putNumber("VMAX", 0)
table.putNumber("EXPOSURE", 10)
table.addEntryListener(valueChanged)

while 1:
    time.sleep(1)
