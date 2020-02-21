import yaml
import networktables as nt

class NetworkTables:
    def __init__(self, yml_data, yml_path, ip=None):
        self.yml_path = yml_path
        self.yml_data = yml_data
        self.need_reload = False
        self.calib_camera = False
        if ip is None:
            self.ip = self.yml_data['network']['nt-ip']
        print(self.ip)
        nt.NetworkTables.initialize(server=self.ip)
        self.table = nt.NetworkTables.getTable('SmartDashboard')
        camera_table = nt.NetworkTables.getTable("CameraPublisher")
        table = camera_table.getSubTable("Camera")
        table.getEntry("streams").setStringArray(["mjpg:http://10.18.16.142:1180/video_feed"])
    def clearTable(self):
        for item in self.yml_data['network']['variables']:
            self.table.putNumber(item, -1)
    def setupCalib(self):
        def valueChanged(table, key, value, isNew):
            with open(self.yml_path) as f:
                list_doc = yaml.safe_load(f)
            if key == "HMIN":
                list_doc['color']['lower']['H'] = value
                with open(self.yml_path, "w") as f:
                    yaml.dump(list_doc, f)
                self.need_reload = True
            elif key == "SMIN":
                list_doc['color']['lower']['S'] = value
                with open(self.yml_path, "w") as f:
                    yaml.dump(list_doc, f)
                self.need_reload = True
            elif key == "VMIN":
                list_doc['color']['lower']['V'] = value
                with open(self.yml_path, "w") as f:
                    yaml.dump(list_doc, f)
                self.need_reload = True
            elif key == "HMAX":
                list_doc['color']['upper']['H'] = value
                with open(self.yml_path, "w") as f:
                    yaml.dump(list_doc, f)
                self.need_reload = True
            elif key == "SMAX":
                list_doc['color']['upper']['S'] = value
                with open(self.yml_path, "w") as f:
                    yaml.dump(list_doc, f)
                self.need_reload = True
            elif key == "VMAX":
                list_doc['color']['upper']['V'] = value
                with open(self.yml_path, "w") as f:
                    yaml.dump(list_doc, f)
                self.need_reload = True
            elif key == "CalibrationCamera":
                self.calib_camera = value
        lower = self.yml_data['color']['lower']
        upper = self.yml_data['color']['upper']
        calibTable = nt.NetworkTables.getTable('SmartDashboard/Calibration')
        calibTable.putNumber("HMIN", lower['H'])
        calibTable.putNumber("HMAX", upper['H'])
        calibTable.putNumber("SMIN", lower['S'])
        calibTable.putNumber("SMAX", upper['S'])
        calibTable.putNumber("VMIN", lower['V'])
        calibTable.putNumber("VMAX", upper['V'])
        calibTable.putNumber("EXPOSURE", 10)
        calibTable.putBoolean("CalibrationCamera", False)
        calibTable.addEntryListener(valueChanged)

    def putValue(self, key, value):
        if isinstance(value, str):
            self.table.putString(key, value)
            return True
        if isinstance(value, float) or isinstance(value, int):
            self.table.putNumber(key, value)
            return True
        if isinstance(value, bool):
            self.table.putBoolean(key, value)
            return True
        return False
