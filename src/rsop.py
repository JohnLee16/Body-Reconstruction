import open3d as o3d
import json


def scan_body():
    # list realsense devices
    rs_devices = o3d.t.io.RealSenseSensor.list_devices()

    config_filename = ''
    with open(config_filename) as cf:
        rs_cfg = o3d.t.io.RealSenseSensorConfig(json.load(cf))
    rs_cfg.enable_stream(rs.stream.accel)
    rs_cfg.enable_stream(rs.stream.gyro)
    
    rs = o3d.t.io.RealSenseSensor()
    rs.init_sensor(rs_cfg, 0, bag_filename)
    rs.start_capture(True)  # true: start recording with capture
    for fid in range(150):
        im_rgbd = rs.capture_frame(True, True)  # wait for frames and align them
        # process im_rgbd.depth and im_rgbd.color

    rs.stop_capture()

