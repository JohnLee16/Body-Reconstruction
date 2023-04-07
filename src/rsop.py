import open3d as o3d
import json
import pyrealsense2 as rs
import numpy as np
import cv2
import copy
import time
from helpers import *
import os

from util.pclib import passthroughfilter


def scan_body():
    # list realsense devices
    rs_devices = o3d.t.io.RealSenseSensor.list_devices()

    config_filename = './config/rs_config.json'
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



def rs_scan():
    VISUALIZE = True
    
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    # config the IMU
    config.enable_stream(rs.stream.accel) # rs.format.motion_xyz32f, 250
    config.enable_stream(rs.stream.gyro) # rs.format.motion_xyz32f, 200

    pipeline.start(config)
    i=1
    try:
        i =  10
        while True:
    
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            
            depth_frame = frames.get_depth_frame()
            
            color_frame = frames.get_color_frame()
            accel = frames[2].as_motion_frame().get_motion_data()
            gyro = frames[3].as_motion_frame().get_motion_data()
            
            print(accel)
            print(gyro)
            
            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
            profile = frames.get_profile()
            if not depth_frame or not color_frame:
                continue
    
            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            #480*640
            color_image = np.asanyarray(color_frame.get_data())

            key = cv2.waitKey(1)
            
            if i > 0:
                print("type of depth_image:",type(depth_image))
                print("shape of depth_image:",depth_image.shape)

                o3d_color = o3d.geometry.Image(color_image)
                o3d_depth = o3d.geometry.Image(depth_image)
                rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_color, o3d_depth,
                                                                                depth_scale=1000.0,
                                                                                depth_trunc=3.0,
                                                                                convert_rgb_to_intensity=False)

                intrinsics = profile.as_video_stream_profile().get_intrinsics()
                # 转换为open3d中的相机参数
                pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
                    intrinsics.width, intrinsics.height,
                    intrinsics.fx, intrinsics.fy,
                    intrinsics.ppx, intrinsics.ppy
                )

                o3d_result = o3d.geometry.PointCloud.create_from_rgbd_image(
                        rgbd_image,
                        pinhole_camera_intrinsic
                    )

                pcd_pass = passthroughfilter(o3d_result, 'z', 0.01, 1)
                o3d.visualization.draw_geometries([pcd_pass])
                data_name = './data/' + str(i) + '.ply'
                o3d.io.write_point_cloud(data_name, pcd_pass, write_ascii=True)

                # #ply to pcd
                # mesh_ply = o3d.io.read_triangle_mesh("./data/1.ply")
                # mesh_ply.compute_vertex_normals()

                # # V_mesh 为ply网格的顶点坐标序列，shape=(n,3)，这里n为此网格的顶点总数，其实就是浮点型的x,y,z三个浮点值组成的三维坐标
                # V_mesh = np.asarray(mesh_ply.vertices)

                # print("ply info:", mesh_ply)
                # print("ply vertices shape:", V_mesh.shape)
                # o3d.visualization.draw_geometries([mesh_ply], window_name="ply", mesh_show_wireframe=True)

                # # ply -> stl
                # mesh_stl = o3d.geometry.TriangleMesh()
                # mesh_stl.vertices = o3d.utility.Vector3dVector(V_mesh)

                # mesh_stl.compute_vertex_normals()
                # # print("stl info:", mesh_stl)
                # o3d.visualization.draw_geometries([mesh_stl], window_name="stl")
                # o3d.io.write_triangle_mesh("./data/1.stl", mesh_stl)

                # # stl/ply -> pcd
                # pcd = o3d.geometry.PointCloud()
                # pcd.points = o3d.utility.Vector3dVector(V_mesh)
                # print("pcd info:", pcd)
                # print("type of pcd:", type(pcd))
                # # print("shape of pcd:", pcd.shape)
                # o3d.visualization.draw_geometries([pcd], window_name="pcd")

                # # save pcd
                # o3d.io.write_point_cloud("./data/1.pcd", pcd)
                i-=1

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    
            # Stack both images horizontally
            images = np.hstack((color_image, depth_colormap))
    
            # Show images
            # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            # cv2.imshow('RealSense', images)
    
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    
    
    finally:
    
        # Stop streaming
        pipeline.stop()



if __name__ == '__main__':
    rs_scan()