import depthai as dai
import json

with dai.Device() as device:
    calib = device.readCalibration()

    def get_camera_info(cam_socket):
        width = 640 # default width
        height = 480 # default height
        intrinsics = calib.getCameraIntrinsics(cam_socket, width, height)
        dist_coeffs = calib.getDistortionCoefficients(cam_socket)

        return {
            "resolution": {"width": width, "height": height},
            "fx": intrinsics[0][0],
            "fy": intrinsics[1][1],
            "cx": intrinsics[0][2],
            "cy": intrinsics[1][2],
            "intrinsic_matrix": intrinsics,
            "distortion_coefficients": dist_coeffs
        }

    intrinsics_data = {
        "CamL": get_camera_info(dai.CameraBoardSocket.CAM_B),
        "CamR": get_camera_info(dai.CameraBoardSocket.CAM_C)
    }

    with open("cameraIntrinsics.json", "w") as f:
        json.dump(intrinsics_data, f, indent=4)
