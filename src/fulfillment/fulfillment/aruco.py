import cv2
import cv2.aruco as aruco
import numpy as np
import time


class arucoDetector():
    def __init__(self):

        # 정확한 카메라 행렬 및 왜곡 계수는 카메라 보정을 통해 얻어야 함
        self.camera_matrix = np.array([[666.21311517, 0.,    630.12891911],
                                       [0.,    663.99283008, 363.5997193],
                                       [0.,      0.,       1.]], dtype=np.float32)
        # 보정된 값으로 교체 필요
        self.dist_coeffs = np.array(
            [-0.03645762, 0.08774861, -0.00096308, -0.00120847, -0.06589077])

        # Aruco 마커 크기 (단위: 미터)
        self.marker_length = 0.105  # 10cm
        # Aruco Dictionary 및 탐지 파라미터
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_100)
        self.parameters = aruco.DetectorParameters_create()

    def detect(self, frame):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Aruco 마커 탐지
        corners, ids, rejected = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.parameters)

        # 자세와 위치 추정
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

        ret = []
        if ids is not None:

            for i, marker_id in enumerate(ids):
                rvec, tvec = rvecs[i][0], tvecs[i][0]

                # 거리 계산
                distance = np.linalg.norm(tvec)
                # print(
                #     f"ID: {marker_id[0]}, 거리: {distance:.2f}m, tvec: {tvec}, rvec: {rvec}")
                ret.append(
                    {"id": marker_id[0], "corners": corners, "distance": distance, "tvec": tvec, "rvec": rvec})
        return ret


def calculate_angle_between_axes(rvec1, rvec2, axis1="z", axis2="y"):
    # Convert rotation vectors to rotation matrices
    rot_mat1, _ = cv2.Rodrigues(rvec1)
    rot_mat2, _ = cv2.Rodrigues(rvec2)

    # Select the specified axes from rotation matrices
    axis_map = {"x": 0, "y": 1, "z": 2}
    vec1 = rot_mat1[:, axis_map[axis1]]
    vec2 = rot_mat2[:, axis_map[axis2]]

    # Compute the angle between the two vectors
    dot_product = np.dot(vec1, vec2)
    norm_product = np.linalg.norm(vec1) * np.linalg.norm(vec2)
    angle_rad = np.arccos(np.clip(dot_product / norm_product, -1.0, 1.0))

    # Convert angle to degrees
    angle_deg = np.degrees(angle_rad)
    return angle_deg


def calculate_distance_from_marker_to_point(rvec1, tvec1, rvec2, tvec2, p2):
    # Convert rotation vectors to rotation matrices
    rot_mat2, _ = cv2.Rodrigues(rvec2)

    # Transform point p2 from marker m2's local coordinates to the camera coordinate system
    p2_camera = rot_mat2 @ p2 + tvec2.flatten()

    # Camera coordinates of marker m1's origin (tvec1)
    p1_camera = tvec1.flatten()

    # Compute Euclidean distance between p1 and transformed p2
    distance = np.linalg.norm(p2_camera - p1_camera)
    return distance


def calculate_relative_coordinates(rvec1, tvec1, rvec2, tvec2):
    # Convert rotation vectors to rotation matrices
    rot_mat1, _ = cv2.Rodrigues(rvec1)
    rot_mat2, _ = cv2.Rodrigues(rvec2)

    # Compute the relative rotation and translation
    # Rotation of m2 in m1's coordinate system
    relative_rot_mat = rot_mat1.T @ rot_mat2
    # Translation of m2's origin relative to m1
    relative_tvec = rot_mat1.T @ (tvec2 - tvec1)

    ret = []
    for i in range(3):
        ret.append(round(relative_tvec.flatten()[i], 2))
    return ret


def main():

    # 카메라 설정
    cap = cv2.VideoCapture(2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    detector = arucoDetector()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break

        ret = detector.detect(frame)
        rvecs = [[], []]
        tvecs = [[], []]

        def prt(l):
            ret = ""
            for i in l:
                ret += f"{i:.2f} "
            return ret
        if ret:

            # 축 그리기
            for item in ret:
                tvec = item["tvec"]
                rvec = item["rvec"]
                corners = item["corners"]

                id = item["id"]

                cv2.drawFrameAxes(frame, detector.camera_matrix,
                                  detector.dist_coeffs, rvec, tvec, detector.marker_length)

                # 마커 표시
                aruco.drawDetectedMarkers(frame, corners)
                distance = np.linalg.norm(tvec)
                print(
                    f"ID: {id}, 거리: {distance:.2f}m, tvec: {prt(tvec)}, rvec: {prt(rvec)}")

                if id == 0:
                    rvecs[0] = rvec
                    tvecs[0] = tvec
                elif id == 50:
                    rvecs[1] = rvec
                    tvecs[1] = tvec

            if len(rvecs) == 2 and len(rvecs[0]) != len([]) and len(rvecs[1]) != len([]):
                print("-"*20)
                print(
                    f'각도(id0 y축, id50 z축):  \t{calculate_angle_between_axes(rvecs[0], rvecs[1], "y", "z"): .3f}')
                print(
                    f"거리(id0 원점, id50 z+0.4m점):\t{calculate_distance_from_marker_to_point(rvecs[0], tvecs[0], rvecs[1], tvecs[1], np.array([0, 0, 0.4])):.3f}")
                print(
                    f"거리(id0 원점, id50 원점):\t{calculate_distance_from_marker_to_point(rvecs[0], tvecs[0], rvecs[1], tvecs[1], np.array([0, 0, 0.])):.3f}")
                print(
                    f"상대좌표(id0 원점, id50 원점):\t{calculate_relative_coordinates(rvecs[0], tvecs[0], rvecs[1], tvecs[1])}")
        else:
            print("마커를 찾을 수 없습니다.")

        cv2.imshow('Aruco Marker Detection', frame)

        # ESC 키로 종료
        input = cv2.waitKey(1) & 0xFF
        if input == 27:
            break
        elif input == ord('s'):
            cv2.imwrite(str(time.time())+".jpg", frame)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()