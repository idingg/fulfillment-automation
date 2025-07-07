# Vision AI를 이용한 Fulﬁllment Automation

이 프로젝트는 YOLOv8 객체 탐지와 ArUco 마커 인식을 결합한 Vision AI와 Manipulator와 AMR을 이용한 물류 자동화(Fulfillment Automation) 시스템입니다. 컨베이어 벨트, 로봇 매니퓰레이터, Vision AI를 활용해 물류 현장의 자동 분류, 추적, 적재, 이송 등의 작업을 수행할 수 있도록 설계되었습니다. 본 시스템은 물류, 스마트 팩토리 등 다양한 분야의 자동화 솔루션에 적용할 수 있습니다.

## 주요 기능
### 1. 실시간 객체 탐지 및 추적
- YOLOv8 기반의 Vision AI로 다양한 물체를 실시간 탐지
- ArUco 마커를 이용해 일반 RGB 카메라로 개별 물체의 고유 식별 및 3차원 위치 인식

### 2. 자동 분류 및 이송
- 명령을 통해 물체를 인식 및 컨베이어를 통해 지정 위치로 이송
- 로봇 매니퓰레이터와 연동하여 픽 앤 플레이스(Pick & Place) 작업 수행

### 3. 작업 관리 및 시각화
- 작업 진행 중 일시정지 또는 완전정지 및 재시작 가능
- 컨베이어 등 제어 불가 등 이상 감지 시 관리자 알림
- 각 작업의 이력 및 상태를 실시간으로 기록 및 모니터링
- OpenCV, PyQt 기반 GUI로 작업 현황 시각화

## 사용 기술
- 로봇 프레임워크: <img src="https://img.shields.io/badge/ROS2-22314E?style=for-the-badge&logo=ros&logoColor=white">
- 프로그래밍 언어: <img src="https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white">
- GUI 프레임워크: <img src="https://img.shields.io/badge/PyQt-41CD52?style=for-the-badge&logo=qt&logoColor=white">
- 컴퓨터 비젼 : <img src="https://img.shields.io/badge/-YOLO-FFCC00?style=for-the-badge&logo=yolo&logoColor=white"><img src="https://img.shields.io/badge/ArUco-FFFFFF?style=for-the-badge&logo=aruco&logoColor=white">


## 활용 분야
- 스마트 물류센터 자동화
- 생산/포장 라인 자동 분류 및 적재

## 향후 개선 방향
- 다양한 객체 탐지 알고리즘 및 마커 지원
- 다중 로봇 협업 및 작업 최적화
- 클라우드 기반 데이터 연동 및 분석

## 시연 이미지 및 영상
![ArUco marker 인식](https://github.com/idingg/fulfillment-automation/blob/main/images/1.png?raw=true)
![YOLOv8 객체 학습](https://github.com/idingg/fulfillment-automation/blob/main/images/2.png?raw=true)
![ArUco 마커 인식 및 거리측정](https://github.com/idingg/fulfillment-automation/blob/main/images/3.png?raw=true)
![시스템 흐름 제어](https://github.com/idingg/fulfillment-automation/blob/main/images/4.png?raw=true)
![이상 감지 알림](https://github.com/idingg/fulfillment-automation/blob/main/images/5.png?raw=true)

## 시연 영상