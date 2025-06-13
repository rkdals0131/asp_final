## /marker_detections 토픽을 받아서 내부 저장소에 저장하고 추후 지정된 경로에 .csv로 출력
# id와 마커의 월드좌표계를 저장하고 csv로 저장. 
# 저장 경로는 multi_tracker/marker_stack.py 파일이 있는 경로에 result라는 디렉토리가 있으므로 multi_tracker/multi_tracker/result 에 저장. 
# rviz2에 탐지한 마커의 위치를 마커어레이로 표시. frame은 map에 표시하고 0.5m 크기 정사각형으로 표시, 현재 보고 있는 마커는 초록색으로, 저장된 마커는 흰색으로 표시
# 마커는 6개이며 0~5까지 고유한 id가 있고 detection3darray에 나와있음. 
"""
❯ ros2 topic echo /marker_detections --once
header:
  stamp:
    sec: 5999
    nanosec: 304000000
  frame_id: map
detections:
- header:
    stamp:
      sec: 5999
      nanosec: 304000000
    frame_id: map
  results:
  - hypothesis:
      class_id: '3'
      score: 1.0
    pose:
      pose:
        position:
          x: -85.23358123216184
          y: 113.21224722556136
          z: 13.577542253986708
        orientation:
          x: 0.13338740214564088
          y: 0.689407082775716
          z: 0.15733498513445907
          w: 0.6943856116166676
      covariance:
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
  bbox:
    center:
      position:
        x: 0.0
        y: 0.0
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    size:
      x: 0.0
      y: 0.0
      z: 0.0
  id: ''
---

"""
