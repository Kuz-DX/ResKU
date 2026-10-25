# cputest 실행 안내

테스트 입력은 rosbag이 아니라 jecs에서 실행 중인 네 카메라의 실시간
`CompressedImage` 토픽이다. 실행과 로그 생성은 jecs에서, 그래프와 요약표 생성은
ROS가 없는 노트북에서 수행한다.

- 로봇 PC 실행 절차: [howtorun_jecs.md](howtorun_jecs.md)
- 노트북 분석 절차: [howtorun_laptop.md](howtorun_laptop.md)

기존 `person_detection` 노드와 테스트 노드를 동시에 실행하지 않는다. 모델 경로와
실제 카메라 토픽은 jecs 환경에 맞게 확인한 뒤 명령에 넣는다.
