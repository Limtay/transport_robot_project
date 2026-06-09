#!/bin/bash
# 노트북 코드를 Orin으로 쏘는 스크립트
echo "Orin으로 코드를 전송합니다..."
rsync -avz --delete ~/tp_ws/orin_ws/src/orin_firmware_bridge/ swarm@10.251.24.214:~/orin_ws/src/orin_firmware_bridge
echo "전송 완료! SSH로 접속해서 빌드하세요: ssh swarm@10.251.24.214 / 192.168.55.1"