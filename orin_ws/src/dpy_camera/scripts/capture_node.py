#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
카메라 캡처 노드.

명령 예시:
    # num_shots 파라미터 값만큼 캡처
    ros2 service call /dpy_camera/capture std_srvs/srv/Trigger {}

    # 한 번에 찍을 장수를 바꾸고 싶으면
    ros2 param set /dpy_camera num_shots 5

    # 3장을 찍었다면 장별로 따로 볼 수 있다 (**1-base** — 찍은 순서 그대로 1,2,3)
    ros2 topic echo /dpy_camera/image_raw_1 --once
    ros2 topic echo /dpy_camera/image_raw_2 --once
    ros2 topic echo /dpy_camera/image_raw_3 --once

`~/image_raw` 는 depth=1 래치라 **버스트의 마지막 장만** 남는다. 몇 장을 찍었든
전부 확인하려면 `~/image_raw_<N>` (N=1,2,3,...) 을 봐야 한다 — 각각 depth=1 래치라
그 번호의 "가장 최근 촬영 결과"가 항상 남아 있다.

⚠ **번호는 1부터다** (2026-08-13 변경, 종전 0-base). RViz2 에서 Image 디스플레이
하나만 띄워 두고 토픽 이름 끝의 숫자만 `_1 → _2 → _3` 으로 바꿔 가며 버스트를
훑어보게 하려는 것 — "몇 장 찍었나"(num_shots=3)와 "몇 번을 보면 되나"(1~3)가
어긋나지 않아야 헷갈리지 않는다.

번호 토픽은 **매 호출의 num_shots 에 맞춰 정리된다** — 예를 들어 4장 찍은 뒤
num_shots=3 으로 다시 찍으면 `image_raw_4` 는 그 호출 시작 시점에 아예 없어진다
(낡은 4번째 사진이 다음 버스트의 일부처럼 남아 있지 않도록).
"""

import glob
import os
import stat
import time
from datetime import datetime

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
)


class CaptureNode(Node):
    def __init__(self):
        super().__init__('dpy_camera')

        # --- 파라미터 ---
        # 촬영에 쓸 장치. **여기 지정한 것만 연다** — 실패해도 다른 `/dev/video*` 를
        # 뒤지지 않는다 (`_resolve_device` 참조). 숫자를 주면 `/dev/video<N>` 로 해석한다.
        # 번호는 USB 열거 순서에 따라 바뀌므로 `/dev/v4l/by-id/...` 권장.
        self.declare_parameter('video_device', '/dev/video6')
        self.declare_parameter('frame_id', 'dpy_camera')
        self.declare_parameter('image_width', 1920)
        self.declare_parameter('image_height', 1080)
        # 한 번 명령에 찍을 장수
        self.declare_parameter('num_shots', 1)
        # 캡처 직전 워밍업 '시간'(초). 카메라가 막 열린 직후 자동노출(AE)/
        # 화이트밸런스(AWB)/오토포커스(AF)가 수렴할 때까지 이 시간 동안 프레임을
        # 계속 읽고 버린다. 프레임 수가 아닌 시간 기준이라 fps 변동에 안정적이다.
        self.declare_parameter('warmup_sec', 1.0)
        # 연속 촬영 시 프레임 간 간격(초)
        self.declare_parameter('shot_interval', 0.0)
        # MJPEG 으로 받을지 여부 (USB 대역폭/디코딩 부하에 유리)
        self.declare_parameter('use_mjpg', True)
        # 파일 저장 여부 및 저장 경로 (빈 문자열이면 저장 안 함)
        self.declare_parameter('save_dir', '')
        # 저장 파일 보관 개수 — 촬영이 끝날 때마다 **가장 최근 N장만 남기고** 오래된 것을
        # 지운다. 0(또는 음수) 이면 무제한(정리 안 함).
        #
        # ⚠ 버스트 단위가 아니라 **장(파일) 단위**다. 3장씩 찍으면서 N=10 이면 마지막
        #   3~4 버스트가 섞여 남는다. 버스트 경계를 맞추고 싶으면 num_shots 의 배수로 준다
        #   (예: 3장 × 최근 5회 = 15).
        # ⚠ 지우는 대상은 이 노드가 쓴 `shot_*.jpg` 패턴뿐이다 — save_dir 에 사람이 둔
        #   다른 파일은 건드리지 않는다 (경로를 잘못 줬을 때의 사고 방지).
        self.declare_parameter('max_saved_shots', 3)
        # 토픽 발행 여부
        self.declare_parameter('publish_image', True)

        latched_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # ← 래치 핵심
        )       
        self._latched_qos = latched_qos   # 인덱스 게시자를 나중에 만들 때 재사용
        self.pub = self.create_publisher(Image, '~/image_raw', latched_qos)
        # 장별 번호 토픽 (`~/image_raw_1`, `_2`, ... — **1-base**) — `~/image_raw` 는
        # depth=1 이라 여러 장을 찍으면 마지막 장만 남는다. 장마다 별도 래치 토픽을 두면
        # 버스트 전체를 나중에도(구독자가 캡처 이후에 붙어도) 장별로 확인할 수 있다.
        # num_shots 가 늘어날 수 있으므로 처음 쓰일 때 게시자를 만든다(lazy).
        self._pub_indexed = {}

        # 서비스: 호출되면 num_shots 장 캡처
        self.srv = self.create_service(Trigger, '~/capture', self.on_capture)

        self.get_logger().info(
            "on-demand 캡처 노드 준비 완료. "
        )

    # ------------------------------------------------------------------
    def on_capture(self, request, response):
        device = self.get_parameter('video_device').value
        width = self.get_parameter('image_width').value
        height = self.get_parameter('image_height').value
        num_shots = self.get_parameter('num_shots').value
        warmup_sec = self.get_parameter('warmup_sec').value
        interval = self.get_parameter('shot_interval').value
        use_mjpg = self.get_parameter('use_mjpg').value
        save_dir = self.get_parameter('save_dir').value
        publish_image = self.get_parameter('publish_image').value
        max_saved = self.get_parameter('max_saved_shots').value

        self.get_logger().info(f'캡처 시작: {num_shots}장 @ {device}')

        # ⚠ num_shots 가 이전 호출보다 줄면, 그보다 큰 번호의 토픽(`image_raw_N`)이
        # **낡은 사진을 래치한 채로 계속 남는다.** (예: 4장 찍고 → num_shots=3 으로 다시
        # 찍으면 image_raw_4 는 4장짜리 버스트의 옛날 사진을 계속 물고 있어서, 이번 3장
        # 버스트의 일부처럼 오인될 수 있다.) 이번 호출의 의도(num_shots)보다 큰 번호는
        # 게시자를 아예 정리해 `ros2 topic list` 에서도 사라지게 한다.
        # 번호가 1-base 이므로 남길 범위는 1..num_shots — 경계는 `>` 다 (`>=` 아님).
        if publish_image:
            for shot_no in list(self._pub_indexed.keys()):
                if shot_no > max(1, num_shots):
                    self.destroy_publisher(self._pub_indexed.pop(shot_no))

        # ⚠ **지정한 장치 하나만 연다 — 다른 번호로 대체 시도하지 않는다.**
        #   (2026-08-13 사용자 요구) 실제로 `/dev/video2` 로 설정된 채 내장 웹캠이 찍히고
        #   있었는데 아무도 몰랐다. "어떻게든 찍히는" 것보다 **안 찍히고 왜 안 되는지
        #   말해 주는 것**이 낫다.
        cap_source, err = self._resolve_device(device)
        if err:
            self.get_logger().error(err)
            response.success = False
            response.message = err
            return response

        cap = cv2.VideoCapture(cap_source, cv2.CAP_V4L2)
        if not cap.isOpened():
            # 여기서 다른 장치를 찾아보지 않는다. 대부분 이미 다른 프로세스가 잡고 있는
            # 경우다 (dpy_camera 를 두 번 띄웠거나, rqt/guvcview 가 열어 둔 상태).
            msg = (f'카메라를 열 수 없습니다: {cap_source} '
                   f'(다른 프로세스가 사용 중이거나 캡처 불가 노드일 수 있다. '
                   f'`fuser -v {cap_source}` 로 점유 확인) — 다른 장치로 대체하지 않는다')
            self.get_logger().error(msg)
            response.success = False
            response.message = msg
            return response

        try:
            if use_mjpg:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            # 버퍼를 최소화해 최신 프레임을 받도록 시도(드라이버 따라 무시될 수 있음)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            # 워밍업: 지정한 시간 동안 프레임을 계속 읽고 버려 AE/AWB/AF 가
            # 수렴할 시간을 확보한다. (프레임 수가 아닌 시간 기준)
            warmup_end = time.monotonic() + max(0.0, warmup_sec)
            warmup_count = 0
            while time.monotonic() < warmup_end:
                ok, _ = cap.read()
                if not ok:
                    # 읽기가 계속 실패하면 busy-loop 방지를 위해 잠깐 쉼
                    time.sleep(0.01)
                warmup_count += 1
            self.get_logger().info(
                f'워밍업 {warmup_sec:.2f}s 동안 {warmup_count}프레임 버림'
            )

            captured = 0
            saved_paths = []
            for i in range(max(1, num_shots)):
                ok, frame = cap.read()
                if not ok or frame is None:
                    self.get_logger().warn(f'{i + 1}번째 프레임 읽기 실패')
                    continue

                stamp = self.get_clock().now().to_msg()

                shot_no = i + 1   # 토픽/frame_id 번호는 1-base (찍은 순서 그대로)

                if publish_image:
                    img_msg = self._to_image_msg(frame, stamp, shot_no=shot_no)
                    # 장당 1회 발행이면 충분하다 — 소비자(RViz2 설정 `dpy_camera.rviz`)가
                    # RELIABLE + TRANSIENT_LOCAL 로 붙어서 래치된 사진을 언제든 가져간다.
                    # (2026-08-13 잠시 rqt 대응으로 반복 발행을 넣었다가 되돌렸다 —
                    #  촬영이 4.5초 길어지는데 rqt 는 미리 켜 둔 경우만 해결됐다.
                    #  경위와 재도입 방법은 CAMERA_ACTION.md §15.)
                    self.pub.publish(img_msg)                          # 마지막 장 = 최신값
                    self._indexed_publisher(shot_no).publish(img_msg)  # 장별 번호 토픽

                if save_dir:
                    p = self._save(frame, save_dir, shot_no)
                    if p:
                        saved_paths.append(p)

                captured += 1
                if interval > 0 and i < num_shots - 1:
                    time.sleep(interval)

        finally:
            cap.release()

        # 보관 정리는 **버스트가 끝난 뒤 한 번만** 한다. 장마다 돌리면 같은 버스트를
        # 찍는 도중에 그 버스트의 앞 장을 지우는 순서가 되어, 실패한 장이 섞이면
        # 남는 장수가 들쭉날쭉해진다.
        pruned = 0
        if save_dir and max_saved and max_saved > 0:
            pruned = self._prune(save_dir, max_saved)

        msg = f'{captured}/{num_shots}장 캡처 완료'
        if saved_paths:
            msg += f' | 저장 {len(saved_paths)}장 → {save_dir}'
            if pruned:
                msg += f' (오래된 {pruned}장 삭제, 최근 {max_saved}장 유지)'
        self.get_logger().info(msg)
        response.success = captured > 0
        response.message = msg
        return response

    # ------------------------------------------------------------------
    def _resolve_device(self, device):
        """`video_device` 를 **정확히 그 장치 하나**로 확정한다.

        반환 `(경로, 오류메시지)` — 오류메시지가 있으면 촬영을 시작하지 않는다.

        ## 왜 열기 전에 검사하는가

        `cv2.VideoCapture` 는 실패해도 예외 없이 `isOpened()==False` 만 준다. 없는
        경로인지, 권한이 없는지, 이미 점유 중인지 구분이 안 돼 원인 추적이 오래 걸린다.
        여기서 미리 갈라 두면 로그만 보고 바로 안다.
        """
        # 숫자(또는 숫자 문자열) → /dev/videoN 으로 **명시적** 변환
        if isinstance(device, int):
            device = f'/dev/video{device}'
        elif isinstance(device, str) and device.isdigit():
            device = f'/dev/video{device}'

        if not isinstance(device, str) or not device:
            return None, f'video_device 파라미터가 비어 있거나 형식이 잘못됐다: {device!r}'

        # by-id/by-path 는 심볼릭 링크다 — 실제 노드로 풀어 로그에 남긴다.
        real = os.path.realpath(device)
        if not os.path.exists(real):
            return None, (f'video_device 가 존재하지 않는다: {device}'
                          + (f' → {real}' if real != device else '')
                          + '. 카메라 연결과 YAML 의 video_device 를 확인할 것 '
                            '(다른 번호로 대체 시도하지 않는다)')
        if not stat.S_ISCHR(os.stat(real).st_mode):
            return None, f'video_device 가 문자 장치가 아니다: {device} → {real}'
        if not os.access(real, os.R_OK | os.W_OK):
            return None, (f'video_device 접근 권한이 없다: {real} '
                          f'(사용자를 video 그룹에 넣을 것)')

        # 어떤 카메라인지 로그에 남긴다 — `/dev/video*` 번호는 USB 열거 순서에 따라
        # 바뀌므로, 번호만으로는 "무엇이 찍혔는지" 를 나중에 알 수 없다.
        # (2026-08-13: video2 로 설정된 채 Arducam 대신 내장 웹캠이 찍히고 있었다.)
        name = self._v4l_name(real)
        self.get_logger().info(
            f'카메라 장치 확정: {device}'
            + (f' → {real}' if real != device else '')
            + (f'  [{name}]' if name else ''))
        return real, None

    @staticmethod
    def _v4l_name(dev_path):
        """`/sys/class/video4linux/<node>/name` 을 읽어 카메라 모델명을 돌려준다."""
        try:
            with open(f'/sys/class/video4linux/{os.path.basename(dev_path)}/name') as f:
                return f.read().strip()
        except OSError:
            return None

    # ------------------------------------------------------------------
    def _indexed_publisher(self, shot_no):
        """shot_no(1-base) 번째 장 전용 래치 토픽(`~/image_raw_<N>`) 게시자. 없으면 만든다."""
        pub = self._pub_indexed.get(shot_no)
        if pub is None:
            pub = self.create_publisher(Image, f'~/image_raw_{shot_no}', self._latched_qos)
            self._pub_indexed[shot_no] = pub
        return pub

    # ------------------------------------------------------------------
    def _to_image_msg(self, frame, stamp, shot_no=None):
        """OpenCV BGR 프레임을 sensor_msgs/Image(bgr8) 로 변환 (cv_bridge 불필요).

        `shot_no` 는 토픽 번호와 같은 **1-base** 값이다 — `frame_id` 만 보고도 어느
        `image_raw_N` 의 사진인지 알 수 있게 맞춰 둔다.
        """
        msg = Image()
        msg.header.stamp = stamp
        base_frame_id = self.get_parameter('frame_id').value
        msg.header.frame_id = (
            base_frame_id if shot_no is None else f'{base_frame_id}_{shot_no}'
        )
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = frame.shape[1] * frame.shape[2]
        msg.data = frame.tobytes()
        return msg

    # 이 노드가 쓰는 파일 이름 규칙. _prune 이 지울 대상을 이 패턴으로만 한정한다.
    SHOT_GLOB = 'shot_*.jpg'

    def _save(self, frame, save_dir, shot_no):
        """`shot_<날짜>_<시각>_<N>.jpg` 로 저장. N 은 토픽 번호와 같은 1-base 다."""
        os.makedirs(save_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        path = os.path.join(save_dir, f'shot_{ts}_{shot_no}.jpg')
        if not cv2.imwrite(path, frame):
            # 경로가 없거나 권한이 없으면 imwrite 는 예외 없이 False 만 준다 —
            # 조용히 넘기면 "찍혔다는데 파일이 없다" 가 된다.
            self.get_logger().error(f'저장 실패: {path}')
            return None
        self.get_logger().info(f'저장: {path}')
        return path

    def _prune(self, save_dir, keep):
        """save_dir 의 `shot_*.jpg` 중 최신 keep 장만 남기고 삭제. 삭제한 개수를 반환."""
        try:
            paths = glob.glob(os.path.join(save_dir, self.SHOT_GLOB))
        except OSError as e:
            self.get_logger().warn(f'보관 정리 건너뜀 (목록 실패): {e}')
            return 0
        if len(paths) <= keep:
            return 0
        # mtime 기준 최신순. 같은 버스트는 mtime 이 겹칠 수 있어 파일명(타임스탬프+idx)을
        # 보조 키로 둔다 — 이름 형식이 고정폭이라 사전순 == 시간순이다.
        def sort_key(p):
            try:
                return (os.path.getmtime(p), os.path.basename(p))
            except OSError:
                return (0.0, os.path.basename(p))
        paths.sort(key=sort_key, reverse=True)

        removed = 0
        for p in paths[keep:]:
            try:
                os.remove(p)
                removed += 1
            except OSError as e:
                self.get_logger().warn(f'삭제 실패: {p} ({e})')
        if removed:
            self.get_logger().info(
                f'보관 정리: 오래된 {removed}장 삭제 (최근 {keep}장 유지)')
        return removed


def main(args=None):
    rclpy.init(args=args)
    node = CaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
