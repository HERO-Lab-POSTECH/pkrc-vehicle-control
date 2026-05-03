"""USB camera manager for HERO robot.

Responsibilities:
- USB camera open/reopen with retry FSM (no while-True thread; ROS2 timer based).
- Frame capture at fixed rate (default 40 Hz read attempt, 15 Hz publish).
- ROS2 publish: CompressedImage(JPEG) + reservation for raw Image.
- Recording toggle (MJPG → .avi in node-local video/ dir).

Replaces the previous threading-based camera_loop in main.py.
"""

import os
import threading
import time
from datetime import datetime

import cv2
from sensor_msgs.msg import CompressedImage


DEFAULT_CAMERA_DEVICE = (
    '/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._exploreHD_USB_Camera_SN00009-video-index0'
)


class CameraManager:
    """USB 카메라 캡처/발행/녹화 관리.

    사용:
        cam = CameraManager(node, device=DEFAULT_CAMERA_DEVICE)
        # node가 spin되면 timer가 자동 read+publish.
        cam.toggle_recording()
        ...
        cam.shutdown()
    """

    def __init__(self, node, device=DEFAULT_CAMERA_DEVICE,
                 capture_rate_hz=40.0, publish_rate_hz=15.0,
                 reconnect_interval_sec=3.0, frame_timeout_sec=2.0,
                 video_dir=None):
        self.node = node
        self.logger = node.get_logger()
        self.device = device
        self.publish_rate_hz = publish_rate_hz
        self.reconnect_interval_sec = reconnect_interval_sec
        self.frame_timeout_sec = frame_timeout_sec

        self.camera = None
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.last_frame_time = time.time()
        self.last_publish_time = 0.0
        self._reconnect_wait_until = 0.0

        # 녹화 상태
        self.video_lock = threading.Lock()
        self.is_recording = False
        self.video_writer = None
        self.recording_start_time = None
        self.recording_frame_count = 0
        self.current_video_filename = None
        self.video_dir = video_dir or os.path.join(os.path.dirname(__file__), '..', 'video')
        os.makedirs(self.video_dir, exist_ok=True)

        # ROS2 publishers
        self.pub_camera_compressed = node.create_publisher(
            CompressedImage, '/camera/image/compressed', 10
        )

        # ROS2 timer (블로킹 스레드 대신)
        period = 1.0 / capture_rate_hz
        self._timer = node.create_timer(period, self._on_tick)

        # 첫 카메라 시도
        self._open_camera()

    # ───────── 카메라 open/reopen ─────────

    def _open_camera(self):
        """카메라 초기화 (재연결 지원). 성공 시 True."""
        if self.camera is not None:
            try:
                self.camera.release()
            except Exception as e:
                self.logger.debug(f'camera release: {e}')
            self.camera = None

        try:
            self.camera = cv2.VideoCapture(self.device, cv2.CAP_V4L2)

            if not self.camera.isOpened() and self.device != '/dev/video0':
                self.logger.warn(f'⚠️  {self.device} 열기 실패, /dev/video0로 재시도')
                self.camera.release()
                self.camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
                if self.camera.isOpened():
                    self.device = '/dev/video0'

            if self.camera and self.camera.isOpened():
                self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                self.camera.set(cv2.CAP_PROP_FPS, 30)
                self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                w = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
                self.logger.info(f'✅ USB 카메라 초기화 완료 ({self.device}, {w}x{h})')
                self.last_frame_time = time.time()
                return True
            self.logger.warn('⚠️  USB 카메라를 찾을 수 없습니다')
            return False
        except Exception as e:
            self.logger.error(f'❌ 카메라 초기화 실패: {e}')
            return False

    # ───────── timer tick (read + publish) ─────────

    def _on_tick(self):
        now = time.time()

        # 카메라 미연결 → 재연결 backoff
        if self.camera is None or not self.camera.isOpened():
            if now >= self._reconnect_wait_until:
                self.logger.warn('🔄 카메라 재연결 시도...')
                if self._open_camera():
                    self._reconnect_wait_until = 0.0
                else:
                    self._reconnect_wait_until = now + self.reconnect_interval_sec
            return

        # 프레임 읽기
        try:
            ret, frame = self.camera.read()
        except Exception as e:
            self.logger.error(f'❌ 카메라 읽기 오류: {e}')
            ret = False
            frame = None

        if not ret or frame is None:
            # 타임아웃 시 reopen 예약
            if now - self.last_frame_time > self.frame_timeout_sec:
                self.logger.warn(f'⚠️  카메라 프레임 타임아웃 ({self.frame_timeout_sec}초)')
                try:
                    self.camera.release()
                except Exception as e:
                    self.logger.debug(f'release: {e}')
                self.camera = None
                self._reconnect_wait_until = now + self.reconnect_interval_sec
            return

        self.last_frame_time = now
        with self.frame_lock:
            self.latest_frame = frame.copy()

        # 녹화
        if self.is_recording:
            with self.video_lock:
                if self.video_writer is not None:
                    try:
                        self.video_writer.write(frame)
                        self.recording_frame_count += 1
                    except Exception as e:
                        self.logger.error(f'❌ 프레임 저장 실패: {e}')

        # ROS2 publish (rate-limited)
        if now - self.last_publish_time >= 1.0 / self.publish_rate_hz:
            self.last_publish_time = now
            try:
                msg = CompressedImage()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.header.frame_id = 'camera'
                msg.format = 'jpeg'
                _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                msg.data = jpeg.tobytes()
                self.pub_camera_compressed.publish(msg)
            except Exception as e:
                self.logger.debug(f'compressed publish 실패: {e}')

    # ───────── 녹화 ─────────

    def start_recording(self):
        if self.is_recording:
            self.logger.warn('⚠️  이미 녹화 중입니다')
            return
        if self.latest_frame is None:
            self.logger.error('❌ 카메라 프레임이 없습니다')
            return

        with self.frame_lock:
            height, width = self.latest_frame.shape[:2]

        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.current_video_filename = os.path.join(self.video_dir, f'video_{ts}.avi')

        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        with self.video_lock:
            self.video_writer = cv2.VideoWriter(
                self.current_video_filename, fourcc, 30.0, (width, height)
            )
            if not self.video_writer.isOpened():
                self.logger.error('❌ 비디오 파일 생성 실패')
                self.video_writer.release()
                self.video_writer = None
                return

        self.is_recording = True
        self.recording_start_time = time.time()
        self.recording_frame_count = 0
        self.logger.info(
            f'🔴 녹화 시작: {os.path.basename(self.current_video_filename)} '
            f'({width}x{height})'
        )

    def stop_recording(self):
        if not self.is_recording:
            self.logger.warn('⚠️  녹화 중이 아닙니다')
            return

        self.is_recording = False
        with self.video_lock:
            if self.video_writer is not None:
                self.video_writer.release()
                self.video_writer = None

        duration = time.time() - self.recording_start_time
        self.logger.info(
            f'⏹️  녹화 중지 (길이: {duration:.1f}초, 프레임: {self.recording_frame_count}개)'
        )
        self.logger.info(f'💾 저장 완료: {os.path.basename(self.current_video_filename)}')

        if os.path.exists(self.current_video_filename):
            size_mb = os.path.getsize(self.current_video_filename) / (1024 * 1024)
            self.logger.info(f'📦 파일 크기: {size_mb:.2f} MB')
        else:
            self.logger.error('❌ 파일이 생성되지 않았습니다!')

        self.current_video_filename = None
        self.recording_frame_count = 0

    def toggle_recording(self):
        if self.is_recording:
            self.stop_recording()
        else:
            self.start_recording()

    # ───────── shutdown ─────────

    def shutdown(self):
        try:
            if self.is_recording:
                self.stop_recording()
        except Exception as e:
            self.logger.debug(f'shutdown stop_recording: {e}')
        try:
            if self.camera and self.camera.isOpened():
                self.camera.release()
        except Exception as e:
            self.logger.debug(f'shutdown camera release: {e}')
