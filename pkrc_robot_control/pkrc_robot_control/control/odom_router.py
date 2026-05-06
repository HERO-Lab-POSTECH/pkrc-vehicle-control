"""Routes Fast-LIO and Cartographer odometry to controllers.

Fan-outs each odometry topic to both HoveringController and PIDModeController,
which independently track latest pose for their respective control modes.
"""

from nav_msgs.msg import Odometry


class OdometryRouter:
    """`/slam/fast_lio/odometry`, `/slam/fast_lio_loc/odometry`,
    `/slam/cartographer/odometry` 를 구독하여 hovering/pid 컨트롤러
    양쪽에 동시 전달. fast_lio LIO/loc 모드는 SLAM 측에서 동시에
    publish 되지 않으며, 두 토픽 모두 동일한 fastlio 콜백으로 fan-in.
    """

    FASTLIO_TOPIC = '/slam/fast_lio/odometry'
    FASTLIO_LOC_TOPIC = '/slam/fast_lio_loc/odometry'
    CARTO_TOPIC = '/slam/cartographer/odometry'

    def __init__(self, node, hovering_ctrl, pid_ctrl, qos=10):
        self.node = node
        self.hovering = hovering_ctrl
        self.pid = pid_ctrl

        self.fastlio_sub = node.create_subscription(
            Odometry, self.FASTLIO_TOPIC, self._on_fastlio, qos
        )
        self.fastlio_loc_sub = node.create_subscription(
            Odometry, self.FASTLIO_LOC_TOPIC, self._on_fastlio, qos
        )
        self.carto_sub = node.create_subscription(
            Odometry, self.CARTO_TOPIC, self._on_carto, qos
        )

    def _on_fastlio(self, msg: Odometry):
        self.hovering.update_fastlio_odometry(msg)
        self.pid.update_fastlio_odometry(msg)

    def _on_carto(self, msg: Odometry):
        self.hovering.update_carto_odometry(msg)
        self.pid.update_carto_odometry(msg)
