"""Routes Fast-LIO and Cartographer odometry to controllers.

Fan-outs each odometry topic to both HoveringController and PIDModeController,
which independently track latest pose for their respective control modes.
"""

from nav_msgs.msg import Odometry


class OdometryRouter:
    """`/fast_lio/odometry` 와 `/cartographer_2d/odometry` 를 구독하여
    hovering/pid 컨트롤러 양쪽에 동시 전달.
    """

    FASTLIO_TOPIC = '/fast_lio/odometry'
    CARTO_TOPIC = '/cartographer_2d/odometry'

    def __init__(self, node, hovering_ctrl, pid_ctrl, qos=10):
        self.node = node
        self.hovering = hovering_ctrl
        self.pid = pid_ctrl

        self.fastlio_sub = node.create_subscription(
            Odometry, self.FASTLIO_TOPIC, self._on_fastlio, qos
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
