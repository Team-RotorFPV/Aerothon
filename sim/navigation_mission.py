"""Navigation-only object fixture. Production delivery is not exercised here.

The headless world has no FCU motor/IMU evidence. This explicitly omitted
delivery leaf keeps the hook stowed and marks delivery unconfirmed, allowing
the real search, return corridor and landing behaviours to be tested without
inventing telemetry or declaring a delivery. Rendered acceptance always uses
the original production mission entry point.
"""
import math
import py_trees
from mission_bt import mission_tree


class UnqualifiedHeadlessDelivery(py_trees.behaviour.Behaviour):
    def __init__(self,mav,drop_alt=5.,cruise_alt=10.,**_):
        super().__init__('UnqualifiedHeadlessDelivery')
        self.mav=mav
        self.cruise_alt=cruise_alt

    def initialise(self):
        self.xy=self.mav.pos()[:2]
        self.yaw=self.mav.yaw()
        self.mav.delivery_confirmed=False
        self.mav.delivery_offset_m=None
        self.mav.delivery_note='delivery not exercised: object fixture has no qualified motor telemetry'
        self.mav.log(self.mav.delivery_note,warn=True)

    def update(self):
        w=self.mav.winch_status
        payout=w.get('payout_estimate_m',w.get('payout_m'))
        if w.get('fault') or not isinstance(payout,(int,float)) or not math.isfinite(payout) or payout<0:
            self.mav.winch('stop')
            self.mav.abort_reason='invalid winch state in navigation-only fixture'
            return py_trees.common.Status.FAILURE
        if payout>.02:
            self.mav.winch('stow')
            return py_trees.common.Status.RUNNING
        self.mav.winch('stop')
        self.mav.goto(*self.xy,self.cruise_alt,self.yaw)
        if self.mav.reached(*self.xy,self.cruise_alt,.6):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


def main():
    # This substitution exists only in the test process launched by fly_headless.
    mission_tree.WinchDrop=UnqualifiedHeadlessDelivery
    mission_tree.STATE_NAMES['UnqualifiedHeadlessDelivery']='DELIVERY_UNQUALIFIED'
    mission_tree.main()


if __name__=='__main__':main()
