"""Stop an invalidated search segment, then replan from fresh measured pose."""
import math
from .search_planner import point_in_exclusion, routing_obstacles


class SearchBrakingGate:
    def __init__(self):
        self.hold = None
        self.last_velocity_stamp = None
        self.settled_samples = 0
        self.completed_hold = None
        self.brake_started_stamp = None
        self.escape_active = False
        self.escape_blocked = False

    def _revalidate_hold(self, stage, current):
        """A stationary command still has to respect new red observations."""
        mav=stage.mav
        anchor=self.hold[:2] if self.hold is not None else self.completed_hold
        if anchor is None:
            return
        # Preserve an already certified outward escape instead of turning
        # it into another stop inside the newly observed margin.
        if stage.router._segment_safe(mav,anchor,exclusions=current):
            return
        try:
            now=stage.clock_fn();inputs=mav.delivery_inputs
            pose_stamp=float(inputs['pose_stamp']);stamp=float(inputs['velocity_stamp']);speed=float(inputs['speed'])
            if not (all(math.isfinite(v) for v in (now,pose_stamp,stamp,speed))
                    and 0<=now-pose_stamp<=.5 and 0<=now-stamp<=.5 and speed>=0.):
                return
        except (AttributeError,KeyError,TypeError,ValueError):
            return
        x,y,z=mav.pos()
        containing=[r for r in routing_obstacles(current,stage.router.clearance_m)
                    if r[0]<=x<=r[1] and r[2]<=y<=r[3]]
        target=(x,y)
        if containing:
            x0,x1,y0,y1=containing[0]
            target=min(((x0-.25,y),(x1+.25,y),(x,y0-.25),(x,y1+.25)),
                       key=lambda p:math.dist((x,y),p))
        if not stage.router._segment_safe(mav,target,exclusions=current):
            if not self.escape_blocked:
                # New paint can obstruct the escape itself. Cancel that
                # travel at the fresh measured position once, then keep
                # the anchor latched until a certified escape is available.
                self.hold=(x,y,z,mav.yaw())
                self.completed_hold=None
                self.escape_active=self.escape_blocked=True
                self.brake_started_stamp=now
                self.last_velocity_stamp=None
                self.settled_samples=0
                mav.log('search escape blocked by new red observations; holding measured position')
            return
        self.hold=(*target,z,mav.yaw())
        self.completed_hold=None
        self.escape_active=bool(containing)
        self.escape_blocked=False
        self.brake_started_stamp=now
        self.last_velocity_stamp=None
        self.settled_samples=0
        stage.router.reset()
        mav.log('search braking anchor invalidated by red observations; taking certified nearest-edge escape')

    def tick(self, stage):
        mav = stage.mav
        current = stage._current_exclusions()
        self._revalidate_hold(stage,current)
        goal = getattr(stage, '_streamed_search_goal', None)
        commanded = getattr(mav, 'position_target_xy', None)
        if callable(commanded):
            goal = commanded()
        # A completed braking hold is a stationary command, not the next
        # route segment. Let the leaf finish a stationary turn while fresh
        # settled-motion evidence continues to support this exact hold.
        # A replacement position command must pass the route certificate.
        completed = self.completed_hold
        self.completed_hold = None
        if completed is not None and goal is not None and tuple(goal) == completed:
            self.completed_hold = completed
            try:
                now = stage.clock_fn()
                inputs = mav.delivery_inputs
                pose_stamp = float(inputs['pose_stamp'])
                stamp = float(inputs['velocity_stamp'])
                speed = float(inputs['speed'])
                if (all(math.isfinite(v) for v in (now, pose_stamp, stamp, speed))
                        and 0 <= now-pose_stamp <= .5
                        and 0 <= now-stamp <= .5 and 0 <= speed <= .1):
                    return False
            except (AttributeError, KeyError, TypeError, ValueError):
                pass
            # Keep the existing stationary command while motion evidence is
            # unusable or no longer settled. Re-latching a new pose here would
            # make the hold chase estimator updates. Do not let the leaf
            # issue a new travel command until fresh settled motion returns.
            stage.feedback_message = 'holding completed braking anchor; waiting for fresh settled FCU motion'
            return True
        # Share the router's segment certificate, including its narrow
        # nearest-edge escape when already inside an inflated margin. A
        # blanket intersection check cancels that escape and brakes there.
        if self.hold is None and goal is not None and (
                not all(math.isfinite(v) for v in goal)
                or not stage.router._segment_safe(mav, goal, exclusions=current)):
            self.hold = (*mav.pos(), mav.yaw())
            self.last_velocity_stamp = None
            self.settled_samples = 0
            self.brake_started_stamp = stage.clock_fn() if callable(stage.clock_fn) else None
            stage.router.reset()
            mav.log('search segment invalidated by observed exclusion; braking before replanning')
        if self.hold is None:
            return False
        mav.goto(*self.hold)
        stage.feedback_message = 'braking invalidated segment; waiting for fresh settled FCU motion'
        now = stage.clock_fn() if callable(stage.clock_fn) else None
        inputs = getattr(mav, 'delivery_inputs', {})
        try:
            pose_stamp = float(inputs['pose_stamp'])
            stamp = float(inputs['velocity_stamp'])
            speed = float(inputs['speed'])
            fresh = (now is not None and all(math.isfinite(x) for x in (pose_stamp, stamp, speed, now))
                     and 0 <= now-pose_stamp <= .5 and 0 <= now-stamp <= .5 and speed >= 0.)
        except (KeyError, TypeError, ValueError):
            fresh = False
        if not fresh or speed > .1:
            self.settled_samples = 0
            self.last_velocity_stamp = None
            return True
        if self.brake_started_stamp is None:
            self.brake_started_stamp = now
        if self.last_velocity_stamp is not None and now-self.last_velocity_stamp > .5:
            self.settled_samples = 0
            self.last_velocity_stamp = None
        # Consume delivered FCU captures, rather than aliasing their settled
        # intervals to the slower mission timer. Every accepted capture stays
        # post-brake, distinct, fresh and below the same .10 m/s limit.
        history = getattr(mav, 'delivery_velocity_samples', None)
        samples = [(stamp, speed)] if history is None else list(history)
        for captured, measured in samples:
            if not all(math.isfinite(v) for v in (captured, measured)):
                self.settled_samples = 0
                self.last_velocity_stamp = None
                continue
            if captured <= self.brake_started_stamp or now-captured > .5:
                continue
            if captured > now or measured < 0:
                self.settled_samples = 0
                self.last_velocity_stamp = None
                continue
            if self.last_velocity_stamp is not None:
                if captured < self.last_velocity_stamp:
                    continue
                if captured == self.last_velocity_stamp:
                    if measured > .1:
                        self.settled_samples = 0
                    continue
                if captured-self.last_velocity_stamp > .5:
                    self.settled_samples = 0
            self.last_velocity_stamp = captured
            self.settled_samples = self.settled_samples + 1 if measured <= .1 else 0
        if self.settled_samples < 3:
            return True
        if self.escape_active and point_in_exclusion(*mav.pos()[:2],stage.router.clearance_m,current):
            # Low measured speed inside the margin does not complete the
            # escape or authorize a replacement search segment.
            return True
        # The replan reads mav.pos() now. The braking target is not a measured
        # arrival position, and three copies of one velocity sample are not
        # independent evidence that the aircraft has stopped.
        stage._replan_mid_sweep()
        stopped = (*mav.pos(), mav.yaw())
        mav.goto(*stopped)
        stage._pace_hold = stopped
        stage._turn_hold = stopped[:2]
        stage._turn_best = math.inf
        stage._turn_stall = 0
        stage._streamed_search_goal = None
        self.completed_hold = stopped[:2]
        self.hold = None
        self.last_velocity_stamp = None
        self.settled_samples = 0
        self.brake_started_stamp = None
        self.escape_active = False
        self.escape_blocked = False
        return True
