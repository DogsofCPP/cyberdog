"""
Navigation State Machine for CyberDog Race.
Manages the 6 segment states and transitions.
"""
import enum
import time
import threading
import math


class RaceState(enum.Enum):
    """Race segment states."""
    IDLE = 0
    INIT = 1           # Standing up, getting ready
    SEGMENT_1 = 2      # Stone Path - slow walk
    SEGMENT_2 = 3      # Wild Ball Search - hit orange balls
    SEGMENT_3 = 4      # Curved Sprint - high-speed line follow
    SEGMENT_4 = 5      # Deep Tunnel - object recognition + obstacles
    SEGMENT_5 = 6      # Single Bridge - narrow beam crossing + jump
    SEGMENT_6 = 7      # Gold Rush - kick ball to goal
    FINISHED = 8
    ERROR = 9


class RaceStateMachine:
    """State machine managing all 6 race segments.

    Each segment is implemented as a coroutine-like execute() function.
    The machine calls the appropriate segment handler based on current state.
    """

    def __init__(self, ctrl, perception, speaker=None):
        self.ctrl = ctrl
        self.perception = perception
        self.speaker = speaker

        self._state = RaceState.IDLE
        self._prev_state = RaceState.IDLE
        self._segment_start_time = 0.0
        self._race_start_time = 0.0
        self._race_elapsed = 0.0
        self._running = False
        self._lock = threading.Lock()

        # Navigator for world-frame reactive navigation
        from .navigator import Navigator
        self.navigator = Navigator(perception)

        # Per-segment state
        self._seg1_balls_hit = 0
        self._seg2_balls_hit = 0
        self._seg2_all_balls_found = False
        self._seg4_objects_announced = set()
        self._seg5_on_bridge = False

        # Shared segment counters
        self._orange_balls_hit = 0
        self._total_orange_balls = 4  # 4x4 grid, one orange target per row/column

    # ------------------------------------------------------------------
    # State management
    # ------------------------------------------------------------------

    @property
    def state(self):
        with self._lock:
            return self._state

    @state.setter
    def state(self, new_state):
        with self._lock:
            self._prev_state = self._state
            self._state = new_state
            self._segment_start_time = time.perf_counter()

    def transition_to(self, new_state):
        """Transition to a new state with logging."""
        prev = self.state
        self.state = new_state
        elapsed = self._race_elapsed
        print(f"[SM] {elapsed:.1f}s: {prev.name} -> {new_state.name}")

    def is_running(self):
        return self._running

    # ------------------------------------------------------------------
    # Main run loop
    # ------------------------------------------------------------------

    def run(self, timeout_s=900.0):
        """Run the state machine until FINISHED, ERROR, or timeout.

        Args:
            timeout_s: Maximum race time in seconds (default 15min = 900s)
        """
        self._running = True
        self._race_start_time = time.perf_counter()
        print("[SM] Race started")
        self.transition_to(RaceState.INIT)

        try:
            while self._running:
                self._race_elapsed = time.perf_counter() - self._race_start_time

                if self._race_elapsed > timeout_s:
                    print(f"[SM] Race timeout ({timeout_s}s), stopping")
                    self.transition_to(RaceState.FINISHED)
                    break

                state = self.state  # snapshot

                if state == RaceState.INIT:
                    self._do_init()
                elif state == RaceState.SEGMENT_1:
                    self._do_segment_1()
                elif state == RaceState.SEGMENT_2:
                    self._do_segment_2()
                elif state == RaceState.SEGMENT_3:
                    self._do_segment_3()
                elif state == RaceState.SEGMENT_4:
                    self._do_segment_4()
                elif state == RaceState.SEGMENT_5:
                    self._do_segment_5()
                elif state == RaceState.SEGMENT_6:
                    self._do_segment_6()
                elif state == RaceState.FINISHED:
                    self._do_finished()
                    break
                elif state == RaceState.ERROR:
                    self._do_error()
                    break

                time.sleep(0.01)

        except KeyboardInterrupt:
            print("[SM] Interrupted")
            self.transition_to(RaceState.ERROR)
        except Exception as e:
            print(f"[SM] Exception: {e}")
            self.transition_to(RaceState.ERROR)
        finally:
            self._running = False
            self.ctrl.shutdown()
            print("[SM] Race ended")

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

    def _do_init(self):
        """Stand up and prepare for race start."""
        print("[SM] INIT: Standing up")
        self.ctrl.stand_up_and_wait(height=0.24, wait_s=2.0)
        time.sleep(0.5)
        print("[SM] INIT: Ready to race")
        self.transition_to(RaceState.SEGMENT_1)

    # ------------------------------------------------------------------
    # Segment handlers (overridden by concrete implementations)
    # ------------------------------------------------------------------

    def _do_segment_1(self):
        """Stone Path - slow walk along stone road.
        
        Walk to (3,0) and turn to face 90° (+Y direction) for next segment.
        """
        # Re-calibrate position offset for Reset World scenario
        self._check_and_calibrate_position()
        from .segments.segment_1 import execute as seg1_exec
        done = seg1_exec(self.ctrl, self.perception, self.navigator)
        if done:
            self.transition_to(RaceState.SEGMENT_2)

    def _check_and_calibrate_position(self):
        """Re-check and calibrate position offset if Gazebo was reset."""
        status = self.perception.sensor_status()
        if status.get('gazebo', False):
            raw_x, raw_y, raw_yaw = self.perception.pose
            # If position is near origin (within 0.5m), likely a fresh start or reset
            dist_from_origin = math.sqrt(raw_x**2 + raw_y**2)
            if dist_from_origin < 0.5:
                offset_x = -raw_x
                offset_y = -raw_y
                self.perception.set_position_offset(offset_x, offset_y, 0.0)
                print(f"[SM] Reset World detected, recalibrated position offset to ({offset_x:.3f}, {offset_y:.3f})")
            else:
                print(f"[SM] Using existing position offset, current pos=({raw_x:.3f}, {raw_y:.3f})")

    def _do_segment_2(self):
        """Wild Ball Search - find and hit all orange balls."""
        from .segments.segment_2 import execute as seg2_exec
        done = seg2_exec(self.ctrl, self.perception, self.speaker, self.navigator)
        if done:
            self.transition_to(RaceState.SEGMENT_3)

    def _do_segment_3(self):
        """Curved Sprint - high-speed line following."""
        from .segments.segment_3 import execute as seg3_exec
        done = seg3_exec(self.ctrl, self.perception, self.navigator)
        if done:
            self.transition_to(RaceState.SEGMENT_4)

    def _do_segment_4(self):
        """Deep Tunnel - object recognition and obstacle navigation."""
        # Segment 4 can be skipped for debugging segment 5.
        from .segments.segment_4 import execute as seg4_exec
        done = seg4_exec(self.ctrl, self.perception, self.speaker,
                         self.navigator, announced=self._seg4_objects_announced)
        if done:
            self.transition_to(RaceState.SEGMENT_5)

    def _do_segment_5(self):
        """Single Bridge - traverse narrow beam and jump off."""
        from .segments.segment_5 import execute as seg5_exec
        done = seg5_exec(self.ctrl, self.perception, self.navigator)
        if done:
            self.transition_to(RaceState.SEGMENT_6)

    def _do_segment_6(self):
        """Gold Rush - kick ball out to goal, then lie down."""
        from .segments.segment_6 import execute as seg6_exec
        done = seg6_exec(self.ctrl, self.perception, self.speaker, self.navigator)
        if done:
            self.transition_to(RaceState.FINISHED)

    def _do_finished(self):
        """Race finished - make sure robot is lying down."""
        # Do not force pure_damper here; some workflows want to keep standing.
        return

    def _do_error(self):
        """Error state - stop robot safely."""
        self.ctrl.pure_damper()
