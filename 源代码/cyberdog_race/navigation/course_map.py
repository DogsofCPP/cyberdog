"""
Course Map - Global World-Frame Coordinate System.

All coordinates are in Gazebo world frame:
  X = right, Y = up
  Heading yaw in degrees: 0=+X, 90=+Y, 180=-X, -90=-Y (matching odom_pose)

Segment transitions (user-provided):
  S1 end / S2 start : (3.0,  0.0, yaw= 90.0°)
  S2 end / S3 start : (-0.3,  4.0, yaw= 90.0°)
  S3 end / S4 start : ( 3.0,  7.0, yaw=180.0°)
  S4 end / S5 start : ( 3.0,  7.0, yaw= 90.0°)
  S5 end / S6 start : ( 3.0, 13.0, yaw=-90.0°)
"""
import math

# ---------------------------------------------------------------------------
# Segment start positions: (x, y, yaw_deg)
# ---------------------------------------------------------------------------
SEGMENT_STARTS = {
    1: (0.0,  0.0,   0.0),   # start, facing +X
    2: (3.0,  0.0,  90.0),   # right edge, facing +Y
    3: (-0.3, 4.0,  90.0),   # S3 entry / S2 exit
    4: ( 3.0, 7.0, 180.0),   # right side top, facing -X
    5: ( 3.0, 7.0,  90.0),   # right edge mid, facing +Y
    6: ( 3.0, 13.0, -90.0),  # right edge top, facing -Y
}

# ---------------------------------------------------------------------------
# Coordinate conversion helpers
# ---------------------------------------------------------------------------

def deg_to_rad(deg):
    return deg * math.pi / 180.0


def rad_to_deg(rad):
    return rad * 180.0 / math.pi


def seg_start(seg_num):
    """Return (x, y, yaw_deg) for segment start."""
    return SEGMENT_STARTS[seg_num]


def seg_exit(seg_num):
    """Return (x, y, yaw_deg) for segment exit = next segment's entry."""
    return SEGMENT_STARTS.get(seg_num + 1, None)


def world_pose(x, y, yaw_deg=None):
    """Return full world-frame pose dict."""
    return {
        'x': x,
        'y': y,
        'yaw_deg': yaw_deg if yaw_deg is not None else None,
        'yaw_rad': deg_to_rad(yaw_deg) if yaw_deg is not None else None,
    }


# ---------------------------------------------------------------------------
# Segment 1 - Stone Path
# From (0,0) facing +X. Cross stone slabs to connector, enter S2.
# ---------------------------------------------------------------------------
# Dog starts at (0,0) facing +X. Walk straight (+X) to end of stones at x=2.7,
# then turn left 90° to face +Y, then enter S2 connector.
S1_WAYPOINTS = [
    (2.9,  0.0,  0.0),   # end of stone slab straight
    (2.9,  0.0, 90.0),   # turn to face +Y (S2 direction)
    (3.0,  0.0, 90.0),   # S2 entry
]


# ---------------------------------------------------------------------------
# Segment 2 - Wild Ball Search
# From (3, 0) facing +Y. Snake pattern: go to mid-point between rows,
# turn 90°, hit ball, repeat.
#
# Orange balls in the bundled race.world:
#   - (0.8, 1.34)  - row 1, column 2
#   - (2.0, 2.18)  - row 2, column 3
#   - (3.2, 3.02)  - row 3, column 4
#   - (-0.4, 3.86) - row 4, column 1
#
# Strategy: move to a short approach point near each orange ball, face the
# ball, bump it, then return to the route and continue toward the exit.
# ---------------------------------------------------------------------------
_S2_ENTRY = (3.0, 0.0)   # world x, y

# Snake pattern waypoints: approach points before planned bumps.
# Format: (x, y, target_heading_deg)
# Heading: 90=+Y (north), 0=+X (east), 180/-180=-X (west), -90=-Y (south)
S2_WAYPOINTS = [
    (3.0,  0.90,  90.0),
    (0.70,  1.08,  90.0),  # approach ball #1 from south 0.8
    (1.4,  1.75,   0.0),
    (2.0,  2.05,  90.0),  # approach ball #2 from south
    (2.6,  2.55,   0.0),
    (3.0,  2.9,  90.0),  # approach ball #3 from south 2.76
    (2.0,  3.35, 180.0),
    (-0.1,  3.45, 180.0),
    (-0.3, 3.86, 160.0),  # approach ball #4 from east 
]

# Ball hit waypoints: when robot reaches these positions, it should auto-hit
# Format: (x, y, heading, ball_number)
S2_BALL_HIT_WAYPOINTS = [
    (0.8,   1.34,  90.0, 1),   # Ball #1 
    (2.0,   2.18,  90.0, 2),   # Ball #2
    (3.2,   3.02,  90.0, 3),   # Ball #3
    (-0.4,  3.86, 180.0, 4),   # Ball #4
]

S2_EXIT_WAYPOINTS = [
    (-0.3, 4.0, 90.0),     # S3 entry / S2 exit
]

# Also include the explicit exit coordinate
S2_EXIT_COORD = (-0.3, 4.0)


# ---------------------------------------------------------------------------
# Segment 3 - Curved Sprint
# Curved lane sampled from measured points on the bend.
# ---------------------------------------------------------------------------
S3_WAYPOINTS = [
    (-0.300, 4.00,  90.0),
    (-0.300, 4.850,  37.9),
    ( 0.127, 5.30,  27.8),
    ( 0.420, 5.450,   9.5),
    ( 1.020, 5.550,   6.4),
    ( 1.464, 5.600,  10.6),
    ( 2.000, 5.700,   8.0),
    ( 2.400, 5.760,  40.4),
    ( 2.800, 6.10,   76.0),
    ( 2.900, 6.300,  81.9),
    ( 3.0, 7.20,  180.0),
]


# ---------------------------------------------------------------------------
# Segment 4 - Deep Tunnel
# From (3, 7) facing -X. Navigate to objects at world coordinates, then exit to bridge.
#
# Object positions (from race.world):
#   - football:    (2.1,  10.8)
#   - orange_ball: (0.95, 11.1)
#   - coke bottle: (-0.1,  11.1)
# ---------------------------------------------------------------------------
S4_WAYPOINTS = [
    ( 3.0,  7.0, 180.0),   # entry position
    ( 3.0, 10.5, 180.0),   # move forward into tunnel
    ( 2.1, 10.8, 180.0),   # approach football position
    ( 0.95, 11.1, 180.0),  # approach orange ball position
    (-0.1, 11.1, 180.0),   # approach coke bottle position
    ( 1.5, 11.5,   0.0),   # turn and prepare for bridge
    ( 3.0, 12.0, -90.0),   # move toward bridge entry
]


# ---------------------------------------------------------------------------
# Segment 5 - Single Bridge
# From (3, 7) facing +Y. Cross bridge, jump near (3, 13).
# ---------------------------------------------------------------------------
S5_WAYPOINTS = [
    ( 3.0,  9.0,  90.0),   # onto bridge
    ( 3.0, 11.0,  90.0),   # mid bridge
    ( 3.0, 12.0,  90.0),   # near end, jump zone
    ( 3.0, 13.0, -90.0),   # S6 entry
]


# ---------------------------------------------------------------------------
# Segment 6 - Gold Rush
# From (3, 13) facing -Y. Kick football, reach finish zone.
# Football position from race.world: (0.4, 14.7) in world frame
# ---------------------------------------------------------------------------
S6_WAYPOINTS = [
    ( 2.0, 13.5, -90.0),   # approach from bridge
    ( 0.5, 14.5, -90.0),   # near football
    ( 0.5, 15.5, -90.0),    # finish zone
]


# ---------------------------------------------------------------------------
# All segment waypoint tables (for Navigator)
# ---------------------------------------------------------------------------
SEGMENT_WAYPOINTS = {
    1: S1_WAYPOINTS,
    2: S2_WAYPOINTS,
    3: S3_WAYPOINTS,
    4: S4_WAYPOINTS,
    5: S5_WAYPOINTS,
    6: S6_WAYPOINTS,
}


# ---------------------------------------------------------------------------
# Default navigation tolerances
# ---------------------------------------------------------------------------
WAYPOINT_TOL_M = 0.15       # distance tolerance for "arrived"
HEADING_TOL_DEG = 5.0       # heading tolerance for "facing correct direction"
STUCK_SPEED_THRESHOLD = 0.005  # m/s below which robot is "stuck"
STUCK_TIME_S = 3.0          # seconds of stuck before recovery triggers
DEVIATION_THRESHOLD_M = 0.4  # how far off plan before recovery mode
