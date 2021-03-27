# Simulator module
import time
import enum
import math

running: bool

class rig:
    wheelbase: float = 1.15
    width: float = 1.2
    rear_bumper: float = 0.3
    front_bumper: float = wheelbase + 0.3
    max_throttle: float = 1.0
    max_brake: float = 1.0
    max_abs_steering_rate: float = 1.6
    max_abs_steering_angle: float = 0.45
    throttle_force: float = 3.0
    brake_force: float = 5.0
    drag: float = 0.003
    rolling_resistance: float = 0.1


class Gear(enum.Enum):
    FORWARD: int = 1
    REVERSE: int = 2


class ControlSignal:
    def __init__(self, throttle: float = 0, brake: float = 0, steering_rate: float = 0, gear: Gear = Gear.FORWARD):
        self.throttle = throttle
        self.brake = brake
        self.steering_rate = steering_rate
        self.gear = gear

    def __str__(self):
        return f"T= {self.throttle:.3f}  B= {self.brake:.3f}  S_R= {self.steering_rate:.3f}  G= {self.gear}"


class Position:
    def __init__(self, x: float = 0, y: float = 0):
        self.x = x
        self.y = y


class State:
    def __init__(self, position: Position = Position(0, 0), orientation: float = 0, steering_angle: float = 0,
                 speed: float = 0, gear: Gear = Gear.FORWARD):
        self.position = position
        self.orientation = orientation
        self.steering_angle = steering_angle
        self.speed = speed
        self.gear = gear

    def __str__(self):
        return f"x= {self.position.x:.2f}  y= {self.position.y:.2f}  fi= {self.orientation:.2f}"


def close():
    global running
    running = False


def check_control_constraints(u: ControlSignal):
    if rig.max_abs_steering_rate < abs(u.steering_rate):
        raise Exception('Maximum allowed steering rate exceeded')
    if u.throttle < 0 or 1 < u.throttle:
        raise Exception('Throttle control out of allowed boundaries');
    if u.brake < 0 or 1 < u.brake:
        raise Exception('Brake control out of allowed boundaries');
    if 0 < u.brake and 0 < u.throttle:
        raise Exception('Brake and torque simultaneous control is not allowed');


def is_zero(v: float):
    return abs(v) < 1.0e-5


def move_direction(gear: Gear):
    if gear == Gear.FORWARD:
        return 1
    else:
        return -1


def clamp(v, min_v, max_v):
    return min(max(v, min_v), max_v)


def run(update_callback):
    x: State = State()

    global running
    running = True
    t = 0.0
    dt = 0.1
    ddt = dt / 10
    start_time = time.time()
    while running:
        u = update_callback(x, dt, t)
        # Raises exception in case of violation
        check_control_constraints(u)
        # Switch gears
        if is_zero(x.speed):
            if x.gear != u.gear:
                x.speed = 0
            x.gear = u.gear
        # Simulate
        for i in range(0, 10):
            dir_speed: float = x.speed * move_direction(x.gear)
            # Change position
            x.position.x += dir_speed * math.cos(x.orientation) * ddt
            x.position.y += dir_speed * math.sin(x.orientation) * ddt
            # Change orientation
            x.orientation += dir_speed * math.tan(x.steering_angle) / rig.wheelbase * ddt
            x.steering_angle = clamp(x.steering_angle + u.steering_rate * ddt, -rig.max_abs_steering_angle,
                                     rig.max_abs_steering_angle)
            # Change speed
            acceleration: float = rig.throttle_force * u.throttle - \
                                  rig.brake_force * u.brake - \
                                  rig.drag * x.speed * x.speed - \
                                  rig.rolling_resistance
            x.speed = max(x.speed + acceleration * ddt, 0.0)
        # Normalize orientation
        x.orientation = math.atan2(math.sin(x.orientation), math.cos(x.orientation))
        # Switch gears (again - if it was not possible to switch gears before, but now is, so this is the place to try it again)
        if is_zero(x.speed):
            if x.gear != u.gear:
                x.speed = 0
            x.gear = u.gear
        # Prepare for the next step
        t = t + dt
        next_start_time = time.time()
        time.sleep(max(dt - (time.time() - start_time), 0))
        start_time = time.time()
