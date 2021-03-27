import numpy as np
from scipy.optimize import minimize
import simulator as sim


# Optimiser signal to throttle/break output
def signal_to_output(gear: sim.Gear, u):
    if u >= 0 and gear == sim.Gear.FORWARD:
        t = u
        b = 0.0
    elif u >= 0 and gear == sim.Gear.REVERSE:
        t = 0.0
        b = u
    elif u < 0 and gear == sim.Gear.FORWARD:
        t = 0.0
        b = -u
    elif u < 0 and gear == sim.Gear.REVERSE:
        t = -u
        b = 0.0
    return t, b


def distance_to_line_sq(state, waypoint1):
    # d = ((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1)) / sqrt((x2-x1)**2 + (y2-y1)**2)
    x0 = state.position.x
    y0 = state.position.y
    x1 = waypoint1[0]
    y1 = waypoint1[1]
    x2 = waypoint1[0] + np.cos(waypoint1[2])
    y2 = waypoint1[1] + np.sin(waypoint1[2])
    return ((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1))**2 / ((x2 - x1) ** 2 + (y2 - y1) ** 2)


class ModelPredictiveControl:
    def __init__(self):
        self.horizon: int = 8
        self.bounds = []
        for i in range(self.horizon):
            self.bounds += [[-1.0, 1.0]]
            self.bounds += [[-sim.rig.max_abs_steering_rate, sim.rig.max_abs_steering_rate]]
        self.u_in = np.ones(2 * self.horizon)*0.1

    def plant_model(self, prev_state: sim.State, dt, u, steering_rate):
        x_t = prev_state.position.x
        y_t = prev_state.position.y
        fi_t = prev_state.orientation
        v_t = prev_state.speed
        theta_t = prev_state.steering_angle
        gear_t = prev_state.gear
        ddt = dt

        # Set gear
        if sim.is_zero(v_t):
            if u >= 0.0:
                gear_t = sim.Gear.FORWARD
            else:
                gear_t = sim.Gear.REVERSE

        throttle, brake = signal_to_output(gear_t, u)

        dir_speed: float = v_t * sim.move_direction(gear_t)
        # Change position
        x_t += dir_speed * np.cos(fi_t) * ddt
        y_t += dir_speed * np.sin(fi_t) * ddt
        # Change orientation
        fi_t += dir_speed * np.tan(theta_t) / sim.rig.wheelbase * ddt
        theta_t = sim.clamp(theta_t + steering_rate * ddt, -sim.rig.max_abs_steering_angle,
                            sim.rig.max_abs_steering_angle)
        # Change speed
        acceleration: float = sim.rig.throttle_force * throttle - \
                              sim.rig.brake_force * brake - \
                              sim.rig.drag * v_t * v_t - \
                              sim.rig.rolling_resistance
        v_t = max(v_t + acceleration * ddt, 0.0)
        # Set gear again
        if sim.is_zero(v_t):
            if u >= 0.0:
                gear_t = sim.Gear.FORWARD
            else:
                gear_t = sim.Gear.REVERSE

        return sim.State(sim.Position(x_t, y_t), fi_t, theta_t, v_t, gear_t)

    def cost_function(self, u, *args):
        state = args[0]
        ref = args[1]
        dt = args[2]
        cost = 0.0

        for k in range(0, self.horizon):
            state = self.plant_model(state, dt, u[k*2], u[k*2+1])

            # Distance from goal cost
            distance = ((ref[0] - state.position.x)**2 + abs(ref[1] - state.position.y)**2)**0.5
            cost += (3 * distance)**0.5

            # Wrong orientation cost
            delta_fi = ref[2] - state.orientation
            delta_fi = np.arctan2(np.sin(delta_fi), np.cos(delta_fi))
            cost += (abs(delta_fi))**0.7 + abs(delta_fi)/3.0

            # Distance from axis-of-goal cost
            cost += 20 * distance_to_line_sq(state, ref)
        return cost

    def get_control_signals(self, state, ref, prediction_time_step, recalculate):
        if recalculate:
            self.u_in = np.random.rand(2 * self.horizon) - 0.5
        u_solution = minimize(self.cost_function,
                              x0=self.u_in,
                              args=(state, ref, prediction_time_step),
                              method='SLSQP',
                              bounds=self.bounds,
                              tol=1e-8)

        # Recycle optimiser output
        self.u_in = np.append(u_solution.x, [u_solution.x[-2], u_solution.x[-1]])
        self.u_in = self.u_in[2:]

        # Set gear
        gear = state.gear
        if sim.is_zero(state.speed):
            if u_solution.x[0] >= 0.0:
                gear = sim.Gear.FORWARD
            else:
                gear = sim.Gear.REVERSE
        throttle, brake = signal_to_output(gear, u_solution.x[0])

        return sim.ControlSignal(throttle=throttle, brake=brake, steering_rate=u_solution.x[1], gear=gear)
