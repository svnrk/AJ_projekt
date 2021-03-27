import simulator
import visualizer
import mpc2
import pygame
import numpy as np

active_parking_spot = 1
recalculate = False
simulation_time = 40
cs_s = []
counter = 0


def controller(x: simulator.State, dt: float, time: float):
    global active_parking_spot, recalculate
    running = True
    for event in pygame.event.get():
        if event.type == pygame.QUIT: running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_0:
                active_parking_spot = 0
            elif event.key == pygame.K_1:
                active_parking_spot = 1
            elif event.key == pygame.K_2:
                active_parking_spot = 2
            elif event.key == pygame.K_3:
                active_parking_spot = 3
            elif event.key == pygame.K_4:
                active_parking_spot = 4
            elif event.key == pygame.K_q:
                running = False
            recalculate = True
    print(f"time: {time:.1f}")

    if car_is_in_box(x, p_list[active_parking_spot]):
        if simulator.is_zero(x.speed):
            active_parking_spot += 1
            if active_parking_spot == len(p_list): active_parking_spot = 0
    visualizer.update(x.position.x, x.position.y, x.orientation, active_parking_spot)
    recalculate = True if ((not recalculate and simulator.is_zero(x.speed)) or time % 10 < 0.1) else False

    cs = mpc.get_control_signals(x, p_list[active_parking_spot], 0.3, recalculate=recalculate)

    if simulation_time < time or not running:
        x.position.x, x.position.y, x.orientation = 0, 0, 0
        simulator.run(replay)

    cs_s.append(cs)
    return cs


def replay(x: simulator.State, dt: float, time: float):
    print(f"time: {time:.1f}")
    global counter
    running = True if counter < len(cs_s) - 1 else False
    for event in pygame.event.get():
        if event.type == pygame.QUIT: running = False

    visualizer.update(x.position.x, x.position.y, x.orientation, -1, replay=True)
    if simulation_time < time or not running:
        simulator.close()
        visualizer.close()

    cs = cs_s[counter]
    counter += 1
    return cs


half_length = 0.875
half_width = 0.6
def car_is_in_box(state, ref):
    delta_x = state.position.x - ref[0]
    delta_y = state.position.y - ref[1]
    if delta_x < 0.5 and delta_y < 0.5:
        delta_fi = ref[2] - state.orientation
        delta_fi = np.arctan2(np.sin(delta_fi), np.cos(delta_fi))

        hl_1 = half_length * np.cos(delta_fi) + half_width * np.sin(delta_fi)
        hw_1 = half_width * np.cos(delta_fi) + half_length * np.sin(delta_fi)

        R = np.array(((np.cos(-ref[2]), -np.sin(-ref[2])),
                      (np.sin(-ref[2]), np.cos(-ref[2]))))
        delta_1 = R.dot([delta_x, delta_y])
        if abs(delta_1[0]) < 0.9 - hw_1 and abs(delta_1[1]) < 1.175 - hl_1 and delta_fi < 0.411:
            return True
    return False


def list_from_file(file_in):
    output_list = []
    with open(file_in) as file:
        for line in file:
            p = line.split(':')
            p_ = [float(i) for i in p]
            output_list.append(p_)
    return output_list


p_list = list_from_file("parking_spots.txt")
mpc = mpc2.ModelPredictiveControl()
visualizer.visualize(p_list)
simulator.run(controller)
