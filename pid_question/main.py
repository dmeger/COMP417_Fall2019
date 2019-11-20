#!/usr/bin/python

__author__ = "Travis Manderson"
__copyright__ = "Copyright 2018, Travis Manderson"

import sim_env
import pid_plotter_vispy
from multiprocessing import Process, Manager, Lock
import multiprocessing
import sys


def run_simulator(graph_index, graph_time, graph_position, graph_error, graph_fan, graph_target, validation_mode, target_height):
    global xdata
    global ydata
    env = sim_env.env(graph_index, graph_time, graph_position, graph_error, graph_fan, graph_target)
    if validation_mode:
        env.run_validation(target_height)
    else:
        env.run()

def run_pid_plotter(graph_index, graph_time, graph_position, graph_error, graph_fan, graph_target):
    plotter = pid_plotter_vispy.PIDPlotter(graph_index, graph_time, graph_position, graph_error, graph_fan, graph_target)
    plotter.run()
    return


if __name__ == '__main__':
    print("welcome to the ping-pong ball simulator")
    validation_mode = False
    target_height = 0.0
    exit = False
    while not exit:
        print('e - Experiment Mode')
        print('v [target height] - Validation Mode')
        inputString = raw_input('Select Job To Run: ')
        # inputString = "v 0.5"
        # inputString = "e"
        commands = inputString.split(";")
        for command in commands:
            argList = command.strip()
            argList = argList.split(" ")
            job = argList[0]
            if job == 'v':
                if len(argList) > 1:
                    validation_mode = True
                    target_height = float(argList[1])
                else:
                    print('please enter a target height')
                    break
            if job == 'e' or job == 'v':
                # graph_lock = Lock()
                max_size = 432000 #1 hour at 120 fps
                graph_time = multiprocessing.Array('d', max_size)
                graph_position = multiprocessing.Array('d', max_size)
                graph_error = multiprocessing.Array('d', max_size)
                graph_fan = multiprocessing.Array('d', max_size)
                graph_target = multiprocessing.Array('d', max_size)
                graph_index = multiprocessing.Value('i')
                graph_index.value = 0

                if not sys.platform == "darwin":
                    processes = []
                    process_plotter = Process(target=run_pid_plotter,
                                              args=(graph_index, graph_time, graph_position, graph_error, graph_fan, graph_target,))
                    process_plotter.start()
                    processes.append(process_plotter)

                    process_sim = Process(target=run_simulator, args=(graph_index, graph_time, graph_position, graph_error, graph_fan, graph_target,validation_mode,target_height))
                    process_sim.start()
                    processes.append(process_sim)

                    for process in processes:
                        process.join()
                    print("Exiting Main Thread")
                else:
                    env = sim_env.env(graph_index, graph_time, graph_position, graph_error, graph_fan, graph_target)
                    if validation_mode:
                        env.run_validation(target_height)
                    else:
                        env.run()
