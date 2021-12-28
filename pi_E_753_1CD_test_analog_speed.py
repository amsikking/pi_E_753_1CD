import time
import numpy as np
import pi_E_753_1CD
import ni_PXIe_6739

# init:
ao_piezo_channel = 6
ao = ni_PXIe_6739.DAQ(num_channels=ao_piezo_channel, rate=1e4, verbose=False)
piezo = pi_E_753_1CD.Controller(which_port = 'COM6', verbose=False)

# configure:
num_moves = 31
ao_play_seconds = 0.005 # is ~5ms enough step and settle time? (test and check)
z_min, z_max = piezo.z_min, piezo.z_max
moves = np.linspace(z_min, z_max, num_moves)
piezo.set_analog_control_limits(0, 10, z_min, z_max) # -10 -> 10 = best control

# run servo:
print('Testing servo control speed:')
start = time.perf_counter()
for move_um in moves:
    piezo.move_um(move_um, relative=False)
servo_time_s = time.perf_counter() - start
print('servo  time (s) = ', servo_time_s)

# run analog:
print('Testing analog control speed:')
voltages = []
for move_um in moves:
    z_voltage = piezo.get_voltage_for_move_um(move_um, relative=False)
    ao_volts = np.zeros((ao.s2p(ao_play_seconds), ao.num_channels), 'float64')
    ao_volts[:, ao_piezo_channel - 1] = z_voltage
    voltages.append(ao_volts)
voltages = np.concatenate(voltages, axis=0)

start = time.perf_counter()
piezo.set_analog_control_enable(True)
ao.play_voltages(voltages, force_final_zeros=False)
piezo.set_analog_control_enable(False)
analog_time_s = time.perf_counter() - start
print('analog time (s) = ', analog_time_s)

# difference:
speed_up = servo_time_s / analog_time_s
print('Analog speed up ~ %0.1fx'%speed_up)

# tidy up:
piezo.move_um(0, relative=False)
piezo.close()
ao.close()
