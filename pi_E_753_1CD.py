import time
import serial

class Controller:
    '''
    Basic device adaptor for PI E-753.1CD, high-speed, single-axis, digital
    piezo controller. Many more commands are available and have not been
    implemented.
    '''
    def __init__(self,
                 which_port,            # COM port for controller
                 z_tol_um=0.25,         # tolerance on positional error (um)
                 name='E-753.1CD',      # optional name
                 verbose=True,          # False for max speed
                 very_verbose=False):   # True for debug
        self.z_tol_um = z_tol_um
        self.name = name
        self.verbose = verbose
        self.very_verbose = very_verbose
        if self.verbose: print('%s: Opening...'%name, end='')
        try:
            self.port = serial.Serial(
                port=which_port, baudrate=115200, timeout=5)
        except serial.serialutil.SerialException:
            raise IOError('%s: No connection on port %s'%(name, which_port))
        if self.verbose: print(" done.")
        # get device identity:
        self.identity = self._send('*IDN?')[0]
        # get physical units:
        self.port.write(b'SPA? 1 0x07000601\n')
        self.unit =self.port.readline(
            ).rstrip().decode('latin-1').split('=')[1]
        assert self.unit == 'µM'
        # get position and position limits:
        self.get_position(verbose=False)
        self.z_min = float(self._send('TMN?')[0].split('=')[1])
        self.z_max = float(self._send('TMX?')[0].split('=')[1])
        self._moving = False
        self._analog_control = False
        # move z to legal position if needed:
        if self.z <= self.z_min + self.z_tol_um:
            self.move_um(self.z_min + self.z_tol_um, relative=False)
        if self.z >= self.z_max - self.z_tol_um:
            self.move_um(self.z_max - self.z_tol_um, relative=False)
        # set state:
        self._set_servo_enable(True) # closed loop control
        self._set_analogue_control_limits = False
        self._send('CCL 1 advanced', respond=False) # need >= 'cmd level 1'
        self._send('SPA 1 0x06000500 0', respond=False) # disable analog
        if self.verbose:
            self._print_attributes()
        return None

    def _print_attributes(self):
        print("%s: device identity"%self.name)
        print("%s:  = %s"%(self.name, self.identity))
        print("%s: units"%self.name)
        print("%s:  = %s"%(self.name, self.unit))
        print("%s: position (z)"%self.name)
        print("%s:  = %10.06f (um)"%(self.name, self.z))
        print("%s: position limits (z_min, z_max)"%self.name)
        print("%s:  = %10.06f, %10.06f (um)"%(
            self.name, self.z_min, self.z_max))

    def _send(self, cmd, respond=True):
        if self.very_verbose:
            print("%s: sending cmd = "%self.name, cmd)
        cmd = bytes(cmd, encoding='ascii')
        self.port.write(cmd + b'\n')
        if respond:
            responses = []
            while True:
                response = self.port.readline()
                assert response.endswith(b'\n') # default terminator
                responses.append(response.rstrip().decode('ascii')) # strip ' '
                if len(response) == 1: break # = 1 for self._reboot()
                if response[-2] != 32: break # ASCII #32 = space -> not finished
        else:
            responses = None
        if self.very_verbose:
            print("%s:  response   = "%self.name, responses)
        assert self.port.in_waiting == 0
        self._check_errors()
        return responses

    def _check_errors(self):
        self.port.write(b'ERR?\n')  # Get Error Number -> check with manual
        self.error = self.port.readline()
        if self.error != b'0\n':    # 0 = no error
            raise RuntimeError(
                "%s: error = "%self.name, self.error.decode("ascii"))
        return None

    def _get_cmd_list(self):
        if self.verbose:
            print("%s: getting list of available commands"%self.name)
        self.cmd_list = self._send('HLP?')
        if self.verbose:
            print("%s:  available commands -> "%self.name)
            for cmd in self.cmd_list:
                print(cmd)
        return self.cmd_list

    def _get_parameter_list(self):
        if self.verbose:
            print("%s: getting list of available parameters"%self.name)
        self.parameter_list = self._send('HPA?')
        if self.verbose:
            print("%s:  available parameters -> "%self.name)
            for parameter in self.parameter_list:
                print(parameter)
        return self.parameter_list

    def _get_parameter(self, p_id):
        if self.verbose:
            print("%s: getting parameter %s"%(self.name, p_id))
        value = self._send('SPA? 1 %s'%p_id)
        if self.verbose:
            print("%s:  parameter value %s"%(self.name, value))
        return value

    def _set_parameter(self, p_id, value): # type(p_id) = int, float or char
        # -> check with docs or ._get_parameter_list() to see type and options
        v = str(value)
        if self.verbose:
            print("%s: setting parameter %s = "%(self.name, p_id) + v )
        self._send('SPA 1 '+ p_id + ' ' + v + ' ', respond=False)
        if self.verbose:
            print("%s:  finished setting parameter"%self.name)
        return None

    def _set_servo_enable(self, enable):
        if enable:
            self._send('SVO 1 1', respond=False)
        if not enable:
            self._send('SVO 1 0', respond=False)
        if self.very_verbose:
            print("%s: servo enabled = %s"%(self.name, enable))
        return None

    def _finish_moving(self):
        if not self._moving:
            return None
        while True:
            self.port.write(b'\x05') # Request Motion Status
            response = self.port.read(2)
            if response == b'0\n': break
        self._moving = False
        if self.verbose: print('%s:  -> finished moving'%self.name)
        self._check_errors()
        return None

    def get_position(self, verbose=True):
        if verbose: print("%s: position (z)"%self.name)
        self.z = float(self._send('POS?')[0].split('=')[1])
        if verbose: print("%s:  = %10.06f (um)"%(self.name, self.z))
        return self.z

    def move_um(self, z, relative=True, block=True):
        self._finish_moving()
        assert not self._analog_control
        if relative:
            self.z = float(self.z + z)
            cmd = 'MOV 1 %0.9f'%self.z
        if not relative: # Abolute move
            self.z= float(z)
            cmd = 'MOV 1 %0.9f'%self.z
        assert self.z_min <= self.z <= self.z_max
        if self.verbose:
            print("%s: moving to (z)"%self.name)
            print("%s:  = %10.06f (um)"%(self.name, self.z))
        self._send(cmd, respond=False)
        self._moving = True
        if block:
            self._finish_moving()
        return None

    def set_analog_control_limits(
        self, v_min=None, v_max=None, z_min_ai=None, z_max_ai=None):
        if self.verbose:
            print("%s: setting analog control limits"%self.name,
                  "(v_min, v_max) and (z_min_ai, z_max_ai)")
        assert v_min < v_max            # max and min voltages (v)
        assert -10 <= v_min <= 10 and -10 <= v_max <= 10
        assert z_min_ai < z_max_ai      # max and min positions (um)
        assert self.z_min <= z_min_ai and z_max_ai <= self.z_max
        assert self.z >= z_min_ai - self.z_tol_um, 'current z out of range'
        assert self.z <= z_max_ai + self.z_tol_um, 'current z out of range'
        # gain and offset from manual:
        self.gain = 0.1 * (z_max_ai - z_min_ai) / (v_max - v_min)
        self.offset = z_max_ai - self.gain * (10 * v_max)
        self._send('SPA 2 0x02000300 %0.9f'%float(self.gain), respond=False)
        self._send('SPA 2 0x02000200 %0.9f'%float(self.offset), respond=False)
        if self.verbose:
            print("%s:  = (%5.03f, %5.03f)v and (%5.03f, %5.03f)um"%(
                self.name, v_min, v_max, z_min_ai, z_max_ai))
        self.v_min, self.v_max = v_min, v_max
        self.z_min_ai, self.z_max_ai = z_min_ai, z_max_ai
        self._set_analogue_control_limits = True
        return None

    def get_voltage_for_move_um(self, z, relative=True):
        if self.verbose:
            print("%s: voltage for move um (z_voltage)"%self.name)
        assert self._set_analogue_control_limits, 'these must be set first'
        if relative: z = self.z + z
        assert z >= self.z_min_ai - self.z_tol_um, 'requested position too low'
        assert z <= self.z_max_ai + self.z_tol_um, 'requested position too high'
        # ScaledValue = OFFSET + GAIN * NormalizedValue (from manual)
        # z = self.offset + self.gain * (10 * z_voltage)
        z_voltage = float(z - self.offset) / (self.gain * 10)
        if z_voltage < self.v_min: z_voltage = self.v_min # legalize edge cases
        if z_voltage > self.v_max: z_voltage = self.v_max
        if self.verbose:
            print("%s:  = %10.06f (v)"%(self.name, z_voltage))
        return z_voltage

    def set_analog_control_enable(self, enable):
        if self.verbose:
            print("%s: setting analog control enable = %s"%(self.name, enable))
        assert self._set_analogue_control_limits, 'these must be set first'        
        if enable:
            self._send('SPA 1 0x06000500 2', respond=False) # enable analog
        if not enable: # read z from voltage on controller for current position
            z_target = float(self._send('TSP? 2')[0].split('=')[1])
            if z_target < self.z_min: z_target = self.z_min # legalize edges
            if z_target > self.z_max: z_target = self.z_max
            self._send('SPA 1 0x06000500 0', respond=False) # disable analog
            self._send('MOV 1 %0.9f'%z_target, respond=False) # zero movement
        self._analog_control = enable
        return None
        
    def close(self):
        if self.verbose: print("%s: closing..."%self.name)
        self._finish_moving()
        if self._analog_control:
            self.set_analog_control_enable(False)
        self.port.close()
        if self.verbose: print("%s: closed."%self.name)
        return None
    
if __name__ == '__main__':
    piezo = Controller(which_port = 'COM6', verbose=True, very_verbose=False)
    # test developer functions:
##    piezo._get_cmd_list()
##    piezo._get_parameter_list()
##    piezo._get_parameter('0x07000901')
##    piezo._set_parameter('0x07000901', 0.00000000)

    print('\nAbsolute and relative moves:')
    piezo.move_um(0, relative=False)
    piezo.move_um(10)

    print('\nNon-blocking call:')
    piezo.move_um(10, relative=False, block=False)
    print(' do something else...')
    piezo.move_um(0, relative=False)

    print('\nSwitch to analog control and back:')
    piezo.move_um(50, relative=False)               # move somewhere
    piezo.set_analog_control_limits(0, 10, 0, 100)  # configure analog
    # get voltage to feed into analog control 
    z_voltage = piezo.get_voltage_for_move_um(0)    # for ~zero motion
    piezo.set_analog_control_enable(True)           # enable analog control
    print(' (do some fast analog motion...)')
##    input() # optional pause and play with voltage e.g. apply z_voltage
    piezo.set_analog_control_enable(False)          # return to servo control
    piezo.move_um(0, relative=False)                # move somewhere else
    
    piezo.close()
