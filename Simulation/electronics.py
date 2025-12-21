import numpy as np
from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from scipy.optimize import fsolve

def filter_ODE(t, state, vi_interp):
    vo = state[0]
    
    k = 1 / ((R_r2 + R_l2) * C_d)
    vdot = k * (vi_interp(t) - vo)
    return [vdot]

v_D_dot_PREV = 0.0

def filter2_ODE(t, state, v_Ri_interp, v_Li_interp):
    v_R, v_L, v_D = state
    global v_D_dot_PREV

    v_Ro = -C_d*R_r2*v_D_dot_PREV + v_R
    # v_Ro = v_R
    v_Lo = v_D - v_Ro

    v_R_dot = (1/C_r) * ((-1/R_r2 - 1/R_r1)*v_R + (1/R_r2)*v_Ro + (1/R_r1)*v_Ri_interp(t))
    v_L_dot = (1/C_l) * ((-1/R_l2 - 1/R_l1)*v_L + (1/R_l2)*v_Lo + (1/R_l1)*v_Li_interp(t))
    v_D_dot = -(1/(2*C_d)) * ((-v_R + v_Ro) / R_r2 + (v_L - v_Lo) / R_l2)

    v_D_dot_PREV = v_D_dot

    print(f'{t:.4f}, {v_D:.3e}')
    return [v_R_dot, v_L_dot, v_D_dot]

def exact_filter_dae(t, y, vRi_func, vLi_func):
    v_R, v_L, v_D = y
    v_Ri = vRi_func(t)
    v_Li = vLi_func(t)
    
    # Reconstruct exact output node voltages (algebraic constraints)
    # v_Ro = v_R - R_R2 * C_D * v_D_dot   # but we don't know v_D_dot yet → will be solved implicitly
    # v_Lo = v_L + R_L2 * C_D * v_D_dot   # same
    
    # We actually need to solve for the derivatives → use residual form
    # Residual function F(v̇_R, v̇_L, v̇_D) = 0
    
    def residual(dy):
        vdot_R, vdot_L, vdot_D = dy
        
        # Exact output nodes using the trial v̇_D
        v_Ro_trial = v_R - R_r2 * C_d * vdot_D
        v_Lo_trial = v_L + R_l2 * C_d * vdot_D
        
        # KCL residuals
        res1 = C_r * vdot_R - ( (v_Ri - v_R)/R_r1 - (v_R - v_Ro_trial)/R_r2 )
        res2 = C_l * vdot_L - ( (v_Li - v_L)/R_l1 - (v_L - v_Lo_trial)/R_l2 )
        res3 = C_d * vdot_D - ( (v_Ro_trial - v_R)/R_r2 - (v_Lo_trial - v_L)/R_l2 )
        
        return [res1, res2, res3]
    
    # Solve the 3×3 nonlinear system at each time step (very small and cheap)

    dy0 = [0.0, 0.0, 0.0]  # initial guess
    sol = fsolve(residual, dy0, xtol=1e-12)
    
    print(f'{t:.4f}, {sol[2]:.3e}')
    return sol  # returns [v̇_R, v̇_L, v̇_D]

def bridge(ve_plus, ve_neg, strain_R, strain_L, type='half', combine=True):
    '''
    :param ve_plus: positive excitation
    :param ve_neg: negative excitation
    :param G_L: resistance of left strain gauge
    :param G_R: resistance of right strain gauge
    :param type: bridge configuration. Both gauges on same face of part
    '''
    G_R = R_gr * (1 + GF*strain_R)
    G_L = R_gl * (1 + GF*strain_L)

    if type == 'quarter':
        v_Ri = (R_br*ve_plus + G_R*ve_neg) / (G_R + R_br)
        v_Li = (R_blq*ve_plus + R_bl*ve_neg) / (R_bl + R_blq)
        
    elif type == 'half':
        v_Ri = (R_br*ve_plus + G_R*ve_neg) / (G_R + R_br)
        v_Li = (G_L*ve_plus + R_bl*ve_neg) / (R_bl + G_L)

    else:
        raise ValueError('Bridge type not allowed. quarter or half')
    
    if combine:
        return v_Ri - v_Li
    else:
        return [v_Ri, v_Li]


class DAQ_Simulator():
    def __init__(self, t_span, dt, input_type='strain', filter_type='pi', bridge_type='half',
                 excitation_voltage=5.0):
        self.t_span = t_span
        self.t_array = np.arange(t_span[0], t_span[1]+dt, dt)
        
        self.input_type = input_type
        self.filter_type = filter_type
        self.bridge_type = bridge_type

        self.excitation_voltage = excitation_voltage
        if self.filter_type == 'diff':
            self.combine_flag = True
        elif self.filter_type== 'pi':
            self.combine_flag = False
        else:
            raise ValueError("Invalid filter type. 'diff' or 'pi'")


    def precompute_filter_input(self, A, freq):
        if self.input_type == 'bridge voltage':
            bridge_voltage = A * (np.sign(np.sin(2*np.pi*freq*self.t_array)) + 1) / 2

            electrical_noise = np.random.normal(0, noise_std, len(self.t_array))
            self.vi_precomp = bridge_voltage + electrical_noise
            self.vi_interp = interp1d(self.t_array, self.vi_precomp, kind='linear')

        elif self.input_type == 'strain':
            self.strain_R = A * (np.sign(np.cos(2*np.pi*freq*self.t_array + 3*np.pi/2)) + 1) / 2
            # self.strain_R = A * (np.cos(2*np.pi*freq*self.t_array + 3*np.pi/2) + 1) / 2
            self.strain_R[0] = 0.0
            self.strain_L = self.strain_R     # assuming left strain and right strain are equal
            bridge_excitation = self.excitation_voltage + np.random.normal(0, noise_std, len(self.t_array))
            bridge_voltage = bridge(bridge_excitation, 0.0, self.strain_R, self.strain_L, self.bridge_type, self.combine_flag)
            if self.filter_type == 'diff':
                self.vi_precomp = bridge_voltage
                self.vi_interp = interp1d(self.t_array, self.vi_precomp, kind='linear')
            
            elif self.filter_type == 'pi':
                self.v_Ri_precomp, self.v_Li_precomp = bridge_voltage
                self.v_Ri_interp = interp1d(self.t_array, self.v_Ri_precomp, kind='linear')
                self.v_Li_interp = interp1d(self.t_array, self.v_Li_precomp, kind='linear')

                self.vi_precomp = self.v_Ri_precomp - self.v_Li_precomp
                self.vi_interp = interp1d(self.t_array, self.vi_precomp, kind='linear')
            

            # Compute ideal (noise-free) bridge output for the two strain states
            strain_low  = 0.0
            strain_high = A                 # amplitude is negative for compression, but bridge is linear

            v_bridge_low  = bridge(self.excitation_voltage, 0.0, strain_low,  strain_low,  self.bridge_type)
            v_bridge_high = bridge(self.excitation_voltage, 0.0, strain_high, strain_high, self.bridge_type)

            self.v_bridge_steady_low  = v_bridge_low
            self.v_bridge_steady_high = v_bridge_high

        else:
            raise ValueError('Invalid precompute method')

    def run_sim(self):
        if self.filter_type == 'diff':
            state0 = [0.0]
            sol = solve_ivp(filter_ODE, self.t_span, state0, args=(self.vi_interp,), atol=1e-12, rtol=1e-12)
            self.time = sol.t
            self.vi = self.vi_interp(self.time)
            self.vo = sol.y[0]
         
        elif self.filter_type == 'pi':
            state0 = [self.excitation_voltage/2]*2 + [0.0]
            sol = solve_ivp(filter2_ODE, self.t_span, state0, args=(self.v_Ri_interp, self.v_Li_interp), t_eval=self.t_array)#, atol=1e-3, rtol=1e-3)
            # sol = solve_ivp(exact_filter_dae, self.t_span, state0, args=(self.v_Ri_interp, self.v_Li_interp), atol=1e-12, rtol=1e-12)
            self.time = sol.t
            self.vi = self.vi_interp(self.time)
            self.vo = sol.y[2]

        print(f'Num time steps: {len(self.time)}. From {self.time[0]} - {self.time[-1]} seconds.')


    def digitize_adc_ads1115(self, pga_gain=None, data_rate=860):
        """
        Digitize the filter output using real ADS1115 behaviour in differential mode.
        Assumes direct connection (no external amplifier).
        
        Parameters
        ----------
        pga_gain : {2/3, 1, 2, 4, 8, 16}, optional
            ADS1115 internal programmable gain amplifier setting.
            If None, uses the global ADC_GAIN defined at the top of the script.
        data_rate : int
            Samples per second: 8, 16, 32, 64, 128, 250, 475, 860 (default 860)
        """
        # Use explicit argument or fall back to the global ADC_GAIN you already defined
        gain = pga_gain if pga_gain is not None else ADC_GAIN
        print(gain)
        
        fsr_map = {2/3: 6.144, 1: 4.096, 2: 2.048, 4: 1.024, 8: 0.512, 16: 0.256}
        if gain not in fsr_map:
            raise ValueError(f'Invalid PGA gain {gain}. Must be one of {list(fsr_map.keys())}')
        
        FSR = fsr_map[gain]
        LSB = FSR / 32768.0

        # Resample filter output at the chosen ADC data rate
        t_adc = np.arange(self.time[0], self.time[-1] + 1/(2*data_rate), 1.0/data_rate)
        vo_adc = np.interp(t_adc, self.time, self.vo * gain)

        # Direct differential input → no external gain multiplication
        counts = np.round(vo_adc / LSB).astype(np.int32)
        counts = np.clip(counts, -32768, 32767)

        # Store results
        self.adc_time        = t_adc
        self.adc_counts      = counts.astype(np.int16)
        self.adc_voltage     = counts * LSB                 # reconstructed ideal voltage
        self.adc_lsb         = LSB
        self.adc_fsr         = FSR
        self.adc_pga         = gain
        self.adc_sps         = data_rate

    def display_simulation(self):
        # Bridge and ADC voltages
        plt.figure(1, figsize=(10, 6))
        plt.plot(self.time, self.vi, label='Bridge Voltage', linestyle='--')
        plt.plot(self.time, self.vo, label='Filter Output')
        if hasattr(self, 'adc_counts'):
            plt.step(self.adc_time, self.adc_voltage, where='post', label='ADS1115 Output', linewidth=0.5)
            plt.axhline(self.v_bridge_steady_high*ADC_GAIN*0.95, c='purple')
            plt.axhline(self.v_bridge_steady_high*ADC_GAIN*0.05, c='purple')
        plt.axvline(0.01, c='green')
        plt.axvline(0.11, c='green')

        if self.input_type == 'bridge voltage':
            plt.axhline(amplitude*0.95, c='red')
            plt.axhline(amplitude*0.05, c='red')
        elif self.input_type == 'strain':
            plt.axhline(self.v_bridge_steady_high*0.95, c='red', label=r'95% Final Value')
            plt.axhline(self.v_bridge_steady_high*0.05, c='red')


        plt.xlabel('Time (s)')
        plt.ylabel('Voltage (V)')
        plt.title('Simulation of DAQ Performance')
        plt.ylim(max(-5, np.min(self.vo)), min(5, np.max(self.vo)))
        plt.legend()
        plt.grid(True)
        
        # if self.input_type == 'strain':
        #     plt.figure(2, figsize=(10, 6))
        #     plt.plot(self.t_array, self.strain_R*1e6, label='strain_R')
        #     plt.plot(self.t_array, self.strain_L*1e6, label='strain_L')
        #     plt.xlabel('Time (s)')
        #     plt.ylabel('Strain (µε)')
        #     plt.legend()
        #     plt.grid(True)


if __name__ == "__main__":
    # Component values
        # Analog-to-Digital-Converter
    ADC_GAIN = 16
        # filter
    R_r2 = 300      # ohm, resistor on right leg of differential RC filter
    R_l2 = R_r2         # ohm, resistor on left leg of differential RC filter
    C_d = 0.1e-6         # farad, capacitor between legs of differential RC filter
    R_r1 = 3000         # ohm, resistor on right leg of single RC filter
    R_l1 = R_r1       # ohm, resistor on left leg of single RC filter
    C_r = 1e-6         # farad, capacitor between right leg and GND in single RC filter
    C_l = C_r         # farad, capacitor between left leg and GND in single RC filter

        # strain gauge wheatstone bridge
    R_br = 350          # ohm, resistor on right side of wheatstone bridge
    R_bl = R_br         # ohm, resistor on left side of wheatstone bridge
    R_blq = R_bl        # ohm, resistor of left side of wheatstone bridge in quarter config. Replaces left strain gauge.
    R_gr = 350          # ohm, unstrained resistance of right strain gauge
    R_gl = R_gr         # ohm, unstrained resistance of left strain gauge
    GF = 2.0            # gauge factor of strain gauges. Assumed identical for right and left gauges

    # Signal characteristics
    # input_type            # 'bridge voltage' controls bridge voltage directly as input. 'strain' uses strain as input, passing through the wheatstone bridge
    sig_freq = 5            # Hz. 5 is one rise/fall per 0.1 seconds
    amplitude = -1000e-6       # volt for 'bridge voltage', strain for 'strain', negative strain is compression
    noise_std = 50e-3       # standard deviation of electical noise injected on top of input
                            # volt for 'bridge voltage', added to generated bridge voltage
                            # volt for 'strain', added to bridge excitation 
    np.random.seed(42)

    # Initialize simulator
    sim = DAQ_Simulator(t_span=[0, 0.2], dt=0.001, input_type='strain', excitation_voltage=5.0, filter_type='pi')

    # Create interpolator for consistent input to different functions
    sim.precompute_filter_input(amplitude, sig_freq)   # allow solve_ivp to interpolate between generated input steps

    # Solve the ODE using the interpolated input
    sim.run_sim()
    # sim.digitize_adc_ads1115(data_rate=860)
    
    # Plot the results
    sim.display_simulation()
    plt.show()