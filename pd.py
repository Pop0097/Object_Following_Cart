import time # Provides time-related functions

# Creates class object for the PD controller
class PD:
    """
        Parameters:
            @param kproportional = proportional gain
            @param kderivative  = dereivative gain
            @param minimum_output = minimum PWM output possible
            @param maximum_output = maximum PWM output possible

        @return => void function
    """

    def __init__(self, kproportional, kderivative, minimum_output, maximum_output):
        self.kp = kproportional
        self.kd = kderivative
        self.min_out = minimum_output  # Below this and motors won't spin
        self.max_out = maximum_output  # Above this and the wheels will slip
        self.past_positions = [0, 0]  # Stores past positions of the object for derivative calculations
        self.measurement_time = [time.time(), 0] # Stores the times that past measurements were taken at (for derivative calculations)

    """
        Parameters:
            @param current = current value (this will be an angle measurement or a distance measurement depending on the PD object being called)
            @param desired = desired value (this will be an angle measurement or a distance measurement depending on the PD object being called)

        @return => PWM value for motors (motor power)
    """

    def get_output(self, current, desired):

        # Calculates the error between the current and desired values
        error = abs(desired - current)

        # Created some tolerance in the error. If the error is less than 1 cm or 1 degree, then we don't power the motors.
        # The goal is to ensure the car only moves when it needs to, conserving power.
        if error < 1:
            return 0

        # Adjust past position arrays to include the most recent "current" measurement
        self.past_positions[1] = self.past_positions[0] 
        self.past_positions[0] = current

        # Adjusts the indeces in the measurement_time array to include the most recent "current" measurement
        self.measurement_time[1] = self.measurement_time[0]
        self.measurement_time[0] = time.time()

        # Calculates derivative value
        derivative = current - self.past_positions[1]

        # Calculates required motor output and ensures it stays within min and max values
        motor_output = round(error * self.kp +  derivative * self.kd)

        if motor_output < self.min_out:
            motor_output = self.min_out
        elif motor_output > self.max_out:
            motor_output = self.max_out

        return motor_output  # Returns the required PWM value




