class PID_Controller:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.setpoint = setpoint  # Desired target value

        self.prev_error = 0  # Store previous error for derivative term
        self.integral = 0  # Store accumulated integral

    def update(self, current_value, dt):
        # Calculate error
        error = self.setpoint - current_value

        # Proportional term
        P = self.kp * error

        # Integral term
        self.integral += error * dt
        I = self.ki * self.integral

        # Derivative term
        derivative = (error - self.prev_error) / dt
        D = self.kd * derivative

        # Update previous error
        self.prev_error = error

        # PID output
        output = P + I + D
        return output
