class PID_Controller:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint

        self.prev_error = 0  # Store previous error for derivative term
        self.integral = 0  # Store accumulated integral

    def update(self, current_value, dt):
        error = self.setpoint - current_value
        P = self.kp * error

        self.integral += error * dt
        I = self.ki * self.integral

        derivative = (error - self.prev_error) / dt
        D = self.kd * derivative

        self.prev_error = error

        output = P + I + D
        return output
