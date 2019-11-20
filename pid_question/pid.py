class PIDController:
    def __init__(self, target_pos):
        self.target_pos = target_pos
        self.Kp = 0.0
        self.Ki = 0.0
        self.Kd = 0.0
        self.bias = 0.0
        return

    def reset(self):
        return

#TODO: Complete your PID control within this function. At the moment, it holds
#      only the bias. Your final solution must use the error between the 
#      target_pos and the ball position, plus the PID gains. You cannot
#      use the bias in your final answer. 
    def get_fan_rpm(self, vertical_ball_position):
        output = self.bias
        return output
