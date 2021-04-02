import rospy


class PID:
	def __init__(self, ref, K_P = 1.0, K_I = 1.0, K_D = 1.0):
		self.ref = ref

		self.K_P = K_P
		self.K_I = K_I
		self.K_D = K_D

		self.sum = 0.

		self.last_timestamp = rospy.get_time()

		self.last_error = 0.


  	def get_next_command(self, pos, epsilon = 1e-20):
		err = pos - self.ref

		now = rospy.get_time()
		dT = now - self.last_timestamp

		# Proportial term
		P = self.K_P * err

		# Integration term
		self.sum += err * dT

		I = self.K_I * self.sum

		# Derivative term
		dErr = (err - self.last_error) / (dT + epsilon)

		D = self.K_D * dErr

		cmd = P + I + D

		self.last_timestamp = now

		return cmd

	def set_coefficients(K_P, K_I, K_D):
		self.K_P = K_P
		self.K_I = K_I
		self.K_D = K_D