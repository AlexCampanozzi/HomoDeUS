import rospy


class PID:
	def __init__(self, ref_x, ref_y, K_P = 1., K_I = 1., K_D = 1.):
		self.ref_x = ref_x
		self.ref_y = ref_y

		self.K_P = K_P
		self.K_I = K_I
		self.K_D = K_D

		self.sum_x = 0.
		self.sum_y = 0.

		self.last_timestamp = rospy.get_time()

		self.last_error_x = 0.
		self.last_error_y = 0.


  	def get_next_command(self, pos_x, pos_y, epsilon = 1e-20):
		err_x = pos_x - self.ref_x
		err_y = pos_y - self.ref_y

		now = rospy.get_time()
		dT = now - self.last_timestamp

		# Proportial term
		Px = self.K_P * err_x
		Py = self.K_P * err_y

		# Integration term
		self.sum_x += err_x * dT
		self.sum_y += err_y * dT

		Ix = self.K_I * self.sum_x
		Iy = self.K_I * self.sum_y

		# Derivative term
		dErr_x = (err_x - self.last_error_x) / (dT + epsilon)
		dErr_y = (err_y - self.last_error_y) / (dT + epsilon)

		Dx = self.K_D * dErr_x
		Dy = self.K_D * dErr_y

		cmd_x = Px + Ix + Dx
		cmd_y = Py + Iy + Dy

		self.last_timestamp = now

		return cmd_x, cmd_y

	def set_coefficients(K_P, K_I, K_D):
		self.K_P = K_P
		self.K_I = K_I
		self.K_D = K_D