#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    :2024/10/5
# @Author  :WuJunYi
# @File    :Motion.py
# @Software:PyCharm
import time
import xgolib
import math

# 定义Motion类，用于控制机器的运动相关操作
class Motion:
	def __init__(self):
		"""
		类的构造函数，用于初始化Motion类的实例。

		在这里创建了一个与机器设备通信的对象，并获取机器初始的偏航角（yaw）值，
		以便后续在运动控制中作为参考基准。
		"""
		self.move = xgolib.XGO("/dev/ttyAMA0")
		self.start_yaw = self.move.read_yaw()
		self.st_time = time.time()
		self.offsetk = -0.0

		# 死区补偿参数
		self.dead_zone_threshold = 6.8
		self.min_effective_speed = 15  # 略高于死区阈值

		# 低通滤波器参数 (简化实现)
		self.alpha = 0.5  # 滤波系数 (0-1，越小越平滑)
		self.prev_filtered = 0.0

		# PID参数
		self.kp = 0.6
		self.ki = 0.1  # 新增积分项
		self.kd = 0.2   # 新增微分项
		self.prev_error = 0.0
		self.integral = 0.0
		self.integral_limit = 15.0  # 积分限幅

        # 平滑控制参数
		self.smooth_zone = math.radians(10)  # 平滑过渡区角度(弧度)

	def dead_zone_compensation(self, speed):
		"""死区补偿函数"""
		if speed > self.dead_zone_threshold:
			return speed - self.dead_zone_threshold
		elif speed < -self.dead_zone_threshold:
			return speed + self.dead_zone_threshold
		return 0.0

	def lowpass_filter(self, angle):
		"""简易低通滤波器实现"""
		filtered = self.alpha * angle + (1 - self.alpha) * self.prev_filtered
		self.prev_filtered = filtered
		return filtered

	def normalize_angle_deg(self, angle_deg):
		"""把角度归一化到 [0, 360) 区间，避免 -90° 和 270° 的歧义。"""
		return ((angle_deg % 360) + 360) % 360

	def calculate_error(self, target_angle, current_angle):
		"""计算带方向的最短角度误差，支持 -90° / 270° 这类同一物理方向的表示。"""
		target_deg = self.normalize_angle_deg(math.degrees(target_angle))
		current_deg = self.normalize_angle_deg(math.degrees(current_angle))

		# 计算当前角度到目标角度的最短转角，结果落在 [-180, 180)
		error_deg = (target_deg - current_deg + 540) % 360 - 180
		error_rad = math.radians(error_deg)
		return abs(error_rad), 1.0 if error_deg >= 0 else -1.0

	def smooth_sign(self, error_rad):
		"""平滑过渡函数"""
		abs_error = abs(error_rad)
		if abs_error > self.smooth_zone:
			return 1.0 if error_rad > 0 else -1.0

		# 在平滑区内创建渐变响应
		# return error_rad / self.smooth_zone
		return math.sin(error_rad * math.pi / (2 * self.smooth_zone))

	def change_yaw(self, target_angle_deg):
		"""
		优化版偏航角控制
		:param target_angle_deg: 目标偏航角(角度制)
		"""
		# 重置滤波状态
		self.prev_filtered = 0.0

		# 角度转换
		target_angle = math.radians(target_angle_deg)
		timeout = 6.0
		start_time = time.time()

		while True:
			# 1. 读取并滤波当前角度
			raw_angle_rad = math.radians(
				self.move.read_yaw() - self.start_yaw - self.offsetk*(time.time()-self.st_time)
			)
			current_angle = self.lowpass_filter(raw_angle_rad)

			# 2. 计算带方向的角度误差
			error_rad, error_sign = self.calculate_error(target_angle, current_angle)
			error_deg = math.degrees(error_rad)
			signed_error_deg = error_deg * error_sign

			# 3. PID控制器计算
			# P项
			p_term = self.kp * signed_error_deg

			# I项 (带积分限幅)
			self.integral += self.ki * signed_error_deg
			self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))

			# D项 (仅当有足够历史数据时)
			d_term = 0.0
			if self.prev_error is not None:
				d_term = self.kd * (signed_error_deg - self.prev_error) / 0.1
			self.prev_error = signed_error_deg

			# 合成输出
			pid_output = p_term + self.integral + d_term
			pid_output = max(-225, min(pid_output, 225))  # 限幅

			# 4. 控制输出计算
			if error_rad < math.radians(0.8):  # 接近目标区
				output_speed = 0.0
			else:
				# 平滑过渡计算
				sign_factor = self.smooth_sign(error_rad * error_sign)
				base_speed = self.min_effective_speed * sign_factor

				# 根据误差大小叠加增量
				scaled_speed = pid_output * 0.5  # 比例系数
				output_speed = base_speed + scaled_speed

				# 死区补偿
				output_speed = self.dead_zone_compensation(output_speed)

			# 5. 执行控制
			#print(output_speed)
			self.move.turn(output_speed)

			# 6. 退出条件检查
			current_time = time.time()
			if error_rad < math.radians(0.8):  # 成功条件
				self.move.turn(0.0)
				print(f"控制完成，最终误差: {signed_error_deg:.2f}度")
				break
			elif current_time - start_time > timeout:
				self.move.turn(0.0)
				print(f"超时退出，当前误差: {signed_error_deg:.2f}度")
				break

			time.sleep(0.1)

		time.sleep(0.5)

		# 重置控制状态
		self.prev_error = None
		self.integral = 0.0

	def go_straight(self, speed, target_angle_deg):
		"""
		使机器狗在保持目标偏航角的同时直线运动（PID闭环纠偏）
		:param speed: 前进速度 (move_x)
		:param target_angle_deg: 期望保持的绝对偏航角(角度制)，0表示初始正前方
		"""
		# 1. 角度转换
		target_angle = math.radians(target_angle_deg)

		# 2. 读取并滤波当前角度 (参考 change_yaw 里的偏航校准逻辑)
		raw_angle_rad = math.radians(
			self.move.read_yaw() - self.start_yaw - self.offsetk*(time.time()-self.st_time)
		)
		current_angle = self.lowpass_filter(raw_angle_rad)

		# 3. 计算误差
		error_rad, error_sign = self.calculate_error(target_angle, current_angle)
		error_deg = math.degrees(error_rad)
		signed_error_deg = error_deg * error_sign

		# 4. PID计算纠偏速度 (使用类中定义的 kp, ki, kd)
		p_term = self.kp * signed_error_deg
		self.integral += self.ki * signed_error_deg
		self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))

		d_term = 0.0
		if self.prev_error is not None:
			# 这里的 0.05 对应主程序循环的 sleep 时间
			d_term = self.kd * (signed_error_deg - self.prev_error) / 0.05
		self.prev_error = signed_error_deg

		pid_output = p_term + self.integral + d_term

		# 限制纠偏转速，防止直行时剧烈晃动 (参考 dead_zone_compensation 逻辑)
		pid_output = max(-60, min(pid_output, 60))

		# 5. 执行控制：设定前进速度 + 叠加 PID 算出来的转向纠偏速度
		self.move.move_x(speed)
		self.move.turn(pid_output)

	def to_first_image(self):
		"""
        控制设备移动到第一个目标位置的操作。

        先使设备在x轴方向上以一定速度移动一段时间，然后停止移动并暂停一段时间。
        """
		self.move.move_x(10)
		time.sleep(2.5)
		self.move.move_x(0)
		time.sleep(0.5)

	def to_second_image(self):
		"""
        控制设备移动到第二个目标位置的操作。

        先使设备绕yaw轴旋转一定角度，然后在x轴方向上以一定速度移动一段时间，最后停止移动并暂停一段时间。
        """
		self.move.turn(25)
		time.sleep(2.5)
		self.move.turn(0)
		time.sleep(0.5)

		self.move.move_x(19)
		time.sleep(5.0)
		self.move.move_x(0)
		time.sleep(0.5)
	# def to_second_image(self):
	# 	"""
	# 	控制设备移动到第二个目标位置的操作。
	# 	先使设备绕yaw轴旋转一定角度，然后在x轴方向上以一定速度移动一段时间，最后停止移动并暂停一段时间。
	# 	"""
	# 	self.go_left()
	# 	time.sleep(0.5)
	# 	self.go_left()
	# 	time.sleep(0.5)
	# 	self.go_a_block()
	# 	time.sleep(0.5)

	def to_connect1(self):
		"""
        控制设备移动到连接点1的操作。

        涉及到设备在x轴方向和yaw轴方向上的多次移动和旋转操作，以逐步到达目标位置并调整到特定的偏航角状态。
        """
		self.change_yaw(-45)
		time.sleep(0.5)

		self.move.move_x(18)
		time.sleep(5.8)
		self.move.move_x(0)
		time.sleep(0.5)
		'''
		self.move.turn(22)
		time.sleep(3)
		self.move.turn(0)
		time.sleep(0.5)'''

		self.change_yaw(0)

		self.move.move_x(19)
		time.sleep(4.0)
		self.move.move_x(0)
		time.sleep(0.5)
		'''
		self.move.turn(32)
		time.sleep(3)
		self.move.turn(0)
		time.sleep(0.5)'''

		self.change_yaw(90)

		self.move.move_x(20)
		time.sleep(8.0)
		self.move.move_x(0)
		time.sleep(0.5)
