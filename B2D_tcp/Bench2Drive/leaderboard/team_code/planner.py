import os
from collections import deque
import logging
import numpy as np
import math
EARTH_RADIUS_EQUA = 6378137.0

# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

DEBUG = int(os.environ.get('HAS_DISPLAY', 0))

class Plotter(object):
	def __init__(self, size):
		self.size = size
		self.clear()
		self.title = str(self.size)

	def clear(self):
		from PIL import Image, ImageDraw
		self.img = Image.fromarray(np.zeros((self.size, self.size, 3), dtype=np.uint8))
		self.draw = ImageDraw.Draw(self.img)

	def dot(self, pos, node, color=(255, 255, 255), r=2):
		x, y = 5.5 * (pos - node)
		x += self.size / 2
		y += self.size / 2
		self.draw.ellipse((x-r, y-r, x+r, y+r), color)

	def show(self):
		if not DEBUG:
			return
		import cv2
		cv2.imshow(self.title, cv2.cvtColor(np.array(self.img), cv2.COLOR_BGR2RGB))
		cv2.waitKey(1)

class RoutePlanner(object):
	def __init__(self, min_distance, max_distance, debug_size=256, lat_ref=0.0, lon_ref=0.0):
		print("planner init")

		self.route = deque()
		
		self.min_distance = min_distance
		
		self.max_distance = max_distance
		
		self.mean = np.array([0.0, 0.0])
		
		self.scale = np.array([111324.60662786, 111319.490945])
		
		self.debug = Plotter(debug_size)
		
		self.lat_ref = lat_ref
		
		self.lon_ref = lon_ref
		
		self.logger = logger

	def set_route(self, global_plan, gps=False, global_plan_world=None):
		self.route.clear()
		self.logger.info(f"Setting route with {len(global_plan)} waypoints")

		if global_plan_world:
			for (pos, cmd), (pos_word, _) in zip(global_plan, global_plan_world):
				self.logger.info(f"* Waypoint: pos={pos}, cmd={cmd}, world_pos={pos_word}")
				if gps:
					if not isinstance(pos, dict) or 'lat' not in pos or 'lon' not in pos:
						self.logger.error(f"Invalid GPS data: {pos}")
						continue
					pos_array = np.array([pos['lat'], pos['lon']])
					if not self._is_valid_gps(pos_array):
						self.logger.error(f"Invalid GPS coordinates: {pos_array}")
						continue
					pos = self.gps_to_location(pos_array)
				else:
					if not isinstance(pos, dict) or 'position' not in pos:
						self.logger.error(f"Invalid position data: {pos}")
						continue
					pos = np.array([pos['position']['x'], pos['position']['y']])
				self.route.append((pos, cmd, pos_word))
		else:
			for pos, cmd in global_plan:
				# self.logger.info(f"Waypoint: pos={pos}, cmd={cmd}")
				if gps:
					if not isinstance(pos, dict) or 'lat' not in pos or 'lon' not in pos:
						self.logger.error(f"Invalid GPS data: {pos}")
						continue
					pos_array = np.array([pos['lat'], pos['lon']])
					if not self._is_valid_gps(pos_array):
						self.logger.error(f"Invalid GPS coordinates: {pos_array}")
						continue
					pos = self.gps_to_location(pos_array)
				else:
					if not isinstance(pos, dict) or 'position' not in pos:
						self.logger.error(f"Invalid position data: {pos}")
						continue
					pos = np.array([pos['position']['x'], pos['position']['y']])
				self.route.append((pos, cmd))
		self.logger.info(f"planner.set_route() finished")

	def _is_valid_gps(self, gps):
		lat, lon = gps
		if not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
			return False
		if np.isnan(lat) or np.isnan(lon) or np.isinf(lat) or np.isinf(lon):
			return False
		return True

	def run_step(self, gps):
		self.debug.clear()
		if len(self.route) == 1:
			return self.route[0]

		to_pop = 0
		farthest_in_range = -np.inf
		cumulative_distance = 0.0

		for i in range(1, len(self.route)):
			if cumulative_distance > self.max_distance:
				break
			cumulative_distance += np.linalg.norm(self.route[i][0] - self.route[i-1][0])
			distance = np.linalg.norm(self.route[i][0] - gps)
			if distance <= self.min_distance and distance > farthest_in_range:
				farthest_in_range = distance
				to_pop = i
			r = 255 * int(distance > self.min_distance)
			g = 255 * int(self.route[i][1] == 4)
			b = 255
			self.debug.dot(gps, self.route[i][0], (r, g, b))

		for _ in range(to_pop):
			if len(self.route) > 2:
				self.route.popleft()

		self.debug.dot(gps, self.route[0][0], (0, 255, 0))
		self.debug.dot(gps, self.route[1][0], (255, 0, 0))
		self.debug.dot(gps, gps, (0, 0, 255))
		self.debug.show()

		return self.route[1]

	def gps_to_location(self, gps):
		try:
			lat, lon = gps
			self.logger.info(f"Converting GPS: lat={lat}, lon={lon}")
			if not self._is_valid_gps(gps):
				self.logger.error(f"Invalid GPS coordinates: lat={lat}, lon={lon}")
				return np.array([0.0, 0.0])

			scale = math.cos(self.lat_ref * math.pi / 180.0)
			tan_arg = (lat + 90) * math.pi / 360.0
			if abs(tan_arg - math.pi / 2) < 1e-6:
				self.logger.warning(f"Near pole: lat={lat}, adjusting to avoid tan(π/2)")
				return np.array([0.0, 0.0])
			tan_value = math.tan(tan_arg)
			if tan_value <= 0:
				self.logger.warning(f"Invalid tan value: {tan_value}, returning default")
				return np.array([0.0, 0.0])

			my = math.log(tan_value) * (EARTH_RADIUS_EQUA * scale)
			mx = (lon * (math.pi * EARTH_RADIUS_EQUA * scale)) / 180.0
			y = scale * EARTH_RADIUS_EQUA * math.log(math.tan((90.0 + self.lat_ref) * math.pi / 360.0)) - my
			x = mx - scale * self.lon_ref * math.pi * EARTH_RADIUS_EQUA / 180.0
			self.logger.info(f"Converted to x={x}, y={y}")
			return np.array([x, y])
		except Exception as e:
			self.logger.error(f"GPS conversion failed: {str(e)}")
			return np.array([0.0, 0.0])