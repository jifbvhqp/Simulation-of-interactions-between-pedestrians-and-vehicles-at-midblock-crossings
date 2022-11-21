from math import sqrt


def normalize(vector):
	"""
	:return: The vector scaled to a length of 1
	"""
	norm = sqrt(vector[0] ** 2 + vector[1] ** 2)
	return vector[0] / norm, vector[1] / norm


def dot(vector1, vector2):
	"""
	:return: The dot (or scalar) product of the two vectors
	"""
	return vector1[0] * vector2[0] + vector1[1] * vector2[1]


def edge_direction(point0, point1):
	"""
	:return: A vector going from point0 to point1
	"""
	return point1[0] - point0[0], point1[1] - point0[1]


def orthogonal(vector):
	"""
	:return: A new vector which is orthogonal to the given vector
	"""
	return vector[1], -vector[0]


def vertices_to_edges(vertices):
	"""
	:return: A list of the edges of the vertices as vectors
	"""
	return [edge_direction(vertices[i], vertices[(i + 1) % len(vertices)])
			for i in range(len(vertices))]


def project(vertices, axis):
	"""
	:return: A vector showing how much of the vertices lies along the axis
	"""
	dots = [dot(vertex, axis) for vertex in vertices]
	return [min(dots), max(dots)]


def overlap(projection1, projection2):
	"""
	:return: Boolean indicating if the two projections overlap
	"""
	return min(projection1) <= max(projection2) and \
		   min(projection2) <= max(projection1)


def separating_axis_theorem(vertices_a, vertices_b):
	edges = vertices_to_edges(vertices_a) + vertices_to_edges(vertices_b)
	axes = [normalize(orthogonal(edge)) for edge in edges]

	for axis in axes:
		projection_a = project(vertices_a, axis)
		projection_b = project(vertices_b, axis)

		overlapping = overlap(projection_a, projection_b)

		if not overlapping:
			return False

	return True