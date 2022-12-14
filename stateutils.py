"""Utility functions to process state."""
from typing import Tuple

import numpy as np
from numba import njit
import numpy.linalg as LA


@njit
def vector_angles(vecs: np.ndarray) -> np.ndarray:
	 """Calculate angles for an array of vectors
	 :param vecs: nx2 ndarray
	 :return: nx1 ndarray
	 """
	 ang = np.arctan2(vecs[:, 1], vecs[:, 0])  # atan2(y, x)
	 return ang
#@njit
#def vector_angles(a: np.ndarray,b: np.ndarray) -> np.ndarray:
#	"""Calculate angles for an array of vectors
#	:param vecs: nx2 ndarray
#	:return: nx1 ndarray
#	"""
#	inner = np.inner(a, b)
#	norms = LA.norm(a) * LA.norm(b)
#
#	cos = inner / norms
#	rad = np.arccos(np.clip(cos, -1.0, 1.0))
#	deg = np.rad2deg(rad)
#	return deg[0]
#
@njit
def left_normal(vecs: np.ndarray) -> np.ndarray:
	vecs = np.fliplr(vecs) * np.array([-1.0, 1.0])
	return vecs


@njit
def right_normal(vecs: np.ndarray) -> np.ndarray:
	vecs = np.fliplr(vecs) * np.array([1.0, -1.0])
	return vecs


@njit
def normalize(vecs: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
	"""Normalize nx2 array along the second axis
	input: [n,2] ndarray
	output: (normalized vectors, norm factors)
	"""
	norm_factors = []
	for line in vecs:
		norm_factors.append(np.linalg.norm(line))
	norm_factors = np.array(norm_factors)
	normalized = vecs / np.expand_dims(norm_factors, -1)
	# get rid of nans
	for i in range(norm_factors.shape[0]):
		if norm_factors[i] == 0:
			normalized[i] = np.zeros(vecs.shape[1])
	return normalized, norm_factors


@njit
def desired_directions(state: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
	"""Given the current state and destination, compute desired direction."""
	destination_vectors = state[:, 4:6] - state[:, 0:2]
	directions, dist = normalize(destination_vectors)
	return directions, dist
@njit
def self_vec_diff(vecs: np.ndarray) -> np.ndarray:
	"""r_ab
	r_ab := r_a − r_b.
	"""
	diff = np.expand_dims(vecs, 1) - np.zeros((np.expand_dims(vecs, 0)).shape)
	return diff

def self_diff(vecs: np.ndarray, keepdims=False) -> np.ndarray:
	"""
	:param vecs: nx2 array
	:return: diff with diagonal elements removed
	"""
	diff = self_vec_diff(vecs)
	# diff = diff[np.any(diff, axis=-1), :]	 # get rid of zero vectors
	diff = diff[
		~np.eye(diff.shape[0], dtype=bool), :
	]  # get rif of diagonal elements in the diff matrix
	if keepdims:
		diff = diff.reshape(vecs.shape[0], -1, vecs.shape[1])

	return diff


@njit
def vec_diff(vecs: np.ndarray) -> np.ndarray:
	"""r_ab
	r_ab := r_a − r_b.
	"""
	diff = np.expand_dims(vecs, 1) - np.expand_dims(vecs, 0)
	return diff


def each_diff(vecs: np.ndarray, keepdims=False) -> np.ndarray:
	"""
	:param vecs: nx2 array
	:return: diff with diagonal elements removed
	"""
	diff = vec_diff(vecs)
	# diff = diff[np.any(diff, axis=-1), :]	 # get rid of zero vectors
	diff = diff[
		~np.eye(diff.shape[0], dtype=bool), :
	]  # get rif of diagonal elements in the diff matrix
	if keepdims:
		diff = diff.reshape(vecs.shape[0], -1, vecs.shape[1])

	return diff
	
@njit
def vecs_diff(vec_1: np.ndarray,vec_2: np.ndarray) -> np.ndarray:
	"""r_ab
	r_ab := r_a − r_b.
	"""
	diff = np.expand_dims(vec_1, 1) - np.expand_dims(vec_2, 0)
	return diff
	
def each_diffs(vec_1: np.ndarray,vec_2: np.ndarray, keepdims=False) -> np.ndarray:
	"""
	:param vecs: nx2 array
	:return: diff with diagonal elements removed
	"""
	diff = vecs_diff(vec_1,vec_2)
	# diff = diff[np.any(diff, axis=-1), :]	 # get rid of zero vectors
	#diff = diff[
	#	 ~np.eye(diff.shape[0], dtype=bool), :
	#]	# get rif of diagonal elements in the diff matrix
	diff = diff.reshape((-1,2))
	if keepdims:
		diff = diff.reshape(vecs.shape[0], -1, vecs.shape[1])

	return diff
@njit
def speeds(state: np.ndarray) -> np.ndarray:
	"""Return the speeds corresponding to a given state."""
	#	  return np.linalg.norm(state[:, 2:4], axis=-1)
	speed_vecs = state[:, 2:4]
	speeds_array = np.array([np.linalg.norm(s) for s in speed_vecs])
	return speeds_array


@njit
def center_of_mass(vecs: np.ndarray) -> np.ndarray:
	"""Center-of-mass of a given group"""
	return np.sum(vecs, axis=0) / vecs.shape[0]


@njit
def minmax(vecs: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
	x_min = np.min(vecs[:, 0])
	y_min = np.min(vecs[:, 1])
	x_max = np.max(vecs[:, 0])
	y_max = np.max(vecs[:, 1])
	return (x_min, y_min, x_max, y_max)
