"""Sensor processing: LIDAR, odometry, mapping."""

from .mapping import ObstacleMap, MapConfig

__all__ = ["ObstacleMap", "MapConfig"]
