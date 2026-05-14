import numpy as np
import pymap3d as pm
from scipy.spatial.transform import Rotation as R


def compute_error(
    attitude_q: list[float, float, float, float],
    gps_coords: list[float, float, float],
    gs_coords: list[float, float, float],
):
    # VERY IMPORTANT TO MAKE SURE QUATERNION IS FORMATTED AS [x, y, z, w].
    # GPS must be formatted as [lat, lon, alt]
    # gs_coords must be formatted as [lat, lon, alt].

    gs_lat, gs_lon, gs_alt = gs_coords
    lat, lon, alt = gps_coords
    n, e, d = pm.geodetic2ned(
        gs_lat,
        gs_lon,
        gs_alt,
        lat,
        lon,
        alt,
    )

    r_err = np.array([n, e, d])
    x = r_err/np.linalg.norm(r_err)

    world_to_body = R.from_quat(attitude_q).as_matrix()
    direction = world_to_body @ x
    azimuth_error_rad = np.arctan2(direction[1], direction[0])
    horizontal_distance = np.sqrt(direction[0]**2 + direction[1]**2)
    pitch_error_rad = np.arctan2(direction[2], horizontal_distance) 

    return azimuth_error_rad, pitch_error_rad
