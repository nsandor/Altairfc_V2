from controls.error_computation import compute_error
import pytest
import numpy as np
import pymap3d as pm
import controls.error_computation as ec


@pytest.mark.parametrize("bal_gps, desired_ned, attitude_q, expected_az, expected_pitch", [
    # ECEF r_err along X -> az=0, pitch=0
    ([45.5048, -73.5772, 1060.0], [0.0, 1000.0, 1000.0], [0.0, 0.0, 0.0, 1.0], np.pi/2, np.pi/4),
    ([45.5048, -73.5772, 1060.0], [1000.0, 0.0, 1000.0], [0.0, 0.0, 0.0, 1.0], 0.0, np.pi/4),
    ([45.5048, -73.5772, 1060.0], [0.0, 0.0, 1000.0], [0.0, 0.0, 0.0, 1.0], 0.0, np.pi/2),


])
def test_error_computation(bal_gps, desired_ned, attitude_q, expected_az, expected_pitch):

    gs_lat, gs_lon, gs_alt = pm.ned2geodetic(
        desired_ned[0],
        desired_ned[1],
        desired_ned[2],
        bal_gps[0],
        bal_gps[1],
        bal_gps[2],
    )

    gs_coords = [gs_lat, gs_lon, gs_alt]

    azimuth_error_rad, pitch_error_rad = compute_error(attitude_q, bal_gps, gs_coords)

    assert np.isfinite(azimuth_error_rad) and np.isfinite(pitch_error_rad)
    assert abs(azimuth_error_rad - expected_az) < 1e-6
    assert abs(pitch_error_rad - expected_pitch) < 1e-6
    print(azimuth_error_rad, pitch_error_rad)