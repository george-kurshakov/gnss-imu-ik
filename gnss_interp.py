import data_reader
from matplotlib import pyplot as plt
from datetime import timedelta
import numpy as np
import ahrs


def get_overlap_interval(streams):
    """Find overlapping time interval across multiple time-sorted lists."""
    starts = [stream[0].time for stream in streams]
    ends = [stream[-1].time for stream in streams]
    return max(starts), min(ends)


def generate_time_grid(start_time, end_time, freq_hz):
    """Generate a list of datetime objects at a fixed frequency."""
    step = timedelta(seconds=1.0 / freq_hz)
    times = []
    t = start_time
    while t <= end_time:
        times.append(t)
        t += step
    return times


def interpolate_vector_field(times_target, times_src, values_src):
    """Interpolate a vector (NxM) field over time."""
    times_sec = np.array([(t - times_src[0]).total_seconds()
                         for t in times_src])
    result = []
    for i in range(values_src.shape[1]):
        interp = np.interp(
            [(t - times_src[0]).total_seconds() for t in times_target],
            times_sec,
            values_src[:, i]
        )
        result.append(interp)
    return np.stack(result, axis=1)


def interpolate_matrix_field(times_target, times_src, values_src):
    """Interpolate a matrix (Nx3x3) field over time."""
    times_sec = np.array([(t - times_src[0]).total_seconds()
                         for t in times_src])
    result = []
    for t in times_target:
        t_sec = (t - times_src[0]).total_seconds()
        M = np.zeros((3, 3))
        for r in range(3):
            for c in range(3):
                M[r, c] = np.interp(t_sec, times_sec, values_src[:, r, c])
        result.append(M)
    return result


def nearest_neighbor(times_target, times_src, values_src):
    """Find nearest neighbor for each target time."""
    times_sec = np.array([(t - times_src[0]).total_seconds()
                         for t in times_src])
    return [
        values_src[np.argmin(
            np.abs(times_sec - (t - times_src[0]).total_seconds()))]
        for t in times_target
    ]


def interpolate_imu_stream(imu_stream, times_target):
    times_src = [e.time for e in imu_stream]
    acc = np.array([e.acc for e in imu_stream])
    gyr = np.array([e.gyr for e in imu_stream])
    mag = np.array([e.mag for e in imu_stream])
    quat = np.array([e.quat for e in imu_stream])

    acc_interp = interpolate_vector_field(times_target, times_src, acc)
    gyr_interp = interpolate_vector_field(times_target, times_src, gyr)
    mag_interp = interpolate_vector_field(times_target, times_src, mag)
    quat_interp = interpolate_vector_field(times_target, times_src, quat)

    return [
        data_reader.IMU_epoch(time=t, acc=acc_interp[i], gyr=gyr_interp[i],
                       mag=mag_interp[i], quat=quat_interp[i])
        for i, t in enumerate(times_target)
    ]


def interpolate_gnss_stream(gnss_stream, times_target):
    times_src = [e.time for e in gnss_stream]
    baselines = np.array([e.baseline for e in gnss_stream])
    velocities = np.array([e.vel for e in gnss_stream])
    covariances = np.array([e.cov for e in gnss_stream])
    Q_values = np.array([e.Q for e in gnss_stream])
    nsat_values = np.array([e.nsat for e in gnss_stream])

    baseline_interp = interpolate_vector_field(
        times_target, times_src, baselines)
    vel_interp = interpolate_vector_field(times_target, times_src, velocities)
    cov_interp = interpolate_matrix_field(times_target, times_src, covariances)
    Q_interp = nearest_neighbor(times_target, times_src, Q_values)
    nsat_interp = nearest_neighbor(times_target, times_src, nsat_values)

    return [
        data_reader.GNSS_epoch(time=t, baseline=baseline_interp[i], vel=vel_interp[i],
                        Q=int(Q_interp[i]), nsat=int(nsat_interp[i]), cov=cov_interp[i])
        for i, t in enumerate(times_target)
    ]


def synchronize_all(imu_streams, gnss_streams, imu_freq=100):
    """
    Synchronize 3 IMU and 3 GNSS streams to a common time base.

    :param imu_streams: list of 3 IMU lists
    :param gnss_streams: list of 3 GNSS lists
    :return: tuple of 3 IMU lists and 3 GNSS lists, all aligned
    """
    all_streams = imu_streams + gnss_streams
    start_time, end_time = get_overlap_interval(all_streams)
    time_grid = generate_time_grid(start_time, end_time, imu_freq)

    synced_imus = [interpolate_imu_stream(
        stream, time_grid) for stream in imu_streams]
    synced_gnss = [interpolate_gnss_stream(
        stream, time_grid) for stream in gnss_streams]

    return (*synced_imus, *synced_gnss)


def write_trc(gnss_data, path):
    with open(path, "w") as f:
        f.write(
            "\t".join(["PathFileType", "4", "(X/Y/Z)", "subject01_walk1.trc"]) + "\n")
        f.write("\t".join(["DataRate", "CameraRate", "NumFrames", "NumMarkers",
                "Units", "OrigDataRate", "OrigDataStartFrame", "OrigNumFrames"]) + "\n")
        f.write("\t".join([str(100.0), str(100.0), str(len(gnss_data[0])), str(
            3), "m", str(100.0), str(1), str(len(gnss_data[0]))]) + "\n")
        f.write("\t".join(["Frame#", "Time", "torso_gnss", "",
                "", "calcn_l_gnss", "", "", "calcn_r_gnss"]) + "\n")
        f.write("\t".join(["", "", "X1", "Y1",
                "Z1", "X2", "Y2", "Z2", "X3", "Y3", "Z3"]) + "\n")
        for i in range(len(gnss_data[0])):
            f.write("\t".join([str(i+1), str((gnss_data[0][i].time - gnss_data[0][0].time).total_seconds()),
                               str(gnss_data[0][i].baseline[0]), str(
                                   -gnss_data[0][i].baseline[2]), str(gnss_data[0][i].baseline[1]),
                               str(gnss_data[1][i].baseline[0]), str(
                                   -gnss_data[1][i].baseline[2]), str(gnss_data[1][i].baseline[1]),
                               str(gnss_data[2][i].baseline[0]), str(-gnss_data[2][i].baseline[2]), str(gnss_data[2][i].baseline[1])]) + "\n")


def write_sto(imu_data, path):
    with open(path, "w") as f:
        f.write("DataRate=100.000000\n")
        f.write("DataType=Quaternion\n")
        f.write("version=3\n")
        f.write("OpenSimVersion=4.5.2-2025-04-07-2c9fc5bc9\n")
        f.write("endheader\n")
        f.write("time\ttorso_imu\tcalcn_l_imu\tcalcn_r_imu\n")
        for i in range(len(imu_data[0])):
            quat_t_str = ",".join([str(imu_data[0][i].quat[j])
                                  for j in range(4)])
            quat_l_str = ",".join([str(imu_data[1][i].quat[j])
                                  for j in range(4)])
            quat_r_str = ",".join([str(imu_data[2][i].quat[j])
                                  for j in range(4)])
            f.write("\t".join([str((imu_data[0][i].time - imu_data[0][0].time).total_seconds()),
                               quat_t_str, quat_l_str, quat_r_str]) + "\n")


subject = 's8'
speed = 'speed_fast'

gnss_t, _, _ = data_reader.read_gnss(
    f"D:\\OneDrive - Lake Lucerne Institute AG\\DART.Gait-MQ - GMQ-12.AirKinematics - Dataset\\{subject}\\gnss\\{speed}\\straight\\mdp\\t_mdp.pos")
imu_t = data_reader.read_imu(
    f"D:\\OneDrive - Lake Lucerne Institute AG\\DART.Gait-MQ - GMQ-12.AirKinematics - Dataset\\{subject}\\gnss\\{speed}\\straight\\t.txt", "x-sens")
gnss_l, _, _ = data_reader.read_gnss(
    f"D:\\OneDrive - Lake Lucerne Institute AG\\DART.Gait-MQ - GMQ-12.AirKinematics - Dataset\\{subject}\\gnss\\{speed}\\straight\\mdp\\lf_mdp.pos")
imu_l = data_reader.read_imu(
    f"D:\\OneDrive - Lake Lucerne Institute AG\\DART.Gait-MQ - GMQ-12.AirKinematics - Dataset\\{subject}\\gnss\\{speed}\\straight\\lf.txt", "x-sens")
gnss_r, _, _ = data_reader.read_gnss(
    f"D:\\OneDrive - Lake Lucerne Institute AG\\DART.Gait-MQ - GMQ-12.AirKinematics - Dataset\\{subject}\\gnss\\{speed}\\straight\\mdp\\rf_mdp.pos")
imu_r = data_reader.read_imu(
    f"D:\\OneDrive - Lake Lucerne Institute AG\\DART.Gait-MQ - GMQ-12.AirKinematics - Dataset\\{subject}\\gnss\\{speed}\\straight\\rf.txt", "x-sens")

# imu, gnss = shoe.synchronize_gnss_to_imu(imu, gnss)

imu_t, imu_l, imu_r, gnss_t, gnss_l, gnss_r = synchronize_all(
    [imu_t, imu_l, imu_r], [gnss_t, gnss_l, gnss_r], imu_freq=100)

q_ahrs = ahrs.filters.madgwick.Madgwick(np.array([epoch.gyr for epoch in imu_t]), np.array([
                                        epoch.acc for epoch in imu_t]))
for i in range(len(imu_t)):
    imu_t[i].quat = q_ahrs.Q[i]

q_ahrs = ahrs.filters.madgwick.Madgwick(np.array([epoch.gyr for epoch in imu_l]), np.array([
                                        epoch.acc for epoch in imu_l]))
for i in range(len(imu_l)):
    imu_l[i].quat = q_ahrs.Q[i]

q_ahrs = ahrs.filters.madgwick.Madgwick(np.array([epoch.gyr for epoch in imu_r]), np.array([
                                        epoch.acc for epoch in imu_r]))
for i in range(len(imu_r)):
    imu_r[i].quat = q_ahrs.Q[i]

# # Compute the mean baseline for each marker to preserve their relative positions
# mean_t = np.mean([e.baseline for e in gnss_t], axis=0)
# mean_l = np.mean([e.baseline for e in gnss_l], axis=0)
# mean_r = np.mean([e.baseline for e in gnss_r], axis=0)

# # Stack the means to estimate the main direction
# all_means = np.vstack([mean_t, mean_l, mean_r])
# xy = all_means[:, :2]
# X = xy[:, 0].reshape(-1, 1)
# y = xy[:, 1]
# reg = LinearRegression().fit(X, y)
# angle = np.arctan(reg.coef_[0])
# theta = -angle


# def rotate_z(vec, theta):
#     c, s = np.cos(theta), np.sin(theta)
#     R = np.array([[c, -s, 0],
#                   [s,  c, 0],
#                   [0,  0, 1]])
#     return R @ vec


# # Compute the centroid of all baselines (for rigid-body rotation)
# all_baselines = np.vstack([
#     np.array([e.baseline for e in gnss_t]),
#     np.array([e.baseline for e in gnss_l]),
#     np.array([e.baseline for e in gnss_r])
# ])
# centroid = np.mean(all_baselines, axis=0)

# fig = plt.figure(figsize=(12, 6))

# # Before rotation
# ax1 = fig.add_subplot(121, projection='3d')
# ax1.set_title('Trajectories Before Rotation')
# for gnss, label, color in zip(
#     [gnss_t, gnss_l, gnss_r],
#     ['torso', 'left', 'right'],
#     ['r', 'g', 'b']
# ):
#     traj = np.array([e.baseline for e in gnss])
#     ax1.plot(traj[:, 0], traj[:, 1], traj[:, 2], color, label=label)
# ax1.legend()
# ax1.set_xlabel('X')
# ax1.set_ylabel('Y')
# ax1.set_zlabel('Z')

# # Rotate all baselines rigidly around the centroid
# for stream in (gnss_t, gnss_l, gnss_r):
#     for e in stream:
#         e.baseline = rotate_z(e.baseline - centroid, theta) + centroid

write_trc((gnss_t, gnss_l, gnss_r), "my_trc.trc")
write_sto((imu_t, imu_l, imu_r), "my_sto.sto")

# plt.show()


# After rotation
# Recompute rotated trajectories for plotting

# ax2 = fig.add_subplot(122, projection='3d')
# ax2.set_title('Trajectories After Rotation')
# for gnss, label, color in zip(
#     [gnss_t, gnss_l, gnss_r],
#     ['torso', 'left', 'right'],
#     ['r', 'g', 'b']
# ):
#     traj = np.array([e.baseline for e in gnss])
#     ax2.plot(traj[:, 0], traj[:, 1], traj[:, 2], color, label=label)
# ax2.legend()
# ax2.set_xlabel('X')
# ax2.set_ylabel('Y')
# ax2.set_zlabel('Z')

# # Set equal aspect for both plots


# def set_axes_equal(ax):
#     x_limits = ax.get_xlim3d()
#     y_limits = ax.get_ylim3d()
#     z_limits = ax.get_zlim3d()
#     x_range = abs(x_limits[1] - x_limits[0])
#     y_range = abs(y_limits[1] - y_limits[0])
#     z_range = abs(z_limits[1] - z_limits[0])
#     max_range = max([x_range, y_range, z_range])
#     mid_x = np.mean(x_limits)
#     mid_y = np.mean(y_limits)
#     mid_z = np.mean(z_limits)
#     ax.set_xlim3d([mid_x - max_range/2, mid_x + max_range/2])
#     ax.set_ylim3d([mid_y - max_range/2, mid_y + max_range/2])
#     ax.set_zlim3d([mid_z - max_range/2, mid_z + max_range/2])


# set_axes_equal(ax1)
# set_axes_equal(ax2)

# plt.tight_layout()
# plt.show()
