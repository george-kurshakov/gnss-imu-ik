with open("static.mot", "w") as f:
    f.write("inDegrees=yes\n")
    f.write("name=walk.mot\n")
    f.write("DataType=double\n")
    f.write("version=3\n")
    f.write("OpenSimVersion=4.5\n")
    f.write("endheader\n")
    columns = ["time", "pelvis_tilt", "pelvis_list", "pelvis_rotation", "hip_flexion_r", "hip_adduction_r", "hip_rotation_r", "knee_angle_r", "knee_angle_r_beta", "ankle_angle_r", "subtalar_angle_r", "mtp_angle_r", "hip_flexion_l", "hip_adduction_l", "hip_rotation_l", "knee_angle_l",
               "knee_angle_l_beta", "ankle_angle_l", "subtalar_angle_l", "mtp_angle_l", "lumbar_extension", "lumbar_bending", "lumbar_rotation", "arm_flex_r", "arm_add_r", "arm_rot_r", "elbow_flex_r", "pro_sup_r", "wrist_flex_r", "wrist_dev_r", "arm_flex_l", "arm_add_l", "arm_rot_l", "elbow_flex_l", "pro_sup_l", "wrist_flex_l", "wrist_dev_l"]
    f.write("\t".join(columns) + "\n")
    coordinates = len(columns) * [0.0]
    # coordinates[5] = 0.94  # Pelvis tY
    # coordinates = [str(item) for item in coordinates]
    for i in range(101):
        coordinates[0] = 0.01*i
        f.write("\t".join([str(item) for item in coordinates]) + "\n")
