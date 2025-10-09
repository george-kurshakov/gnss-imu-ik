import numpy as np
from opensim import Vec3, Rotation
import opensim as osim
import sys
import os

accuracy = 0.001
visualize = True

# Processing setup files
imu_placer_setup = "myIMUPlacer_Setup.xml"
imu_ik_setup = "myIMUIK_Setup.xml"

# Input data files
# .sto quaternion orientations file
quat_file = "my_sto.sto"
# .trc GNSS data file (working as markers)
marker_file = "my_trc.trc"

# Output file
joint_file = "recording.sto"

# def get_theta():
#     """Estimate the heading angle from the general walking direction"""
#     p = np.polyfit(np.array([epoch.baseline[0] for epoch in l] + [epoch.baseline[0] for epoch in r]),
#                    np.array([epoch.baseline[1] for epoch in l] + [epoch.baseline[1] for epoch in r]), 1)
#     p = np.array([1, p[0]])
#     p = p / np.linalg.norm(p)    

def main():
    # Import and rotate IMU orientations
    quatTable = osim.TimeSeriesTableQuaternion(quat_file)
    # Rotation to apply around vertical axis
    theta = np.pi * 3/4
    # -90 degrees around X axis and some rotation around Y axis to align sensor to OpenSim frame
    sensor_to_opensim_rot = Rotation(
        osim.SpaceRotationSequence, -np.pi/2, osim.CoordinateAxis(0), theta, osim.CoordinateAxis(1))
    osim.OpenSenseUtilities.rotateOrientationTable(
        quatTable, sensor_to_opensim_rot)  # IMU to OpenSim frame
    # osim.STOFileAdapterQuaternion.write(quatTable, 'my_sto_rotated.sto')

    myIMUPlacer = osim.IMUPlacer(imu_placer_setup)
    myIMUPlacer.run(visualizeResults=False)

    model = osim.Model("s1_calibrated.osim")
    model.initSystem()
    model.finalizeConnections()

    coordinates = model.getCoordinateSet()
    ikReporter = osim.TableReporter()
    ikReporter.setName('ik_reporter')
    for coord in coordinates:
        ikReporter.addToReport(coord.getOutput('value'), coord.getName())
    model.addComponent(ikReporter)

    # Import markers
    marker_table = osim.TimeSeriesTableVec3(marker_file)
    # Set up marker weights
    marker_weights = osim.SetMarkerWeights()
    marker_weights.cloneAndAppend(osim.MarkerWeight('torso_gnss', 1.0))
    marker_weights.cloneAndAppend(osim.MarkerWeight('calcn_l_gnss', 0.1))
    marker_weights.cloneAndAppend(osim.MarkerWeight('calcn_r_gnss', 0.1))
    mRefs = osim.MarkersReference(marker_table, marker_weights)

    # Set up orientation weights
    quat_weights = osim.SetOientationWeights()
    quat_weights.cloneAndAppend(osim.OrientationWeight('torso_imu', 1.0))
    quat_weights.cloneAndAppend(osim.OrientationWeight('calcn_l_imu', 1.0))
    quat_weights.cloneAndAppend(osim.OrientationWeight('calcn_r_imu', 1.0))
    orientationsData = osim.OpenSenseUtilities.convertQuaternionsToRotations(
        quatTable)
    oRefs = osim.OrientationsReference(orientationsData, quat_weights)

    coordinateReferences = osim.SimTKArrayCoordinateReference()
    constraint_var = 10.0

    init_state = model.initSystem()
    if visualize:
        model.setUseVisualizer(True)
    init_state = model.initSystem()
    s0 = init_state

    times = marker_table.getIndependentColumn()

    # solver = osim.InverseKinematicsTool()
    # # "myIMUIK_Setup.xml"
    # IMUikSolver = osim.IMUInverseKinematicsTool()
    # IMUikSolver.runInverseKinematicsWithOrientationsFromFile(
    #     model, "my_sto_rotated.sto", visualizeResults=visualize)

    ikSolver = osim.InverseKinematicsSolver(
        model, mRefs, oRefs, coordinateReferences, constraint_var)
    ikSolver.setAccuracy(accuracy)

    for i in range(marker_table.getNumRows()):
        time = times[i]  # passo un tempo alla volta
        print(f"Time: {time}")
        s0.setTime(time)
        ikSolver.assemble(s0)
        ikSolver.track(s0)
        model.realizePosition(s0)
        model.realizeReport(s0)
        if visualize:
            model.getVisualizer().show(s0)
            model.getVisualizer().getSimbodyVisualizer().setShowSimTime(True)

    ik_results = ikReporter.getTable()
    osim.STOFileAdapter.write(ik_results, joint_file)

if __name__ == "__main__":
    main()
