# gnss-imu-ik

Opensim code for human inverse kinematics in outdoor environments based on GNSS and IMU data. Developed by Georgii A. Kurshakov as a part of his PhD project at University of Genoa.

## Dependencies

It is essential to install OpenSim (version 4.5 was used) and set up Python scripting: <https://opensimconfluence.atlassian.net/wiki/spaces/OpenSim/pages/53085346/Scripting+in+Python>. It is recommended to use the **conda** package for OpenSim. Before running the code, make sure that the OpenSim installation folder is added to PATH, otherwise the script will not be able to find the *simbody-visualizer* and body geometry files. Other Python packages required include:

* **numpy** - you might need to install numpy<2 for compatibility
* **ahrs**

## Workflow

* *data_reader.py* is a library file to assist in reading input files, which are:
  * GNSS files in .pos format, a standard output format of RTKLIB.
  * IMU files in .txt format, in our case converted from .mtb proprietary format.
* *gnss_interp.py* is a file to synchronize the GNSS and IMU data and to interpolate GNSS to the IMU frequency.
* *ik_markers.py* is a file to run the OpenSim inverse kinematics computation using both GNSS and IMU data as sources.
