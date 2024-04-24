{ buildRosPackage, rclpy, ament-index-python, python3Packages, rqt-gui
, rqt-gui-py, input-manager }:
buildRosPackage {
  pname = "aleph2-gui";
  version = "0.0.0";

  src = ./.;

  doCheck = true;

  buildType = "ament_python";
  propagatedBuildInputs = [
    rclpy
    ament-index-python
    python3Packages.qtpy
    python3Packages.pyqt5
    rqt-gui
    rqt-gui-py
    input-manager
  ];
}
