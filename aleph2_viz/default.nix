{ buildRosPackage, ament-cmake, robot-state-publisher, joint-state-publisher
, joint-state-publisher-gui, rviz2, aleph2-description }:
buildRosPackage {
  pname = "aleph2-viz";
  version = "0.0.0";

  src = ./.;

  doCheck = true;

  buildType = "ament_cmake";
  buildInputs = [ ament-cmake ];
  propagatedBuildInputs = [
    robot-state-publisher
    joint-state-publisher
    joint-state-publisher-gui
    rviz2
    aleph2-description
  ];
  nativeBuildInputs = [ ament-cmake ];
}
