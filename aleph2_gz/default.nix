{ buildRosPackage, ament-cmake, gz-plugin-vendor, gz-sim-vendor, ros-gz-sim
, aleph2-description }:
buildRosPackage {
  pname = "aleph2-gz";
  version = "0.0.0";

  src = ./.;

  doCheck = true;

  buildType = "ament_cmake";
  buildInputs = [ ament-cmake gz-plugin-vendor gz-sim-vendor ];
  propagatedBuildInputs = [ ros-gz-sim aleph2-description ];
  nativeBuildInputs = [ ament-cmake ];
}
