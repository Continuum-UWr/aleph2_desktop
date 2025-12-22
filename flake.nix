{
  inputs = {
    nixpkgs.follows = "aleph2-common/nixpkgs";
    flake-utils.follows = "aleph2-common/flake-utils";
    nix-ros-overlay.follows = "aleph2-common/nix-ros-overlay";
    aleph2-common.url = "git+https://gitlab.continuum.ii.uni.wroc.pl/continuum/software/aleph2_common";
  };
  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
      nix-ros-overlay,
      aleph2-common,
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs =
          (import nixpkgs {
            system = system;
            overlays = [ (final : prev : {freeimage = prev.callPackage nix/freeimage.nix {};}) nix-ros-overlay.overlays.default];
          }).pkgs;

        aleph2-description = aleph2-common.packages.${system}.aleph2-description;
        input-manager = aleph2-common.packages.${system}.input-manager;

        aleph2-gui = pkgs.rosPackages.jazzy.callPackage (import ./aleph2_gui) { inherit input-manager; };
        aleph2-gz = pkgs.rosPackages.jazzy.callPackage (import ./aleph2_gz) { inherit aleph2-description; };
        aleph2-viz = pkgs.rosPackages.jazzy.callPackage (import ./aleph2_viz) { inherit aleph2-description; };

      in
      {
        packages = {
          inherit aleph2-gui aleph2-gz aleph2-viz;
          default = aleph2-viz;
          freeimage = pkgs.freeimage;
        };
        devShells.default = pkgs.mkShell {
          nativeBuildInputs = [
            (pkgs.buildEnv {
              paths = [
                pkgs.rosPackages.jazzy.ros-core
                aleph2-gui
                aleph2-gz
                aleph2-viz
              ];
            })
          ];
        };
        formatter = pkgs.nixfmt;
      }
    );
}
