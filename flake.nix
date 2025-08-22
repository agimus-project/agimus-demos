{
  description = "Description of the environments for the AGIMUS demos";

  inputs = {
    gepetto.url = "github:nim65s/gepetto-nix/gazebo";
    flake-parts.follows = "gepetto/flake-parts";
    nixpkgs.follows = "gepetto/nixpkgs";
    nix-ros-overlay.follows = "gepetto/nix-ros-overlay";
    systems.follows = "gepetto/systems";
    treefmt-nix.follows = "gepetto/treefmt-nix";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = import inputs.systems;
      imports = [
        inputs.gepetto.flakeModule
        {
          gepetto-pkgs.overlays = [
            (
              final: prev:
              let
                fileset = final.lib.fileset;
              in
              {
                rosPackages = prev.rosPackages // {
                  humble = prev.rosPackages.humble.overrideScope (
                    humble-final: humble-prev: {
                      agimus-demo-00-franka-controller =
                        humble-prev.agimus-demo-00-franka-controller.overrideAttrs
                          (super: {
                            src = fileset.toSource {
                              root = ./.;
                              fileset = fileset.unions [
                                ./agimus_demo_00_franka_controller
                              ];
                            };
                          });
                      agimus-demo-01-lfc-alone = humble-prev.agimus-demo-01-lfc-alone.overrideAttrs (super: { });
                      agimus-demo-02-simple-pd-plus = humble-prev.agimus-demo-02-simple-pd-plus.overrideAttrs (
                        super: { }
                      );
                      agimus-demo-02-simple-pd-plus-tiago-pro =
                        humble-prev.agimus-demo-02-simple-pd-plus-tiago-pro.overrideAttrs
                          (super: { });
                      agimus-demo-03-mpc-dummy-traj = humble-prev.agimus-demo-03-mpc-dummy-traj.overrideAttrs (
                        super: { }
                      );
                      agimus-demo-03-mpc-dummy-traj-tiago-pro =
                        humble-prev.agimus-demo-03-mpc-dummy-traj-tiago-pro.overrideAttrs
                          (super: { });
                      agimus-demo-04-visual-servoing = humble-prev.agimus-demo-04-visual-servoing.overrideAttrs (
                        super: { }
                      );
                      agimus-demo-05-pick-and-place = humble-prev.agimus-demo-05-pick-and-place.overrideAttrs (
                        super: { }
                      );
                      agimus-demos = humble-prev.agimus-demos.overrideAttrs (super: { });
                      agimus-demos-common = humble-prev.agimus-demos-common.overrideAttrs (super: {
                        src = fileset.toSource {
                          root = ./.;
                          fileset = fileset.unions [
                            ./agimus_demos_common
                          ];
                        };
                      });
                    }
                  );
                };
              }
            )
          ];
        }
      ];
      perSystem =
        {
          lib,
          pkgs,
          self',
          ...
        }:
        {
          devShells.default = pkgs.mkShell {
            packages = [
              self'.packages.default
            ];
          };
          packages = {
            default =
              with pkgs.rosPackages.humble;
              buildEnv {
                postBuild = ''
                  rosWrapperArgs+=(
                  --set QT_QPA_PLATFORM_PLUGIN_PATH ${pkgs.qt5.qtbase.bin}/lib/qt-${pkgs.qt5.qtbase.version}/plugins/platforms
                  --prefix IGN_CONFIG_PATH : "$out/share/ignition"
                  )
                '';
                paths = [
                  # keep-sorted start
                  agimus-demo-00-franka-controller
                  agimus-demo-01-lfc-alone
                  agimus-demo-02-simple-pd-plus
                  agimus-demo-02-simple-pd-plus-tiago-pro
                  agimus-demo-03-mpc-dummy-traj
                  agimus-demo-03-mpc-dummy-traj-tiago-pro
                  agimus-demo-04-visual-servoing
                  agimus-demo-05-pick-and-place
                  agimus-demos
                  agimus-demos-common
                  gz-cmake
                  gz-common
                  gz-fuel-tools
                  gz-gui
                  gz-launch
                  gz-math
                  gz-msgs
                  gz-physics
                  gz-plugin
                  gz-rendering
                  gz-sensors
                  gz-sim
                  gz-tools
                  gz-transport
                  gz-utils
                  pkgs.qt5.wrapQtAppsHook
                  sdformat
                  # keep-sorted end
                ];
              };
          };
        };
    };
}
