{
  description = "Description of the environments for the AGIMUS demos";

  inputs = {
    gepetto.url = "github:nim65s/gepetto-nix/agimus-demos";
    flake-parts.follows = "gepetto/flake-parts";
    gazebo-sim-overlay.follows = "gepetto/gazebo-sim-overlay";
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
                      agimus-demos-common = humble-prev.agimus-demos-common.overrideAttrs (super: { });

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
              pkgs.gz-harmonic
              self'.packages.default
            ];
          };
          packages = {
            default =
              with pkgs.rosPackages.humble;
              buildEnv {
                paths = [
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
                ];
              };
          };
        };
    };
}
