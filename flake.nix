{
  description = "Whole Body Model Predictive Control in the AGIMUS architecture";

  inputs = {
    gazebros2nix.url = "github:gepetto/gazebros2nix";
    flake-parts.follows = "gazebros2nix/flake-parts";
    nixpkgs.follows = "gazebros2nix/nixpkgs";
    nix-ros-overlay.follows = "gazebros2nix/nix-ros-overlay";
    systems.follows = "gazebros2nix/systems";
    treefmt-nix.follows = "gazebros2nix/treefmt-nix";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, ... }:
      {
        systems = [ "x86_64-linux" ];
        imports = [
          inputs.gazebros2nix.flakeModule
          {
            gazebros2nix = {
              rosDistros = [ "humble" ];
              rosShellDistro = "humble";
              rosPackages = {
                agimus-demo-00-franka-controller = _final: _ros-final: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./agimus_demo_00_franka_controller
                    ];
                  };
                };

                agimus-demo-01-lfc-alone = _final: _ros-final: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./agimus_demo_01_lfc_alone
                    ];
                  };
                };

                agimus-demo-02-simple-pd-plus = _final: _ros-final: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./agimus_demo_02_simple_pd_plus
                    ];
                  };
                };

                agimus-demo-02-simple-pd-plus-tiago-pro = _final: _ros-final: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./agimus_demo_02_simple_pd_plus_tiago_pro
                    ];
                  };
                };

                agimus-demo-03-mpc-dummy-traj = _final: _ros-final: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./agimus_demo_03_mpc_dummy_traj
                    ];
                  };
                };

                agimus-demo-03-mpc-dummy-traj-tiago-pro = _final: _ros-final: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./agimus_demo_03_mpc_dummy_traj_tiago_pro
                    ];
                  };
                };

                # TODO: need demo 9
                # agimus-demo-04-dual-arm-tiago-pro = _final: _ros-final: {
                #   src = lib.fileset.toSource {
                #     root = ./.;
                #     fileset = lib.fileset.unions [
                #       ./agimus_demo_04_dual_arm_tiago_pro
                #     ];
                #   };
                # };

                agimus-demo-04-visual-servoing = _final: _ros-final: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./agimus_demo_04_visual_servoing
                    ];
                  };
                };

                agimus-demo-05-pick-and-place = _final: _ros-final: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./agimus_demo_05_pick_and_place
                    ];
                  };
                };

                agimus-demo-06-regrasp = _final: _ros-final: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./agimus_demo_06_regrasp
                    ];
                  };
                };

                # TODO: need pytroller
                # agimus-demo-07-deburring = _final: _ros-final: {
                #   src = lib.fileset.toSource {
                #     root = ./.;
                #     fileset = lib.fileset.unions [
                #       ./agimus_demo_07_deburring
                #     ];
                #   };
                # };

                agimus-demo-08-collision-avoidance = _final: _ros-final: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./agimus_demo_08_collision_avoidance
                    ];
                  };
                };

                agimus-demos = _final: _ros-final: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./agimus_demos
                    ];
                  };
                };

                agimus-demos-common = _final: _ros-final: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./agimus_demos_common
                    ];
                  };
                };

                agimus-demos-controllers = _final: _ros-final: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./agimus_demos_controllers
                    ];
                  };
                };
              };
            };
          }
        ];
      }
    );
}
