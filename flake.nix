{
  description = "Whole Body Model Predictive Control in the AGIMUS architecture";

  inputs = {
    gazebros2nix.url = "github:gepetto/gazebros2nix/fork";
    flake-parts.follows = "gazebros2nix/flake-parts";
    flakoboros.follows = "gazebros2nix/flakoboros";
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
            flakoboros = {
              rosDistros = [ "humble" ];
              rosShellDistro = "humble";
              rosOverrideAttrs = {
                agimus-demo-00-franka-controller = _: _: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./agimus_demo_00_franka_controller
                    ];
                  };
                };

                agimus-demo-01-lfc-alone = _: _: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./agimus_demo_01_lfc_alone
                    ];
                  };
                };

                agimus-demo-02-simple-pd-plus = _: _: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./agimus_demo_02_simple_pd_plus
                    ];
                  };
                };

                agimus-demo-02-simple-pd-plus-tiago-pro = _: _: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./agimus_demo_02_simple_pd_plus_tiago_pro
                    ];
                  };
                };

                agimus-demo-03-mpc-dummy-traj = _: _: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./agimus_demo_03_mpc_dummy_traj
                    ];
                  };
                };

                agimus-demo-03-mpc-dummy-traj-tiago-pro = _: _: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./agimus_demo_03_mpc_dummy_traj_tiago_pro
                    ];
                  };
                };

                # TODO: need demo 9
                # agimus-demo-04-dual-arm-tiago-pro = _:_:{
                #   src = lib.fileset.toSource {
                #     root = ./.;
                #     fileset = lib.fileset.unions [
                #       ./agimus_demo_04_dual_arm_tiago_pro
                #     ];
                #   };
                # };

                agimus-demo-04-visual-servoing = _: _: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./agimus_demo_04_visual_servoing
                    ];
                  };
                };

                agimus-demo-05-pick-and-place = _: _: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./agimus_demo_05_pick_and_place
                    ];
                  };
                };

                agimus-demo-06-regrasp = _: _: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./agimus_demo_06_regrasp
                    ];
                  };
                };

                # TODO: need pytroller
                # agimus-demo-07-deburring = _:_:{
                #   src = lib.fileset.toSource {
                #     root = ./.;
                #     fileset = lib.fileset.unions [
                #       ./agimus_demo_07_deburring
                #     ];
                #   };
                # };

                agimus-demo-08-collision-avoidance = _: _: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./agimus_demo_08_collision_avoidance
                    ];
                  };
                };

                agimus-demos = _: _: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./agimus_demos
                    ];
                  };
                };

                agimus-demos-common = _: _: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./agimus_demos_common
                    ];
                  };
                };

                agimus-demos-controllers = _: _: {
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
