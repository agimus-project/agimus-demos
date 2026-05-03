{
  description = "Whole Body Model Predictive Control in the AGIMUS architecture";

  inputs.gepetto.url = "github:gepetto/nix";

  outputs =
    inputs:
    inputs.gepetto.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        rosDistros = [
          "humble"
          "jazzy"
        ];
        rosShellDistro = "jazzy";
        rosOverrideAttrs = {
          agimus-demo-00-franka-controller = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./agimus_demo_00_franka_controller;
            };
          };

          agimus-demo-01-lfc-alone = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./agimus_demo_01_lfc_alone;
            };
          };

          agimus-demo-02-simple-pd-plus = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./agimus_demo_02_simple_pd_plus;
            };
          };

          agimus-demo-02-simple-pd-plus-tiago-pro = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./agimus_demo_02_simple_pd_plus_tiago_pro;
            };
          };

          agimus-demo-03-mpc-dummy-traj = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./agimus_demo_03_mpc_dummy_traj;
            };
          };

          agimus-demo-03-mpc-dummy-traj-tiago-pro = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./agimus_demo_03_mpc_dummy_traj_tiago_pro;
            };
          };

          # TODO: need demo 9
          # agimus-demo-04-dual-arm-tiago-pro = {
          #   src = lib.fileset.toSource {
          #     root = ./.;
          #     fileset = ./agimus_demo_04_dual_arm_tiago_pro;
          #   };
          # };

          agimus-demo-04-visual-servoing = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./agimus_demo_04_visual_servoing;
            };
          };

          agimus-demo-05-pick-and-place = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./agimus_demo_05_pick_and_place;
            };
          };

          agimus-demo-06-regrasp = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./agimus_demo_06_regrasp;
            };
          };

          # TODO: need pytroller
          # agimus-demo-07-deburring = {
          #   src = lib.fileset.toSource {
          #     root = ./.;
          #     fileset = ./agimus_demo_07_deburring;
          #   };
          # };

          agimus-demo-08-collision-avoidance = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./agimus_demo_08_collision_avoidance;
            };
          };

          agimus-demos = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./agimus_demos;
            };
          };

          agimus-demos-common = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./agimus_demos_common;
            };
          };

          agimus-demos-controllers = {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = ./agimus_demos_controllers;
            };
          };
        };
      }
    );
}
