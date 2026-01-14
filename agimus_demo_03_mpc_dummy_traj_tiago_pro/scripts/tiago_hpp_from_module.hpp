from agimus_demo_03_mpc_dummy_traj_tiago_pro.hpp_square_motion import
    HPPLeftArmSquareMotion

        def
        main()
    : urdf_filename =
    "package://example-robot-data/robots/tiago_pro_description/robots/"
    "tiago_pro.urdf" srdf_filename = ""

    motion = HPPLeftArmSquareMotion(urdffilename, srdf_filename)

                 if __name__
             == "__main__" : main()
