#!/usr/bin/env python
# Copyright (c) 2025 CNRS
# Author: Florent Lamiraux
#

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.

class Helper:
    def __init__(self, ps, graph):
        self.robot = ps.robot
        self.problem = ps.client.basic.problem
        self.graph = graph
        self.problem.resetConstraints()
        self.problem.addNumericalConstraints("cp", ["tiago_pro/left grasps reinforcment_bar/left",
            "tiago_pro/right grasps reinforcment_bar/right",
            "preplace_reinforcment_bar",
            "locked_plate/root_joint",
            "place_reinforcment_bar/complement"],
            [0, 0, 0, 0, 0])

    # Generate a preplace configuration from reachable from goal
    def generateIntermediateConfigs(self, q_init, q_goal):
        q1 = q_goal[:]
        # Build projector to project a configuration in preplacement with placement
        # complement initialized with q_init
        self.problem.setRightHandSideFromConfigByName("place_reinforcment_bar/complement", q_init)
        self.problem.setRightHandSideFromConfigByName("locked_plate/root_joint", q_init)
    
        for i in range(2000):
            q = self.robot.shootRandomConfig()
            # Project random configuration in pregrasp reachable from q_goal
            edge = "tiago_pro/left > reinforcment_bar/left | f_01"
            res, q2, err = self.graph.generateTargetConfig(edge, q1, q)
            if not res: continue
            res, _ = self.robot.isConfigValid(q2)
            if not res: continue
            # Generate corresponding grasp configuration
            edge = "tiago_pro/left > reinforcment_bar/left | f_12"
            res, q3, err = self.graph.generateTargetConfig(edge, q2, q2)
            if not res: continue
            res, _ = self.robot.isConfigValid(q3)
            # Generate corresponding preplacement configuration
            edge = "tiago_pro/left > reinforcment_bar/left | f_23"
            res, q4, err = self.graph.generateTargetConfig(edge, q3, q3)
            if not res: continue
            res, _ = self.robot.isConfigValid(q4)
            if not res: continue
            # Project result on preplacement with place_reinforcment_bar/complement
            # initialized with q_init
            res, q5, err = self.problem.applyConstraints(q4)
            res, _ = self.robot.isConfigValid(q5)
            if res: break
        if not res:
            raise RuntimeError(
                "Failed to generate intermediate configurations to solve the problem."
                )
        self.problem.resetConstraints()
        return q4, q5

