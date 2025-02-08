#!/usr/bin/env python
#
# Copyright (c) 2023 CNRS
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

from hpp.corbaserver.manipulation import ConstraintGraph, \
    ConstraintGraphFactory, Rule, Constraints, CorbaClient, SecurityMargins

class Factory(ConstraintGraphFactory):
    def __init__(self, ps, graph):
        self.ps = ps
        ConstraintGraphFactory.__init__(self, graph)

    def generate(self):
        part = self.objects[0]
        box = self.objects[1]
        # create preplacement and placement constraints for the object
        self.ps.createTransformationConstraint(f"place_{part}", "",
            f"{part}/root_joint", [0,0,0,0,0,0,1], 6*[False])
        self.ps.createLockedJoint(f"place_{part}/complement",
            f"{part}/root_joint", [0,0,0,0,0,0,1])
        self.ps.setConstantRightHandSide(f"place_{part}/complement", False)
        self.ps.createTransformationConstraint(f"preplace_{part}",
            f"{box}/root_joint", f"{part}/root_joint", [0, 0, .30, 0, 0, 0, 1],
            [False, False, True, False, False, False])
        self.ps.createLockedJoint(f"place_{box}/complement",
            f"{box}/root_joint", [0,0,0,0,0,0,1])
        self.ps.setConstantRightHandSide(f"place_{box}/complement", False)
        self.ps.createTransformationConstraint(f"vertical_{part}", "",
            f"{part}/root_joint", [0,0,0,0,0,0,1],
            [True, True, False, True, True, True])
        self.ps.setConstantRightHandSide(f"vertical_{part}", False)
        ConstraintGraphFactory.generate(self)
        ig = 0; gripper = self.grippers[ig]
        for ih, h in enumerate(self.handles):
            if 'goal_handle' in h: continue
            edge = f"{gripper} > {h} | f_23"
            self.graph.addConstraints(edge = edge,
                constraints = Constraints(numConstraints =
                [f"vertical_{part}"])
            )
            edge = f"{gripper} < {h} | {ig}-{ih}_32"
            self.graph.addConstraints(edge = edge,
                constraints = Constraints(numConstraints =
                [f"vertical_{part}"])
            )

factory = None
# Create a handle and a gripper for the goal position of the object
# Create specific graph with vertical preplace motions
# The last handle is the center of the part.
def makeGraph(ps, robot, grippers, objects, handles, rules, possibleGrasps):
    global factory
    graph = ConstraintGraph(robot, 'graph')

    factory = Factory(ps, graph)
    factory.constraints.removeEmptyConstraints = False
    factory.setGrippers(grippers)
    factory.setObjects(objects, handles, [[], []])
    factory.setRules(rules)
    factory.setPossibleGrasps(possibleGrasps)
    factory.generate()
    return factory, graph