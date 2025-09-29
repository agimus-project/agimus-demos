# Copyright CNRS
# Authors: Florent Lamiraux

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from math import fabs

import numpy as np
from hpp.corbaserver import wrap_delete as wd


def concatenatePaths(paths):
    if len(paths) == 0:
        return None
    p = paths[0].asVector()
    for q in paths[1:]:
        assert np.linalg.norm(np.array(p.end()) - np.array(q.initial())) < 1e-8
        p.appendPath(q)
    return p


def getMaxVelocity(path):
    res = np.zeros(7)
    t = 0
    while t < path.length():
        v = path.derivative(t, 1)
        for i in range(7):
            res[i] = max(fabs(v[i]), res[i])
        t += 0.01
    return res


class PathPlanner(object):
    def __init__(self, ps, cg, gripper):
        self.gripper = gripper
        self.ps = ps
        self.graph = cg
        self.cproblem = wd(ps.hppcorba.problem.getProblem())
        self.cgraph = wd(self.cproblem.getConstraintGraph())
        self.crobot = wd(self.cproblem.robot())
        self.cdistance = wd(
            ps.hppcorba.problem.createDistance("Weighed", self.cproblem)
        )
        self.croadmap = wd(
            ps.client.manipulation.problem.createRoadmap(self.cdistance, self.crobot)
        )
        self.croadmap.constraintGraph(self.cgraph)
        self.transitionPlanner = ps.client.basic.problem.createPathPlanner(
            "TransitionPlanner", self.cproblem, self.croadmap
        )
        self.transitionPlanner.setPathProjector("Progressive", 0.02)
        self.transitionPlanner.maxIterations(1000)
        self.transitionPlanner.addPathOptimizer("RandomShortcut")
        self.transitionPlanner.addPathOptimizer("SplineGradientBased_bezier3")
        self.transitionPlanner.setEdge(cg.edges["Loop | f"])

    def generateRandomInitConfig(self, q_init):
        q = q_init.copy()
        q[:7] = q_init[:7] + np.random.normal(0.0, 0.5, 7)
        while not self.ps.robot.configIsValid(q):
            q[:7] = q_init[:7] + np.random.normal(0.0, 0.5, 7)
        return q

    def checkConfigurationValid(self, q_init):
        return self.ps.robot.configIsValid(q_init)

    def planPregrasp(self, handle, q_init, q_guesses=[]):
        qpg, qg = self.generateConfigurationsForHole(
            handle, q_init, q_guesses, testCollision=True
        )

        # print(q_init, flush=True)
        # print(q_guesses, flush=True)
        # print(handle, flush=True)

        print(qpg, qg, flush=True)

        if not qpg or not qg:
            return None
        # Plan between q_init and qpg
        prefix = f"{self.gripper} > {handle} | f"
        edge = prefix + "_01"
        self.transitionPlanner.setEdge(self.graph.edges[edge])
        return wd(self.transitionPlanner.planPath(q_init, [qpg], True))

    # Generate a pregrasp and a grasp configurations for a given hole
    def generateConfigurationsForHole(
        self, handle, q_init, q_guesses=[], testCollision=True
    ):
        prefix = f"{self.gripper} > {handle} | f"
        init_guesses = q_guesses + [
            self.ps.robot.shootRandomConfig() for i in range(1000)
        ]
        for q in init_guesses:
            edge = prefix + "_01"
            res, qpg, err = self.graph.generateTargetConfig(edge, q_init, q)
            if not res:
                continue
            if testCollision:
                res, msg = self.transitionPlanner.validateConfiguration(
                    qpg, self.graph.edges[edge]
                )
                if not res:
                    continue
            edge = prefix + "_12"
            res, qg, err = self.graph.generateTargetConfig(edge, qpg, qpg)
            if not res:
                continue
            if testCollision:
                res, msg = self.transitionPlanner.validateConfiguration(
                    qg, self.graph.edges[edge]
                )
                if not res:
                    continue
            if res:
                break
        if res:
            return qpg, qg
        return None, None

    def planPathToHole(self, handle, q_init, q_guesses=[]):
        qpg, qg = self.generateConfigurationsForHole(
            handle, q_init, q_guesses, testCollision=True
        )
        if not qpg or not qg:
            return None
        # Plan between q_init and qpg
        prefix = f"{self.gripper} > {handle} | f"
        edge = prefix + "_01"
        self.transitionPlanner.setEdge(self.graph.edges[edge])
        p1 = self.transitionPlanner.planPath(q_init, [qpg], True)
        if not p1:
            return None
        edge = prefix + "_12"
        self.transitionPlanner.setEdge(self.graph.edges[edge])
        p2, res, msg = self.transitionPlanner.directPath(qpg, qg, True)
        p21 = p2.asVector()
        p22 = self.transitionPlanner.timeParameterization(p21)
        p2.deleteThis()
        p21.deleteThis()
        p2 = p22
        if not res:
            if p2:
                p2.deleteThis()
            p1.deleteThis()
            return None
        p3, res, msg = self.transitionPlanner.directPath(qg, qpg, True)
        p31 = p3.asVector()
        p32 = self.transitionPlanner.timeParameterization(p31)
        p3.deleteThis()
        p31.deleteThis()
        p3 = p32
        assert res
        assert p3
        result = concatenatePaths([p1, p2, p3])
        p1.deleteThis()
        p2.deleteThis()
        p3.deleteThis()
        return wd(result)
