"""
This client connects to a Qualisys (motion capture) server with an asynchronous subprocess
and exposes 6d position and velocity of given bodies.
Thomas FLAYOLS - LAAS CNRS
(Updated to support multiple bodies)
"""

import asyncio
import qtm
import numpy as np
import pinocchio

from multiprocessing import Process
from multiprocessing.sharedctypes import Value, Array
from ctypes import c_double
from pinocchio.explog import log

class QualisysClient:
    def __init__(self, ip="127.0.0.1", bodies={"object": 1}):
        self.bodies = bodies
        self.num_bodies = len(self.bodies.keys())
        # Shared c_double arrays sized for N bodies
        self.shared_bodyPosition = Array(c_double, 3 * self.num_bodies, lock=False)
        self.shared_bodyVelocity = Array(c_double, 3 * self.num_bodies, lock=False)
        self.shared_bodyOrientationQuat = Array(c_double, 4 * self.num_bodies, lock=False)
        self.shared_bodyOrientationMat9 = Array(c_double, 9 * self.num_bodies, lock=False)
        self.shared_bodyAngularVelocity = Array(c_double, 3 * self.num_bodies, lock=False)
        self.shared_timestamp = Value(c_double, lock=False)

        args = (
            ip,
            self.bodies,
            self.shared_bodyPosition,
            self.shared_bodyVelocity,
            self.shared_bodyOrientationQuat,
            self.shared_bodyOrientationMat9,
            self.shared_bodyAngularVelocity,
            self.shared_timestamp,
        )
        self.p = Process(target=self.qualisys_process, args=args)
        self.p.start()

    def stop(self):
        self.p.terminate()
        self.p.join()

    def getPositions(self):
        """Returns shape (num_bodies, 3)"""
        return np.array(self.shared_bodyPosition).reshape(self.num_bodies, 3)

    def getVelocities(self):
        """Returns shape (num_bodies, 3)"""
        return np.array(self.shared_bodyVelocity).reshape(self.num_bodies, 3)

    def getAngularVelocities(self):
        """Returns shape (num_bodies, 3)"""
        return np.array(self.shared_bodyAngularVelocity).reshape(self.num_bodies, 3)

    def getOrientationQuats(self):
        """Returns shape (num_bodies, 4)"""
        return np.array(self.shared_bodyOrientationQuat).reshape(self.num_bodies, 4)

    def qualisys_process(
        self,
        ip,
        bodies,
        shared_bodyPosition,
        shared_bodyVelocity,
        shared_bodyOrientationQuat,
        shared_bodyOrientationMat9,
        shared_bodyAngularVelocity,
        shared_timestamp,
    ):
        print(f"Qualisys process started tracking bodies: {bodies}!")
        """ This will run on a different process"""
        shared_timestamp.value = -1
        
        def on_packet(packet):
            """Callback function called everytime a data packet arrives from QTM"""
            timestamp = packet.timestamp * 1e-6
            is_first_frame = (shared_timestamp.value == -1)
            dt = 0 if is_first_frame else (timestamp - shared_timestamp.value)

            for idx, (body_name, body_id) in enumerate(bodies.items()):
                # Ensure the body_id exists in the packet to avoid index errors
                try:
                    position = packet.get_6d()[1][body_id][0]
                    orientation = packet.get_6d()[1][body_id][1]
                except IndexError:
                    continue

                if np.isnan(position[0]):
                    continue

                p_idx = idx * 3
                q_idx = idx * 4
                m_idx = idx * 9

                # Read last state from shared memory to compute velocity
                last_position = np.array([
                    shared_bodyPosition[p_idx], 
                    shared_bodyPosition[p_idx+1], 
                    shared_bodyPosition[p_idx+2]
                ])
                last_rotation = np.array([
                    [shared_bodyOrientationMat9[m_idx], shared_bodyOrientationMat9[m_idx+1], shared_bodyOrientationMat9[m_idx+2]],
                    [shared_bodyOrientationMat9[m_idx+3], shared_bodyOrientationMat9[m_idx+4], shared_bodyOrientationMat9[m_idx+5]],
                    [shared_bodyOrientationMat9[m_idx+6], shared_bodyOrientationMat9[m_idx+7], shared_bodyOrientationMat9[m_idx+8]]
                ])
                last_se3 = pinocchio.SE3(last_rotation, last_position)

                # Process new state
                pos_arr = np.array([position.x, position.y, position.z]) * 1e-3
                rot_arr = np.array(orientation.matrix).reshape(3, 3).transpose()
                se3 = pinocchio.SE3(rot_arr, pos_arr)
                xyzquat = pinocchio.SE3ToXYZQUAT(se3)

                # Write Positions and Quaternions
                shared_bodyPosition[p_idx:p_idx+3] = xyzquat[0:3]
                shared_bodyOrientationQuat[q_idx:q_idx+4] = xyzquat[3:7]

                # Write Rotation Matrix
                shared_bodyOrientationMat9[m_idx] = orientation.matrix[0]
                shared_bodyOrientationMat9[m_idx+1] = orientation.matrix[3]
                shared_bodyOrientationMat9[m_idx+2] = orientation.matrix[6]
                shared_bodyOrientationMat9[m_idx+3] = orientation.matrix[1]
                shared_bodyOrientationMat9[m_idx+4] = orientation.matrix[4]
                shared_bodyOrientationMat9[m_idx+5] = orientation.matrix[7]
                shared_bodyOrientationMat9[m_idx+6] = orientation.matrix[2]
                shared_bodyOrientationMat9[m_idx+7] = orientation.matrix[5]
                shared_bodyOrientationMat9[m_idx+8] = orientation.matrix[8]

                # Compute world velocity
                if is_first_frame or dt <= 0:
                    shared_bodyVelocity[p_idx:p_idx+3] = [0.0, 0.0, 0.0]
                    shared_bodyAngularVelocity[p_idx:p_idx+3] = [0.0, 0.0, 0.0]
                else:
                    shared_bodyVelocity[p_idx:p_idx+3] = (pos_arr - last_position) / dt
                    bodyAngularVelocity = log(last_se3.rotation.T.dot(se3.rotation)) / dt
                    shared_bodyAngularVelocity[p_idx:p_idx+3] = bodyAngularVelocity[:]

            shared_timestamp.value = timestamp

        async def setup():
            connection = await qtm.connect(ip)
            if connection is None:
                print("no connection with qualisys!")
                return
            print("Connected to Qualisys")
            try:
                await connection.stream_frames(components=["6d"], on_packet=on_packet)
            except:
                print("connection with qualisys lost")

        asyncio.ensure_future(setup())
        asyncio.get_event_loop().run_forever()
        
def exampleOfUse():
    import time
    import matplotlib.pyplot as plt

    id = 17
    print(f"ID: {id}")
    qc = QualisysClient(ip="140.93.1.100", bodies={"robot": id})
    N = 1000000
    positions = np.zeros((3, N))
    quaternions = np.zeros((4, N))
    lin_velocities = np.zeros((3, N))
    ori_velocities = np.zeros((3, N))

    for i in range(N):
        print("======================================")
        print("position: ", qc.getPositions())
        print("orientation: ", qc.getOrientationQuats())
        print("linear velocity: ", qc.getVelocities())
        print("angular velocity: ", qc.getAngularVelocities()*180/np.pi)
        positions[:, i] = qc.getPositions()
        quaternions[:, i] = qc.getOrientationQuats()
        lin_velocities[:, i] = qc.getVelocities()
        ori_velocities[:, i] = qc.getAngularVelocities() *180/np.pi

        time.sleep(0.003)
    qc.stop()

def discover(ip="140.93.1.100"):
    """
    Connect to a Qualisys QTM server and print all 6D bodies with their index,
    name, and current position. Use this to find the IDs to pass to QualisysClient.

    Usage:
        python3 qualisys.py discover
        python3 qualisys.py discover 192.168.1.10
    """
    import xml.etree.ElementTree as ET

    async def _run():
        connection = await qtm.connect(ip)
        if connection is None:
            print(f"Could not connect to Qualisys at {ip}")
            return

        # Get named body list from QTM parameters
        params_xml = await connection.get_parameters(parameters=["6d"])
        root = ET.fromstring(params_xml)
        names = [el.find("Name").text for el in root.iter("Body") if el.find("Name") is not None]

        # Grab one packet to get current positions
        future = asyncio.get_event_loop().create_future()

        def on_packet(packet):
            if not future.done():
                future.set_result(packet)

        await connection.stream_frames(components=["6d"], on_packet=on_packet)
        packet = await future
        await connection.stream_frames_stop()

        _, bodies = packet.get_6d()

        print(f"\n{'ID':>4}  {'Name':<30}  {'x (mm)':>10}  {'y (mm)':>10}  {'z (mm)':>10}  {'visible'}")
        print("-" * 80)
        for i, (pos, _) in enumerate(bodies):
            name = names[i] if i < len(names) else "?"
            visible = not (pos.x != pos.x)  # NaN check
            if visible:
                print(f"{i:>4}  {name:<30}  {pos.x:>10.1f}  {pos.y:>10.1f}  {pos.z:>10.1f}  yes")
            else:
                print(f"{i:>4}  {name:<30}  {'---':>10}  {'---':>10}  {'---':>10}  NO (not tracked)")

    asyncio.ensure_future(_run())
    asyncio.get_event_loop().run_forever()


if __name__ == "__main__":
    import sys
    if len(sys.argv) >= 2 and sys.argv[1] == "discover":
        ip = sys.argv[2] if len(sys.argv) >= 3 else "140.93.1.100"
        discover(ip)
    else:
        exampleOfUse()