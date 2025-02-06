import numpy as np

def concatenatePaths(paths):
    if len(paths) == 0: return None
    p = paths[0].asVector()
    for q in paths[1:]:
        assert(p.end() == q.initial())
        p.appendPath(q)
    return p

def split_path(path):
    path = path.flatten()
    grasp_path_idxs = [0]
    placing_path_idxs = []
    freefly_path_idxs = []

    for idx in range(1, path.numberPaths()):
        if path_move_object(path.pathAtRank(idx)):
            placing_path_idxs.append(idx)
        else:
            if len(placing_path_idxs) == 0:
                grasp_path_idxs.append(idx)
            else:
                freefly_path_idxs.append(idx)
    grasp_path = concatenatePaths([path.pathAtRank(idx) for idx in grasp_path_idxs])
    placing_path = concatenatePaths([path.pathAtRank(idx) for idx in placing_path_idxs])
    freefly_path = concatenatePaths([path.pathAtRank(idx) for idx in freefly_path_idxs])
    return grasp_path, placing_path, freefly_path


def path_move_object(path):
    object_init_pose = np.array(path.initial()[9:12])
    object_end_pose = np.array(path.end()[9:12])
    eps = 1e-4
    if np.linalg.norm(object_end_pose - object_init_pose) < eps:
        return False
    else:
        return True

class BaseObject(object):
    rootJointType = "freeflyer"

    def __init__(self, urdf_path: str, srdf_path: str, name: str):
        self.urdfFilename = urdf_path
        self.srdfFilename = srdf_path
        self.name = name
    

