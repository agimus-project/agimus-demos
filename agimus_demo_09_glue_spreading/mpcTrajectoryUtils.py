import pinocchio as pin
import numpy as np
from pinocchio import SE3
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import interp1d, RBFInterpolator


class PatternGenerator:
    """
    A class to generate patterns for glue spreading.
    """
    def __init__(self, dims_or_dict, center=None):
        """
        Initialize the PatternGenerator.

        Accepted forms:
        - dims_or_dict is a dict with keys 'length','width','height','center'
        - dims_or_dict is an iterable (length, width, height) and `center` is provided
        """
        # support dict input
        if isinstance(dims_or_dict, dict):
            data = dims_or_dict
            self.object_length = float(data.get('length', 1.0))
            self.object_width = float(data.get('width', 1.0))
            self.object_height = float(data.get('height', 0.0))
            self.object_center = tuple(data.get('center', (0.0, 0.0, 0.0)))
        else:
            # expect iterable dims and a center tuple
            if center is None:
                raise ValueError("When passing dims as iterable you must also provide `center` (x,y,z)")
            self.object_length = float(dims_or_dict[0])
            self.object_width = float(dims_or_dict[1])
            self.object_height = float(dims_or_dict[2])
            self.object_center = tuple(center)

    def generate_pattern(self, pattern_type, step=10, stride=0.2, orientation='vertical'):
        if pattern_type == 'zigzag':
            x,y,z = self.zigzag(step=step, stride=stride, orientation=orientation)
            waypoints : list = []
            for i in range (len(x)):
                waypoints.append(np.array([x[i], y[i], z[i]]))
            return waypoints
        elif pattern_type == 'spiral':
            x,y,z = self.spiral_from_center(stride=stride)
            waypoints : list = []
            for i in range (len(x)):
                waypoints.append(np.array([x[i], y[i], z[i]]))
            return waypoints
        elif pattern_type == 'zigzag_curve':
            return self.zig_zag_curve(step=step, stride=stride, orientation=orientation)
        else:
            raise ValueError("Unknown pattern type")

    def zigzag(self, step=10, stride=0.2, orientation='vertical'):
        """
        Draws a zigzag pattern.

        Parameters:
        - start: starting point (x, y, z)
        - length: length of each zigzag segment
        - nb: number of zigzag segments
        - step: number of points per segment
        - stride: offset between segments
        - orientation: 'vertical', 'horizontal', or 'diagonal'
        """
        x, y, z = [], [], []
        if (orientation == 'vertical'):
            x_tmp, y_tmp, z_tmp = self.object_center[0] - self.object_length / 2, \
                self.object_center[1] - self.object_width / 2, \
                self.object_center[2] + self.object_height / 2
            num_lines = int(self.object_length // stride)
            if self.object_length % stride == 0:
                num_lines += 1
        elif (orientation == 'horizontal'):
            x_tmp, y_tmp, z_tmp = self.object_center[0] - self.object_length / 2, \
                self.object_center[1] - self.object_width / 2, \
                self.object_center[2] + self.object_height / 2
            # Ensure the last line is on the edge if division is exact
            num_lines = int(self.object_width / stride)
            if self.object_width % stride == 0:
                num_lines += 1
        else:
            raise ValueError("orientation must be 'vertical', 'horizontal'")

        for i in range(num_lines):
            direction = 1 if i % 2 == 0 else -1  # alternate direction

            for j in range(step + 1):
                if orientation == 'vertical':
                    x.append(x_tmp)
                    y.append(y_tmp + direction * j * self.object_width / step)
                    z.append(z_tmp)
                elif orientation == 'horizontal':
                    x.append(x_tmp + direction * j * self.object_length / step)
                    y.append(y_tmp)
                    z.append(z_tmp)

            if orientation == 'vertical':
                x_tmp += stride
                y_tmp = y[-1]  # continue from last y
                z_tmp = z[-1]
            elif orientation == 'horizontal':
                y_tmp += stride
                x_tmp = x[-1]  # continue from last x
                z_tmp = z[-1]
        return x, y, z

    def zig_zag_curve(self, step=10, stride=0.2, orientation='vertical'):
        """
        Génère un motif en zigzag où les courbes de demi-tour restent
        STRICTEMENT à l'intérieur des limites de l'objet.
        """
        # 1. Récupération des dimensions et limites
        x_c, y_c, z_c = self.object_center
        L, W, H = self.object_length, self.object_width, self.object_height
        z_ref = z_c + H / 2

        # Rayon de la courbe de demi-tour
        R = stride / 2.0

        # 2. Configuration des axes
        if orientation == 'vertical':
            # Stride sur X, Sweep (balayage) sur Y
            axis_stride, axis_sweep = 0, 1
            stride_start = x_c - L/2
            stride_end = x_c + L/2
            # Limites du balayage (bords de l'objet sur Y)
            sweep_min = y_c - W/2
            sweep_max = y_c + W/2
            stride_len = L
            
        elif orientation == 'horizontal':
            # Stride sur Y, Sweep (balayage) sur X
            axis_stride, axis_sweep = 1, 0
            stride_start = y_c - W/2
            stride_end = y_c + W/2
            # Limites du balayage (bords de l'objet sur X)
            sweep_min = x_c - L/2
            sweep_max = x_c + L/2
            stride_len = W
        else:
            raise ValueError("orientation must be 'vertical' or 'horizontal'")

        # 3. Calcul du nombre de lignes
        target_strides = np.arange(stride_start, stride_end, stride)
    
        # FORCE LA COUVERTURE DU DERNIER BORD :
        # Si la dernière ligne n'est pas exactement sur le bord max, on ajoute une ligne sur le bord max.
        if target_strides[-1] < (stride_end - 1e-5):
            target_strides = np.append(target_strides, stride_end)
        else:
            target_strides[-1] = stride_end # On s'assure que c'est préci
        
        num_lines = len(target_strides)
        
        trajectory_chunks = []
        current_stride_pos = stride_start

        for i in range(num_lines):
            # Direction: 1 = vers le haut/droite, -1 = vers le bas/gauche
            direction = 1 if i % 2 == 0 else -1
            
            # --- A. LIGNE DROITE ---
            # Logique de confinement :
            # Si on va vers le MAX (+1) : on s'arrête à (MAX - Rayon) pour laisser la place à la courbe
            # Si on va vers le MIN (-1) : on s'arrête à (MIN + Rayon)
            
            # Cas spécial : Le tout début de la trajectoire (i=0, start) commence au bord absolu
            if i == 0 and direction == 1:
                p_start = sweep_min
            else:
                p_start = sweep_min + R if direction == 1 else sweep_max - R

            # Cas spécial : La fin de la ligne droite
            # Si c'est la dernière ligne, on va jusqu'au bout.
            # Sinon, on s'arrête avant le bord pour faire la courbe.
            if i == num_lines - 1:
                p_end = sweep_max if direction == 1 else sweep_min
            else:
                p_end = sweep_max - R if direction == 1 else sweep_min + R

            # Génération des points de la ligne
            t_line = np.linspace(0, 1, step + 1)
            coords_line = np.zeros((len(t_line), 3))
            coords_line[:, 2] = z_ref
            coords_line[:, axis_stride] = current_stride_pos
            # Interpolation linéaire entre p_start et p_end
            coords_line[:, axis_sweep]  = p_start + (p_end - p_start) * t_line
            
            trajectory_chunks.append(coords_line)

            # --- B. COURBE DE DEMI-TOUR (Interne) ---
            if i < num_lines - 1:
                # On génère un arc de 180 degrés (pi radians)
                # On exclut le premier point (t=0) pour éviter le doublon avec la ligne
                t_arc = np.linspace(0, np.pi, (step // 2) + 1)[1:]
                
                coords_arc = np.zeros((len(t_arc), 3))
                coords_arc[:, 2] = z_ref
                
                # --- Calcul sur l'axe STRIDE (Avancement) ---
                # On part de current_stride_pos et on va à current_stride_pos + stride
                # Formule : pos + R * (1 - cos(t))
                # t=0 -> pos, t=pi -> pos + 2R (= pos + stride)
                coords_arc[:, axis_stride] = current_stride_pos + R * (1 - np.cos(t_arc))
                
                # --- Calcul sur l'axe SWEEP (Oscillation) ---
                # L'arc doit "bomber" vers le mur sans le traverser.
                # Si direction était +1 (on est en haut), on doit bomber vers le haut puis redescendre.
                # Le centre de rotation est à (p_end)
                # Formule : center + R * sin(t) * direction
                
                # Point pivot (fin de la ligne précédente)
                pivot = coords_line[-1, axis_sweep]
                coords_arc[:, axis_sweep] = pivot + R * np.sin(t_arc) * direction
                
                trajectory_chunks.append(coords_arc)
                
                # Mise à jour pour la prochaine ligne
                current_stride_pos += stride

        # 4. Assemblage et Conversion
        full_matrix = np.vstack(trajectory_chunks)
        
        # Conversion en liste de np.array([x,y,z]) comme demandé
        waypoints = [row for row in full_matrix]
        
        return waypoints


    def curve_arc(self, start_point, radius, dir, angle=np.pi, step=10):
        """
        Dessine un arc de cercle dans le plan XY.

        Args:
            start_point (tuple): point de départ (x, y, z)
            radius (float): rayon de la courbe
            dir (str): direction de la courbe ('+x', '-x', '+y', '-y')
            angle (float): angle de l'arc en radians (par défaut = pi/2 = 90°)
            step (int): nombre de points pour dessiner la courbe

        Returns:
            x, y, z (list): listes des coordonnées des points de la courbe
        """
        x, y, z = [], [], []
        angle_step = angle / step

        for i in range(1, step):
            theta = i * angle_step
            if dir == '+x':
                xi = start_point[0] + radius * np.sin(theta)
                yi = start_point[1] + radius * (1 - np.cos(theta))
            elif dir == '-x':
                xi = start_point[0] - radius * np.sin(theta)
                yi = start_point[1] + radius * (1 - np.cos(theta))
            elif dir == '+y':
                xi = start_point[0] + radius * (1 - np.cos(theta))
                yi = start_point[1] + radius * np.sin(theta)
            elif dir == '-y':
                xi = start_point[0] + radius * (1 - np.cos(theta))
                yi = start_point[1] - radius * np.sin(theta)
            else:
                raise ValueError("dir must be '+x', '-x', '+y', or '-y'")

            x.append(xi)
            y.append(yi)
            z.append(start_point[2])  # z constant

        return x, y, z

    def spiral_from_center(self, stride=1.0):
        """
        Génère une spirale polygonale (carrée) qui part du centre et s'étend vers l'extérieur,
        sans dépasser les dimensions de l'objet.

        Args:
            stride (float): espacement entre chaque "tour"

        Returns:
            x, y, z: listes des coordonnées des points de la spirale
        """
        # Initialisation
        x = [self.object_center[0]]
        y = [self.object_center[1]]
        z = [self.object_center[2] + self.object_height / 2]

        # Limites de l'objet
        x_min = self.object_center[0] - self.object_length / 2
        x_max = self.object_center[0] + self.object_length / 2
        y_min = self.object_center[1] - self.object_width / 2
        y_max = self.object_center[1] + self.object_width / 2

        # Directions: droite, haut, gauche, bas
        directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        lengths = [self.object_length, self.object_width]  # longueur côté courant
        current_lengths = [stride, stride]  # longueur à parcourir pour chaque direction
        dir_idx = 0  # index de direction
        steps = 0

        while True:
            dx, dy = directions[dir_idx % 4]
            # Détermine la longueur maximale possible sans dépasser les bords
            if dx != 0:
                # Mouvement en x
                if dx > 0:
                    max_len = min(current_lengths[0], x_max - x[-1])
                else:
                    max_len = min(current_lengths[0], x[-1] - x_min)
            else:
                # Mouvement en y
                if dy > 0:
                    max_len = min(current_lengths[1], y_max - y[-1])
                else:
                    max_len = min(current_lengths[1], y[-1] - y_min)

            # Si la longueur à parcourir est nulle ou négative, on s'arrête
            if max_len <= 0:
                break

            # Ajoute le nouveau point
            new_x = x[-1] + dx * max_len
            new_y = y[-1] + dy * max_len
            x.append(new_x)
            y.append(new_y)
            z.append(z[-1])

            # Prépare la longueur pour le prochain tour
            if dir_idx % 2 == 1:
                current_lengths[0] += stride
                current_lengths[1] += stride

            dir_idx += 1
            steps += 1

            # Arrête si on touche les bords
            if not (x_min <= new_x <= x_max and y_min <= new_y <= y_max):
                break

        return x, y, z


class SplineGenerator:
    """
    Class that interpolates the `waypoints` into a spline and calculates the orientation between them so that the X axis faces the nex waypoint and the Z axis faces downward
    """
    def __init__(self, start_pos, start_ori, waypoints,
                 v_start=1.0, v_spread=1.0,
                 start_kernel='linear', spread_kernel='cubic'):
        if len(waypoints) < 2:
            raise ValueError("At least two waypoints are required.")

        self.start_pos = np.array(start_pos)
        self.start_ori = np.array(start_ori)
        self.waypoints = np.array(waypoints)
        self.v_start = v_start
        self.v_spread = v_spread
        self.start_kernel = start_kernel
        self.spread_kernel = spread_kernel

        self.start_traj = None
        self.spread_traj = None
        self.last_valid_orientation_ref = np.array([0.0, 0.0, 0.0])
        self.t_total = 0

        self._compute_and_set_times()
        self._compute_full_traj()
        self._compute_full_ori()

    def _compute_and_set_times(self):
        """ Computes time durations based on distances and velocities. """
        # Distance for first segment (Current Robot Pose -> Pattern Start)
        d_start = np.linalg.norm(self.waypoints[0] - self.start_pos)
        
        # Handle distances for the start segment with protection against zero velocity
        t_start = d_start / self.v_start if self.v_start > 0 else 0.0

        # Distances for the spread segment
        diffs = np.diff(self.waypoints, axis=0)
        d_spread = np.sum(np.linalg.norm(diffs, axis=1))

        t_spread = d_spread / self.v_spread if self.v_spread > 0 else 0.0

        self.t_start = t_start
        self.t_total = t_start + t_spread

        # Update time arrays
        self.time_start = np.array([0.0, self.t_start])[:, None]
        self.time_spread = np.linspace(self.t_start, self.t_total, len(self.waypoints))[:, None]

    def _compute_start_trj(self):
        """
        Initiates the interpolator for the trajectory between the start pose and the first waypoint

        Args:
            None
        Returns:
            None
        Sets:
            self.start_traj (RBFInterpolator)
        """
        points = np.vstack([self.start_pos, self.waypoints[0]])
        self.start_traj = RBFInterpolator(self.time_start, points, kernel=self.start_kernel)

    def _compute_spread_traj(self):
        """
        Initiates the interpolator for the trajectory between the first waypoint and the last

        Args:
            None
        Returns:
            None
        Sets:
            self.spread_traj (RBFInterpolator)
        """
        points = self.waypoints[0:]
        self.spread_traj = RBFInterpolator(self.time_spread, points, kernel=self.spread_kernel)

    def _compute_full_traj(self):
        """
        Initiates the interpolators for the trajectory between the start pose and the first waypoint and between the first waypoint and the last.

        Args:
            None
        Returns:
            None
        Sets:
            self.start_traj : (RBFInterpolator) interpolator for the pose of the start trajectory
            self.spread_traj : (RBFInterpolator) interpolator for the pose of the spread trajectory
        """
        self._compute_start_trj()
        self._compute_spread_traj()

    def get_interpolated_pose(self, t):
        """
        Returns the pose of the end effector at a given `t` (secs)
        Args:
            t : time in seconds (float)
        Returns:
            pose : (list[float]) pose of end effector for t
        Sets:
            None
        """
        t = np.clip(t, 0.0, self.t_total)
        if t <= self.t_start:
            return self.start_traj(np.array([[t]]))[0]
        else:
            return self.spread_traj(np.array([[t]]))[0]

    def _compute_start_orientation(self, spread_orientations):
        """
        Instanciates the interpolator from the start orientation to the orientation of the first pattern waypoint
        Args:
            spread_orientations : list of the orientations for the spreading part of the trajectory
        Returns:
            None
        Sets:
            start_ori_traj : (RBFInterpolator) interpolator for the orientation of the start trajectory
        """
        orientations = np.vstack((self.start_ori, spread_orientations[0]))
        self.start_ori_traj = RBFInterpolator(self.time_start, orientations, kernel=self.start_kernel)

    def _compute_spread_orientation(self):
        """
        Computes the orientation of the end effector waypoint by waypoint so that the X axis points to the next waypoint and the Z axis points down.
        Also initializes the interpolator between the first waypoint to the last.
        Args:
            None
        Returns:
            spread_orientations : (list[float]) list of orientations for the spread part of the trajectory
        Sets:
            start_ori_traj : (RBFInterpolator) interpolator for the orientation of the spread trajectory
        """
        points = self.waypoints[0:]
        spread_orientations = []
        for i in range(len(points)-1):
            spread_orientations.append(self._compute_local_orientation(points[i], points[i+1]))
        spread_orientations.append(spread_orientations[-1]) # repeat last orientation to match the number of waypoints
        self.spread_ori_traj = RBFInterpolator(self.time_spread, spread_orientations, kernel=self.spread_kernel)
        return spread_orientations

    def _compute_local_orientation(self, current_point, next_point):
        """
        Computes the orientation between two points so that the X axis points from the `current_point` to the `next_point` and the Z axis points down
        Args:
            None
        Returns:
            orientation : (list[float]) Roll Pitch Yaw orientation of the effector
        Sets:
            None
        """
        direction_vector = next_point - current_point
        roll = np.pi
        pitch = 0
        yaw = np.arctan2(direction_vector[1], direction_vector[0])
        orientation = np.array([roll, pitch, yaw])
        return orientation

    def _compute_full_ori(self):
        """
        Computes the orientation for the full trajectory
        Args:
            None
        Returns:
            None
        Sets:
            None
        """
        spread_ori = self._compute_spread_orientation()
        self._compute_start_orientation(spread_ori)

    def get_interpolated_ori(self,t):
        """
        Returns the orientation of the end effector at a given `t` (secs)
        Args:
            t : time in seconds (float)
        Returns:
            orientation : (list[float]) Roll Pitch Yaw of the end effector
        Sets:
            None
        """
        t = np.clip(t, 0.0, self.t_total)
        if t <= self.t_start:
            return self.start_ori_traj(np.array([[t]]))[0]
        else:
            return self.spread_ori_traj(np.array([[t]]))[0]
    
    def transform(self, translation, rotation):
        """
        Apply a full SE3 transform (rotation + translation) to all waypoints AND orientations.
        This keeps the spline consistent with the new reference frame.

        Args:
            translation: geometry_msgs/Vector3
            rotation: geometry_msgs/Quaternion
        """

        # --- 1. Normalize quaternion (ROS can give non-normalized Q) ---
        quat_ros = np.array([rotation.x, rotation.y, rotation.z, rotation.w])
        quat_ros = quat_ros / np.linalg.norm(quat_ros)

        trans_ros = np.array([translation.x, translation.y, translation.z])

        # --- 2. Build the SE3 transform ---
        q_pin = pin.Quaternion(quat_ros).normalize()
        R_tf = q_pin.toRotationMatrix()
        T = pin.SE3(R_tf, trans_ros)
        T_matrix = T.homogeneous

        # --- 3. Transform all waypoint positions ---
        assert self.waypoints.shape[1] == 3, (
            f"Waypoints should have shape (N,3), got {self.waypoints.shape}"
        )

        homogeneous_points = np.hstack([
            self.waypoints,
            np.ones((self.waypoints.shape[0], 1))
        ])

        transformed_points = (T_matrix @ homogeneous_points.T).T
        self.waypoints = transformed_points[:, :3]

        # --- 4. Transform all orientations (RPY) ---
        new_orientations = []
        for i in range(len(self.waypoints) - 1):
            # Recompute local orientation
            wp_ori = self._compute_local_orientation(
                self.waypoints[i], self.waypoints[i + 1]
            )

            # Convert to rotation matrix
            R_local = pin.utils.rpyToMatrix(*wp_ori)

            # Apply transform rotation
            R_global = R_tf @ R_local

            # Convert back to RPY
            rpy_global = pin.utils.matrixToRpy(R_global)
            new_orientations.append(rpy_global)

        # Duplicate last orientation to keep same size
        new_orientations.append(new_orientations[-1])
        self.transformed_orientations = np.array(new_orientations)

        # --- 5. Rebuild interpolation based on new points & orientations ---
        self._compute_and_set_times()
        self._compute_full_traj()

        # Replace spread orientations with computed ones
        self.spread_ori_traj = RBFInterpolator(
            self.time_spread, self.transformed_orientations, kernel=self.spread_kernel
        )

        # Recompute the start orientation interpolator
        start_oris = np.vstack((self.start_ori, self.transformed_orientations[0]))
        self.start_ori_traj = RBFInterpolator(
            self.time_start, start_oris, kernel=self.start_kernel
        )

    def update_interpolators(self):
        """
        Recomputes the interpolators after a transformation
        Args:
            None
        Returns:
            None
        Sets:
            self.t_total : (float) time of the complete trajectory
            self.start_traj : (RBFInterpolator) interpolator for the pose of the start trajectory
            self.spread_traj : (RBFInterpolator) interpolator for the pose of the spread trajectory
        """
        self._compute_and_set_times() 
        self._compute_full_traj()
        self._compute_full_ori()
class MockPoint:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z

class MockQuaternion:
    def __init__(self, x, y, z, w):
        self.x, self.y, self.z, self.w = x, y, z, w

def draw_frame(ax, pose: SE3,scale=[1, 1, 1]):
    """
    Draws 3 arrows in the `ax` plot showing XYZ pose and RPY rotation.
    Args:
        ax : (Axes) plot
    Returns:
        None
    """
    origin = pose.translation
    R = pose.rotation

    colors = ['r', 'g', 'b']
    for i in range(3):
        axis = R[:, i]  # vecteur direction
        ax.quiver(
            origin[0], origin[1], origin[2],
            axis[0]*scale[i], axis[1]*scale[i], axis[2]*scale[i],
            color=colors[i], arrow_length_ratio = 0.01, length=0.05
        )

if __name__=="__main__":
    object= {
            'width': 1.0,
            'length': 1.0,
            'height': 0.0,
            'center': [0.,0.,0.2],
        }
    patternGen = PatternGenerator(object)
    positions = patternGen.generate_pattern('zigzag_curve',stride=0.45)

    # positions = [np.array([0.5, 0.0, 0.2]),
    #             np.array([ 0.5, 0.0, 0.5]),
    #             np.array([0.35, 0.35, 0.5]),
    #             np.array([0.35, 0.35, 0.2]),
    #             np.array([0.0, 0.5, 0.2]),
    #             np.array([0.0, 0.5, 0.5]),
    #             np.array([-0.35, 0.35, 0.5]),
    #             np.array([-0.35, 0.35, 0.2]),
    #             np.array([-0.5, 0.0, 0.2]),
    #             np.array([-0.5, 0.0, 0.5]),
    #             np.array([-0.35, -0.35, 0.5]),
    #             np.array([-0.35, -0.35, 0.2]),
    #             np.array([0.0, -0.5, 0.2]),
    #             np.array([0.0, -0.5, 0.5]),
    #             np.array([0.35, -0.35,  0.5]),
    #             np.array([0.35, -0.35,  0.2]),
    #             np.array([0.5, 0.0, 0.2])]

    duration = 7
    start_pose = [ 3.33970764e-01, -3.08143602e-16,  5.40159383e-01]
    start_ori = [ 2.77295717, -0.34585912 , 0.85050551]
    spline = SplineGenerator(start_pose, start_ori, waypoints=positions)

    
    # Debug of trajectory
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    t = 0
    while t < spline.t_total:
        orientation = spline.get_interpolated_ori(t)
        roll = orientation[0]
        pitch = orientation[1]
        yaw = orientation[2]
        pose = spline.get_interpolated_pose(t)
        R = pin.utils.rpyToMatrix(roll, pitch, yaw)
        print((R))
        print((pose))
        pose_6d = pin.SE3(R, pose)
        draw_frame(ax, pose_6d)
        ax.scatter(*pose, marker="^", c="r",alpha=0.5,s=15)
        t = t + 0.02


    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title("Interpolation position + orientation 3D (RBF)")
    ax.legend()

    trans = MockPoint(0,0,0)
    rot = MockQuaternion(0, 0, 0.7071, 0.7071)
    rot = MockQuaternion(0.7071, 0, 0.0, 0.7071)


    spline.transform(trans, rot)
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111, projection='3d')
    t = 0
    while t < spline.t_total:
        orientation = spline.get_interpolated_ori(t)
        roll = orientation[0]
        pitch = orientation[1]
        yaw = orientation[2]
        pose = spline.get_interpolated_pose(t)
        R = pin.utils.rpyToMatrix(roll, pitch, yaw)
        print((R))
        print((pose))
        pose_6d = pin.SE3(R, pose)
        draw_frame(ax2, pose_6d)
        ax2.scatter(*pose, marker="^", c="r",alpha=0.5,s=15)
        t = t + 0.02


    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.set_title("Interpolation position + orientation 3D (RBF) Transformed")
    ax.legend()
    plt.tight_layout()
    plt.show()

    # traj = []
    # dt = 0.01
    # times = np.arange(0, duration + dt, dt)
    # for t in times:
    #     pose = spline.interpolate_pose(t)
    #     traj.append(pose)
    #     # print("pose:" + str(pose))
    # traj = np.array(traj)
    # wp_pos = np.array(positions)
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], label="Trajectoire interpolée", marker=".", color='blue',alpha=0.5)
    # ax.scatter(wp_pos[:, 0], wp_pos[:, 1], wp_pos[:, 2], label="Waypoints", color='red', s=50)
    # # ax.plot(x,y,z, label="Pattern generator", marker=".", color='cyan')
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # ax.set_zlabel('Z')
    # ax.set_title("Interpolation position 3D (RBF)")
    # ax.legend()
    # ax.set_box_aspect([1, 1, 1])
    # plt.tight_layout()
    # plt.show()