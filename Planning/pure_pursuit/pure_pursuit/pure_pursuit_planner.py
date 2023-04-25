import numpy as np


class AdvancedPurePursuitPlanner:
    def __init__(self, conf, wb=0.33):
        """
        conf: NameSpace
        """

        self.wheelbase = wb
        self.conf = conf
        self.max_reacquire = 20.

        self.drawn_waypoints = []
        # waypoint index
        self.wpt_xind = self.conf.wpt_xind
        self.wpt_yind = self.conf.wpt_yind
        self.wpt_vind = self.conf.wpt_vind
        self.waypoints_xyv = None
        self.waypoints = None
        self.load_waypoints(conf)
        self.wpNum = self.waypoints.shape[0]

        # advanced pure pursuit
        self.minL = self.conf.minL
        self.maxL = self.conf.maxL
        self.minP = self.conf.minP
        self.maxP = self.conf.maxP
        self.Pscale = self.conf.Pscale
        self.Lscale = self.conf.Lscale
        self.D = self.conf.D
        self.vel_scale = self.conf.vel_scale
        self.prev_error = 0.0
        self.interpScale = self.conf.interpScale

        # ittc

        self.debug = self.conf.debug

    def _change_waypoint_xyv_idx(self, new_x_idx, new_y_idx, new_v_idx):
        self.wpt_xind = new_x_idx
        self.wpt_yind = new_y_idx
        self.wpt_vind = new_v_idx
        print('change waypoint x, y, v idx')

    def load_waypoints(self, conf):
        """
        loads waypoints
        """
        waypoints = np.loadtxt(conf.wpt_path, delimiter=conf.wpt_delim, skiprows=conf.wpt_rowskip)
        waypoints = np.vstack((waypoints[:, 1], waypoints[:, 2], waypoints[:, 5], waypoints[:, 3], waypoints[:, 0])).T
        self.waypoints = waypoints
        self.waypoints_xyv = waypoints[:, :3]

    def render_waypoints(self, e):
        """
        update waypoints being drawn by EnvRenderer
        """

        # points = self.waypoints

        points = np.vstack((self.waypoints[:, 0], self.waypoints[:, 1])).T

        scaled_points = 50. * points

        for i in range(points.shape[0]):
            if len(self.drawn_waypoints) < points.shape[0]:
                b = e.batch.add(1, GL_POINTS, None, ('v3f/stream', [scaled_points[i, 0], scaled_points[i, 1], 0.]),
                                ('c3B/stream', [183, 193, 222]))
                self.drawn_waypoints.append(b)
            else:
                self.drawn_waypoints[i].vertices = [scaled_points[i, 0], scaled_points[i, 1], 0.]

    def _get_current_waypoint(self, lookahead_distance, position, theta, waypoints):
        """
        gets the current waypoint to follow
        """
        nearest_p, nearest_dist, t, i = nearest_point(position, self.waypoints[:, 0:2])
        # import ipdb;
        # ipdb.set_trace()
        if nearest_dist < lookahead_distance:
            lookahead_point, i2, t2 = intersect_point(position,
                                                      lookahead_distance,
                                                      self.waypoints[:, 0:2],
                                                      i + t,
                                                      wrap=True)
            if i2 is None:
                all_distance = np.linalg.norm(self.waypoints[:, 0:2] - position, axis=1)
                all_distance_lh = np.abs(all_distance - lookahead_distance)
                best_p_idx = np.argmin(all_distance_lh)
                return self.waypoints[best_p_idx, :]
            current_waypoint = np.array([self.waypoints[i2, 0], self.waypoints[i2, 1], self.waypoints[i, 2]])
            return current_waypoint
        elif nearest_dist < self.max_reacquire:
            return self.waypoints[i, :]
        else:
            return None

    def get_L(self, curr_v):
        return curr_v * (self.maxL - self.minL) / self.Lscale + self.minL

    def plan(self, pose_x, pose_y, pose_theta, curr_v, waypoints):

        # get L, P with speed
        L = curr_v * (self.maxL - self.minL) / self.Lscale + self.minL
        P = self.maxP - curr_v * (self.maxP - self.minP) / self.Pscale

        position = np.array([pose_x, pose_y])
        lookahead_point, new_L, nearest_dist = get_wp_xyv_with_interp(L, position, pose_theta, waypoints, waypoints.shape[0], self.interpScale)
        self.nearest_dist = nearest_dist

        speed, steering, error = \
            get_actuation_PD(pose_theta, lookahead_point, position, new_L, self.wheelbase, self.prev_error, P, self.D)
        speed = speed * self.vel_scale
        self.prev_error = error

        # if self.debug:
        #     print(f'target_speed: {speed},  current_speed: {curr_v}, steering: {steering}')
        #     print(f'L: {L},  P:{P}, error: {error}')
        #     print(f'L:{L}, new_L:{new_L}')
        # if np.any(np.isnan(np.array([steering, speed]))):
        #     import ipdb
        #     ipdb.set_trace()
        return steering, speed
