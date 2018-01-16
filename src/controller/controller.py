import http.client, json
import time
import winsound
from logging import getLogger
from math import atan2, sin, sqrt, pi, hypot

from controller.util import heading, UnexpectedResponse

logger = getLogger('controller')

# Headers sent with every POST speed requests
HEADERS = {"Content-type": "application/json", "Accept": "text/json"}

ROBOT_WIDTH = 0.9  # actual is 0.4
g_lookahead = 1.0
g_blocked_times = 0


class Controller:
    """
    Controller base class which contains methods to send requests to the MRDS server and the travel monitoring
    routine.
    """

    class UnexpectedResponse(Exception):
        """
        Custom exception class raised when a HTTP request fails.
        """
        pass

    def __init__(self, mrds_url, lin_spd=1.0):
        """
        Initializes a new instance of Controller.
        :param mrds_url: url which the MRDS server listens on
        :type mrds_url: str
        :param lin_spd: linear speed value to automatically send in the requests
        :type lin_spd: float
        :param delta_pos: minimum distance to a point determine if the robot reached that point
        :type delta_pos: float

        """
        self.__mrds_url = mrds_url
        self._lin_spd = lin_spd

        logger.info('Controller set with linear speed={}'.format(lin_spd))

    def post_speed(self, angular_speed, linear_speed):
        """
        Sends a speed command to the MRDS server.

        :param angular_speed: value of angular speed
        :type angular_speed: float
        :param linear_speed: value of linear speed
        :type linear_speed: float
        """
        params = json.dumps({'TargetAngularSpeed': angular_speed, 'TargetLinearSpeed': linear_speed})
        self.__mrds = http.client.HTTPConnection(self.__mrds_url)
        self.__mrds.request('POST', '/lokarria/differentialdrive', params, HEADERS)
        response = self.__mrds.getresponse()
        status = response.status
        response.close()
        if status == 204:
            return response
        else:
            raise self.UnexpectedResponse(response)

    def getPose(self):
        """Reads the current position and orientation from the MRDS"""
        self.__mrds = http.client.HTTPConnection(self.__mrds_url)
        self.__mrds.request('GET', '/lokarria/localization')
        response = self.__mrds.getresponse()
        if (response.status == 200):
            poseData = response.read()
            response.close()
            return json.loads(poseData.decode())
        else:
            return UnexpectedResponse(response)

    def get_laser_scan(self):
        """
        Requests the current laser scan from the MRDS server and parses it into a dict.
        """
        self.__mrds = http.client.HTTPConnection(self.__mrds_url)
        self.__mrds.request('GET', '/lokarria/laser/echoes')
        response = self.__mrds.getresponse()
        if response.status == 200:
            laser_data = response.read().decode('utf-8')
            response.close()
            return json.loads(laser_data)
        else:
            return self.UnexpectedResponse(response)

    def get_laser_scan_angles(self):
        """
        Requests the current laser properties from the MRDS server and returns a list of the laser angles.
        """
        self.__mrds = http.client.HTTPConnection(self.__mrds_url)
        self.__mrds.request('GET', '/lokarria/laser/properties')
        response = self.__mrds.getresponse()
        if response.status == 200:
            laser_data = response.read().decode('utf-8')
            response.close()
            properties = json.loads(laser_data)
            beamCount = int((properties['EndAngle'] - properties['StartAngle']) / properties['AngleIncrement'])
            a = properties['StartAngle']  # +properties['AngleIncrement']
            angles = []
            while a <= properties['EndAngle']:
                angles.append(a)
                a += pi / 180  # properties['AngleIncrement']
            # angles.append(properties['EndAngle']-properties['AngleIncrement']/2)
            return angles
        else:
            raise self.UnexpectedResponse(response)

    def is_within_lookahead(self, position, nextPosition):
        """
         Checks whether the distance between two positions is less than LOOKAHEAD
         Input: position - A postition, usually the robots current position
         Input: nextPosition - A postition
         Output: Bool - True if the distance between two positions is less than LOOKAHEAD, else false.
        """

        return self.get_distance(position, nextPosition) <= g_lookahead

    def get_distance(self, position1, position2):
        """
         Get distance between to positions
         Input: position1 - A postition
         Input: position2 - A postition
         Output: The distance between two positions
        """

        dy = abs(position1['Y'] - position2['Y'])
        dx = abs(position1['X'] - position2['X'])

        return hypot(dy, dx)

    def get_angle(self, pose, nextPosition):
        """
        Get angle between robots heading and nextPosition
        Input:  pose - Robot's current pose
                nextPosition - Position to get angle to
        Output: The angle between pose and nextPosition
        """

        currentPosition = pose['Pose']['Position']
        dy = nextPosition['Y'] - currentPosition['Y']
        dx = nextPosition['X'] - currentPosition['X']
        currentBearing = heading(pose['Pose']['Orientation'])

        robotAngle = atan2(currentBearing['Y'], currentBearing['X'])
        angleToPosition = atan2(dy, dx)
        angle = robotAngle - angleToPosition

        # Fixes weird angle bug
        while angle > pi:
            angle = -  pi
        while angle < -pi:
            angle = +  pi

        return angle

    def get_next_carrot_node(self, pose, cur_node_num, path):
        '''
        Get the next node to "aim" at
        Input:  pose - Robot's current pose
                curNodeNum - Index of the current goal node
                path - The path to follow
        Output: The carrot (goal) node
        '''

        while (self.is_within_lookahead(pose['Pose']['Position'],
                                        path[cur_node_num]['Pose']['Position']) and
               (cur_node_num < len(path) - 1)):
            cur_node_num += 1

        return cur_node_num

    def get_next_carrot_position(self, pose, cur_node_num, path):
        '''
        Get the exact position of the carrot point on the path segment
        between node curNodeNum and the next one
        Input:  pose - Robot's current pose
                curNodeNum - Index of the current goal node
                path - The path to follow
        Output: The carrot (goal) position
        '''

        if cur_node_num == len(path) - 1:
            return path[cur_node_num]['Pose']['Position']

        x1 = path[cur_node_num]['Pose']['Position']['X']
        x2 = path[cur_node_num + 1]['Pose']['Position']['X']
        y1 = path[cur_node_num]['Pose']['Position']['Y']
        y2 = path[cur_node_num + 1]['Pose']['Position']['Y']

        distance_to_add = g_lookahead - self.get_distance(pose['Pose']['Position'],
                                                          path[cur_node_num]['Pose']['Position'])

        dy = y2 - y1
        dx = x2 - x1
        if dx == 0:
            x_to_add = 0
            y_to_add = distance_to_add
        else:
            dydx = dy / dx
            x_to_add = sqrt((distance_to_add ** 2) / (1 + dydx ** 2))
            y_to_add = dydx * x_to_add

        new_x = x1 + x_to_add
        new_y = y1 + y_to_add

        carrot_position = {'X': new_x, 'Y': new_y}

        return carrot_position

    def get_pure_angular_speed(self, pose, goal_position, linear_speed):
        '''
        Returns the necessary angular speed to follow a generated circle
        segment between the robot and goal point.
        Warning: the speed might be higher than the maximum
        angular speed of the robot.
        Input:  pose - Robot's current pose
                goalPosition - The current goal position
                linearSpeed - The given linear speed
        Output: The angular speed
        '''

        L = g_lookahead
        y = sin(self.get_angle(pose, goal_position)) * L
        r = (L ** 2) / (2 * y)
        Y = 1 / r

        angular_velocity = linear_speed * Y

        return - angular_velocity

    def stop(self):
        self.post_speed(0, 0)
        time.sleep(1)

    def set_speed_and_avoid_obstacles(self, sound, angular_speed, linear_speed):
        """
        Adjusts the given angular and linear speeds in order to avoid obstacles
        found by using the laser scanner
        Input: pose - Robot's current pose
               lsr - A laser scan
               lsrAngles - The angles of the provided laser scan
               angularSpeed - The given angular speed
               linearSpeed - The given linear speed
        """
        global g_blocked_times
        blocked_distance = 1.25 * ROBOT_WIDTH
        blocked, is_right_angle = self.is_blocked(blocked_distance)
        if blocked:
            self.stop()
            self.unblock(is_right_angle, g_blocked_times * blocked_distance)
            g_blocked_times += 1
            if g_blocked_times > 2:

                if sound:
                    winsound.Beep(2000, 50)
                return True  # blocked too many times, so replan

        self.post_speed(angular_speed, linear_speed)

    def is_blocked(self, blocked_distance):
        half_width = 15  # in indices
        zero_angle = 135

        left_angle = zero_angle - half_width
        right_angle = zero_angle + half_width
        lsr = self.get_laser_scan()['Echoes']
        blocked = False

        shortest_distance = 10
        is_right_angle = False

        for i in range(left_angle, zero_angle):
            if lsr[i] < blocked_distance:
                blocked, is_right_angle, shortest_distance = self.handle_blocking_laser(blocked, False, lsr[i],
                                                                                        shortest_distance)

        for i in range(zero_angle, right_angle):
            if lsr[i] < blocked_distance:
                blocked, is_right_angle, shortest_distance = self.handle_blocking_laser(blocked, True, lsr[i],
                                                                                        shortest_distance)

        return blocked, is_right_angle

    def blocked_speeds(self, is_right_angle):
        if is_right_angle:
            angular_speed = -1
        else:
            angular_speed = 1
        linear_speed = -0.6
        return 0, linear_speed

    def unblock(self, is_right_angle, blocked_distance):
        global g_blocked_times
        blocked = True
        angular_speed, linear_speed = self.blocked_speeds(is_right_angle)
        self.post_speed(angular_speed, linear_speed)
        time.sleep(0.5)
        while blocked:
            blocked, new_is_right_angle = self.is_blocked(blocked_distance)
            if blocked and new_is_right_angle != is_right_angle:
                is_right_angle = new_is_right_angle
                angular_speed, linear_speed = self.blocked_speeds(is_right_angle)

            self.post_speed(angular_speed, linear_speed)
            time.sleep(0.5)

    def handle_blocking_laser(self, blocked, is_right_angle, lsr_dist, shortest_distance):
        if not blocked:
            blocked = True
        if lsr_dist < shortest_distance:
            shortest_distance = lsr_dist
        return blocked, is_right_angle, shortest_distance

    def go_fast(self, path, sound=False):
        """
        Follow the path using pure pursuit along with algorithms to avoid
        obstacles and to take shortcuts if possible.
        Input: path - a path of nodes to follow
        """
        global g_blocked_times
        g_blocked_times = 0
        pose = self.getPose()
        start_node_num = 0
        if len(path) > 1:
            cur_node_num = self.get_next_carrot_node(pose, start_node_num, path)
        else:
            cur_node_num = start_node_num

        next_pose = path[cur_node_num]['Pose']
        angle = self.get_angle(pose, next_pose['Position'])

        # If not facing the first position on the new path, rotates until it does.
        # The robot is  usually facing an obstacle when reaching a goal, and the other goal is in the opposite direction.
        # This avoids it colliding with that obstacle or triggering the reactive stop.
        if abs(angle) > pi / 8:
            self.post_speed(-6 * angle, 0)
        while abs(angle) > pi / 8:
            angle = self.get_angle(self.getPose(), next_pose['Position'])
            time.sleep(0.1)

        while not ((cur_node_num == len(path) - 1) and
                   (self.get_distance(pose['Pose']['Position'], path[-1]['Pose']['Position']) < ROBOT_WIDTH)):

            linear_speed = 1.0  # max is 1

            angle = self.get_angle(pose, next_pose['Position'])
            if abs(angle) > pi / 2:
                angular_speed = -1.5 * (2 / pi) * angle
                linear_speed = 0
            else:
                angular_speed = self.get_pure_angular_speed(pose, next_pose['Position'], linear_speed)
                # Slow down when doing sharp turns
                while abs(angular_speed) > 1.8:
                    linear_speed -= 0.01
                    angular_speed = self.get_pure_angular_speed(pose, next_pose['Position'], linear_speed)

            if self.set_speed_and_avoid_obstacles(sound, angular_speed, linear_speed):
                self.stop()
                return False  # blocked, so stop and replan

            time.sleep(0.04)

            pose = self.getPose()

            cur_node_num = self.get_next_carrot_node(pose, cur_node_num, path)
            logger.debug("CurNodeNum: " + str(cur_node_num) + " of " + str(len(path)))
            new_carrot_position = self.get_next_carrot_position(pose, cur_node_num, path)
            next_pose = {'Position': new_carrot_position}

            logger.debug("Current node: " + str(cur_node_num) + " of " + str(len(path)))
            logger.debug(
                "distance to node: " + str(
                    self.get_distance(pose['Pose']['Position'], path[cur_node_num]['Pose']['Position'])))

        self.post_speed(0, 0)
