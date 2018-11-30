import gym
import gym_cap
import numpy as np
import policy.roomba
import policy.random

# Linking Gym with Gazebo
import rospkg
import fnmatch
import time
import rospy, tf, os
from std_msgs.msg import *
from geometry_msgs.msg import *
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState, GetWorldProperties
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class CtfNode:

    def __init__(self, maxsteps=100, map_size=20, map_offset=10, agents=["TS1","TS2","TS3","TS4","TS5","TS6","TS7","TS8",]):

        env = gym.make("cap-v0") # initialize the environment
        self.done = False
        self.steps = 0
        self.score = 0
        self.maxsteps = maxsteps
        self.map_offset = map_offset
        self.observation = env.reset(map_size=map_size,
                        policy_blue=policy.random.PolicyGen(env.get_map, env.get_team_blue),
                        policy_red=policy.roomba.PolicyGen(env.get_map, env.get_team_red))
        self.env = env
        self.flags = {}
        self.agents = agents
        self.env.step()
        self.goals = self.get_agent_positions()
        self.env.render()
        self.flag_names = ["flag_blue", "flag_red"]
        self.gazebo_model_check = False
        # Get initial agent and obstacles locations
        obs = self.get_obstacle_positions()
        bots = self.get_agent_positions()
        self.team_flags = self.get_flag_positions()
        print(self.team_flags)

        # Start ROS node for ROS components
        rospy.init_node('cap_gazebo_node')
        self.r = rospy.Rate(20) # 50hz

        # Setup ROS subscribers and publishers
        goalTopics = [str(bot)+'/goal' for bot in agents]
        flagTopics = [[str(bot)+'/flag/ready',str(bot)] for bot in agents]
        self.pubs = [rospy.Publisher(topic, Point, queue_size=20) for topic in goalTopics]
        self.subs = [rospy.Subscriber(topic, Bool, self.flagCallback, str(bot)) for topic,bot in flagTopics]
        self.gazsub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.gazCallback)

        # Load all Gazebo models for initial spawning
        self.load_gazebo_models()
        # gzModels = self.get_gazebo_world_models()
        while not self.gazebo_model_check:
            print("Sleeping")
            self.r.sleep()
        # self.clear_obstacles(self.models[0])
        self.move_models(agents,bots, True)
        if len(self.models[1]) is 0:
            self.spawn_flags(self.team_flags)
        else:
            self.move_models(self.flag_names,self.team_flags)


        # self.spawn_obstacles(obs)
        self.clear_rviz_obstacles()
        self.spawn_rviz_obstacles(obs)
        self.spawn_rviz_flags(self.team_flags)


    def move_models(self, names, positions, isBot=False):
        rospy.wait_for_service("/gazebo/set_model_state")
        g_set_state = rospy.ServiceProxy("/gazebo/set_model_state",SetModelState)

        for i in xrange(0,len(positions)):
            state = ModelState()

            pose = Pose()
            pose.position.x = positions[i][0] - self.map_offset;
            pose.position.y = positions[i][1] - self.map_offset;
            if isBot: pose.position.z = 0.2
            else: pose.position.z = 0

            quats = tf.transformations.quaternion_from_euler(0,0,0)
            pose.orientation.x = quats[0]; pose.orientation.y = quats[1]; pose.orientation.z = quats[2]; pose.orientation.w = quats[3]

            state.model_name = names[i]
            state.pose = pose
            try:
                ret = g_set_state(state)
                print names[i],ret.status_message
                # self.r.sleep()
            except Exception, e:
                rospy.logerr('Error on calling service: %s',str(e))

    def flagCallback(self, data, args):
        self.flags[args] = data.data

    def gazCallback(self, data):
        models = data.name
        gzObs = [name for name in models if fnmatch.fnmatch(name, 'obstacle_*')]
        gzGoals = [name for name in models if fnmatch.fnmatch(name, 'flag_*')]
        self.models = [gzObs,gzGoals]
        self.gazebo_model_check = True

    def step(self):
        self.observation, self.score, self.done, _ = self.env.step()  # feedback from environment
        self.render()
        self.steps += 1

    def render(self):
        # render and sleep are not needed for score analysis
        self.env.render()

    def loop(self):
        while not self.done:
            while not self.update_gazebo_flags():
                self.update_gazebo_goals()
                self.spawn_rviz_flags(self.team_flags)
                # self.r.sleep()

            print("Stepping")
            self.goals = self.get_agent_positions()
            self.step()
            self.update_gazebo_goals()
            if self.steps == self.maxsteps:
                break

    def run(self):
        while True:
            self.loop()
            self.done = False
            print("Score: %.2f" % self.reward)

    def get_agent_positions(self):
        locations = []

        team1 = self.env.get_team_blue
        for agent in team1:
            locations.append(agent.get_loc())

        team2 = self.env.get_team_red
        for agent in team2:
            locations.append(agent.get_loc())

        return locations

    def get_flag_positions(self):
        locations = []

        for y in range(len(self.env._env)):
            for x in range(len(self.env._env[0])):
                if self.env._env[x][y] == 6:
                    locations.append([x,y])
                if self.env._env[x][y] == 7:
                    locations.append([x,y])

        return locations

    def get_obstacle_positions(self):
        locations = []

        for y in range(len(self.env._env)):
            for x in range(len(self.env._env[0])):
                if self.env._env[x][y] == 8:
                    locations.append([x,y])

        return locations

    def update_gazebo_goals(self):
        goals = self.goals
        goal = Point()
        for i in range(0,len(goals)):
            tmpGoal = goals[i]
            goal.x = tmpGoal[0] - self.map_offset
            goal.y = tmpGoal[1] - self.map_offset
            self.pubs[i].publish(goal)
            self.r.sleep()

    def update_gazebo_flags(self):
        flags = self.flags
        # print flags.values()
        isReady = all(flag==True for flag in flags.values() )
        if len(flags.values()) is 0:
            isReady = False
        print("isReady: ",flags)
        return isReady

    def load_gazebo_models(self):
        # Get Gazebo Models
        rospack = rospkg.RosPack()
        models_root = rospack.get_path('terrasentia_models')
        obstacle_path = os.path.join(models_root,'1m_cube_obstacle/model.sdf')
        red_flag_path = os.path.join(models_root,'flag_red/model.sdf')
        blue_flag_path = os.path.join(models_root,'flag_blue/model.sdf')
        # Store obstacle model SDF
        with open(obstacle_path, "r") as f:
            self.obstacle_xml = f.read()

        # Store red flag model SDF
        with open(red_flag_path, "r") as f:
            self.red_flag_xml = f.read()
        # Store blue flag model SDF
        with open(blue_flag_path, "r") as f:
            self.blue_flag_xml = f.read()

    def spawn_obstacles(self, positions):
        print("Waiting for gazebo services...")

        pose = Pose()
        quats = tf.transformations.quaternion_from_euler(0,0,0)

    	for num in xrange(0,len(positions)):
            model_name   =   "obstacle_{0}".format(num)
            print("Spawning model:%s", model_name)
            pose.position.x = positions[num][0] - self.map_offset;
            pose.position.y = positions[num][1] - self.map_offset;
            pose.position.z = 0
            pose.orientation.x = quats[0]; pose.orientation.y = quats[1]; pose.orientation.z = quats[2]; pose.orientation.w = quats[3]
            model_pose   =   pose

            rospy.wait_for_service("gazebo/spawn_sdf_model")
            s = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
            try:
                ret = s(model_name, self.obstacle_xml, "", model_pose, "world")
                print ret.status_message
            except Exception, e:
                rospy.logerr('Error on calling service: %s',str(e))
            # self.r.sleep()
        s.close()

    def spawn_rviz_obstacles(self, positions):
        topic = 'visualization_marker_array'
        publisher = rospy.Publisher(topic, MarkerArray)
        markerArray = MarkerArray()
        count = 0
    	for num in xrange(0,len(positions)):
            marker = Marker()
            marker.header.frame_id = "/world"
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.header.stamp = rospy.get_rostime()
            marker.id = num
            marker.ns = "/obstacles"
            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.pose.position.x = positions[num][0] - self.map_offset
            marker.pose.position.y = positions[num][1] - self.map_offset
            marker.pose.position.z = 0
            marker.pose.orientation.w = 1.0

            # if(count > MARKERS_MAX):
            #     markerArray.markers.pop(0)
            markerArray.markers.append(marker)
            id = 0
            for m in markerArray.markers:
                m.id = id
                id += 1
            publisher.publish(markerArray)
            count +=1
            self.r.sleep()

    def spawn_rviz_flags(self, positions):
        topic = 'visualization_flags'
        publisher = rospy.Publisher(topic, Marker, queue_size=2)
        publisher1 = rospy.Publisher('rviz/blue_flag', Marker, queue_size=2)
        publisher2 = rospy.Publisher('rviz/red_flag', Marker, queue_size=2)
        count = 0
    	for num in xrange(0,len(positions)):
            marker = Marker()
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD
            marker.header.frame_id = "/world"
            marker.mesh_resource = "package://terrasentia_models/flag/meshes/flag.dae"
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0

            marker.pose.position.x = positions[num][0] - self.map_offset
            marker.pose.position.y = positions[num][1] - self.map_offset
            marker.pose.position.z = 0
            marker.pose.orientation.w = 1.0

            if num is 0:
                marker.ns = "/blue_flag"
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 1.0
                publisher1.publish(marker)
                self.r.sleep()
            elif num is 1:
                marker.ns = "/red_flag"
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                publisher2.publish(marker)
                self.r.sleep()
            else:
                dump = None
                # publisher.publish(marker)
                count +=1
                self.r.sleep()




    def spawn_flags(self, positions):
        print("Waiting for gazebo services...")

        pose = Pose()
        quats = tf.transformations.quaternion_from_euler(0,0,0)

    	for num in xrange(0,len(positions)):
            if num is 0:
                model_name = "flag_blue"
                model_xml = self.blue_flag_xml
            elif num is 1:
                model_name = "flag_red"
                model_xml = self.red_flag_xml
            else:
                dump = None

            print("Spawning model:%s", model_name)
            pose.position.x = positions[num][0] - self.map_offset;
            pose.position.y = positions[num][1] - self.map_offset;
            pose.position.z = 0
            pose.orientation.x = quats[0]; pose.orientation.y = quats[1]; pose.orientation.z = quats[2]; pose.orientation.w = quats[3]
            model_pose   =   pose

            rospy.wait_for_service("gazebo/spawn_sdf_model")
            s = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
            try:
                ret = s(model_name, model_xml, "", model_pose, "world")
                print ret.status_message
            except Exception, e:
                rospy.logerr('Error on calling service: %s',str(e))
        s.close()

    def get_gazebo_world_models(self):
        print("Waiting for Gazebo service [gazebo/get_world_properties]...")
        rospy.wait_for_service("gazebo/get_world_properties")
        gzWorld = rospy.ServiceProxy("gazebo/get_world_properties", GetWorldProperties)
        world = gzWorld()

        gzObs = [name for name in world.model_names if fnmatch.fnmatch(name, 'obstacle_*')]
        gzGoals = [name for name in world.model_names if fnmatch.fnmatch(name, 'flag*')]

        gzWorld.close()
        return [gzObs, gzGoals]

    def clear_obstacles(self, models):
        nObs = len(models)
        print(nObs)
        # rospy.wait_for_service("gazebo/delete_model")
        d = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        for num in xrange(0,nObs):
            try:
                model_name = "obstacle_{0}".format(num)
                print("Deleting model:%s", model_name)
                ret = d(str(model_name))
                print ret.status_message
                # rosservice call gazebo/delete_model '{model_name: coffee_cup}'
                # d.close()
            except Exception, e:
                rospy.logerr('Error on calling service: %s',str(e))
            # self.r.sleep()

        d.close()

    def clear_rviz_obstacles(self):
        topic = 'visualization_marker_array'
        publisher = rospy.Publisher(topic, MarkerArray)
        markerArray = MarkerArray()
    	for num in xrange(0,2):
            marker = Marker()
            marker.header.frame_id = "/world"
            marker.type = marker.CUBE
            marker.action = marker.DELETEALL
            marker.header.stamp = rospy.get_rostime()
            # marker.id = num
            marker.ns = "/obstacles"
            # marker.scale.x = 1
            # marker.scale.y = 1
            # marker.scale.z = 0.2
            # marker.color.a = 1.0
            # marker.color.r = 1.0
            # marker.color.g = 1.0
            # marker.color.b = 1.0
            # marker.pose.position.x = positions[num][0] - self.map_offset
            # marker.pose.position.y = positions[num][1] - self.map_offset
            # marker.pose.position.z = 0
            # marker.pose.orientation.w = 1.0

            publisher.publish(markerArray)
            self.r.sleep()

if __name__ == '__main__':
    cf = CtfNode()
    cf.loop()
