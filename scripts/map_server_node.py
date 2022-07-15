#! /usr/bin/python3
import ctypes

import behavior_tree_navigation_v3.msg as msgs
import nav_msgs.msg as nav_msgs
import nav_msgs.srv as nav_srvs

import actionlib
import os
import rospy
import sys
import tf_conversions
import _thread as thread

import yaml
import sdl2.sdlimage as sdl


class MapServer:
    """"""
    TRINARY = 0
    SCALE = 1
    RAW = 2

    def __init__(self, map_file=None):
        """

        :param map_file:
        """
        self.map_pub = None
        self.ok = True
        self.meta_data_pub = None
        self.map_file = map_file
        if self.map_file is None:
            self.get_map_file()
        rospy.loginfo(f"Got map file '{self.map_file}'")

        self.status = False
        self.publisher = 1

        self.service = None
        self.map_response = nav_srvs.GetMapResponse()
        self.load_map_file()
        self.init_publishers()
        self.serve_map()
        # self.subscribers()

        self.server = actionlib.SimpleActionServer("serve_map", msgs.LoadMapAction, self.execute, False)
        self.server.start()

    def __call__(self, *args, **kwargs):
        rospy.on_shutdown(self.handle_shutdown)
        rospy.spin()

    def get_map_file(self):
        """
        Try to get a user inputted path to map_file or use the default.

        :return:
        """
        map_file = "../data/map.yaml"
        prompt = f"""
        Available map file: 
            1. {map_file}
        Input number (e.g. 1) to select a file from the above listed.
        Input 0 to enter the path to a different map file.
        """
        while True:
            print(prompt)
            input_ = input()
            if input_ == '1':
                self.map_file = map_file
                break
            elif input_ == '0':
                input_ = input("Enter the absolute path to a map_file. ")
                if os.path.exists(input_):
                    self.map_file = input_
                    break
                else:
                    print(f"No file found at {input_}")
            else:
                print("Invalid Input.")

    def execute(self, action):
        """
        Subsequently we would use action to determine when to change map or something. It's not used for now.
        After waiting for 10 seconds to someone to request map, abort.

        :param action:
        :return:
        """
        result = msgs.LoadMapResult()
        rate = rospy.Rate(1)
        count = 0
        while not self.status and count < 10:
            rate.sleep()
            count += 1

        if self.status:
            result.status = 1
        else:
            result.status = 0
        self.server.set_succeeded(result)

    def load_map_file(self):
        """We didn't handle exceptions."""
        with open(self.map_file) as file:
            map_data = yaml.safe_load(file)

        map_image_file = map_data["image"]
        path = os.path.dirname(self.map_file)
        map_file_name = f"{path}/{map_image_file}"

        res = map_data["resolution"]
        origin = map_data["origin"]
        negate = map_data["negate"]
        occ_th = map_data["occupied_thresh"]
        free_th = map_data["free_thresh"]
        dic = {
            'trinary': self.TRINARY,
            'scale': self.SCALE,
            'raw': self.RAW
        }

        try:
            mode_ = map_data["mode"]
            mode = dic[mode_]
        except KeyError:
            mode = dic['trinary']

        self.load_map_from_file(map_file_name, res, origin, mode, negate, occ_th, free_th)

        # rospy.loginfo(f"\n{map_file_name}\n{res}\n{negate}\n{occ_th}\n{free_th}\n{origin}\n{mode}")
        rospy.logdebug("Waiting for valid time (make sure use_sime_time is false or a clock server (e.g., gazebo) is "
                       "running)")
        rate = rospy.Rate(10)
        while rospy.get_time() == 0:
            rate.sleep()
        self.map_response.map.info.map_load_time = rospy.Time.now()
        self.map_response.map.header.frame_id = rospy.get_param("frame_id", "map")  # get or set default
        self.map_response.map.header.stamp = rospy.Time.now()
        rospy.logdebug("Got Time")
        rospy.loginfo(f"Read a {self.map_response.map.info.width} x {self.map_response.map.info.height} map @ "
                      f"{self.map_response.map.info.resolution:.3f} m/cell")

    def load_map_from_file(self, file_name, res, origin, mode, negate, occ_th, free_th):
        """
        filename argument to IMG_Load has to be a bytestring.

        :param free_th:
        :param occ_th:
        :param negate:
        :param mode:
        :param file_name:
        :param res:
        :param origin:
        :return:
        """
        map_image = sdl.IMG_Load(bytes(file_name, encoding='utf'))

        img = map_image.contents
        img_w = img.w
        self.map_response.map.info.width = img_w
        img_h = img.h
        self.map_response.map.info.height = img_h
        self.map_response.map.info.resolution = res
        self.map_response.map.info.origin.position.x = origin[0]
        self.map_response.map.info.origin.position.y = origin[1]
        self.map_response.map.info.origin.position.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, origin[2])
        self.map_response.map.info.origin.orientation.x = q[0]
        self.map_response.map.info.origin.orientation.y = q[1]
        self.map_response.map.info.origin.orientation.z = q[2]
        self.map_response.map.info.origin.orientation.w = q[3]

        # allocate space for map data
        self.map_response.map.data = [0] * (img_w * img_h)

        # Get values that we'll need to iterate through the pixels
        row_stride = img.pitch
        img_format = img.format.contents
        n_channels = img_format.BytesPerPixel

        if mode == self.TRINARY or not img_format.Amask:
            avg_channels = n_channels
        else:
            avg_channels = n_channels - 1
        address = img.pixels
        # pixels = self.pointer(address)
        pixels = address

        for j in range(self.map_response.map.info.height):
            for i in range(self.map_response.map.info.width):
                # Compute mean of RGB for this pixel
                p = pixels + (j * row_stride) + (i * n_channels)
                color_sum = 0
                for k in range(avg_channels):
                    color_sum += self.pointer(p + k)
                color_avg = color_sum / avg_channels

                if n_channels == 1:
                    alpha = 1
                else:
                    alpha = self.pointer(p + n_channels - 1)
                if negate:
                    color_avg = 255 - color_avg
                if mode == self.RAW:
                    value = color_avg
                    self.map_response.map.data[self.map_index(img_w, i, img_h - j - 1)] = value
                    continue

                # If negate is true, we consider blacker pixels free, and whiter pixels occupied.
                # Otherwise, it's vice versa.
                occ = (255 - color_avg) / 255

                if occ > occ_th:
                    value = 100  # +100
                elif occ < free_th:
                    value = 0
                elif mode == self.TRINARY or alpha < 1:
                    value = -1
                else:
                    ratio = (occ - free_th) / (occ_th - free_th)
                    # fixme or (1 + 98) * ratio
                    value = 1 + (98 * ratio)

                self.map_response.map.data[self.map_index(img_w, i, img_h - j - 1)] = value

        data = self.map_response.map.data
        rospy.logdebug(f"Len of data: {len(data)}\n"
                       f"-1s: {len(list(filter(lambda i: i == -1, data)))}\n"
                       f"0s: {len(list(filter(lambda i: i == 0, data)))}\n"
                       f"100s: {len(list(filter(lambda i: i == 100, data)))}")

    @staticmethod
    def map_index(sx, i, j):
        # fixme or sx * (j + i)?
        return (sx * j) + i

    @staticmethod
    def pointer(address):
        e = ctypes.string_at(address, 1)
        return int(f"0x{e.hex()}", 0)

    def serve_map(self):
        """
        Serve Map and run a separate thread that publishes map and meta_data messages.

        :return:
        """
        self.service = rospy.Service('static_map', nav_srvs.GetMap, self.map_service_callback)
        tid = thread.start_new_thread(MapServer.publish_data, (self,))

    def map_service_callback(self, request):
        """
        request is empty; we ignore it

        :param request:
        :return:
        """
        rospy.loginfo("Sending map")
        self.status = True
        return self.map_response

    def init_publishers(self):
        """"""
        self.meta_data_pub = rospy.Publisher("map_metadata", nav_msgs.MapMetaData, queue_size=1)
        self.map_pub = rospy.Publisher("map", nav_msgs.OccupancyGrid, queue_size=1)

    def subscribers(self):
        """To be used to compare my map data. Do not run when we are serving map data."""
        rospy.Subscriber("map", nav_msgs.OccupancyGrid, self.process_map)

    @staticmethod
    def process_map(map_data):
        """Only used for testing purposes. Can be disregarded."""
        data = map_data.data
        print("Subscriber")
        print(f"Len of data: {len(data)}")
        print(f"-1s: {len(list(filter(lambda i: i == -1, data)))}")
        print(f"0s: {len(list(filter(lambda i: i == 0, data)))}")
        print(f"100s: {len(list(filter(lambda i: i == 100, data)))}")

    def publish_data(self, args=None):
        """
        Publish map and meta_data

        :return:
        """
        rate = rospy.Rate(1)
        while self.ok:
            meta_data_message_ = self.map_response.map.info
            self.meta_data_pub.publish(meta_data_message_)
            self.map_pub.publish(self.map_response.map)
            rate.sleep()
        rospy.logdebug("Stopped publishing map and meta_data")

    def handle_shutdown(self):
        self.ok = False


if __name__ == '__main__':
    rospy.init_node("map_action_server")
    rospy.loginfo("Initialised MapActionServer Node.")

    argv = rospy.myargv(argv=sys.argv)
    if len(argv) > 1:
        map_ = argv[1]
    else:
        map_ = None
    server = MapServer(map_file=map_)
    server()
