# SYSTEM IMPORTS
import cv2
import numpy
import os
import pika
import sys

current_dir = os.path.realpath(os.path.dirname(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)
orbslampy_dir = os.path.join(current_dir, "lib")
if orbslampy_dir not in sys.path:
    sys.path.append(orbslampy_dir)


# PYTHON PROJECT IMPORTS
import orbslampy
from messages_pb2 import *


"""
    Strategy: we want to subscribe to a topic called 'filtered_camera_data'.
    Every time something is published, we want our callback to run
    (which will perform slam with the message, and then if there is an update to the
     point cloud, publish the new points on a topic named 'point_cloud')
"""


global_orbslammer = None
global_orbslammer_type = None
global_parameters = None
global_connection = None
global_channel = None
global_subscriber_queue_name = "filtered_camera_data"
global_point_cloud_name = "point_cloud"
global_pose_name = "pose"
global_pose_f_name = "poses.txt"
global_pose_fp = None
global_point_cloud_f_name = "point_clouds.txt"
global_point_cloud_fp = None


def on_connected(connection):
    connection.channel(on_channel_open)


def on_channel_open(new_channel):
    global global_channel, global_subscriber_queue_name, global_point_cloud_name,\
           global_pose_name
    global_channel = new_channel
    global_channel.exchange_declare(exchange=global_point_cloud_name, type="fanout")
    global_channel.exchange_declare(exchange=global_pose_name, type="fanout")
    global_channel.queue_declare(queue=global_subscriber_queue_name, durable=True,
                                 exclusive=False, auto_delete=False, callback=on_queue_declared)


def on_queue_declared(frame):
    global global_channel, global_subscriber_queue_name
    global_channel.basic_consume(slam_callback, queue=global_subscriber_queue_name, no_ack=True)


def parse_cmd_line(commandLine):
    parsed_cmd_values = {}
    parsed_cmd_methods = []
    for cmd in commandLine:
        if '-' in cmd:
            cmd_var = cmd.replace('-', '', 1).split("=")
            if len(cmd_var) == 2:
                # parse list if necessary using:
                # listArgs = commandVariable[1].split(",")
                # if len(listArgs) > 1:
                    # we have a list
                # else
                parsed_cmd_values[cmd_var[0]] = cmd_var[1]
            elif len(cmd_var) == 1:
                parsed_cmd_values[cmd_var[0]] = True
            else:
                print("Unknown command: [%s]" % cmd)
                sys.exit(1)
        else:
            parsed_cmd_methods.append(cmd)
    return (parsed_cmd_methods, parsed_cmd_values)


def convert_protobuf_matrix_to_cv(protobuf_matrix):
    return orbslampy.Mat.from_array(numpy.array(protobuf_matrix.data, dtype="float")
                                         .reshape(protobuf_matrix.rows, protobuf_matrix.cols))


def perform_orbslam_with_message(message_body):
    # this needs to call orbslam with the message depending on the type of camera used
    parsed_msg = None
    global global_orbslammer_type, global_orbslammer
    if global_orbslammer_type == orbslampy.STEREO:
        parsed_msg = StereoCameraFeed()
        parsed_msg.ParseFromString(message_body)
        image_left = convert_protobuf_matrix_to_cv(parsed_msg.image_left)
        image_right = convert_protobuf_matrix_to_cv(parsed_msg.image_right)
        return global_orbslammer.TrackStereo(image_left, image_right, parsed_msg.timestamp)
    elif global_orbslammer_type == orbslampy.MONOCULAR:
        parsed_msg = MonocularCameraFeed()
        parsed_msg.ParseFromString(message_body)
        image = convert_protobuf_matrix_to_cv(parsed_msg.image)
        return global_orbslammer.TrackMonocular(image, parsed_msg.timestamp)
    elif global_orbslammer_type == orbslampy.RGBD:
        parsed_msg = RGBDCameraFeed()
        parsed_msg.ParseFromString(message_body)
        image = convert_protobuf_matrix_to_cv(parsed_msg.image)
        depth_map = convert_protobuf_matrix_to_cv(parsed_msg.depth_map)
        return global_orbslammer.TrackRGBD(image, depth_map, parsed_msg.timestamp)
    else:
        print("unknown orbslampy type: %s" % global_orbslammer_type)
        sys.exit(1)


def convert_point_cloud_to_message(point_cloud):
    point_cloud_msg = PointCloud()
    point = None
    # this will convert each element of the point_cloud to message format
    for i in range(point_cloud.size()):
        # convert it to a point
        point = PointCloud.Point()
        point.x = point_cloud[i][0]
        point.y = point_cloud[i][1]
        point.z = point_cloud[i][2]
        point_cloud_msg.points.append(point)
    return point_cloud_msg.SerializeToString()


def convert_pose_to_message(pose):
    # convert a matrix to a pose message and return bytes
    pose_msg = Pose()
    pose_msg.x = 0.0
    pose_msg.y = 0.0
    pose_msg.z = 0.0
    pose_msg.yaw = 0.0
    pose_msg.pitch = 0.0
    pose_msg.roll = 0.0
    return pose_msg.SerializeToString()


def slam_callback(ch, method, properties, body):
    global global_orbslammer, global_channel, global_point_cloud_name,\
           global_pose_name
    current_pose = perform_orbslam_with_message(body)
    point_cloud = global_orbslammer.GetMostRecentPointCloud()

    global global_pose_fp, global_point_cloud_fp
    global_pose_fp.write(str(current_pose) + "\n")
    # global_channel.basic_publish(exchange=global_pose_name, routing_key="",
    #                              body=convert_pose_to_message(current_pose))
    if (point_cloud.size() > 0):
        global_point_cloud_fp.write("<\n")
        for i in range(point_cloud.size()):
            global_point_cloud_fp.write("\t" + str(point_cloud[i]) + "\n")
        global_point_cloud_fp.write(">\n")
        # global_channel.basic_publish(exchange=global_point_cloud_name, routing_key="",
        #                              body=convert_point_cloud_to_message(point_cloud))


if __name__ == "__main__":
    orbslam_path = os.path.join(".", "..", "src", "ORB_SLAM2")
    vocab_path = os.path.abspath(os.path.join(orbslam_path, "Vocabulary", "ORBVoc.txt"))
    orbslam_type = None
    with_viewer = False

    methods, vals = parse_cmd_line(sys.argv[1:])
    if vals["camera_type"] == "stereo":
        orbslam_type = orbslampy.STEREO
    elif vals["camera_type"] == "monocular":
        orbslam_type = orbslampy.MONOCULAR
    elif vals["camera_type"] == "rgbd":
        orbslam_type = orbslampy.RGBD
    else:
        print("Unknown orbslam_type: %s" % orbslam_type)
        sys.exit(1)
    if "with_viewer" in vals:
        with_viewer = bool(vals["with_viewer"])

    #settings_path = os.path.abspath(os.path.join(orbslam_path, "..", "orbslampy", "config", "camera_params_")) + vals["camera_type"] + ".yaml"
    settings_path = os.path.join(orbslam_path, "Examples", "Stereo", "EuRoC.yaml")

    print("vocab_path: %s" % vocab_path)
    print("settings_path: %s" % settings_path)

    global global_orbslammer_type, global_orbslammer, global_parameters, global_connection,\
           global_channel
    global_orbslammer_type = orbslam_type
    global_orbslammer = orbslampy.OrbSlammer(vocab_path, settings_path, orbslam_type, with_viewer)
    # Step #1: Connect to RabbitMQ using the default parameters
    global_parameters = pika.connection.URLParameters('amqp://guest:guest@localhost:5672/%2F')
    connection = pika.SelectConnection(parameters=global_parameters,
                                       on_open_callback=on_connected)

    global global_pose_f_name, global_pose_fp, global_point_cloud_f_name, global_point_cloud_fp
    global_pose_fp = open(global_pose_f_name, "w")
    global_point_cloud_fp = open(global_point_cloud_f_name, "w")

    try:
        global_connection.ioloop.start()
    except KeyboardInterrupt:
        global_connection.close()
        global_connection.ioloop.start()
