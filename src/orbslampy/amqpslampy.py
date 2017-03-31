# SYSTEM IMPORTS
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


def on_connected(connection):
    connection.channel(on_channel_open)


def on_channel_open(new_channel):
    global global_channel
    global_channel = new_channel
    global_channel.exchange_declare(exchange="point_cloud", type="fanout")
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


def perform_orbslam_with_message(message_body):
    # this needs to call orbslam with the message depending on the type of camera used
    timestamp = None
    global global_orbslammer_type, global_orbslammer
    if global_orbslammer_type == orbslampy.STEREO
        image_left = None
        image_right = None
        global_orbslammer.TrackStereo(image_left, image_right, timestamp)
    elif global_orbslammer_type == orbslampy.MONOCULAR:
        image = None
        global_orbslammer.TrackMonocular(image, timestamp)
    elif global_orbslammer_type == orbslampy.RGBD:
        image = None
        depth_map = None
        global_orbslammer.TrackRGBD(image, depth_map, timestamp)
    else:
        print("unknown orbslampy type: %s" % global_orbslammer_type)
        sys.exit(1)


def convert_point_cloud_to_message(point_cloud):
    converted_point_cloud = list()
    x = None
    y = None
    z = None
    # this will convert each element of the point_cloud to message format
    for i in range(point_cloud.size()):
        # convert it to a point
        x = point_cloud[i][0]
        y = point_cloud[i][1]
        z = point_cloud[i][2]
        converted_point_cloud.append(Point(x, y, z))
    return


def slam_callback(ch, method, properties, body):
    perform_orbslam_with_message(body)

    global global_orbslammer, global_channel
    point_cloud = global_orbslammer.GetMostRecentPointCloud()
    if (point_cloud.size() > 0):
        global_channel.basic_publish(exchange="PointCloud", routing_key="",
                                     body=convert_point_cloud_to_message(point_cloud))


if __name__ == "__main__":
    vocab_path = None
    settings_path = None
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

    global global_orbslammer_type, global_orbslammer, global_parameters, global_connection,\
           global_channel
    global_orbslammer_type = orbslam_type
    global_orbslammer = orbslampy.OrbSlammer(vocab_path, settings_path, orbslam_type, with_viewer)
    # Step #1: Connect to RabbitMQ using the default parameters
    global_parameters = pika.connection.URLParameters('amqp://guest:guest@localhost:5672/%2F')
    connection = pika.SelectConnection(parameters=global_parameters,
                                       on_open_callback=on_connected)

    try:
        global_connection.ioloop.start()
    except KeyboardInterrupt:
        global_connection.close()
        global_connection.ioloop.start()
