"""
helper script to generate params file from template and camera config
"""

# lib
import json
import yaml
import argparse
import math


def main():
    args = parse_arguments()

    # load camera config and template
    with open(args['camera']) as file:
        camera_config = json.load(file)
    with open(args['template']) as file:
        template_params = yaml.safe_load(file)

    # calculate new params
    params = template_params.copy()
    params['Camera.cx'] = camera_config['stream_dimensions']['x'] / 2.0
    params['Camera.cy'] = camera_config['stream_dimensions']['y'] / 2.0
    params['Camera.fx'] = (
            camera_config['stream_dimensions']['x']
            / (2.0 * math.tan(math.radians(0.5 * camera_config['fov'])))
    )
    params['Camera.fy'] = params['Camera.fx']

    # write update params to output file
    with open(args['output'], 'w') as file:
        file.write('%YAML 1.0\n---\n')
        yaml.dump(params, file)


def parse_arguments():
    """
    helper function to parse CLI args

    :return:
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera', '-c', help='path to camera config.json', required=True)
    parser.add_argument('--template', '-t', help='path to template parameters', required=True)
    parser.add_argument('--output', '-o', help='path to generated params file', required=True)

    return vars(parser.parse_args())


if __name__ == '__main__':
    main()
