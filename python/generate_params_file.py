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
    with open(args['template']) as file:
        template_params = yaml.safe_load(file)

    # Read the KITTI camera calibration file to extract the projection matrix
    with open(args['camera']) as file:
        lines = file.readlines()
        for line in lines:
            if line.startswith("P_rect"):
                proj_mat = line[line.find(':')+1:]. \
                    strip('\n').rstrip(' ').lstrip(' ').split(sep=' ')
                proj_mat = list(map(float, proj_mat))

    # calculate new params
    params = template_params.copy()
    params['Camera.cx'] = proj_mat[2]
    params['Camera.cy'] = proj_mat[6]
    params['Camera.fx'] = proj_mat[0]
    params['Camera.fy'] = proj_mat[5]

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
