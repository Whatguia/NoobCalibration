import argparse
import json
import os

import cv2
import numpy

def loadIntrinsic(filename):
    with open(filename) as jsonfile:
        root=json.load(jsonfile)
    assert(len(root["intrinsic"])==3 or len(root["intrinsic"])==9 or len(root["image_size"])==2)

    intrinsic=numpy.array(root["intrinsic"]).reshape((3,3))
    distortion=numpy.array(root["distortion"])
    image_size=numpy.array(root["image_size"])
    
    return intrinsic,distortion,image_size

def main(args):
    intrinsic,distortion,image_size=loadIntrinsic(args.intrinsic_json_path)
    map_x,map_y=cv2.initUndistortRectifyMap(intrinsic,distortion,numpy.eye(3),intrinsic,(image_size[0],image_size[1]),cv2.CV_16SC2)
    image_names=os.listdir(args.image_path)
    for image_name in image_names:
        image=cv2.imread(os.path.join(args.image_path,image_name))
        undistort_image=cv2.remap(image,map_x,map_y,cv2.INTER_LINEAR)
        cv2.imwrite(os.path.join(args.output_path,image_name),undistort_image)
    print("undistort done!")

if __name__=="__main__":
    parser=argparse.ArgumentParser(description='image undistort',formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('image_path',type=str,help='image directory path')
    parser.add_argument('output_path',type=str,help='output directory path')
    parser.add_argument('intrinsic_json_path',type=str,help='json file path')
    args=parser.parse_args()
    main(args)