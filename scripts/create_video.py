import sys
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages') # in order to import cv2 under python3
import cv2
sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages') # append back in order to import rospy
import numpy as np
import os
import os.path


def create_video(dir_path, fps):

    # Count number of files in directory
    img_num = 0
    for path in os.listdir(dir_path):
        if os.path.isfile(os.path.join(dir_path, path)):
            img_num += 1

    # Get shape of image
    img = cv2.imread('{0}frame_0000000001.png'.format(dir_path))
    frameSize = (img.shape[1], img.shape[0])

    # Writer
    out = cv2.VideoWriter('{0}output_video_{1}fps.mp4'.format(dir_path, fps), cv2.VideoWriter_fourcc(*'mp4V'), fps, frameSize)

    for i in range(0,img_num+1):
        # for j in range(0,operator_num+robot_num+1):
        num_zero = 10 - len(str(i))
        prefix = '0' * num_zero
        filename = '{0}frame_{1}{2}.png'.format(dir_path,prefix,i)
        img = cv2.imread(filename)
        out.write(img)

    out.release()


if __name__ == "__main__":
    # dir_path = '/home/genki/GIT/argos-sct/frames/'
    path = os.path.join(os.environ['HOME'], 'GIT/argos-sct/frames/')
    fps = 10
    create_video(path, fps)