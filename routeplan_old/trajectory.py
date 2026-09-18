# import numpy as np
# import matplotlib.pyplot as plt
from collections import defaultdict
from multiprocessing import Lock

# import matplotlib
# matplotlib.use('Agg')
# import matplotlib.pyplot as plt
# plt.switch_backend('agg')

import rospy
from perception_msgs.msg import Trajectory


# plt.figure(1)
# ax1 = plt.subplot(2,2,1)
# ax2 = plt.subplot(2,2,2)
# ax3 = plt.subplot(2,2,3)
# ax4 = plt.subplot(2,2,4)
#
# colors = ["red", "black", "blue", "brown", "green"]
# #colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'w']
# objs = defaultdict(list)
# lock = Lock()
# max_count = 101
#
#
# def gen_image():
#     lock.acquire()
#
#     plt.sca(ax1)
#     plt.cla()
#     plt.sca(ax2)
#     plt.cla()
#     plt.sca(ax3)
#     plt.cla()
#     plt.sca(ax4)
#     plt.cla()
#
#     #lock.acquire()
#     #plt.cla()
#     print(objs.keys())
#     color = 'r'
#
#     print(objs)
#     t = range(len(objs['s']))
#     s = objs['s']
#     yaw = objs['yaw']
#     velocity = objs['velocity']
#     curvature = objs['curvature']
#
#
#     plt.sca(ax1)
#     #plt.cla()
#     plt.plot(t, s, color=color)
#     plt.xlim(0,max_count)
#     plt.ylim(-0.1,40)
#     plt.title('s-t')
#
#     plt.sca(ax2)
#     #plt.cla()
#     plt.plot(t, yaw, color=color)
#     plt.xlim(0,max_count)
#     plt.ylim(-360,360)
#     plt.title('yaw-t')
#
#     plt.sca(ax3)
#     #plt.cla()
#     plt.plot(t, velocity, color=color)
#     plt.xlim(0,max_count)
#     plt.ylim(-0.1,6)
#     plt.title('velocity-t')
#
#     plt.sca(ax4)
#     #plt.cla()
#     plt.plot(t, curvature, color=color)
#     plt.xlim(0,max_count)
#     plt.ylim(-55,55)
#     plt.title('curvature-t')
#
#     lock.release()


def callback(msg):
    print('callback')
    # lock.acquire()
    global objs
    objs = defaultdict(list)
    for obj in msg.trajectoryinfo.trajectorypoints:
        s = obj.s
        yaw = 57.4 * obj.heading
        velocity = obj.velocity
        curvature = min(1.0 / (abs(obj.curvature) + 1e-6), 50)

        # objs['s'].append(s)
        # objs['yaw'].append(yaw)
        # objs['velocity'].append(velocity)
        # objs['curvature'].append(curvature)

    # lock.release()


def main():
    rospy.init_node('display', anonymous=True)
    rospy.Subscriber('/cicv_amr_trajectory', Trajectory, callback)
    rospy.spin()

    # plt.ion()

    # while 1:
    #     print("show")
    # plt.clf()
    # gen_image()
    # plt.pause(0.01)
    # plt.ioff()
    # plt.show()


if __name__ == '__main__':
    main()
