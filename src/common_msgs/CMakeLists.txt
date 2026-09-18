cmake_minimum_required(VERSION 2.8.3)
project(common_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
        )


## Generate messages in the 'msg' folder
add_message_files(
        DIRECTORY msg
        FILES
             FaultInfo.msg
             FaultVec.msg
             Header.msg
             TimeStatistics.msg
             TimeStatus.msg
             DRPoseWithTime.msg
             EulerWithCovariance.msg
             LLH.msg
             NavStatus.msg
             Pose.msg
             PoseEuler.msg
             PoseQuaternion.msg
             Quaternion.msg
             TimeStatistics.msg
             TimeStatus.msg
             Twist.msg
             UnsureVar.msg
             Vector3.msg
             Vector3WithCovariance.msg
             CalibrationResult.msg
             LidarCalibrationResult.msg
             UdpMsg.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
)
