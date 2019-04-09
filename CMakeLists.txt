# CMake 最低版本号要求
CMAKE_MINIMUM_REQUIRED (VERSION 2.8)

# 查找目录下所有源文件
AUX_SOURCE_DIRECTORY(src DIR_SRCS)
AUX_SOURCE_DIRECTORY(src/ctrl DIR_CTRL_SRCS)
AUX_SOURCE_DIRECTORY(src/dev DIR_DEV_SRCS)

# 项目信息
PROJECT(QuadrupedRobotCtrlProg)

# 增加静态库子目录
ADD_SUBDIRECTORY(src/usr_lib)

# 指定生成目标
ADD_EXECUTABLE(QuadrupedRobotCtrlProg ${DIR_SRCS} ${DIR_CTRL_SRCS} ${DIR_DEV_SRCS})

#指定连接库
TARGET_LINK_LIBRARIES(QuadrupedRobotCtrlProg usr_lib m)

