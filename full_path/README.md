一、功能包内容：
1、全覆盖路径规划算法ccpp.cpp
2、将路径发布为Path话题pub_path.cpp
3、纯跟踪算法purepursuit.cpp

二、运行
1、在full_path/scr这一级编译功能包catkin_make
2、额外开启一个roscore和turtlesim_node
3、运行ccpp进行路径规划输出a.yaml文件
4、运行pub_path将a.yaml文件中的路径信息发布为path话题
5、运行purepursuit订阅path并发布ros小海龟的控制话题，控制小海龟进行全覆盖路径的跟踪
