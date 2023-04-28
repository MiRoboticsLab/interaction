# cyberdog visual programming python demo
## 概述
> 当前项目为 cyberdog 可视化编程项目针对高级玩家的示例项目。
> 在此默认高级玩家具备脱离APP基于 cyberdog 能力集进行一系列编程，所以将展示如何调用 cyberdog 能力集的 python API，用户可以根据自己的使用场景丰富具体的使用逻辑。
> 1. build_ros_demo: 基于 cyberdog 能力集 内部初始化 ROS 上下文进行调用 python API 进行简单展示；
> 2. reuse_ros_demo: 基于 cyberdog 能力集和外部 ROS 上下文进行调用 python API 进行简单展示，该 demo 主要想说明：“Python 调用 cyberdog 能力集 python API 时候对 C++ 动态库内的 ROS 上下文无感（不共享）”的现象，所以不论什么场景下调用 cyberdog 能力集的 python API 都需要初始化 ROS 上下文，也就是初始化 cyberdog 的时候需要将第三个参数设置为 True。

> 如：cyberdog = Cyberdog("abilityset_reuse_ros_demo", "demo", <font color=red size=4>True</font>, "")
