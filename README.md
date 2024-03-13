# codeit_moveit 说明



### 一、 配置文件说明

##### 1. 配置文件目录结构

```c++
bin
 |---codeit_dev.exe 			//可执行程序
 |---moveit_config				//机器人配置总文件夹
	 |---d200_config			//d200机器人配置文件夹
	 |---fanuc_config			//fanuc机器人配置文件夹
	 |---panda_config			//panda机器人配置文件夹
	 |---pr2_config				//pr2机器人配置文件夹
	 |---ur5_config				//ur5机器人配置文件夹
		 |---config
			 |---cartesian_limits.yaml			//运动学约束：速度、加速度
			 |---chomp_planning.yaml			//chomp规划器参数
			 |---codeit_controllers.yaml		//moveit_simple_controller_manager执行器配置参数
			 |---codeit_joint_motion_map.yaml	//关节名对应codeit中伺服index
			 |---fake_controllers.yaml			//moveit_fake_controller_manager执行器配置参数
			 |---joint_limits.yaml				//关节约束配置
			 |---kinematics.yaml				//运动学参数配置：求解器、搜索分辨率、超时时间
			 |---ompl_planning.yaml				//ompl规划器参数
		 |---meshes								//描述urdf外观纹理,碰撞体
			 |---ur5
				 |---collision
				 |---visual
		 |---srdf
			 |---ur5.srdf
		 |---urdf
			 |---ur5.urdf
		 |---config.xml							//总体配置文件，重要！！！
	 |---config.yaml							//机器人配置文件夹选择，重要！！！
lib
 |---libfake_node_handle.so
 |---liblog_helper.so
 |---libmove_group.so
 |---libmove_group_capabilities_base.so
 |---libmoveit_plugin_loader.so
plugins
 |---libmove_group_capabilities_default.so					// 必须，
 |---libros_adapter_capability.so							// 可选，
 |---libkinematics_plugins.so								// 必须，
 |---libplanner_plugins.so									// 必须，
 |---libmoveit_default_planning_request_adapter_plugins.so	// 必须，
 |---libcontroller_plugins.so								// 必须，
```



##### 2. config.yaml 配置说明

```yaml
# config.yaml 可执行文件目录下的moveit_config目录中，指向具体的机器人配置文件夹
config : ur5_config
```



##### 3. config.xml 总体配置说明

```c++
// config.xml 为机器人总体配置。包括其他配置文件的路径、插件加载路径、规划配置等。
|---package_location					//替换urdf文件中package://*/所代表的路径，去ros后没有rospackage的概念，无法package定位路径。
|---urdf_path							//urdf文件路径
|---srdf_path							//srdf文件路径
|---capability_plugins_path				//capability插件路径
|---kinematics_plugins_path				//kinematics插件路径
|---planner_plugins_path				//planner插件路径
|---planner_adapter_plugins_path		//planner_adapter插件路径
|---controller_plugins_path				//controller插件路径
|---robot_description_planning
	|---joint_limits_config_path		//关节约束配置文件路径
	|---cartesian_limits_config_path	//运动学约束配置文件路径
|---robot_description_kinematics
	|---kinematics_config_path			//运动学参数配置文件路径
|---move_group
	|---debug
	|---allow_trajectory_execution
	|---max_safe_path_cost
	|---jiggle_fraction
	|---default_planning_pipeline
	|---capabilities
	|---disable_capabilities
	|---octomap							//octomap
		|---octomap_resolution
		|---octomap_frame
	|---more_transform					//额外的静态坐标转换
		|---enable
		|---transform_1
		|---transform_2
	|---planning_scene_monitor			//planning_scene_monitor
		|---publish_geometry_updates
		|---publish_planning_scene
		|---publish_planning_scene_hz
		|---publish_state_updates
		|---publish_transforms_updates
	|---controller						//controller
		|---moveit_controller_manager
		|---moveit_manage_controllers
		|---controller_config_path
		|---joint_motion_map_path
	|---plan_execution					//plan_execution
		|---max_replan_attempts
		|---record_trajectory_state_frequency
	|---trajectory_execution			//trajectory_execution
		|---allowed_execution_duration_scaling
		|---allowed_goal_duration_margin
		|---allowed_start_tolerance
		|---execution_duration_monitoring
		|---execution_velocity_scaling
		|---wait_for_trajectory_completion
	|---planning_pipelines				//planning_pipelines
    	|---ompl
    		|---planning_plugin
    		|---request_adapters
    		|---start_state_max_bounds_error
    		|---config_path
    	|---chomp
```



### 二、新增CodeIt指令

##### 1. MoveIt

___________________________________________________________________________

手册用法

笛卡尔空间场景下规划及运动

___________________________________________________________________________

参数定义
--robottarget_var
						数据类型：robottarget
						说明：期望的笛卡尔位姿。默认值：p0

--pipeline_id
						数据类型：String
						说明：规划器选择。默认值：“ompl”

--group_name
						数据类型：String
						说明：机器人规划组。默认值：“manipulator”

--allowed_planning_time
						数据类型：Double
						说明：规划超时时间。默认值：“5.0”

--exec
						数据类型：Int
						说明：0: 只规划不执行；1:规划并真实执行。2:规划并虚拟执行。默认值：1

--show
						数据类型：Int
						说明：show==0时不起作用；show=1显示当前关节TF，show=2显示当前连杆末端坐标。默认值：0
									group_name有效时显示组内关节和连杆，无效时显示所有关节和连杆。									
									show != 0 时不会规划运行；

___________________________________________________________________________

基本示例
例1

```
VAR --type=robottarget --name=p0 --value={0.1,0.2,0.3,0,0,0,1}
MoveIt --robottarget_var=p0
```



##### 2. ObsM

___________________________________________________________________________

手册用法

笛卡尔空间场景下规划及运动

___________________________________________________________________________

参数定义
--operation
						数据类型：Int
						说明：对障碍物的操作。默认值：0

```c++
          enum Operation
          {
            ADD = 0,
            REMOVE = 1,
            APPEND = 2,
            MOVE = 3,
          };
```

--id
						数据类型：String
						说明：障碍物名称。默认值：“Box_0”

--frame_id
						数据类型：String
						说明：机器人规划组。默认值：“world”

--pose
						数据类型：Pose
						说明：障碍物位置。默认值：p0

--type
						数据类型：Int
						说明：障碍物类型。默认值：1

```c++
          enum Type
          {
            BOX = 1u,       // 盒   dimensions: 3维  长  宽  高
            SPHERE = 2u,    // 球   dimensions: 1维  半径
            CYLINDER = 3u,  // 圆柱 dimensions: 2维  高  半径
            CONE = 4u,      // 圆锥 dimensions: 2维  高  半径
          };
```

--dimensions
						数据类型：Vector
						说明：与type对应，type描述的形状对应的数值。默认值："{0.2,0.2,0.2}"



___________________________________________________________________________

基本示例
例1

```
VAR --type=robottarget --name=p0 --value={0.1,0.2,0.3,0,0,0,1}
MoveIt --robottarget_var=p0
```



##### 3. CheckJS

___________________________________________________________________________

手册用法

检查关节状态是否碰撞

___________________________________________________________________________

参数定义
--jointtarget_var
						数据类型：jointtarget
						说明：关节空间关节状态。默认值：j0

___________________________________________________________________________

基本示例
例1

```
VAR --type=jointtarget --name=j0 --value={0.0,0.0,0.0,0.0,0.5,0.0,0,0,0,0}
MoveIt --jointtarget_var=j0
```



##### 4. Perception

___________________________________________________________________________

手册用法

感知障碍物信息（点云、深度图像），更新moveit场景。

___________________________________________________________________________

参数定义
--type
						数据类型：Int
						说明：type=0获取点云信息；type=1获取深度图信息（暂未支持）。默认值：0

___________________________________________________________________________

基本示例
例1

```
Perception --type=0
```





### 三、编译

##### 1. 编译说明

1. ompl库依赖于boost，planner_plugins中的ompl_planner依赖于ompl，要完整编译整个工程，须安装boost。

	2. 如未安装boost，程序编译不会报错，但ompl库和planner_plugins库不会生成。
 	3. linux下gcc版本使用gcc-8，codeit编译使用的是gcc-8，使用gcc-9编译filesystem库运行时会有问题。



##### 2. windows下安装boost

1. 打开网页 https://sourceforge.net/projects/boost/files/boost-binaries/

2. 选择安装版本，如：1.71.0

3. 选择自己适合的msvc版可执行程序下载。如：boost_1_71_0-msvc-14.2-32.exe

4. 下载完成安装即可。

   

##### 3. linux下安装boost

```shell
# 安装系统匹配版本boost
sudo apt-get install libboost-dev
# 或安装指定版本boost
sudo apt-get install libboost1.71-dev
```



##### 4. 编译步骤

```shell
# 编译
mkdir build && cd build
cmake ..
make -j4
# 安装
make install
```



### 四、使用说明

##### 1. 准备配置文件

   参照d200_config、fanuc_config、panda_config、pr2_config、ur5_config准备配置文件夹如：xxx_config。

   修改config.yaml文件指向新添加的配置文件夹。如：config : xxx_config

   

##### 2. 准备运行库及插件库

   ```c++
   // 运行库
   lib
    |---libfake_node_handle.so
    |---liblog_helper.so
    |---libmove_group.so
    |---libmove_group_capabilities_base.so
    |---libmoveit_plugin_loader.so
   // 默认插件库
   plugins
    |---libmove_group_capabilities_default.so					// 必须，
    |---libros_adapter_capability.so							// 可选，
    |---libkinematics_plugins.so								// 必须，
    |---libplanner_plugins.so									// 必须，
    |---libmoveit_default_planning_request_adapter_plugins.so	// 必须，
    |---libcontroller_plugins.so								// 必须，
       
   // 注意修改xxx_config下的config.xml文件，正确配置插件路径。
   // 可使用自定义的插件库，自定义插件需修改config.xml文件，添加插件路径，多个路径使用空格分开。
   ```



##### 3. 运行程序

   ```shell
   # 配置环境变量，运行可执行程序报错找不到lib中的库文件，需添加环境变量如下：
   export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:../lib
   # 运行可执行程序
   ./codeit_dev
   ```

   

##### 4. 使用新添加的指令。

   ```shell
   Mode
   Enable
   SetRate
   MoveIt
   ObsM
   CheckJS
   ```

