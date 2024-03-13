// Generated by gencpp from file moveit_msgs/PlanningSceneComponents.msg
// DO NOT EDIT!


#ifndef MOVEIT_MSGS_MESSAGE_PLANNINGSCENECOMPONENTS_H
#define MOVEIT_MSGS_MESSAGE_PLANNINGSCENECOMPONENTS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace moveit_msgs
{
template <class ContainerAllocator>
struct PlanningSceneComponents_
{
  typedef PlanningSceneComponents_<ContainerAllocator> Type;

  PlanningSceneComponents_()
    : components(0)  {
    }
  PlanningSceneComponents_(const ContainerAllocator& _alloc)
    : components(0)  {
  (void)_alloc;
    }



   typedef uint32_t _components_type;
  _components_type components;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(SCENE_SETTINGS)
  #undef SCENE_SETTINGS
#endif
#if defined(_WIN32) && defined(ROBOT_STATE)
  #undef ROBOT_STATE
#endif
#if defined(_WIN32) && defined(ROBOT_STATE_ATTACHED_OBJECTS)
  #undef ROBOT_STATE_ATTACHED_OBJECTS
#endif
#if defined(_WIN32) && defined(WORLD_OBJECT_NAMES)
  #undef WORLD_OBJECT_NAMES
#endif
#if defined(_WIN32) && defined(WORLD_OBJECT_GEOMETRY)
  #undef WORLD_OBJECT_GEOMETRY
#endif
#if defined(_WIN32) && defined(OCTOMAP)
  #undef OCTOMAP
#endif
#if defined(_WIN32) && defined(TRANSFORMS)
  #undef TRANSFORMS
#endif
#if defined(_WIN32) && defined(ALLOWED_COLLISION_MATRIX)
  #undef ALLOWED_COLLISION_MATRIX
#endif
#if defined(_WIN32) && defined(LINK_PADDING_AND_SCALING)
  #undef LINK_PADDING_AND_SCALING
#endif
#if defined(_WIN32) && defined(OBJECT_COLORS)
  #undef OBJECT_COLORS
#endif

  enum {
    SCENE_SETTINGS = 1u,
    ROBOT_STATE = 2u,
    ROBOT_STATE_ATTACHED_OBJECTS = 4u,
    WORLD_OBJECT_NAMES = 8u,
    WORLD_OBJECT_GEOMETRY = 16u,
    OCTOMAP = 32u,
    TRANSFORMS = 64u,
    ALLOWED_COLLISION_MATRIX = 128u,
    LINK_PADDING_AND_SCALING = 256u,
    OBJECT_COLORS = 512u,
  };


  typedef std::shared_ptr< ::moveit_msgs::PlanningSceneComponents_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::moveit_msgs::PlanningSceneComponents_<ContainerAllocator> const> ConstPtr;

}; // struct PlanningSceneComponents_

typedef ::moveit_msgs::PlanningSceneComponents_<std::allocator<void> > PlanningSceneComponents;

typedef std::shared_ptr< ::moveit_msgs::PlanningSceneComponents > PlanningSceneComponentsPtr;
typedef std::shared_ptr< ::moveit_msgs::PlanningSceneComponents const> PlanningSceneComponentsConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::moveit_msgs::PlanningSceneComponents_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::moveit_msgs::PlanningSceneComponents_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::moveit_msgs::PlanningSceneComponents_<ContainerAllocator1> & lhs, const ::moveit_msgs::PlanningSceneComponents_<ContainerAllocator2> & rhs)
{
  return lhs.components == rhs.components;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::moveit_msgs::PlanningSceneComponents_<ContainerAllocator1> & lhs, const ::moveit_msgs::PlanningSceneComponents_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace moveit_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::moveit_msgs::PlanningSceneComponents_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::moveit_msgs::PlanningSceneComponents_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::moveit_msgs::PlanningSceneComponents_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::moveit_msgs::PlanningSceneComponents_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::moveit_msgs::PlanningSceneComponents_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::moveit_msgs::PlanningSceneComponents_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::moveit_msgs::PlanningSceneComponents_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bc993e784476960b918b6e7ad5bb58ce";
  }

  static const char* value(const ::moveit_msgs::PlanningSceneComponents_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbc993e784476960bULL;
  static const uint64_t static_value2 = 0x918b6e7ad5bb58ceULL;
};

template<class ContainerAllocator>
struct DataType< ::moveit_msgs::PlanningSceneComponents_<ContainerAllocator> >
{
  static const char* value()
  {
    return "moveit_msgs/PlanningSceneComponents";
  }

  static const char* value(const ::moveit_msgs::PlanningSceneComponents_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::moveit_msgs::PlanningSceneComponents_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This message defines the components that make up the PlanningScene message.\n"
"# The values can be used as a bitfield to specify which parts of the PlanningScene message\n"
"# are of interest\n"
"\n"
"# Scene name, model name, model root\n"
"uint32 SCENE_SETTINGS=1\n"
"\n"
"# Joint values of the robot state\n"
"uint32 ROBOT_STATE=2\n"
"\n"
"# Attached objects (including geometry) for the robot state\n"
"uint32 ROBOT_STATE_ATTACHED_OBJECTS=4\n"
"\n"
"# The names of the world objects\n"
"uint32 WORLD_OBJECT_NAMES=8\n"
"\n"
"# The geometry of the world objects\n"
"uint32 WORLD_OBJECT_GEOMETRY=16\n"
"\n"
"# The maintained octomap \n"
"uint32 OCTOMAP=32\n"
"\n"
"# The maintained list of transforms\n"
"uint32 TRANSFORMS=64\n"
"\n"
"# The allowed collision matrix\n"
"uint32 ALLOWED_COLLISION_MATRIX=128\n"
"\n"
"# The default link padding and link scaling\n"
"uint32 LINK_PADDING_AND_SCALING=256\n"
"\n"
"# The stored object colors\n"
"uint32 OBJECT_COLORS=512\n"
"\n"
"# Bitfield combining options indicated above\n"
"uint32 components\n"
;
  }

  static const char* value(const ::moveit_msgs::PlanningSceneComponents_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::moveit_msgs::PlanningSceneComponents_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.components);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PlanningSceneComponents_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::moveit_msgs::PlanningSceneComponents_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::moveit_msgs::PlanningSceneComponents_<ContainerAllocator>& v)
  {
    s << indent << "components: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.components);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOVEIT_MSGS_MESSAGE_PLANNINGSCENECOMPONENTS_H
