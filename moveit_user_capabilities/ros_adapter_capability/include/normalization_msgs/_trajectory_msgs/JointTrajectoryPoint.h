// Generated by gencpp from file _trajectory_msgs/JointTrajectoryPoint.msg
// DO NOT EDIT!


#ifndef _TRAJECTORY_MSGS_MESSAGE_JOINTTRAJECTORYPOINT_H
#define _TRAJECTORY_MSGS_MESSAGE_JOINTTRAJECTORYPOINT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace _trajectory_msgs
{
template <class ContainerAllocator>
struct JointTrajectoryPoint_
{
  typedef JointTrajectoryPoint_<ContainerAllocator> Type;

  JointTrajectoryPoint_()
    : positions()
    , velocities()
    , accelerations()
    , effort()
    , time_from_start()  {
    }
  JointTrajectoryPoint_(const ContainerAllocator& _alloc)
    : positions(_alloc)
    , velocities(_alloc)
    , accelerations(_alloc)
    , effort(_alloc)
    , time_from_start()  {
  (void)_alloc;
    }



   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _positions_type;
  _positions_type positions;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _velocities_type;
  _velocities_type velocities;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _accelerations_type;
  _accelerations_type accelerations;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _effort_type;
  _effort_type effort;

   typedef ros::Duration _time_from_start_type;
  _time_from_start_type time_from_start;





  typedef std::shared_ptr< ::_trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::_trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> const> ConstPtr;

}; // struct JointTrajectoryPoint_

typedef ::_trajectory_msgs::JointTrajectoryPoint_<std::allocator<void> > JointTrajectoryPoint;

typedef std::shared_ptr< ::_trajectory_msgs::JointTrajectoryPoint > JointTrajectoryPointPtr;
typedef std::shared_ptr< ::_trajectory_msgs::JointTrajectoryPoint const> JointTrajectoryPointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::_trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::_trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::_trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator1> & lhs, const ::_trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator2> & rhs)
{
  return lhs.positions == rhs.positions &&
    lhs.velocities == rhs.velocities &&
    lhs.accelerations == rhs.accelerations &&
    lhs.effort == rhs.effort &&
    lhs.time_from_start == rhs.time_from_start;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::_trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator1> & lhs, const ::_trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace _trajectory_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::_trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::_trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::_trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f3cd1e1c4d320c79d6985c904ae5dcd3";
  }

  static const char* value(const ::_trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf3cd1e1c4d320c79ULL;
  static const uint64_t static_value2 = 0xd6985c904ae5dcd3ULL;
};

template<class ContainerAllocator>
struct DataType< ::_trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "_trajectory_msgs/JointTrajectoryPoint";
  }

  static const char* value(const ::_trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::_trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Each trajectory point specifies either positions[, velocities[, accelerations]]\n"
"# or positions[, effort] for the trajectory to be executed.\n"
"# All specified values are in the same order as the joint names in JointTrajectory.msg\n"
"\n"
"float64[] positions\n"
"float64[] velocities\n"
"float64[] accelerations\n"
"float64[] effort\n"
"duration time_from_start\n"
;
  }

  static const char* value(const ::_trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::_trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.positions);
      stream.next(m.velocities);
      stream.next(m.accelerations);
      stream.next(m.effort);
      stream.next(m.time_from_start);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct JointTrajectoryPoint_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::_trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::_trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator>& v)
  {
    s << indent << "positions[]" << std::endl;
    for (size_t i = 0; i < v.positions.size(); ++i)
    {
      s << indent << "  positions[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.positions[i]);
    }
    s << indent << "velocities[]" << std::endl;
    for (size_t i = 0; i < v.velocities.size(); ++i)
    {
      s << indent << "  velocities[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.velocities[i]);
    }
    s << indent << "accelerations[]" << std::endl;
    for (size_t i = 0; i < v.accelerations.size(); ++i)
    {
      s << indent << "  accelerations[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.accelerations[i]);
    }
    s << indent << "effort[]" << std::endl;
    for (size_t i = 0; i < v.effort.size(); ++i)
    {
      s << indent << "  effort[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.effort[i]);
    }
    s << indent << "time_from_start: ";
    Printer<ros::Duration>::stream(s, indent + "  ", v.time_from_start);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TRAJECTORY_MSGS_MESSAGE_JOINTTRAJECTORYPOINT_H
