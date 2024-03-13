// Generated by gencpp from file _moveit_msgs/ChangeDriftDimensionsRequest.msg
// DO NOT EDIT!


#ifndef _MOVEIT_MSGS_MESSAGE_CHANGEDRIFTDIMENSIONSREQUEST_H
#define _MOVEIT_MSGS_MESSAGE_CHANGEDRIFTDIMENSIONSREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <_geometry_msgs/Transform.h>

namespace _moveit_msgs
{
template <class ContainerAllocator>
struct ChangeDriftDimensionsRequest_
{
  typedef ChangeDriftDimensionsRequest_<ContainerAllocator> Type;

  ChangeDriftDimensionsRequest_()
    : drift_x_translation(false)
    , drift_y_translation(false)
    , drift_z_translation(false)
    , drift_x_rotation(false)
    , drift_y_rotation(false)
    , drift_z_rotation(false)
    , transform_jog_frame_to_drift_frame()  {
    }
  ChangeDriftDimensionsRequest_(const ContainerAllocator& _alloc)
    : drift_x_translation(false)
    , drift_y_translation(false)
    , drift_z_translation(false)
    , drift_x_rotation(false)
    , drift_y_rotation(false)
    , drift_z_rotation(false)
    , transform_jog_frame_to_drift_frame(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _drift_x_translation_type;
  _drift_x_translation_type drift_x_translation;

   typedef uint8_t _drift_y_translation_type;
  _drift_y_translation_type drift_y_translation;

   typedef uint8_t _drift_z_translation_type;
  _drift_z_translation_type drift_z_translation;

   typedef uint8_t _drift_x_rotation_type;
  _drift_x_rotation_type drift_x_rotation;

   typedef uint8_t _drift_y_rotation_type;
  _drift_y_rotation_type drift_y_rotation;

   typedef uint8_t _drift_z_rotation_type;
  _drift_z_rotation_type drift_z_rotation;

   typedef  ::_geometry_msgs::Transform_<ContainerAllocator>  _transform_jog_frame_to_drift_frame_type;
  _transform_jog_frame_to_drift_frame_type transform_jog_frame_to_drift_frame;





  typedef std::shared_ptr< ::_moveit_msgs::ChangeDriftDimensionsRequest_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::_moveit_msgs::ChangeDriftDimensionsRequest_<ContainerAllocator> const> ConstPtr;

}; // struct ChangeDriftDimensionsRequest_

typedef ::_moveit_msgs::ChangeDriftDimensionsRequest_<std::allocator<void> > ChangeDriftDimensionsRequest;

typedef std::shared_ptr< ::_moveit_msgs::ChangeDriftDimensionsRequest > ChangeDriftDimensionsRequestPtr;
typedef std::shared_ptr< ::_moveit_msgs::ChangeDriftDimensionsRequest const> ChangeDriftDimensionsRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::_moveit_msgs::ChangeDriftDimensionsRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::_moveit_msgs::ChangeDriftDimensionsRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::_moveit_msgs::ChangeDriftDimensionsRequest_<ContainerAllocator1> & lhs, const ::_moveit_msgs::ChangeDriftDimensionsRequest_<ContainerAllocator2> & rhs)
{
  return lhs.drift_x_translation == rhs.drift_x_translation &&
    lhs.drift_y_translation == rhs.drift_y_translation &&
    lhs.drift_z_translation == rhs.drift_z_translation &&
    lhs.drift_x_rotation == rhs.drift_x_rotation &&
    lhs.drift_y_rotation == rhs.drift_y_rotation &&
    lhs.drift_z_rotation == rhs.drift_z_rotation &&
    lhs.transform_jog_frame_to_drift_frame == rhs.transform_jog_frame_to_drift_frame;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::_moveit_msgs::ChangeDriftDimensionsRequest_<ContainerAllocator1> & lhs, const ::_moveit_msgs::ChangeDriftDimensionsRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace _moveit_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::_moveit_msgs::ChangeDriftDimensionsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::_moveit_msgs::ChangeDriftDimensionsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_moveit_msgs::ChangeDriftDimensionsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::_moveit_msgs::ChangeDriftDimensionsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_moveit_msgs::ChangeDriftDimensionsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::_moveit_msgs::ChangeDriftDimensionsRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::_moveit_msgs::ChangeDriftDimensionsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4a5ce44f94cdee672e699df89b1ebaf1";
  }

  static const char* value(const ::_moveit_msgs::ChangeDriftDimensionsRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4a5ce44f94cdee67ULL;
  static const uint64_t static_value2 = 0x2e699df89b1ebaf1ULL;
};

template<class ContainerAllocator>
struct DataType< ::_moveit_msgs::ChangeDriftDimensionsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "_moveit_msgs/ChangeDriftDimensionsRequest";
  }

  static const char* value(const ::_moveit_msgs::ChangeDriftDimensionsRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::_moveit_msgs::ChangeDriftDimensionsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# For use with moveit_jog_arm Cartesian planner\n"
"#\n"
"# Allow the robot to drift along these dimensions in a smooth but unregulated way.\n"
"# Give 'true' to enable drift in the direction, 'false' to disable.\n"
"# For example, may allow wrist rotation by drift_x_rotation == true.\n"
"bool drift_x_translation\n"
"bool drift_y_translation\n"
"bool drift_z_translation\n"
"bool drift_x_rotation\n"
"bool drift_y_rotation\n"
"bool drift_z_rotation\n"
"\n"
"# Not implemented as of Jan 2020 (for now assumed to be the identity matrix). In the future it will allow us to transform\n"
"# from the jog control frame to a unique drift frame, so the robot can drift along off-principal axes\n"
"_geometry_msgs/Transform transform_jog_frame_to_drift_frame\n"
"\n"
"================================================================================\n"
"MSG: _geometry_msgs/Transform\n"
"# This represents the transform between two coordinate frames in free space.\n"
"\n"
"Vector3 translation\n"
"Quaternion rotation\n"
"\n"
"================================================================================\n"
"MSG: _geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# _geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"================================================================================\n"
"MSG: _geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::_moveit_msgs::ChangeDriftDimensionsRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::_moveit_msgs::ChangeDriftDimensionsRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.drift_x_translation);
      stream.next(m.drift_y_translation);
      stream.next(m.drift_z_translation);
      stream.next(m.drift_x_rotation);
      stream.next(m.drift_y_rotation);
      stream.next(m.drift_z_rotation);
      stream.next(m.transform_jog_frame_to_drift_frame);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ChangeDriftDimensionsRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::_moveit_msgs::ChangeDriftDimensionsRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::_moveit_msgs::ChangeDriftDimensionsRequest_<ContainerAllocator>& v)
  {
    s << indent << "drift_x_translation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.drift_x_translation);
    s << indent << "drift_y_translation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.drift_y_translation);
    s << indent << "drift_z_translation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.drift_z_translation);
    s << indent << "drift_x_rotation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.drift_x_rotation);
    s << indent << "drift_y_rotation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.drift_y_rotation);
    s << indent << "drift_z_rotation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.drift_z_rotation);
    s << indent << "transform_jog_frame_to_drift_frame: ";
    s << std::endl;
    Printer< ::_geometry_msgs::Transform_<ContainerAllocator> >::stream(s, indent + "  ", v.transform_jog_frame_to_drift_frame);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOVEIT_MSGS_MESSAGE_CHANGEDRIFTDIMENSIONSREQUEST_H
