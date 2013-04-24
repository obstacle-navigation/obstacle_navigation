/* Auto-generated by genmsg_cpp for file /nishome/eysalee/ros/rosbuild_ws/class-code/obstacle_navigation/msg/Blob.msg */
#ifndef OBSTACLE_NAVIGATION_MESSAGE_BLOB_H
#define OBSTACLE_NAVIGATION_MESSAGE_BLOB_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace obstacle_navigation
{
template <class ContainerAllocator>
struct Blob_ {
  typedef Blob_<ContainerAllocator> Type;

  Blob_()
  : area(0)
  , x(0)
  , y(0)
  , left(0)
  , right(0)
  , top(0)
  , bottom(0)
  {
  }

  Blob_(const ContainerAllocator& _alloc)
  : area(0)
  , x(0)
  , y(0)
  , left(0)
  , right(0)
  , top(0)
  , bottom(0)
  {
  }

  typedef uint32_t _area_type;
  uint32_t area;

  typedef uint32_t _x_type;
  uint32_t x;

  typedef uint32_t _y_type;
  uint32_t y;

  typedef uint32_t _left_type;
  uint32_t left;

  typedef uint32_t _right_type;
  uint32_t right;

  typedef uint32_t _top_type;
  uint32_t top;

  typedef uint32_t _bottom_type;
  uint32_t bottom;


  typedef boost::shared_ptr< ::obstacle_navigation::Blob_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::obstacle_navigation::Blob_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Blob
typedef  ::obstacle_navigation::Blob_<std::allocator<void> > Blob;

typedef boost::shared_ptr< ::obstacle_navigation::Blob> BlobPtr;
typedef boost::shared_ptr< ::obstacle_navigation::Blob const> BlobConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::obstacle_navigation::Blob_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::obstacle_navigation::Blob_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace obstacle_navigation

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::obstacle_navigation::Blob_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::obstacle_navigation::Blob_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::obstacle_navigation::Blob_<ContainerAllocator> > {
  static const char* value() 
  {
    return "e280555c57a56f27d360aaacc2ddb12c";
  }

  static const char* value(const  ::obstacle_navigation::Blob_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xe280555c57a56f27ULL;
  static const uint64_t static_value2 = 0xd360aaacc2ddb12cULL;
};

template<class ContainerAllocator>
struct DataType< ::obstacle_navigation::Blob_<ContainerAllocator> > {
  static const char* value() 
  {
    return "obstacle_navigation/Blob";
  }

  static const char* value(const  ::obstacle_navigation::Blob_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::obstacle_navigation::Blob_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uint32 area\n\
uint32 x\n\
uint32 y\n\
uint32 left\n\
uint32 right\n\
uint32 top\n\
uint32 bottom\n\
\n\
";
  }

  static const char* value(const  ::obstacle_navigation::Blob_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::obstacle_navigation::Blob_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::obstacle_navigation::Blob_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.area);
    stream.next(m.x);
    stream.next(m.y);
    stream.next(m.left);
    stream.next(m.right);
    stream.next(m.top);
    stream.next(m.bottom);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Blob_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::obstacle_navigation::Blob_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::obstacle_navigation::Blob_<ContainerAllocator> & v) 
  {
    s << indent << "area: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.area);
    s << indent << "x: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.y);
    s << indent << "left: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.left);
    s << indent << "right: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.right);
    s << indent << "top: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.top);
    s << indent << "bottom: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.bottom);
  }
};


} // namespace message_operations
} // namespace ros

#endif // OBSTACLE_NAVIGATION_MESSAGE_BLOB_H

