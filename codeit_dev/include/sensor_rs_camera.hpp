#ifndef CODEIT_SENSOR_RS_CAMERA_H_
#define CODEIT_SENSOR_RS_CAMERA_H_

#include <codeit/sensor/sensor_base.hpp>

#include <librealsense2/rs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace codeit::sensor {

using stream_index_pair = std::pair<rs2_stream, int>;

const stream_index_pair COLOR{ RS2_STREAM_COLOR, 0 };
const stream_index_pair DEPTH{ RS2_STREAM_DEPTH, 0 };
const stream_index_pair POINT_CLOUD{ RS2_STREAM_ANY, 0 };

class SensorRsCamera;

/**
* @class RsCameraDataParser
* @brief This class is a helper class used to read data stream from SensorRsCamera and
* parse the data stream to rgb image and depth image
*/
class RsCameraDataParser {
public:
	RsCameraDataParser();
	~RsCameraDataParser();
	/**
	* @brief read and parse data from \a sensor which must be the pointer to SensorRsCamera object
	* @param[in] sensor SensorRsCamera object pointer
	* @return true if parse succeed
	* @note \a sensor must be SensorRsCamera object pointer
	*/
	auto parseFrom(Sensor* sensor) -> bool;
	/**
	* @brief copy realsense camera frame data to sensor module of codeit
	* @param[in] sensor SensorRsCamera object pointer
	* @param[in] elem stream index pair you want to copy to the sensor module
	* @param[in] frame reference to frame data from realsense camera
	* @param[in] reference to message data of sensor module
	* @return true if copy succeed
	* @note \a sensor must be SensorRsCamera object pointer
	*/
	auto copyTo(Sensor* sensor, stream_index_pair elem, rs2::frame& frame, core::Msg& data) -> bool;
	/**
	* @brief read rgb image
	* @return rgb image object
	*/
	auto rgb() -> cv::Mat;
	/**
	* @brief read depth image
	* @return depth image object
	*/
	auto depth() -> cv::Mat;
	/**
	* @brief read point cloud
	* @return shared_ptr for point cloud
	*/
	auto pointCloud() ->pcl::PointCloud<pcl::PointXYZ>::ConstPtr;
private:
	/** @brief Judge if we need initialize when calling \ref parseFrom or \ref copyTo */
	auto needInit() -> bool;
	/** @brief Initialize if needed */
	auto init(SensorRsCamera& rs_cam) -> void;

	struct Imp;
	std::unique_ptr<Imp> imp_;
};

class SensorRsCamera : public Sensor {
public:
	auto virtual loadXml(const core::XmlElement &xml_ele) -> void override;
	auto virtual saveXml(core::XmlElement &xml_ele) const -> void override;
	auto virtual start() -> void override;
	auto virtual stop() -> void override;

	auto valid() -> bool;
	auto getImageStreamProfile(stream_index_pair elem) -> rs2::video_stream_profile;

	SensorRsCamera(const std::string &name, const std::string &serial_no, const Size &length = 0);
	~SensorRsCamera();
	CODEIT_REGISTER_TYPE(SensorRsCamera);
	CODEIT_DEFINE_BIG_FOUR(SensorRsCamera);

protected:
	auto virtual init() -> void override;
	auto virtual release() -> void override;
	auto virtual updateData(core::Msg &data) -> void override;

	void processFrame(rs2::frame f, const stream_index_pair& stream, std::map<stream_index_pair, cv::Mat>& images);
	cv::Mat& fixDepthScale(const cv::Mat& from_image, cv::Mat& to_image);
private:
	struct Imp;
	std::unique_ptr<Imp> imp_;
};

} // namespace codeit::sensor

#endif