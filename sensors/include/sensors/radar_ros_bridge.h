#ifndef RADAR_ROS_BRIDGE_H
#define RADAR_ROS_BRIDGE_H

#include <iostream>
#include <iomanip>
#include <ctime>
#include <fstream>
#include <sstream>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/chrono.hpp>
#include <pthread.h>
#include "ros/ros.h"
#include "sensors/RadarHeartbeat.h"
#include "sensors/RadarBasicTrackReport.h"
#include "sensors/RadarNormalTrackReport.h"
#include "sensors/RadarExtendedTrackReport.h"
#include "tracking/RadarCandidate.h"

typedef boost::asio::ip::udp boostUdp;
typedef sensors::RadarHeartbeat Heartbeat;
typedef sensors::RadarBasicTrackReport BasicTrackReport;
typedef sensors::RadarNormalTrackReport NormalTrackReport;
typedef sensors::RadarExtendedTrackReport ExtendedTrackReport;

// Default listening ports
#define HBLSTPORT 6500 // default heartbeat listening port
#define TRLSTPORT 6501 // default track report
#define HEARTBEATBUFFERSIZE 100
#define TRACKREPORTBUFFERSIZE 500
#define FLOATSIZE 4

class RadarRosBridge {
public:
	RadarRosBridge(const uint16_t &heartbeat_listening_port, const uint16_t &track_report_listening_port);
	~RadarRosBridge();
	void main();
private:
	void udp_handle_receive_heartbeat(const boost::system::error_code& error,
									  std::size_t num_bytes);
	void udp_handle_receive_trackreport(const boost::system::error_code& error,
									  	std::size_t num_bytes);
	void basicTrackReportMsgToROS();
	void normalTrackReportMsgToROS();
	void extendedTrackReportMsgToROS();

	void createBasicTrackReportCandidate();
	void createNormalTrackReportCandidate();
	void createExtendedTrackReportCandidate();

	ros::NodeHandle _node_handle;
	ros::NodeHandle _private_node_handle;
	ros::Publisher	_heartbeat_publisher;
	ros::Publisher  _basic_track_report_publisher;
	ros::Publisher  _normal_track_report_publisher;
	ros::Publisher  _extended_track_report_publisher;
	ros::Publisher  _radar_candidate_publisher;

	sensors::RadarHeartbeat _heartbeat_msg;
	sensors::RadarBasicTrackReport _basic_track_report_msg;
	sensors::RadarNormalTrackReport _normal_track_report_msg;
	sensors::RadarExtendedTrackReport _extended_track_report_msg;
	tracking::RadarCandidate _radar_candidate_msg;

	//boost::array<uint8_t, 1> _send_buf;
	boost::array<uint8_t, HEARTBEATBUFFERSIZE> _heartbeat_recv_buf;
	boost::array<uint8_t, TRACKREPORTBUFFERSIZE> _track_report_recv_buf;
	boost::asio::io_service	_io_service;
	boostUdp::endpoint _server_endpoint;
	boostUdp::socket _heartbeat_socket;
	boostUdp::socket _track_report_socket;

	bool _save_raw_binary_msg_data;
	std::string _raw_radar_msg_data_filename;
	std::ofstream _file_raw_heartbeat_msg_data;
	std::ofstream _file_raw_trackreport_msg_data;
};

#endif /* RADAR_ROS_BRIDGE_H */