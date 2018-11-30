#ifndef RADAR_ROS_BRIDGE_H
#define RADAR_ROS_BRIDGE_H

#include <iostream>
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

typedef boost::asio::ip::udp boostUdp;
typedef sensors::RadarHeartbeat Heartbeat;
typedef sensors::RadarBasicTrackReport BasicTrackReport;
typedef sensors::RadarNormalTrackReport NormalTrackReport;
typedef sensors::RadarExtendedTrackReport ExtendedTrackReport;

// Default listening ports
#define HBLSTPORT 6500 // Heartbeat
#define TRLSTPORT 6501 // Track Report
//#define BASICTRACKREPORTSIZE 76 // bytes
//#define NORMALTRACKREPORTSIZE 152 // bytes
//#define EXTENDEDTRACKREPORTSIZE 188 // bytes
#define FLOATSIZE 4

class RadarRosBrdige {
public:
	RadarRosBrdige(const uint16_t &heartbeatListeningPort, const uint16_t &trackReportListeningPort);
	~RadarRosBrdige();
	void main();
private:
	void udp_handle_receive_heartbeat(const boost::system::error_code& error,
									  std::size_t num_bytes);
	void udp_handle_receive_trackreport(const boost::system::error_code& error,
									  	std::size_t num_bytes);
	void basicTrackReportMsgToROS();
	void normalTrackReportMsgToROS();
	void extendedTrackReportMsgToROS();


	ros::NodeHandle _nodeHandle;
	ros::Publisher	_heartBeatPublisher;
	ros::Publisher  _basicTrackReportPublisher;
	ros::Publisher  _normalTrackReportPublisher;
	ros::Publisher  _extendedTrackReportPublisher;

	sensors::RadarHeartbeat _heartbeatMsg;
	sensors::RadarBasicTrackReport _basicTrackReportMsg;
	sensors::RadarNormalTrackReport _normalTrackReportMsg;
	sensors::RadarExtendedTrackReport _extendedTrackReportMsg;

	//boost::array<uint8_t, 1> _sendBuf;
	boost::array<uint8_t, 72> _heartbeatRecvBuf;
	boost::array<uint8_t, 188> _trackReportRecvBuf;
	boost::asio::io_service	_io_service;
	boostUdp::endpoint _server_endpoint;
	boostUdp::socket _heartbeatSocket;
	boostUdp::socket _trackReportSocket;
};



#endif /* RADAR_ROS_BRIDGE_H */