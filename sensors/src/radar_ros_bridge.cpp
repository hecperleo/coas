#include "sensors/radar_ros_bridge.h"

/**
* RadarRosBridge constructor
*/
RadarRosBridge::RadarRosBridge(const uint16_t &heartbeat_listening_port, const uint16_t &track_report_listening_port):
_node_handle(),
_private_node_handle("~"),
_heartbeat_socket(_io_service, boostUdp::endpoint(boostUdp::v4(), heartbeat_listening_port)),
_track_report_socket(_io_service, boostUdp::endpoint(boostUdp::v4(), track_report_listening_port))
{
	_working_frequency = 5;
	_private_node_handle.getParam("working_frequency", _working_frequency);
	_save_raw_binary_msg_data = false;
	_private_node_handle.getParam("save_raw_binary_msg_data", _save_raw_binary_msg_data);
	_private_node_handle.getParam("raw_radar_msg_data_filename", _raw_radar_msg_data_filename);
	if(_save_raw_binary_msg_data)
	{
		auto t = std::time(nullptr);
		auto tm = *std::localtime(&t);
		std::ostringstream oss;
		oss << std::put_time(&tm, "%Y-%m-%d-%H-%M-%S");
		_file_raw_heartbeat_msg_data.open(_raw_radar_msg_data_filename + "_heartbeat"+"_"+oss.str()+".bin", 
										  std::ios_base::out|std::ios_base::binary);
		_file_raw_trackreport_msg_data.open(_raw_radar_msg_data_filename + "_track_report"+"_"+oss.str()+".bin", 
											std::ios_base::out|std::ios_base::binary);
	}
}

/**
* RadarRosBridge destructor
*/
RadarRosBridge::~RadarRosBridge()
{
}

void RadarRosBridge::udp_handle_receive_heartbeat(const boost::system::error_code& error,
										   			 std::size_t num_bytes)
{
	if (!error)
	{
		ROS_INFO("Heartbeat Message Received");

		uint32_t msg_identifier = (uint32_t)_heartbeat_recv_buf[3] << 24  |
      				  			  (uint32_t)_heartbeat_recv_buf[2] << 16 |
      				  			  (uint32_t)_heartbeat_recv_buf[1] << 8  |
      				  			  (uint32_t)_heartbeat_recv_buf[0];

      	float aux_float = 0;
		// Save raw binary data
		if(_save_raw_binary_msg_data)
		{
			_file_raw_heartbeat_msg_data.write((const char*)&num_bytes, sizeof(std::size_t));
			_file_raw_heartbeat_msg_data.write((const char*)_heartbeat_recv_buf.data(), num_bytes);
		}
		
      	// Heartbeat message
      	if(msg_identifier == 0x01AB0101)
      	{
			/*
			//std::cout.write(_recv_buf.data(), len);
			//std::cout << "Something is received" << std::endl; 
			/*
			// If data is received in big-endian format
			num = (uint32_t)buffer[0] << 24 |
      		  	(uint32_t)buffer[1] << 16 |
      	  		(uint32_t)buffer[2] << 8  |
      	  		(uint32_t)buffer[3];
    		// If data is received in little-endian format
			num = (uint32_t)buffer[3] << 24 |
      		  		(uint32_t)buffer[2] << 16 |
      		  		(uint32_t)buffer[1] << 8  |
      	  	  		(uint32_t)buffer[0];
			*/
      		_heartbeat_msg.msg_identifier = msg_identifier;
    		char server_name[Heartbeat::SERVERNAMESIZE];
    		for(int i=0; i<Heartbeat::SERVERNAMESIZE; i++) {
      			server_name[i] = _heartbeat_recv_buf[Heartbeat::SERVERNAMEPOS+i]; 
    		}
			_heartbeat_msg.server_name = std::string(server_name);
			_heartbeat_msg.command_port = (uint16_t)_heartbeat_recv_buf[Heartbeat::CMDPORTPOS+1] << 8 |
								   		  (uint16_t)_heartbeat_recv_buf[Heartbeat::CMDPORTPOS];
			_heartbeat_msg.command_connected = (uint8_t)_heartbeat_recv_buf[Heartbeat::CMDCONNECTEDPOS];
			_heartbeat_msg.command_listen = (uint8_t)_heartbeat_recv_buf[Heartbeat::CMDLISTENPOS];
			std::memcpy (&aux_float, _heartbeat_recv_buf.data() + Heartbeat::RAWDISTTPUTPOS, FLOATSIZE);
			_heartbeat_msg.raw_distribution_throughput = aux_float;
			std::memcpy (&aux_float, _heartbeat_recv_buf.data() + Heartbeat::PRODISTTPUTPOS, FLOATSIZE);
			_heartbeat_msg.pro_distribution_throughput = aux_float;
			std::memcpy (&aux_float, _heartbeat_recv_buf.data() + Heartbeat::RAWRECTPUTPOS, FLOATSIZE);
			_heartbeat_msg.raw_recording_throughput = aux_float;
			_heartbeat_msg.source_started = (uint8_t)_heartbeat_recv_buf[Heartbeat::SRCSTARTEDPOS];
			_heartbeat_msg.buffer_full = (uint8_t)_heartbeat_recv_buf[Heartbeat::BUFFERFULLPOS];
			_heartbeat_msg.cpu_load = (uint16_t)_heartbeat_recv_buf[Heartbeat::CPULOADPOS+1] << 8 |
								      (uint16_t)_heartbeat_recv_buf[Heartbeat::CPULOADPOS];
			_heartbeat_msg.current_source_period = (uint16_t)_heartbeat_recv_buf[Heartbeat::CURRENTSRCPERIODPOS+1] << 8 |
								   			       (uint16_t)_heartbeat_recv_buf[Heartbeat::CURRENTSRCPERIODPOS];
			_heartbeat_msg.n_scans = (uint32_t)_heartbeat_recv_buf[Heartbeat::NSCANSPOS+3] << 24  |
      				  			     (uint32_t)_heartbeat_recv_buf[Heartbeat::NSCANSPOS+2] << 16 |
      				  			     (uint32_t)_heartbeat_recv_buf[Heartbeat::NSCANSPOS+1] << 8  |
      				  			     (uint32_t)_heartbeat_recv_buf[Heartbeat::NSCANSPOS];
			_heartbeat_msg.nav_data_present = (uint8_t)_heartbeat_recv_buf[Heartbeat::NAVDATAPRESENTPOS];
			// std::memcpy(destination, source, size)
      		std::memcpy (&aux_float, _heartbeat_recv_buf.data() + Heartbeat::LATITUDEPOS, FLOATSIZE);
      		_heartbeat_msg.latitude = aux_float;
      		std::memcpy (&aux_float, _heartbeat_recv_buf.data() + Heartbeat::LONGITUDEPOS, FLOATSIZE);
			_heartbeat_msg.longitude = aux_float;

			_heartbeat_publisher.publish(_heartbeat_msg);
      	} else {
      		ROS_WARN("Incorrect Heartbeat Message Reception");
      	}
    }
	_heartbeat_socket.async_receive_from(
			boost::asio::buffer(_heartbeat_recv_buf), _server_endpoint,
			boost::bind(&RadarRosBridge::udp_handle_receive_heartbeat, this,
						boost::asio::placeholders::error,
						boost::asio::placeholders::bytes_transferred));
}

void RadarRosBridge::udp_handle_receive_trackreport(const boost::system::error_code& error,
										   			   std::size_t num_bytes)
{
	if (!error)
	{
		ros::Time reception_time = ros::Time::now();
		_radar_candidate_msg.stamp = reception_time;
		uint32_t msg_identifier = (uint32_t)_track_report_recv_buf[3] << 24  |
      				  			  (uint32_t)_track_report_recv_buf[2] << 16 |
      				  			  (uint32_t)_track_report_recv_buf[1] << 8  |
      				  			  (uint32_t)_track_report_recv_buf[0];

		// Save raw binary data
		if(_save_raw_binary_msg_data)
		{
			_file_raw_trackreport_msg_data.write((const char*)&num_bytes, sizeof(std::size_t));
			_file_raw_trackreport_msg_data.write((const char*)_track_report_recv_buf.data(),num_bytes);
		}
      	if(msg_identifier == 0x01AD0101)
      	{
      		// Basic track report
      		ROS_INFO_STREAM("Basic Track Report Received with size: " << num_bytes);
      		_basic_track_report_msg.msg_identifier = msg_identifier;
      		this->basicTrackReportMsgToROS();
      		_basic_track_report_publisher.publish(_basic_track_report_msg);
			// Candidate publishing
			_radar_candidate_msg.report_level = _radar_candidate_msg.BASICREPORTLEVEL;
			this->createBasicTrackReportCandidate();
			_radar_candidates_list.radar_candidates_list.push_back(_radar_candidate_msg);
		} 
		else if(msg_identifier == 0x01AD0201)
		{
			// Normal track report
			ROS_INFO_STREAM("Normal Track Report Received with size: " << num_bytes);
			_normal_track_report_msg.basic_msg.msg_identifier = msg_identifier;
			this->normalTrackReportMsgToROS();
			_normal_track_report_publisher.publish(_normal_track_report_msg);
			// Candidate publishing
			_radar_candidate_msg.report_level = _radar_candidate_msg.NORMALREPORTLEVEL;
			this->createNormalTrackReportCandidate();
			_radar_candidates_list.radar_candidates_list.push_back(_radar_candidate_msg);
		} 
		else if(msg_identifier == 0x01AD0301)
		{
			// Extended track report
			ROS_INFO_STREAM("Extended Track Report Received with size: " << num_bytes);
			_extended_track_report_msg.normal_msg.basic_msg.msg_identifier = msg_identifier;
			this->extendedTrackReportMsgToROS();
			_extended_track_report_publisher.publish(_extended_track_report_msg);
			// Candidate publishing
			_radar_candidate_msg.report_level = _radar_candidate_msg.NORMALREPORTLEVEL;
			this->createExtendedTrackReportCandidate();
			_radar_candidates_list.radar_candidates_list.push_back(_radar_candidate_msg);
		} 
		else 
		{
			ROS_WARN("Incorrect Track Report Reception");
		}

		_track_report_socket.async_receive_from(
				boost::asio::buffer(_track_report_recv_buf), _server_endpoint,
				boost::bind(&RadarRosBridge::udp_handle_receive_trackreport, this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
	}
}

void RadarRosBridge::basicTrackReportMsgToROS()
{
	float aux_float = 0;
    _basic_track_report_msg.track_id = (uint32_t)_track_report_recv_buf[BasicTrackReport::TRACKIDPOS+1] << 8  |
      				  		 		   (uint32_t)_track_report_recv_buf[BasicTrackReport::TRACKIDPOS];
    _basic_track_report_msg.e_status = (uint8_t)_track_report_recv_buf[BasicTrackReport::ESTATUSPOS];
    _basic_track_report_msg.n_hits = (uint16_t)_track_report_recv_buf[BasicTrackReport::NHITSPOS+1] << 8 |
    					   		 	 (uint16_t)_track_report_recv_buf[BasicTrackReport::NHITSPOS];
    _basic_track_report_msg.n_runs = (uint16_t)_track_report_recv_buf[BasicTrackReport::NRUNSPOS+1] << 8 |
    					   		 	 (uint16_t)_track_report_recv_buf[BasicTrackReport::NRUNSPOS];					   
    std::memcpy (&aux_float, _track_report_recv_buf.data() + BasicTrackReport::ESTXPOS, FLOATSIZE);
    _basic_track_report_msg.estimated_x = aux_float;
    std::memcpy (&aux_float, _track_report_recv_buf.data() + BasicTrackReport::ESTYPOS, FLOATSIZE);
    _basic_track_report_msg.estimated_y = aux_float;
    std::memcpy (&aux_float, _track_report_recv_buf.data() + BasicTrackReport::ESTAZIPOS, FLOATSIZE);
    _basic_track_report_msg.estimated_azi = aux_float;  
    std::memcpy (&aux_float, _track_report_recv_buf.data() + BasicTrackReport::ESTRANGEPOS, FLOATSIZE);
    _basic_track_report_msg.estimated_range = aux_float;
    std::memcpy (&aux_float, _track_report_recv_buf.data() + BasicTrackReport::ESTSPEEDPOS, FLOATSIZE);
    _basic_track_report_msg.estimated_speed = aux_float;
    std::memcpy (&aux_float, _track_report_recv_buf.data() + BasicTrackReport::ESTCOURSEPOS, FLOATSIZE);
    _basic_track_report_msg.estimated_course = aux_float;
    std::memcpy (&aux_float, _track_report_recv_buf.data() + BasicTrackReport::ESTLATPOS, FLOATSIZE);
    _basic_track_report_msg.estimated_lat = aux_float;
    std::memcpy (&aux_float, _track_report_recv_buf.data() + BasicTrackReport::ESTLONPOS, FLOATSIZE);
    _basic_track_report_msg.estimated_lon = aux_float;
    _basic_track_report_msg.time_stamp = (uint64_t)_track_report_recv_buf[BasicTrackReport::TIMESTAMPLSBPOS+7] << 56 |
    						  		 	 (uint64_t)_track_report_recv_buf[BasicTrackReport::TIMESTAMPLSBPOS+6] << 48 |
    						  		 	 (uint64_t)_track_report_recv_buf[BasicTrackReport::TIMESTAMPLSBPOS+5] << 40 |
    						  		 	 (uint64_t)_track_report_recv_buf[BasicTrackReport::TIMESTAMPLSBPOS+4] << 32 |
    						  		 	 (uint64_t)_track_report_recv_buf[BasicTrackReport::TIMESTAMPLSBPOS+3] << 24 |
    						  		 	 (uint64_t)_track_report_recv_buf[BasicTrackReport::TIMESTAMPLSBPOS+2] << 16 |
    						  		 	 (uint64_t)_track_report_recv_buf[BasicTrackReport::TIMESTAMPLSBPOS+1] << 8 |
    						  		 	 (uint64_t)_track_report_recv_buf[BasicTrackReport::TIMESTAMPLSBPOS];
    std::memcpy (&aux_float, _track_report_recv_buf.data() + BasicTrackReport::MEARANGESIZEPOS, FLOATSIZE);
    _basic_track_report_msg.measured_range_size = aux_float;
    std::memcpy (&aux_float, _track_report_recv_buf.data() + BasicTrackReport::MEAAZISIZEPOS, FLOATSIZE);
    _basic_track_report_msg.measured_azimuth_size = aux_float;
}

void RadarRosBridge::normalTrackReportMsgToROS()
{
	float aux_float = 0;
	// Convert basic part of the message
	this->basicTrackReportMsgToROS();
	_normal_track_report_msg.basic_msg = _basic_track_report_msg;
	// Then add fields of normal track report
    std::memcpy (&aux_float, _track_report_recv_buf.data() + NormalTrackReport::MEAXPOS, FLOATSIZE);
    _normal_track_report_msg.measured_x = aux_float;
    std::memcpy (&aux_float, _track_report_recv_buf.data() + NormalTrackReport::MEAYPOS, FLOATSIZE);
    _normal_track_report_msg.measured_y = aux_float;
    std::memcpy (&aux_float, _track_report_recv_buf.data() + NormalTrackReport::MEAAZIPOS, FLOATSIZE);
    _normal_track_report_msg.measured_azi = aux_float;
    std::memcpy (&aux_float, _track_report_recv_buf.data() + NormalTrackReport::MEARANGEPOS, FLOATSIZE);
    _normal_track_report_msg.measured_range = aux_float;
    std::memcpy (&aux_float, _track_report_recv_buf.data() + NormalTrackReport::MEASPEEDPOS, FLOATSIZE);
    _normal_track_report_msg.measured_speed = aux_float;
    std::memcpy (&aux_float, _track_report_recv_buf.data() + NormalTrackReport::MEACOURSEPOS, FLOATSIZE);
    _normal_track_report_msg.measured_course = aux_float;
    std::memcpy (&aux_float, _track_report_recv_buf.data() + NormalTrackReport::PREXPOS, FLOATSIZE);
    _normal_track_report_msg.predicted_x = aux_float;
    std::memcpy (&aux_float, _track_report_recv_buf.data() + NormalTrackReport::PREYPOS, FLOATSIZE);
    _normal_track_report_msg.predicted_y = aux_float;
    std::memcpy (&aux_float, _track_report_recv_buf.data() + NormalTrackReport::PRERANGEPOS, FLOATSIZE);
    _normal_track_report_msg.predicted_range = aux_float;
    std::memcpy (&aux_float, _track_report_recv_buf.data() + NormalTrackReport::PRESPEEDPOS, FLOATSIZE);
    _normal_track_report_msg.predicted_speed = aux_float;
    std::memcpy (&aux_float, _track_report_recv_buf.data() + NormalTrackReport::PRECOURSEPOS, FLOATSIZE);
    _normal_track_report_msg.predicted_course = aux_float;
    std::memcpy (&aux_float, _track_report_recv_buf.data() + NormalTrackReport::SCOREPOS, FLOATSIZE);
    _normal_track_report_msg.score = aux_float;
    std::memcpy (&aux_float, _track_report_recv_buf.data() + NormalTrackReport::WEIGHTPOS, FLOATSIZE);
    _normal_track_report_msg.weight = aux_float;
    std::memcpy (&aux_float, _track_report_recv_buf.data() + NormalTrackReport::STRENGTHPOS, FLOATSIZE);
    _normal_track_report_msg.strength = aux_float;
    std::memcpy (&aux_float, _track_report_recv_buf.data() + NormalTrackReport::DENSITYPOS, FLOATSIZE);
    _normal_track_report_msg.density = aux_float;
}

void RadarRosBridge::extendedTrackReportMsgToROS()
{
	float aux_float = 0;
	// Convert normal part of the message
	this->normalTrackReportMsgToROS();
	_extended_track_report_msg.normal_msg = _normal_track_report_msg;
	// Then add fields of extended track report
    std::memcpy (&aux_float, _track_report_recv_buf.data() + ExtendedTrackReport::GAINXPOS, FLOATSIZE);
    _extended_track_report_msg.gain_x = aux_float;
    std::memcpy (&aux_float, _track_report_recv_buf.data() + ExtendedTrackReport::GAINYPOS, FLOATSIZE);
    _extended_track_report_msg.gain_y = aux_float;
    std::memcpy (&aux_float, _track_report_recv_buf.data() + ExtendedTrackReport::GAINVXPOS, FLOATSIZE);
    _extended_track_report_msg.gain_vx = aux_float;
    std::memcpy (&aux_float, _track_report_recv_buf.data() + ExtendedTrackReport::GAINVYPOS, FLOATSIZE);
    _extended_track_report_msg.gain_vy = aux_float;
    std::memcpy (&aux_float, _track_report_recv_buf.data() + ExtendedTrackReport::INNOERRXPOS, FLOATSIZE);
    _extended_track_report_msg.innovation_error_x = aux_float;
    std::memcpy (&aux_float, _track_report_recv_buf.data() + ExtendedTrackReport::INNOERRYPOS, FLOATSIZE);
    _extended_track_report_msg.innovation_error_y = aux_float;
}

void RadarRosBridge::createBasicTrackReportCandidate()
{
	_radar_candidate_msg.track_id = _basic_track_report_msg.track_id;
	_radar_candidate_msg.e_status = _basic_track_report_msg.e_status;
	_radar_candidate_msg.n_hits = _basic_track_report_msg.n_hits;
	_radar_candidate_msg.estimated_x = _basic_track_report_msg.estimated_x;
	_radar_candidate_msg.estimated_y = _basic_track_report_msg.estimated_y;
	_radar_candidate_msg.estimated_azimuth = _basic_track_report_msg.estimated_azi;
	_radar_candidate_msg.estimated_range = _basic_track_report_msg.estimated_range;
	_radar_candidate_msg.estimated_speed = _basic_track_report_msg.estimated_speed;
	_radar_candidate_msg.estimated_course = _basic_track_report_msg.estimated_course;
	_radar_candidate_msg.estimated_latitude = _basic_track_report_msg.estimated_lat;
	_radar_candidate_msg.estimated_longitude = _basic_track_report_msg.estimated_lon;
	_radar_candidate_msg.time_stamp = _basic_track_report_msg.time_stamp;
	_radar_candidate_msg.measured_range_size = _basic_track_report_msg.measured_range_size;
	_radar_candidate_msg.measured_azimuth_size = _basic_track_report_msg.measured_azimuth_size;
}

void RadarRosBridge::createNormalTrackReportCandidate()
{
	this->createBasicTrackReportCandidate();
	_radar_candidate_msg.measured_x = _normal_track_report_msg.measured_x;
	_radar_candidate_msg.measured_y = _normal_track_report_msg.measured_y;
	_radar_candidate_msg.measured_azimuth = _normal_track_report_msg.measured_azi;
	_radar_candidate_msg.measured_range = _normal_track_report_msg.measured_range;
	_radar_candidate_msg.measured_speed = _normal_track_report_msg.measured_speed;
	_radar_candidate_msg.measured_course = _normal_track_report_msg.measured_course;
	_radar_candidate_msg.score = _normal_track_report_msg.score;
}

void RadarRosBridge::createExtendedTrackReportCandidate()
{
	this->createNormalTrackReportCandidate();
}

void RadarRosBridge::main()
{
	_heartbeat_publisher = _node_handle.advertise<sensors::RadarHeartbeat>("heartbeat", 10);
	_basic_track_report_publisher = _node_handle.advertise<sensors::RadarBasicTrackReport>("basic_track_report", 10);
	_normal_track_report_publisher = _node_handle.advertise<sensors::RadarNormalTrackReport>("normal_track_report", 10);
	_extended_track_report_publisher = _node_handle.advertise<sensors::RadarExtendedTrackReport>("extended_track_report", 10);
	_radar_candidates_list_publisher = _node_handle.advertise<tracking::RadarCandidatesList>("radar_candidates",10);
	//ros::Duration(1).sleep();
	try
	{
		//boostUdp::resolver resolver(_io_service);
		// Query can accept the following parameters
		// 1º Protocol to use, in this case UDP/IPv4
		// 2º Server name (FQDN)
		// 3º Service (or port) you want to access (see UDP and TCP ports list on wikipedia)
		// boostUdp::resolver::query query(boostUdp::v4(), "localhost", "1234");
		// _server_endpoint = *resolver.resolve(query);

		// Print endpoint IP address
		//std::cout << "server IP address" << _server_endpoint.address() << std::endl;

		//boostUdp::socket socket(_io_service);
		//_socket.open(boostUdp::v4());
		//_send_buf[0] = 42;
		//_socket.send_to(boost::asio::buffer(_send_buf), _server_endpoint);

		_heartbeat_socket.async_receive_from(
			boost::asio::buffer(_heartbeat_recv_buf), _server_endpoint,
			boost::bind(&RadarRosBridge::udp_handle_receive_heartbeat, this,
						boost::asio::placeholders::error,
						boost::asio::placeholders::bytes_transferred));
		_track_report_socket.async_receive_from(
			boost::asio::buffer(_track_report_recv_buf), _server_endpoint,
			boost::bind(&RadarRosBridge::udp_handle_receive_trackreport, this,
						boost::asio::placeholders::error,
						boost::asio::placeholders::bytes_transferred));

	}
	catch (std::exception& e)
	{
		ROS_WARN_STREAM("Exception related with UDP socket: " << e.what());
	}
	ros::Rate rate(_working_frequency);
	while(ros::Time::now() == ros::Time(0)); // If in simulation, wait until /clock messages are published
	while(ros::ok())
	{
		//_io_service.poll_one();
		_io_service.poll(); // Execute all ready handlers, then continue
		if(!_radar_candidates_list.radar_candidates_list.empty())
		{
			_radar_candidates_list_publisher.publish(_radar_candidates_list);
			_radar_candidates_list.radar_candidates_list.clear();
		}
		rate.sleep();
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "radar_ros_bridge");
	ros::NodeHandle pnh("~");
	// Get parameters
	int heartbeat_listening_port = HBLSTPORT;		
	pnh.getParam("heartbeat_listening_port", heartbeat_listening_port);
	int track_report_listening_port = TRLSTPORT;
	pnh.getParam("track_report_listening_port", track_report_listening_port);
	//***************
	RadarRosBridge radar_ros_bridge(heartbeat_listening_port, track_report_listening_port);
	radar_ros_bridge.main();
	return 0;
}