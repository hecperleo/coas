#include "sensors/sehirus_conversion.h"

/**
* SehirusConversion constructor
*/
SehirusConversion::SehirusConversion(const uint16_t &heartbeatListeningPort, const uint16_t &trackReportListeningPort):
_nodeHandle(),
_heartbeatSocket(_io_service, boostUdp::endpoint(boostUdp::v4(), heartbeatListeningPort)),
_trackReportSocket(_io_service, boostUdp::endpoint(boostUdp::v4(), trackReportListeningPort))
{
}

/**
* SehirusConversion destructor
*/
SehirusConversion::~SehirusConversion()
{

}

void SehirusConversion::udp_handle_receive_heartbeat(const boost::system::error_code& error,
										   			 std::size_t num_bytes)
{
	if (!error)
	{
		ROS_INFO("Heartbeat Message Received");

		uint32_t msgIdentifier = (uint32_t)_heartbeatRecvBuf[3] << 24  |
      				  			 (uint32_t)_heartbeatRecvBuf[2] << 16 |
      				  			 (uint32_t)_heartbeatRecvBuf[1] << 8  |
      				  			 (uint32_t)_heartbeatRecvBuf[0];

      	float aux_float = 0;
      	// Heartbeat message
      	if ( (msgIdentifier == 0x01AB0101) && (num_bytes == Heartbeat::UDPMSGSIZE) )  
      	{
			/*
			//std::cout.write(_recv_buf.data(), len);
			//std::cout << "Something is received" << std::endl;
			// Si recibimos los datos en formato big-endian
			/*
			num = (uint32_t)buffer[0] << 24 |
      		  	(uint32_t)buffer[1] << 16 |
      	  		(uint32_t)buffer[2] << 8  |
      	  		(uint32_t)buffer[3];
    		// Si recibimos los datos en formato little-endian
			num = (uint32_t)buffer[3] << 24 |
      		  		(uint32_t)buffer[2] << 16 |
      		  		(uint32_t)buffer[1] << 8  |
      	  	  		(uint32_t)buffer[0];
			*/
      		_heartbeatMsg.msgIdentifier = msgIdentifier;
    		char serverName[Heartbeat::SERVERNAMESIZE];
    		for(int i=0; i<=Heartbeat::SERVERNAMESIZE; i++) {
      			serverName[i] = _heartbeatRecvBuf[Heartbeat::SERVERNAMEPOS+i]; 
    		}
			_heartbeatMsg.serverName = std::string(serverName);
			_heartbeatMsg.commandPort = (uint16_t)_heartbeatRecvBuf[Heartbeat::CMDPORTPOS+1] << 8 |
								   		(uint16_t)_heartbeatRecvBuf[Heartbeat::CMDPORTPOS];
			_heartbeatMsg.commandConnected = (uint8_t)_heartbeatRecvBuf[Heartbeat::CMDCONNECTEDPOS];
			_heartbeatMsg.commandListen = (uint8_t)_heartbeatRecvBuf[Heartbeat::CMDLISTENPOS];
			std::memcpy (&aux_float, _heartbeatRecvBuf.data() + Heartbeat::RAWDISTTPUTPOS, FLOATSIZE);
			_heartbeatMsg.rawDistributionThroughput = aux_float;
			std::memcpy (&aux_float, _heartbeatRecvBuf.data() + Heartbeat::PRODISTTPUTPOS, FLOATSIZE);
			_heartbeatMsg.proDistributionThroughput = aux_float;
			std::memcpy (&aux_float, _heartbeatRecvBuf.data() + Heartbeat::RAWRECTPUTPOS, FLOATSIZE);
			_heartbeatMsg.rawRecordingThroughput = aux_float;
			_heartbeatMsg.sourceStarted = (uint8_t)_heartbeatRecvBuf[Heartbeat::SRCSTARTEDPOS];
			_heartbeatMsg.bufferFull = (uint8_t)_heartbeatRecvBuf[Heartbeat::BUFFERFULLPOS];
			_heartbeatMsg.cpuLoad = (uint16_t)_heartbeatRecvBuf[Heartbeat::CPULOADPOS+1] << 8 |
								    (uint16_t)_heartbeatRecvBuf[Heartbeat::CPULOADPOS];
			_heartbeatMsg.currentSourcePeriod = (uint16_t)_heartbeatRecvBuf[Heartbeat::CURRENTSRCPERIODPOS+1] << 8 |
								   			    (uint16_t)_heartbeatRecvBuf[Heartbeat::CURRENTSRCPERIODPOS];
			_heartbeatMsg.nScans = (uint32_t)_heartbeatRecvBuf[Heartbeat::NSCANSPOS+3] << 24  |
      				  			   (uint32_t)_heartbeatRecvBuf[Heartbeat::NSCANSPOS+2] << 16 |
      				  			   (uint32_t)_heartbeatRecvBuf[Heartbeat::NSCANSPOS+1] << 8  |
      				  			   (uint32_t)_heartbeatRecvBuf[Heartbeat::NSCANSPOS];
			_heartbeatMsg.navDataPresent = (uint8_t)_heartbeatRecvBuf[Heartbeat::NAVDATAPRESENTPOS];
			// std::memcpy ( destino, origen, tamaño)
      		std::memcpy (&aux_float, _heartbeatRecvBuf.data() + Heartbeat::LATITUDEPOS, FLOATSIZE);
      		_heartbeatMsg.latitude = aux_float;
      		std::memcpy (&aux_float, _heartbeatRecvBuf.data() + Heartbeat::LONGITUDEPOS, FLOATSIZE);
			_heartbeatMsg.longitude = aux_float;

			_heartBeatPublisher.publish(_heartbeatMsg);
      	} else {
      		ROS_WARN("Incorrect Heartbeat Message Reception");
      	}
    }
	_heartbeatSocket.async_receive_from(
			boost::asio::buffer(_heartbeatRecvBuf), _server_endpoint,
			boost::bind(&SehirusConversion::udp_handle_receive_heartbeat, this,
						boost::asio::placeholders::error,
						boost::asio::placeholders::bytes_transferred));
}

void SehirusConversion::udp_handle_receive_trackreport(const boost::system::error_code& error,
										   			   std::size_t num_bytes)
{
	if (!error)
	{
		uint32_t msgIdentifier = (uint32_t)_trackReportRecvBuf[3] << 24  |
      				  			 (uint32_t)_trackReportRecvBuf[2] << 16 |
      				  			 (uint32_t)_trackReportRecvBuf[1] << 8  |
      				  			 (uint32_t)_trackReportRecvBuf[0];

      	if ( (msgIdentifier == 0x01AD0101) && (num_bytes == BasicTrackReport::UDPMSGSIZE) )
      	{
      		// Basic track report
      		ROS_INFO_STREAM("Basic Track Report Received with size: " << num_bytes);
      		_basicTrackReportMsg.msgIdentifier = msgIdentifier;
      		this->basicTrackReportMsgToROS();
      		_basicTrackReportPublisher.publish(_basicTrackReportMsg);
		} 
		else if ( (msgIdentifier == 0x01AD0201) && (num_bytes == NormalTrackReport::UDPMSGSIZE) ) 
		{
			// Normal track report
			ROS_INFO_STREAM("Normal Track Report Received with size: " << num_bytes);
			_normalTrackReportMsg.basicMsg.msgIdentifier = msgIdentifier;
			this->normalTrackReportMsgToROS();
			_normalTrackReportPublisher.publish(_normalTrackReportMsg);
		} 
		else if ( (msgIdentifier == 0x01AD0301) && (num_bytes == ExtendedTrackReport::UDPMSGSIZE) )
		{
			// Extended track report
			ROS_INFO_STREAM("Extended Track Report Received with size: " << num_bytes);
			_extendedTrackReportMsg.normalMsg.basicMsg.msgIdentifier = msgIdentifier;
			this->extendedTrackReportMsgToROS();
			_extendedTrackReportPublisher.publish(_extendedTrackReportMsg);
		} 
		else 
		{
			ROS_WARN("Incorrect Track Report Reception");
		}

		_trackReportSocket.async_receive_from(
				boost::asio::buffer(_trackReportRecvBuf), _server_endpoint,
				boost::bind(&SehirusConversion::udp_handle_receive_trackreport, this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
	}
}

void SehirusConversion::basicTrackReportMsgToROS()
{
	float aux_float = 0;
    _basicTrackReportMsg.trackID = (uint32_t)_trackReportRecvBuf[BasicTrackReport::TRACKIDPOS+1] << 8  |
      				  		 	   (uint32_t)_trackReportRecvBuf[BasicTrackReport::TRACKIDPOS];
    _basicTrackReportMsg.eStatus = (uint8_t)_trackReportRecvBuf[BasicTrackReport::ESTATUSPOS];
    _basicTrackReportMsg.nHits = (uint16_t)_trackReportRecvBuf[BasicTrackReport::NHITSPOS+1] << 8 |
    					   		 (uint16_t)_trackReportRecvBuf[BasicTrackReport::NHITSPOS];
    _basicTrackReportMsg.nRuns = (uint16_t)_trackReportRecvBuf[BasicTrackReport::NRUNSPOS+1] << 8 |
    					   		 (uint16_t)_trackReportRecvBuf[BasicTrackReport::NRUNSPOS];					   
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + BasicTrackReport::ESTXPOS, FLOATSIZE);
    _basicTrackReportMsg.estX = aux_float;
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + BasicTrackReport::ESTYPOS, FLOATSIZE);
    _basicTrackReportMsg.estY = aux_float;
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + BasicTrackReport::ESTAZIPOS, FLOATSIZE);
    _basicTrackReportMsg.estAzi = aux_float;  
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + BasicTrackReport::ESTRANGEPOS, FLOATSIZE);
    _basicTrackReportMsg.estRange = aux_float;
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + BasicTrackReport::ESTSPEEDPOS, FLOATSIZE);
    _basicTrackReportMsg.estSpeed = aux_float;
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + BasicTrackReport::ESTCOURSEPOS, FLOATSIZE);
    _basicTrackReportMsg.estCourse = aux_float;
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + BasicTrackReport::ESTLATPOS, FLOATSIZE);
    _basicTrackReportMsg.estLat = aux_float;
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + BasicTrackReport::ESTLONPOS, FLOATSIZE);
    _basicTrackReportMsg.estLon = aux_float;
    _basicTrackReportMsg.timeStamp = (uint64_t)_trackReportRecvBuf[BasicTrackReport::TIMESTAMPLSBPOS+7] << 56 |
    						  		 (uint64_t)_trackReportRecvBuf[BasicTrackReport::TIMESTAMPLSBPOS+6] << 48 |
    						  		 (uint64_t)_trackReportRecvBuf[BasicTrackReport::TIMESTAMPLSBPOS+5] << 40 |
    						  		 (uint64_t)_trackReportRecvBuf[BasicTrackReport::TIMESTAMPLSBPOS+4] << 32 |
    						  		 (uint64_t)_trackReportRecvBuf[BasicTrackReport::TIMESTAMPLSBPOS+3] << 24 |
    						  		 (uint64_t)_trackReportRecvBuf[BasicTrackReport::TIMESTAMPLSBPOS+2] << 16 |
    						  		 (uint64_t)_trackReportRecvBuf[BasicTrackReport::TIMESTAMPLSBPOS+1] << 8 |
    						  		 (uint64_t)_trackReportRecvBuf[BasicTrackReport::TIMESTAMPLSBPOS];
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + BasicTrackReport::MEARANGESIZEPOS, FLOATSIZE);
    _basicTrackReportMsg.meaRangeSize = aux_float;
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + BasicTrackReport::MEAAZISIZEPOS, FLOATSIZE);
    _basicTrackReportMsg.meaAzimuthSize = aux_float;
}

void SehirusConversion::normalTrackReportMsgToROS()
{
	float aux_float = 0;
	// Convert basic part of the message
	this->basicTrackReportMsgToROS();
	_normalTrackReportMsg.basicMsg = _basicTrackReportMsg;
	// Then add fields of normal track report
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + NormalTrackReport::MEAXPOS, FLOATSIZE);
    _normalTrackReportMsg.meaX = aux_float;
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + NormalTrackReport::MEAYPOS, FLOATSIZE);
    _normalTrackReportMsg.meaY = aux_float;
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + NormalTrackReport::MEAAZIPOS, FLOATSIZE);
    _normalTrackReportMsg.meaAzi = aux_float;
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + NormalTrackReport::MEARANGEPOS, FLOATSIZE);
    _normalTrackReportMsg.meaRange = aux_float;
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + NormalTrackReport::MEASPEEDPOS, FLOATSIZE);
    _normalTrackReportMsg.meaSpeed = aux_float;
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + NormalTrackReport::MEACOURSEPOS, FLOATSIZE);
    _normalTrackReportMsg.meaCourse = aux_float;
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + NormalTrackReport::PREXPOS, FLOATSIZE);
    _normalTrackReportMsg.preX = aux_float;
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + NormalTrackReport::PREYPOS, FLOATSIZE);
    _normalTrackReportMsg.preY = aux_float;
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + NormalTrackReport::PRERANGEPOS, FLOATSIZE);
    _normalTrackReportMsg.preRange = aux_float;
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + NormalTrackReport::PRESPEEDPOS, FLOATSIZE);
    _normalTrackReportMsg.preSpeed = aux_float;
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + NormalTrackReport::PRECOURSEPOS, FLOATSIZE);
    _normalTrackReportMsg.preCourse = aux_float;
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + NormalTrackReport::SCOREPOS, FLOATSIZE);
    _normalTrackReportMsg.score = aux_float;
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + NormalTrackReport::WEIGHTPOS, FLOATSIZE);
    _normalTrackReportMsg.weight = aux_float;
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + NormalTrackReport::STRENGTHPOS, FLOATSIZE);
    _normalTrackReportMsg.strength = aux_float;
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + NormalTrackReport::DENSITYPOS, FLOATSIZE);
    _normalTrackReportMsg.density = aux_float;
}

void SehirusConversion::extendedTrackReportMsgToROS()
{
	float aux_float = 0;
	// Convert normal part of the message
	this->normalTrackReportMsgToROS();
	_extendedTrackReportMsg.normalMsg = _normalTrackReportMsg;
	// Then add fields of extended track report
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + ExtendedTrackReport::GAINXPOS, FLOATSIZE);
    _extendedTrackReportMsg.gainX = aux_float;
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + ExtendedTrackReport::GAINYPOS, FLOATSIZE);
    _extendedTrackReportMsg.gainY = aux_float;
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + ExtendedTrackReport::GAINVXPOS, FLOATSIZE);
    _extendedTrackReportMsg.gainVX = aux_float;
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + ExtendedTrackReport::GAINVYPOS, FLOATSIZE);
    _extendedTrackReportMsg.gainVY = aux_float;
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + ExtendedTrackReport::INNOERRXPOS, FLOATSIZE);
    _extendedTrackReportMsg.innoErrorX = aux_float;
    std::memcpy (&aux_float, _trackReportRecvBuf.data() + ExtendedTrackReport::INNOERRYPOS, FLOATSIZE);
    _extendedTrackReportMsg.innoErrorY = aux_float;
}

void SehirusConversion::main()
{
	_heartBeatPublisher = _nodeHandle.advertise<sensors::SehirusHeartbeat>("heartbeat", 10);
	_basicTrackReportPublisher = _nodeHandle.advertise<sensors::SehirusBasicTrackReport>("basic_track_report", 10);
	_normalTrackReportPublisher = _nodeHandle.advertise<sensors::SehirusNormalTrackReport>("normal_track_report", 10);
	_extendedTrackReportPublisher = _nodeHandle.advertise<sensors::SehirusExtendedTrackReport>("extended_track_report", 10);
	//ros::Duration(1).sleep();
	try
	{
		//boostUdp::resolver resolver(_io_service);
		// A la consulta se le pasan los siguientes argumentos
		// 1º Protocolo a usar, en este caso UDP/IPv4
		// 2º El nombre del servidor (FQDN)
		// 3º El servicio (o puerto) al que se pretende acceder (ver Lista de puertos TCP y UDP en wikipedia)
		//boostUdp::resolver::query query(boostUdp::v4(), "localhost", "1234");
		//_server_endpoint = *resolver.resolve(query);

		// Imprimir la dirección IP del endpoint
		//std::cout << "dirección IP del servidor " << _server_endpoint.address() << std::endl;

		//boostUdp::socket socket(_io_service);
		//_socket.open(boostUdp::v4());

		//_send_buf[0] = 42;
		//_socket.send_to(boost::asio::buffer(_send_buf), _server_endpoint);

		_heartbeatSocket.async_receive_from(
			boost::asio::buffer(_heartbeatRecvBuf), _server_endpoint,
			boost::bind(&SehirusConversion::udp_handle_receive_heartbeat, this,
						boost::asio::placeholders::error,
						boost::asio::placeholders::bytes_transferred));
		_trackReportSocket.async_receive_from(
			boost::asio::buffer(_trackReportRecvBuf), _server_endpoint,
			boost::bind(&SehirusConversion::udp_handle_receive_trackreport, this,
						boost::asio::placeholders::error,
						boost::asio::placeholders::bytes_transferred));

	}
	catch (std::exception& e)
	{
		ROS_WARN_STREAM("Exception related with UDP socket: " << e.what());
	}
	while (ros::ok())
	{
		_io_service.poll_one();
		//_socket.send_to(boost::asio::buffer(_send_buf), _server_endpoint);
		ros::Duration(0.05).sleep();
	}
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "sehirus_ros_node");
	ros::NodeHandle privateNodeHandle("~");
	// Get parameters
	int heartbeatListeningPort = HBLSTPORT;		
	privateNodeHandle.getParam("heartbeat_listening_port", heartbeatListeningPort);
	int trackReportListeningPort = TRLSTPORT;
	privateNodeHandle.getParam("track_report_listening_port", trackReportListeningPort);
	//***************
	SehirusConversion sehirusConversion(heartbeatListeningPort, trackReportListeningPort);
	sehirusConversion.main();
	return 0;
}