#include "pwm_driver/roscan.h"

ROSCan::ROSCan(const char* dev):_dev(dev){
	_is_ok = true;
	_is_shutdown = false;
}

ROSCan::~ROSCan(){
	quit();
}

bool ROSCan::init(){
	struct sockaddr_can addr;
	struct ifreq ifr;
	int i;

	fprintf(stderr,"CAN Opening\n");

	/* parse CAN frame */
	/* open socket */
	if ((_s_h = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("socket");
		return false;
	}

	addr.can_family = AF_CAN;

	strcpy(ifr.ifr_name, _dev);
	if (ioctl(_s_h, SIOCGIFINDEX, &ifr) < 0) {
		perror("SIOCGIFINDEX");
		::close(_s_h);
		return false;
	}
	addr.can_ifindex = ifr.ifr_ifindex;

	//setsockopt(_s_h, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

	if (bind(_s_h, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return false;
	}

	// needed?
	// enable_fd();
	return true;
}

bool ROSCan::quit(){
	::close(_s_h);
}

bool ROSCan::send(const char* data){
	struct can_frame frame;
	if (parse_canframe(data, &frame)){
		return false;
	}
	return send(frame);
}

bool ROSCan::send(const can_frame& frame){
	int nbytes;
	if ((nbytes = write(_s_h, &frame, sizeof(frame))) != sizeof(frame)) {
		perror("write");
		return false;
	}
	return true;
}

bool ROSCan::read(can_frame& frame){
	int nbytes = ::read(_s_h, &frame, CANFD_MTU);
	switch(nbytes){
		case CAN_MTU:
			return true;
			break;
		case CANFD_MTU:
			return true;
			break;
		case -1:
			return false;
			break;
		default:
			return false;
			break;
	}
	return false;
}

bool ROSCan::set_filter(canid_t id, canid_t mask){
	struct can_filter filter;
	filter.can_id = id;
	filter.can_mask = mask;
	int rc = setsockopt(_s_h, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));
	bool suc = (rc != -1);
	return suc;
}

bool ROSCan::enable_fd(){
	int enable = 1;
	setsockopt(_s_h, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable));
}

bool ROSCan::is_ok(){
	return true;
	//int error = 0;
	//socklen_t len = sizeof(error);
	//int retval = getsockopt(_s_h, SOL_CAN_RAW, SO_ERROR
	//can_frame cf;
}

void ROSCan::set_ok(bool ok){
	this->_is_ok = ok;
}

void ROSCan::shutdown(){
	this->_is_shutdown = true;
}
bool ROSCan::is_shutdown(){
	return _is_shutdown;
}

void ROSCan::set_timeout(float timeout){
	struct timeval tv;
	tv.tv_sec = int(timeout);
	tv.tv_usec = fmod(timeout, 1.0) * 1e6; // == 10 ms
	::setsockopt(_s_h, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

//void ROSCan::step(){
//	// TODO : make set_timeout effect step??
//	// todo : configure timeval ??
//
//	std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
//
//	timeval tv;
//	can_frame cf;
//	tv.tv_sec = 0;
//	tv.tv_usec = 1000; // == 1ms
//
//	FD_ZERO(&_rdfs);
//	FD_SET(_s_h, &_rdfs);
//
//	int rc = select(_s_h+1, &_rdfs, NULL, NULL, &tv);
//
//	if(!rc){
//		std::chrono::duration<double> dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_send);
//		if (dt.count() > 0.2){
//			this->send("181C0100#0260000000000000");
//			last_send = now;
//		}
//	}
//
//	if(FD_ISSET(s_h, &rdfs)){
//		this->read(cf);
//		fprint_long_canframe(stdout, &cf, "\n", 0);
//	}
//}
