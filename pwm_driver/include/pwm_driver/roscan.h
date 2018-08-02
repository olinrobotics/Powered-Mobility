#ifndef __ROSCAN_H__
#define __ROSCAN_H__

extern "C"{
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

#include <net/if.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "lib.h"

}

struct ROSCan{
	const char* _dev; // can device (can0, ...)
	int _s_h; // socket handle
	fd_set _rdfs; // file descriptor
	bool _is_ok;
	bool _is_shutdown;

	ROSCan(const char* dev);
	~ROSCan();
	bool init();
	bool quit();
	bool send(const char* data);
	bool send(const can_frame& frame);
	bool read(can_frame& frame);
	bool set_filter(canid_t id, canid_t mask);
	bool enable_fd();
	bool is_ok();
	void set_ok(bool);
	void set_timeout(float);

	void shutdown();
	bool is_shutdown();
};

#endif
