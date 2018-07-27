#include "roscan.h"
#include <thread>
#include <ctime>
#include <ratio>
#include <chrono>

void read_battery(ROSCan* roscan){
	can_frame cf;
	if(!roscan) return;

	while(! roscan->is_shutdown()){
		printf("<read>\n");
		if(roscan->read(cf)){
			fprintf(stdout, "read!\n");
			//fprint_long_canframe(stdout, &cf, "\n", 0);
		}else{
			fprintf(stderr, "read failed!\n");
			break;
		}
		printf("</read>\n");
	}
}

int main(){
	ROSCan roscan("can0");
	roscan.init();
	roscan.set_filter(0x1C0C0000, 0x1FFFF0FF);
	//roscan.set_timeout(0.01);
	//std::thread b_thread(read_battery, &roscan);

	fd_set rdfs;
	timeval tv;
	can_frame cf;

	tv.tv_sec = 0;
	tv.tv_usec = 10000;

	std::chrono::high_resolution_clock::time_point begin_loop = std::chrono::high_resolution_clock::now();
	std::chrono::high_resolution_clock::time_point last_send = std::chrono::high_resolution_clock::now();
	while(true){
		std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();

		FD_ZERO(&rdfs);
		FD_SET(roscan._s_h, &rdfs);

		int rc = select(roscan._s_h+1, &rdfs, NULL, NULL, &tv);

		if(!rc){
			std::chrono::duration<double> dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_send);
			if (dt.count() > 0.2){
				roscan.send("181C0100#0260000000000000");
				last_send = now;
			}
		}

		if(FD_ISSET(roscan._s_h, &rdfs)){
			roscan.read(cf);
			fprint_long_canframe(stdout, &cf, "\n", 0);
			printf("<<%d>>\n", cf.data[0]);
		}

		//printf("<send>\n");
		//roscan.send("181C0100#0260000000000000");
		//printf("</send>\n");
		//sleep(1);
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		//if(!roscan.send("0x181C0100#0260000000000000")){
		//	fprintf(stderr, "send failed!\n");
		//}
		//if(roscan.read(cf)){
		//	fprint_long_canframe(stdout, &cf, "\n", 0);
		//}
		if(std::chrono::duration_cast<std::chrono::duration<double>>(now - begin_loop).count() > 10.0){
			break;
		}
	}
	roscan.shutdown();
	//b_thread.join();
	return 0;
}
