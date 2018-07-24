#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

#define FRAME_FRONT "/sonar_front"
#define FRAME_BACK "/sonar_back"
#define PIN_BACK 7
#define PIN_FRONT 4

ros::NodeHandle  nh;

sensor_msgs::Range range_msg;
ros::Publisher pub_front("sonar_front", &range_msg);
ros::Publisher pub_back("sonar_back", &range_msg);

unsigned long range_timer;

float getRange(int pin_num){
    long pulse = pulseIn(pin_num, HIGH);
    float distance_inches = pulse / 147.0;
    float distance_meter  = distance_inches * 0.0254;
    return distance_meter;
}

void setup()
{
    nh.initNode();
    nh.advertise(pub_front);
    nh.advertise(pub_back);

    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.header.frame_id = "";
    range_msg.field_of_view = 1.04;
    range_msg.min_range = 0.15;
    range_msg.max_range = 6.0;
}

void loop()
{
    // publish the range value every 50 milliseconds
    //  since it takes that long for the sensor to stabilize
    if ( (millis()-range_timer) > 50){
        float d_back = getRange(PIN_BACK);
        float d_front = getRange(PIN_FRONT);

        range_msg.header.stamp = nh.now();

        range_msg.range = d_back;
        range_msg.header.frame_id = FRAME_BACK;
        pub_back.publish(&range_msg);

        range_msg.range = d_front;
        range_msg.header.frame_id = FRAME_FRONT;
        pub_front.publish(&range_msg);

        range_timer =  millis();
    }
    nh.spinOnce();
}


