#ifndef _COVERT_DETECTION
#define _COVERT_DETECTION

/**
 * Functions to convert SSL vision detection packets to the ROS format.
 * Works for legacy and current SSL vision formats.
 */


#include "pack_protobuf_wrapper/proto/messages_robocup_ssl_detection.pb.h"
#include "pack_msgs/msg/ssl_vision_detection.hpp"

#include "convert_units.h"


namespace pr {
/**
 * Converts an SSL DetectionFrame to the ROS version.
 */
pack_msgs::msg::SSLVisionDetection convert_detection_frame(const SSL_DetectionFrame& protoFrame);


/**
 * Converts a protoBuf DetectionBall to the ROS version.
 */
pack_msgs::msg::SSLVisionDetectionBall convert_detection_ball(SSL_DetectionBall protoBall);


/**
 * Converts a protoBuf DetectionRobot to the ROS version.
 */
pack_msgs::msg::SSLVisionDetectionRobot convert_detection_robot(SSL_DetectionRobot protoBot);
}

#endif // _CONVERT_DETECTION