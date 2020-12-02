
#include "passthrough_config.h"


int main(int argc, char * * argv) {
	ros::init(argc, argv, "passthrough_config");
	
	ros::NodeHandle private_node_handle("~");

	passthrough_config::PassthroughConfig passthrough_filter_node(private_node_handle);

	ros::spin();

	return 0;
}



