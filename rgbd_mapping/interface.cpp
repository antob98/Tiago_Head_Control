#include <ros/ros.h>
#include <iostream>
#include <pal_navigation_msgs/SaveMap.h>

using namespace std;

int main(int argc, char** argv){

	ros::init(argc, argv, "interface");

    ros::NodeHandle n;

    ros::ServiceClient client_interface = n.serviceClient<pal_navigation_msgs::SaveMap>("/pal_map_manager/save_map");

    pal_navigation_msgs::SaveMap save;

    int com = 11;

    cout<<"***************************************************************"<<endl;
    cout<<endl;
    cout<<"           Hello.Take your choice, please:"<<endl;
    cout<<endl;
    cout<<"             * enter 1, if you want to save the map."<<endl;
    cout<<endl;
    cout<<"             * enter 0, if you want to exit"<<endl;
    cout<<endl;
    cout<<"***************************************************************"<<endl;

    while(ros::ok()){
        
        cin>>com;

        if(com == 1){

            client_interface.waitForExistence();
            client_interface.call(save);
	    cout<<"If you want to save again the map, please insert 1."<<endl;
	    cout<<"Otherwise, enter 0 to exit."<<endl;

        }else if(com == 0){

            exit(0);

        }
    }

    ros::spin();

    return 0;
    
}
