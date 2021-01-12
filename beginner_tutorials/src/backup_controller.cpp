#include "ros/ros.h"
#include <iostream> 
#include <vector>
#include <fstream>
#include <sstream>
#include <string.h>
#include <ros/package.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>


class BackUp
{
    private:
        ros::NodeHandle nh;
        int path_cnt;
        int point_cnt;
        
    public:
        std::vector<nav_msgs::Path> path_array;
        BackUp();

};

BackUp::BackUp()
{
    ros::NodeHandle pn("~");
    //pn.param("controller_freq",controller_freq,10);
    path_cnt = 0;
    point_cnt = 0;

    const std::string filePath = ros::package::getPath("stauto_core") + "/src/backup.txt";
    std::ifstream inputFile;
    inputFile.open (filePath.c_str());
    std::string str; std::string substr;
    std::vector<std::string> container;
    std::cout << "BackUp constructor"<< inputFile << std:: endl;

    if (inputFile.is_open())
    {
        while(inputFile)
        {
            getline(inputFile,str);
            std::stringstream ss(str);
            if (str == "*")
            {
                counter += 1;
            }
            else
            {
                while (ss.good())
                {
                    getline(ss,substr,',');
                    container.push_back(substr);
                    // std::cout << "counter : " << counter << "number : " << container << std::endl;
                    
                }
            }
            
        }
    }
    inputFile.close();

}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"backup_controller");
    BackUp backup;
    ros::spin();
    return(0);
}