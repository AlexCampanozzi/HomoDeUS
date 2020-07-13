#include <string>
#include <vector>
#include <map>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

class ArmInterface
{
    private:
        float _planningTime;
        std::string _plannerId;
        std::vector<std::string> _jointsNames;
        moveit::planning_interface::MoveGroupInterface _moveGroup;
        std::map<std::string, double> _target_position;

        bool planTrajectory();
        bool planTrajectoryJ(moveit::planning_interface::MoveGroupInterface::Plan &plan);

    public:
        ArmInterface();

        void setPlanningTime(float val);
        void setPlannerId(std::string Id);
        bool moveTo();
        bool moveToJ(double torso, double j1, double j2, double j3, double j4, double j5, double j6, double j7);

        ~ArmInterface(); 
};