#include "Seq.h"
#include "Robot.h"
#include <iostream>
#include "Controller.h"


using namespace std;

N_AxisSeqPtr Snap_AglSeqPtr;
std::shared_ptr<Robot> ptrRobot;


int main()
{
	//构建机器人对象
	//ptrRobot.reset(Robot::getNewRobot());
	Controller* pCtrl = Controller::createController();
	for (size_t i = 0; i < (pCtrl->pSerialVec).size(); i++)
	{
		pCtrl->pSerialVec[i]->setRobotModelParameter();
	}
	return 0;
}