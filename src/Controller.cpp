#include "Controller.h"

Controller* Controller::m_singleController = nullptr;   ////在此处初始化

//Controller::Controller()
//{
//}
//
//
//Controller::~Controller()
//{
//}

void Controller::initializeController()
{
	const int length = 4;
	int a[length] = { 0 };
	printf("Now initializeController:please input %d robot type:\n", length);
	for (size_t i = 0; i < length; i++)
	{
		scanf("%d", &a[i]);
	}
	std::shared_ptr<SerialRobotModel> _pR;
	for (size_t i = 0; i < length; i++)
	{
		if (a[i] == 6)
		{
			_pR.reset(new General6S());
		}
		else
		{
			_pR.reset(new Scara4S());
		}
		pSerialVec.push_back(_pR);
	}
}
