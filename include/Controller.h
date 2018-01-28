#pragma once
#include "Robot.h"

class Controller
{

public:
	~Controller()
	{
		if (m_singleController)
		{
			delete m_singleController;
		}
	};
	static Controller* createController()
	{
		if (m_singleController == nullptr)
		{
			m_singleController = new Controller();
		}
		return m_singleController;
	};

	std::vector<std::shared_ptr<AbstractRobot> > pSerialVec;

private:
	Controller()
	{
		initializeController();
	};

	void initializeController();

	static Controller* m_singleController;

};

