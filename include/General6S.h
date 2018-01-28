#pragma once
#include "SerialRobotModel.h"
class General6S :
	public SerialRobotModel
{
public:
	General6S();
	~General6S();
	virtual	void setRobotModelParameter();

	virtual ErrorID calcInverseKin_RPY(const Position_MCS_rad& posMCS,
									   const Position_ACS_rad& posLast, Position_ACS_rad& posACS) const;
};

