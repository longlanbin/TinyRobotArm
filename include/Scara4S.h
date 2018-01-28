#pragma once
#include "SerialRobotModel.h"
class Scara4S :
	public SerialRobotModel
{
public:
	Scara4S();
	~Scara4S();
	virtual	void setRobotModelParameter();

	virtual ErrorID calcInverseKin_RPY(const Position_MCS_rad& posMCS,
									   const Position_ACS_rad& posLast, Position_ACS_rad& posACS) const;
};

