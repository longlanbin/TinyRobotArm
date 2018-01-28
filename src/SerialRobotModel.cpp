#include "SerialRobotModel.h"
#include <iostream>

size_t jointLimitNum = 0;
ErrorID rectLimit = RANGE_NOOVERLIMIT;
ErrorID speedLimit = SPEED_NOOVERLIMIT;

SerialRobotModel::SerialRobotModel():transformationMatrix(dmatrix::Identity(4, 4)), baseMatrix(dmatrix::Identity(4, 4)), toolMatrix(Matrix4::Identity(4, 4)), ODD_HANDLE_MARGIN(0.001), ODD_MARGIN(deg2rad(1))
{
	printf("SerialRobotModel::construct!\n");

}


SerialRobotModel::~SerialRobotModel()
{
	printf("SerialRobotModel::destruct!\n");

}



/**
* @brief 机器人正解
*/
ErrorID SerialRobotModel::calcTransMatrix(const Position_ACS_rad& posACS, dmatrix& transMatrix)
{
	if ((static_cast<std::deque<AXIS_REF>::size_type>(posACS.rows())
		!= axes.size()) || (transMatrix.rows() != 4) || (transMatrix.cols()
			!= 4))
	{
		return FORWARD_KIN_TRANS_ARG_ERROR;
	}

	Position_ACS_rad q(posACS);
	dmatrix currentMatrix(dmatrix::Identity(4, 4));

	// For each joint in this model
	for (std::deque<AXIS_REF>::size_type sz = 0; sz<axes.size(); ++sz)
	{
		// Compute the joint using forward kinematics
		if (axes[sz])
		{
			axes[sz]->DHJ.Parameters.Theta = q(sz) + axes[sz]->Offset;// theta = posACS + offset
			currentMatrix = axes[sz]->DHJ.Compute(currentMatrix);
		}
		else
		{
			std::cerr << "DHJ can not be null.";
			return PLCOPEN_NULL_POINTER;
		}
	}
	// Update the model transformation matrix
	transMatrix = currentMatrix * toolMatrix;
	return RBTINTERF_NO_ERROR;
}

/**
* @brief 机器人正解
*/
ErrorID SerialRobotModel::calcForwardKin(const Position_ACS_rad& posACS, Position_MCS_rad& posMCS)
{
	printf("SerialRobotModel::calcForwardKin\n");

	ErrorID lRet;
	if ((static_cast<std::deque<AXIS_REF>::size_type>(posACS.rows())!= axes.size()) || (posMCS.rows() != 6))
	{
		return FORWARD_KIN_ARG_ERROR;
	}
	// Update the model transformation matrix
	lRet = calcTransMatrix(posACS, transformationMatrix);

	// tcp
	posMCS << transformationMatrix.topRightCorner(3, 1),
		tr2rpy(transformationMatrix);
	return lRet;
}

//当前角度、解一、解二
double SerialRobotModel::calcRealAngle(double curAng, double candidateAng1,
	double candidateAng2) const
{
	double curAngCopy(curAng);
	double* allAng[3] =
	{ &candidateAng1, &candidateAng2, &curAngCopy };
	for (int i = 0; i < 3; ++i)
	{
		while (*allAng[i]> M_PI)
		{
			*allAng[i] -= 2 * M_PI;
		}
		while (*allAng[i] < -M_PI)
		{
			*allAng[i] += 2 * M_PI;
		}
	}
	double gap[2] =
	{ 0, 0 };
	bool dir[2] =
	{ true, true };
	for (int i = 0; i < 2; ++i)
	{
		if (*allAng[i] >= curAngCopy)/*=*/
		{
			gap[i] = *allAng[i] - curAngCopy;
			dir[i] = true;
		}
		else
		{
			gap[i] = curAngCopy - *allAng[i];
			dir[i] = false;
		}
		if (gap[i]> M_PI)
		{
			gap[i] = 2 * M_PI - gap[i];
			dir[i] = !dir[i];
		}
	}

	if (gap[0] <= gap[1])/*=*/
	{
		return dir[0] ? curAng + gap[0] : curAng - gap[0];
	}
	else
	{
		return dir[1] ? curAng + gap[1] : curAng - gap[1];
	}
}

//is_Ntimes_90deg_Odd根据实际判断，因为处理过程中考虑了offset
bool SerialRobotModel::is_Near_Odd(double theta_rad, double agl_offset_rad,
	bool is_Ntimes_90deg_Odd) const
{
	//比较的是实际的角度
	double agl_rad = theta_rad - agl_offset_rad;

	//-pi~pi
	if (is_Ntimes_90deg_Odd)
	{
		//-pi/2~pi/2
		while (agl_rad> M_PI / 2)
		{
			agl_rad -= M_PI;
		}
		while (agl_rad < -M_PI / 2)
		{
			agl_rad += M_PI;
		}
		if ((fabs(agl_rad - M_PI / 2)<ODD_MARGIN) || (fabs(agl_rad + M_PI / 2)
			<ODD_MARGIN))
		{
			// DEBUG("ODD %f\r\n", rad2deg(agl_rad));
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		//0~pi
		while (agl_rad> M_PI)
		{
			agl_rad -= M_PI;
		}
		while (agl_rad < 0)
		{
			agl_rad += M_PI;
		}
		if ((fabs(agl_rad)<ODD_MARGIN) || (fabs(agl_rad - M_PI)<ODD_MARGIN))
		{
			//			DEBUG("ODD %f\r\n", rad2deg(agl_rad));
			return true;
		}
		else
		{
			return false;
		}
	}
}



/**
* @brief 判断机器人笛卡尔空间是否超限
*/
ErrorID SerialRobotModel::RobotRangeLimit(const Position_MCS_rad& posMCS)
{
	if (posMCS[0]<range_limit.minX || posMCS[0]>range_limit.maxX)
	{
		return RANGE_OVERLIMIT_X;
	}
	if (posMCS[1]<range_limit.minY || posMCS[1]>range_limit.maxY)
	{
		return RANGE_OVERLIMIT_Y;
	}
	if (posMCS[2]<range_limit.minZ || posMCS[2]>range_limit.maxZ)
	{
		return RANGE_OVERLIMIT_Z;
	}
	return RANGE_NOOVERLIMIT;
}

/**
* @brief 判断机器人关节速度是否超限
*/
ErrorID SerialRobotModel::RobotSpeedLimit(const Position_ACS_rad& pos, const Position_ACS_rad& posACS, double Ts, int num)
{
	for (int i = 0; i < num; i++)
	{
		double speed = rad2deg(pos[i] - posACS[i]) / Ts;
		if (fabs(speed) > axes[i]->paramLmt.maxVel)
		{
			return SPEED_OVERLIMIT + i;
		}
	}
	return SPEED_NOOVERLIMIT;
}