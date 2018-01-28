#include "Robot.h"
#include <iostream>

extern N_AxisSeqPtr Snap_AglSeqPtr;
USER_DAT* Robot::pData = nullptr;


Robot::Robot(std::shared_ptr<SerialRobotModel> pR) : pRobotModel(pR)
{
	pInterp.reset(new Interpolation(pR));
	robotType = 1;        // Note: 机器人类型标号，1表示6轴关机机器人；若要扩展其它类型，请使用枚举
	Ts = CYCLETIME/1000.0;
	totalAxes = TOTAL_AXES;
	for (int i = 0; i < TOTAL_AXES; ++i)
	{
		direction[i] = 1;
		encoderResolution[i] = 17;
		singleTurnEncoder[i] = 0;
		reducRatio[i] = 1;
	}
	robotRunSingle = false;
	robotRunGroup = false;
	int aa = pRobotModel->numAxes();
}


Robot::~Robot()
{
}

/**
* @brief 创建机器人对象
*/
Robot* Robot::getNewRobot()
{

	std::shared_ptr<General6S> _pRobotModel(new General6S());
	_pRobotModel->setRobotModelParameter();
	return (new Robot(_pRobotModel));
}
/**
* @brief 伺服下电
*/
void Robot::servoOff()
{
	//RTN_ERR     ret;
	//I32_T actualPos = 0;
	//for (int i = 0; i < TOTAL_AXES; i++)
	//{
	//	ret = NEC_CoE402GetActualPosition(pData->axes[i], &actualPos);
	//	ret = NEC_CoE402SetTargetPosition(pData->axes[i], actualPos);
	//}
	//Sleep(100);

	//for (int i = 0; i < TOTAL_AXES; i++)
	//{
	//	ret = NEC_CoE402SetCtrlWord(pData->axes[i], 0);
	//	if (ret != ECERR_SUCCESS) { printf("[%d] NEC_CoE402SetCtrlWord 0 error %d \n", i, ret); }
	//}
	//Sleep(100);
}

/**
* @brief 伺服上电
*/
void Robot::servoOn()
{
	//RTN_ERR     ret;
	//I32_T actualPos = 0;
	//for (int i = 0; i < TOTAL_AXES; i++)
	//{
	//	ret = NEC_CoE402GetActualPosition(pData->axes[i], &actualPos);
	//	ret = NEC_CoE402SetTargetPosition(pData->axes[i], actualPos);
	//}
	//Sleep(10);
	//for (int i = 0; i < TOTAL_AXES; i++)
	//{
	//	ret = NEC_CoE402SetCtrlWord(pData->axes[i], 6);
	//	if (ret != ECERR_SUCCESS) { printf("[%d] NEC_CoE402SetCtrlWord 6 error %d \n", i, ret); }
	//}
	//Sleep(10);
	//for (int i = 0; i < TOTAL_AXES; i++)
	//{
	//	ret = NEC_CoE402SetCtrlWord(pData->axes[i], 7);
	//	if (ret != ECERR_SUCCESS) { printf("[%d] NEC_CoE402SetCtrlWord 7 error %d \n", i, ret); }
	//}
	//Sleep(10);
	//for (int i = 0; i < TOTAL_AXES; i++)
	//{
	//	ret = NEC_CoE402SetCtrlWord(pData->axes[i], 15);
	//	if (ret != ECERR_SUCCESS) { printf("[%d] NEC_CoE402SetCtrlWord 15 error %d \n", i, ret); }
	//}
	//Sleep(10);
}

/**
* @brief 设置机器人参数
*/
void Robot::setRobotParam()
{
	getReducRatio();
	getDirection();
	getEncoderResolution();
	getCoupleCoe();
}

/**
* @brief 设置机器人零点位置
*/
void Robot::encoderReset(unsigned int servoNum)
{
	//RTN_ERR     ret;
	//I32_T       actualPos = 0;
	//double		pos;
	//if (servoNum == 0)
	//{
	//	for (size_t i = 0; i < totalAxes; ++i)
	//	{
	//		ret = NEC_CoE402GetActualPosition(pData->axes[i], &actualPos);
	//		if (ret == 0)
	//		{
	//			pos = pulToDeg(actualPos, encoderResolution[i]);
	//			singleTurnEncoder[i] = pos;
	//		}
	//	}
	//}
	//else
	//{
	//	ret = NEC_CoE402GetActualPosition(pData->axes[servoNum - 1], &actualPos);
	//	if (ret == 0)
	//	{
	//		pos = pulToDeg(actualPos, encoderResolution[servoNum - 1]);
	//		singleTurnEncoder[servoNum - 1] = pos;
	//	}
	//}
}

/**
* @brief 单轴运动时，机构角度转换为电机角度
*/
int Robot::targetPosAssign(int& Pos,int axisIndex, N_AxisSeqPtr& SeqPtr)
{
	double angle = 0;
	if (!(SeqPtr[axisIndex]->empty()))
	{
		SeqPtr[axisIndex]->front() = SeqPtr[axisIndex]->front() * direction[axisIndex];
		angle = SeqPtr[axisIndex]->front() * reducRatio[axisIndex];
		angle += singleTurnEncoder[axisIndex];
		Pos = degToPul(angle, encoderResolution[axisIndex]);
	}
	else
	{
		printf("targetPosAssign: SeqPtr[axisIndex]->empty()\n");
		robotRunSingle = false;
		return -1;
	}
	return 0;
}

/**
* @brief 轴组运动时，机构角度转换为电机角度
*/
int Robot::targetPosAssignGroup(double Pos[], N_AxisSeqPtr& SeqPtr)
{
	for (size_t i = 0; i < totalAxes; i++)
	{
		if (!(SeqPtr[i]->empty()))
		{
			SeqPtr[i]->front() = SeqPtr[i]->front() * direction[i];
		}
		else
		{
			printf("targetPosAssignGroup: SeqPtr[i]->empty()\n");
			robotRunGroup = false;
			return -1;
		}
	}

	if (robotType == 2) //SCARA
	{
		SeqPtr[totalAxes - 2]->front() = SeqPtr[totalAxes - 2]->front() - SeqPtr[totalAxes - 1]->front() * coupleCoe.Couple_Coe_5_6;
	}
	else //AR6
	{
		SeqPtr[2]->front() = SeqPtr[2]->front() - SeqPtr[1]->front() * coupleCoe.Couple_Coe_2_3;
		SeqPtr[totalAxes - 2]->front() = SeqPtr[totalAxes - 2]->front() - SeqPtr[totalAxes - 3]->front() * coupleCoe.Couple_Coe_4_5;
		SeqPtr[totalAxes - 1]->front() = SeqPtr[totalAxes - 1]->front() - SeqPtr[totalAxes - 2]->front() * coupleCoe.Couple_Coe_5_6
			- SeqPtr[totalAxes - 3]->front() * coupleCoe.Couple_Coe_4_6;
	}

	for (size_t i = 0; i < totalAxes; i++)
	{
		Pos[i] = SeqPtr[i]->front() * reducRatio[i];
		Pos[i] += singleTurnEncoder[i];
	}
	return 0;
}

/**
* @brief 读取机构当前角度
*/
void Robot::readNowPos(Position &pos)
{
//	RTN_ERR     ret;
//	I32_T       actualPos = 0;

	//for (size_t i = 0; i < totalAxes; i++)
	//{
	//	ret = NEC_CoE402GetActualPosition(pData->axes[i], &actualPos);
	//	pos[i] = pulToDeg(actualPos, encoderResolution[i]);
	//	pos[i] -= singleTurnEncoder[i];
	//	pos[i] /= reducRatio[i];
	//}

	//if (robotType == 2)
	//{
	//	pos[totalAxes - 2] = pos[totalAxes - 2]
	//		+ pos[totalAxes - 1] * coupleCoe.Couple_Coe_5_6;
	//}
	//else
	//{
	//	pos[2] = pos[2] + pos[1] * coupleCoe.Couple_Coe_2_3;
	//	pos[totalAxes - 1] = pos[totalAxes - 1]
	//		+ pos[totalAxes - 2] * coupleCoe.Couple_Coe_5_6
	//		+ pos[totalAxes - 3] * coupleCoe.Couple_Coe_4_6;
	//	pos[totalAxes - 2] = pos[totalAxes - 2]
	//		+ pos[totalAxes - 3] * coupleCoe.Couple_Coe_4_5;
	//}

	//for (size_t i = 0; i < totalAxes; i++)
	//{
	//	pos[i] = pos[i] * direction[i];
	//}
}

/**
* @brief 单轴点到点运动
*/
int Robot::mBt_Ptp_Single()
{
	Position targetPoint(totalAxes);
	Position currentPoint(TOTAL_AXES);
	readNowPos(currentPoint);

	size_t lRet;
	double pos = 0;
	int polynomiType = 0;
	double velPerc = pData->velocity;
	double accPerc = pData->acceleration;
	size_t index = pData->cmdAxisIndex - 1;

	targetPoint = currentPoint;
	targetPoint[index] = pData->targetPos[index];

	lRet = pInterp->JointInterp(targetPoint, currentPoint, Ts, velPerc, accPerc, polynomiType, Snap_AglSeqPtr);
	if (lRet != RBTINTERF_NO_ERROR)
	{
		printf("pInterp->JointInterp error!\n");
		Snap_AglSeqPtr.resize(TOTAL_AXES);
		for (N_AxisSeqPtrIdx idx = 0; idx != TOTAL_AXES; ++idx)
		{
			Snap_AglSeqPtr[idx].reset(new AglSeq());
		}
		return -1;
	}
	//printf("mBt_Ptp: Snap_AglSeqPtr[index]->size() = %d\n", Snap_AglSeqPtr[index]->size());
	return 0;
}

/**
* @brief 轴组点到点运动
*/
int Robot::mBt_Ptp_Group(COORD_SYS coord)
{
	Position targetPoint(totalAxes);
	Position currentPoint(TOTAL_AXES);
	readNowPos(currentPoint);

	size_t lRet;
	double displacement = 0;
	double pos = 0;
	int polynomiType = 0;

	double velPerc = pData->velocity;
	double accPerc = pData->acceleration;

	if (coord == ACS)
	{
		for (size_t i = 0; i < totalAxes; ++i)
		{
			targetPoint[i] = pData->targetPos[i];
		}
	}
	else if (coord == PCS)
	{
		size_t cmdIndex = pData->cmdAxisIndex;
		if (cmdIndex == 0)
		{
			Position_MCS_rad posMCS(6);
			Position_ACS_rad posACS(TOTAL_AXES);
			Position_ACS_rad posLast(TOTAL_AXES);
			for (size_t i = 0; i < 6; ++i)
			{
				posMCS[i] = pData->targetPosPCS[i];
			}
			posMCS = posMCS_deg2rad(posMCS);
			posLast = deg2rad(currentPoint);

			pRobotModel->calcInverseKin_RPY(posMCS, posLast, posACS);
			targetPoint = rad2deg(posACS);
		}
		else
		{
			Position_MCS_rad posMCS(6);
			Position_ACS_rad posACS(TOTAL_AXES);
			Position_ACS_rad currentPointACS(TOTAL_AXES);
			Position_MCS_rad currentPointMCS(6);
			Position_MCS_rad targetPointMCS(6);

			currentPointACS = deg2rad(currentPoint);
			pRobotModel->calcForwardKin(currentPointACS, currentPointMCS);
			targetPointMCS = posMCS_rad2deg(currentPointMCS);
			targetPointMCS[cmdIndex-1] = pData->targetPosPCS[cmdIndex-1];

			posMCS = posMCS_deg2rad(targetPointMCS);

			pRobotModel->calcInverseKin_RPY(posMCS, currentPointACS, posACS);
			targetPoint = rad2deg(posACS);
		}
	}
	else
	{
		printf("error: Coord is not support!\n");
		return -1;
	}

	lRet = pInterp->JointInterp(targetPoint, currentPoint, Ts, velPerc, accPerc, polynomiType, Snap_AglSeqPtr);
	if (lRet != RBTINTERF_NO_ERROR)
	{
		printf("pInterp->JointInterp error!\n");
		Snap_AglSeqPtr.resize(TOTAL_AXES);
		for (N_AxisSeqPtrIdx idx = 0; idx != TOTAL_AXES; ++idx)
		{
			Snap_AglSeqPtr[idx].reset(new AglSeq());
		}
		return -1;
	}
	//printf("mBt_Ptp: Snap_AglSeqPtr.size() = %d\n", Snap_AglSeqPtr[0]->size());

	return 0;
}

/**
* @brief 轴组直线运动
*/
int Robot::mBt_MoveLine_Group()
{
	size_t lRet;
	double maxVel = pData->velocity;
	double maxAcc = pData->acceleration;
	double maxDecel = maxAcc;
	double maxJerk = 10;
	int polynomiType = 0;
	Position_MCS_rad targetPointMCS(6);
	Position_MCS_rad currentPointMCS(6);
	Position_ACS_rad currentPointACS(TOTAL_AXES);

	readNowPos(currentPointACS);
	currentPointACS = deg2rad(currentPointACS);

	pRobotModel->calcForwardKin(currentPointACS, currentPointMCS);

	size_t cmdIndex = pData->cmdAxisIndex;
	if (cmdIndex == 0)
	{
		for (size_t i = 0; i < 6; ++i)
		{
			targetPointMCS[i] = pData->targetPosPCS[i];
		}
	}
	else
	{
		targetPointMCS = posMCS_rad2deg(currentPointMCS);
		targetPointMCS[cmdIndex-1] = pData->targetPosPCS[cmdIndex-1];
	}

	targetPointMCS = posMCS_deg2rad(targetPointMCS);

	lRet = pInterp->LineInterp(targetPointMCS, currentPointMCS, currentPointACS, Ts, maxVel, maxAcc, maxDecel, maxJerk,0,0,0,0,polynomiType, Snap_AglSeqPtr);
	if (lRet != RBTINTERF_NO_ERROR)
	{
		printf("pInterp->LineInterp error! lRet=%d\n",(int)lRet);
		Snap_AglSeqPtr.resize(TOTAL_AXES);
		for (N_AxisSeqPtrIdx idx = 0; idx != TOTAL_AXES; ++idx)
		{
			Snap_AglSeqPtr[idx].reset(new AglSeq());
		}
		return -1;
	}
	//printf("mBt_MoveLine: Snap_AglSeqPtr.size() = %d\n", Snap_AglSeqPtr[0]->size());

	return 0;
}

/**
* @brief 轴组圆弧运动
*/
int Robot::mBt_MoveArc_Group()
{
	size_t lRet;
	double maxVel = pData->velocity;
	double maxAcc = pData->acceleration;
	double maxDecel = maxAcc;
	double maxJerk = 10;
	int polynomiType = 0;
	Position_MCS_rad middlePointMCS(6);
	Position_MCS_rad targetPointMCS(6);
	Position_MCS_rad currentPointMCS(6);
	Position_ACS_rad currentPointACS(TOTAL_AXES);

	readNowPos(currentPointACS);
	currentPointACS = deg2rad(currentPointACS);

	pRobotModel->calcForwardKin(currentPointACS, currentPointMCS);

	for (size_t i = 0; i < 6; ++i)
	{
		targetPointMCS[i] = pData->targetPosPCS[i];
		middlePointMCS[i] = pData->middlePosPCS[i];
	}

	targetPointMCS = posMCS_deg2rad(targetPointMCS);
	middlePointMCS = posMCS_deg2rad(middlePointMCS);
	bool isFullCircle = false;
	if (pData->fullCircleFlag == 1)
	{
		isFullCircle = true;
	}
	lRet = pInterp->ArcInterp(middlePointMCS, targetPointMCS, currentPointMCS, currentPointACS, Ts, maxVel, maxAcc, maxDecel, maxJerk, 0, 0, 0, 0, polynomiType, isFullCircle, Snap_AglSeqPtr);
	if (lRet != RBTINTERF_NO_ERROR)
	{
		printf("pInterp->ArcInterp error! lRet=%d\n",(int) lRet);
		Snap_AglSeqPtr.resize(TOTAL_AXES);
		for (N_AxisSeqPtrIdx idx = 0; idx != TOTAL_AXES; ++idx)
		{
			Snap_AglSeqPtr[idx].reset(new AglSeq());
		}
		return -1;
	}
	//printf("mBt_MoveLine: Snap_AglSeqPtr.size() = %d\n", Snap_AglSeqPtr[0]->size());

	return 0;
}

/**
* @brief 获取各个电机轴的减速比
*/
void Robot::getReducRatio() 
{
	for (AXIS_REFS_INDEX ai = 0; ai < totalAxes; ++ai) 
	{
		reducRatio[ai] = pRobotModel->axes[ai]->reducRatio;
	}
}

/**
* @brief 获取各个电机轴的方向
*/
void Robot::getDirection() 
{
	for (AXIS_REFS_INDEX ai = 0; ai < totalAxes; ++ai) 
	{
		direction[ai] = pRobotModel->axes[ai]->direction;
	}
}

/**
* @brief 获取各个电机轴的编码器位数
*/
void Robot::getEncoderResolution() 
{
	for (AXIS_REFS_INDEX ai = 0; ai < totalAxes; ++ai) 
	{
		encoderResolution[ai] = pRobotModel->axes[ai]->encoderResolution;
	}
}

/**
* @brief 获取耦合比
*/
void Robot::getCoupleCoe() 
{
	coupleCoe = pRobotModel->_coupleCoe;
}

/**
* @brief 将当前位置的角度和空间位姿传递给C#上位机显示
*/
void Robot::printCurPos()
{
	Position_ACS_deg actualPosACS(TOTAL_AXES);
	
	readNowPos(actualPosACS);
	for (int i = 0; i < TOTAL_AXES; i++)
	{
		pData->actualPos[i] = actualPosACS[i];
	}
	Position_ACS_rad positionACS(totalAxes);
	Position_ACS_rad positionMCS(totalAxes);
	positionACS = deg2rad(actualPosACS);
	pRobotModel->calcForwardKin(positionACS, positionMCS);
	positionMCS = posMCS_rad2deg(positionMCS);
	for (int i = 0; i < TOTAL_AXES; i++)
	{
		pData->actualPosPCS[i] = positionMCS[i];
	}
}
