#include "Interpolation.h"
#include <iostream>

/*****************************************************************************/
/**
* @brief Sercos III - 位置控制模式下，笛卡尔坐标系下按直线插补，运行到绝对位置 unit: deg
* @param[in] targetPoint 目标位置，弧度
* @param[in] originPoint 起始位置(MCS)，弧度
* @param[in] originACS 起始位置(ACS)， 弧度
* @param[in] Ts 插补周期
* @param[in] maxVel 最大速度
* @param[in] maxAcc 最大加速度
* @param[in] maxDecel 最大减速度
* @param[in] maxJerk 最大加加速度
* @param[in] maxVelOri 最大角速度
* @param[in] maxAccOri 最大角加速度
* @param[in] maxDecelOri 最大角减速度
* @param[in] maxJerkOri 最大角加加速度
* @param[out] nAglSeqPtr [指针]n组轴的位置插补（关节空间）序列，n为轴数
* @param[out] pGrpTcp [可选][指针]轴组的位置插补（直角空间）序列
* @return 若成功，返回 RBTINTERF_NO_ERROR
* @attention 注意入口参数单位为弧度
* @note 运动距离如果太小，中间有个递归调用
* @todo 角速度限制
*/
/*****************************************************************************/
ErrorID Interpolation::LineInterp(const Position_MCS_rad &targetPoint,
	const Position_MCS_rad &originPoint, const Position_ACS_rad &originACS,
	double Ts, double maxVel, double maxAcc, double maxDecel,
	double maxJerk, double maxVelOri, double maxAccOri, double maxDecelOri,
	double maxJerkOri, int polynomiType, N_AxisSeqPtr &nAglSeqPtr,
	GrpTcpSeq *pGrpTcp/* = NULL*/)
{
	if ((targetPoint.rows() != 6) || (originPoint.rows() != 6))
		return PLCOPEN_MOVE_INSTRUCT_ARG_ERROR;

	ErrorID lRet(RBTINTERF_NO_ERROR);
	AXIS_REFS_INDEX n = pRobot->numAxes();
	AXIS_REFS_INDEX axisNum;
	Position targetACS(n);

	//	若有位置命令未执行完毕，等待各轴进入空闲状态
	//	MC_WaitIsFree();

	//计算路径长度
	double displacement = sqrt(pow((targetPoint[0] - originPoint[0]), 2) + pow(
		(targetPoint[1] - originPoint[1]), 2) + pow((targetPoint[2]
			- originPoint[2]), 2));

	std::deque<double> rlst;

	if (displacement > 0.2)
	{
		if (sqrt(maxAcc * displacement) < maxVel)
		{
			maxVel = sqrt(maxAcc * displacement);
			if (maxAcc / maxVel > 10)
			{
				maxVel = 10 * displacement;
				maxAcc = 100 * displacement;
			}
		}
	}
	else
	{
		if ((lRet = pRobot->calcInverseKin_RPY(targetPoint, originACS, targetACS)) != RBTINTERF_NO_ERROR)
		{
			return lRet;
		}
		else
		{
			targetACS = rad2deg(targetACS);
		}

		displacement = fabs(targetACS[0] - rad2deg(originACS[0]));
		axisNum = 0;
		for (AXIS_REFS_INDEX ai = 1; ai != n; ++ai) //2-n轴
		{
			if (displacement < fabs(targetACS[ai] - rad2deg(originACS[ai])))
			{
				displacement = fabs(targetACS[ai] - rad2deg(originACS[ai]));
				axisNum = ai;
			}
		}

		if (displacement < 0.1)
		{
			return PLCOPEN_NO_MOVE_IS_NEEDED;
		}
		else
		{
			//Decare_Para decare;
			//decare = pRobot->getCurrentlyDecare();
			//maxVel = maxVel / decare.maxvel*(pRobot->axes[axisNum]->paramLmt.maxVel);
			//maxAcc = maxAcc / decare.maxacc*(pRobot->axes[axisNum]->paramLmt.maxAcc);
			//maxDecel = (pRobot->axes[axisNum]->paramLmt.maxDecel);
			if (sqrt(maxAcc * displacement) < maxVel)
			{
				maxVel = sqrt(maxAcc * displacement);
			}
			if (maxAcc / maxVel > 5)
			{
				maxVel = 5 * displacement;
				maxAcc = 25 * displacement;
			}
		}
	}
	lRet = calcInterp_5_1_5(0, 1, Ts, maxVel / displacement, maxAcc / displacement,
		maxDecel / displacement, maxJerk / displacement, polynomiType, rlst);//displacement==times
	if (lRet != RBTINTERF_NO_ERROR)
	{
		std::cout << "error: calcInterp_5_1_5" << std::endl;
		return lRet;
	}
	nAglSeqPtr.resize(n);
	for (N_AxisSeqPtrIdx idx = 0; idx != n; ++idx)
	{
		nAglSeqPtr[idx].reset(new AglSeq());
	}
	//球面线性插补
	Matrix3 mOrigin = rpy2r(originPoint.tail(3));
	Matrix3 mTarget = rpy2r(targetPoint.tail(3));

	Vector3 pOrigin = originPoint.head(3);
	Vector3 pTarget = targetPoint.head(3);
	Vector3 pr;
	dmatrix Tr(4, 4);
	Position_MCS_rad posr(6);
	Position_ACS_rad posACS(n);
	Position_ACS_rad pLast(n);
	dQuaternion qr;
	dQuaternion qTarget(mTarget);
	dQuaternion qOrigin(mOrigin);

	for (std::deque<double>::iterator rit = rlst.begin(); rit != rlst.end(); ++rit)
	{
		double r = *rit;
		qr = qOrigin.slerp(r, qTarget);
		pr = pOrigin*(1 - r) + r*pTarget;
		Tr << qr.toRotationMatrix(), pr, dmatrix::Zero(1, 3), 1;

		posr = tr2MCS(Tr);

		//判断在直角坐标系下是否超限
		rectLimit = pRobot->RobotRangeLimit(posr);
		if (rectLimit != RANGE_NOOVERLIMIT)
		{
			std::cout << "error: RobotRangeLimit!" << std::endl;
			return rectLimit;
		}

		if (rit == rlst.begin())
		{
			pLast = originACS;
		}
		lRet = pRobot->calcInverseKin_RPY(posr, pLast, posACS);
		speedLimit = pRobot->RobotSpeedLimit(pLast, posACS, Ts, n);
		if (speedLimit != SPEED_NOOVERLIMIT)
		{
			std::cout << "error: RobotSpeedLimit!" << std::endl;
			return speedLimit;
		}
		if (lRet != RBTINTERF_NO_ERROR)
		{
			if (pGrpTcp)
				pGrpTcp->clear();
			std::cout << "error: calcInverseKin_RPY!" << std::endl;
			return lRet;
		}
		else
		{
			if (pGrpTcp)
				pGrpTcp->push_back(posr);//记录tcp
			for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) //1-n轴
			{
				nAglSeqPtr[ai]->push_back(rad2deg(posACS[ai]));
			}
			pLast = posACS;
		}
	}
	return RBTINTERF_NO_ERROR;
}
