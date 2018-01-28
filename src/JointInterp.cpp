#include "Interpolation.h"
#include <iostream>
/*****************************************************************************/
/**
* @brief Sercos III - 位置控制模式下，用于运行到绝对位置（不考虑轨迹） unit: deg
* @param[in] targetPoint 目标位置，角度
* @param[in] originPoint 起始位置，角度
* @param[in] Ts 插补周期
* @param[in] velPerc 速度百分比（0.5-95.0，例如：设定以50%的最大速度运行，赋值50）
* @param[in] accPerc 加速度百分比（0.5-95.0）
* @param[out] nAglSeqPtr [指针]n组轴的位置插补（关节空间）序列，n为轴数
* @return 若成功，返回 RBTINTERF_NO_ERROR
* @attention 注意入口参数单位为角度
* @note 运动角度如果太小，中间有个递归调用
*/
/*****************************************************************************/
ErrorID Interpolation::JointInterp(const Position &targetPoint,
									const Position_ACS_deg &originPoint, double Ts, double velPerc,
									double accPerc, int polynomiType, N_AxisSeqPtr &nAglSeqPtr) //输入的是角度值
{
	AXIS_REFS_INDEX n = pRobot->numAxes();

	ErrorID lRet(RBTINTERF_NO_ERROR);
	Position_ACS_rad posacs(n);
	Position_MCS_rad posmcs(6);	//正解求直角坐标位置

	if ((n != AXIS_REFS_INDEX(targetPoint.rows())) || (n
		!= AXIS_REFS_INDEX(originPoint.rows())))
		return JOINT_INTERPLATE_ARG_ERROR;


	//	若有位置命令未执行完毕，等待各轴进入空闲状态
	//	MC_WaitIsFree();

	Position_ACS_deg posOrigin(originPoint);
	Position_ACS_deg posTarget(targetPoint);

	double jointLimit[32];
	for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai)
	{
		jointLimit[2 * ai] = pRobot->axes[ai]->paramLmt.posSWLimit;
		jointLimit[2 * ai + 1] = pRobot->axes[ai]->paramLmt.negSWLimit;
	}

	//计算各轴角位移偏移量
	Position_ACS_deg posOffset(n);
	posOffset = posTarget - posOrigin;

	dvector t(n); //若全程匀速，各轴时间
	double T = 0; //关节运动最长的那个时间
	dvector tA(n); //各轴加速时间
	double tAcc = 0; //关节加速过程最长的那个时间

_AdjustVel:
	//第一步：若全程视作匀速运动，确定各轴最大运动时间T
	velPerc = velPerc / 100;
	accPerc = accPerc / 100;

	for (AXIS_REFS_INDEX ai = 0; ai<n; ++ai) /*1-n轴*/
	{
		t[ai] = fabs(posOffset[ai]) / (velPerc*(pRobot->axes[ai]->paramLmt.maxVel));
	}
	//比较时间大小，取最大值赋值给T
	T = t[0];
	for (AXIS_REFS_INDEX ai = 1; ai != n; ++ai) //2-n轴
	{
		if (T<t[ai])
		{
			T = t[ai];
		}
	}
	if (!T)
	{
		return PLCOPEN_NO_MOVE_IS_NEEDED;
	}

	//第二步：确定加（减）速段时间tAcc
	for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n轴*/
	{
		tA[ai] = (velPerc * (pRobot->axes[ai]->paramLmt.maxVel))
			/ (accPerc * (pRobot->axes[ai]->paramLmt.maxAcc));
	}
	tAcc = tA[0];
	for (AXIS_REFS_INDEX ai = 1; ai != n; ++ai) //2-n轴
	{
		if (tAcc > tA[ai])
		{
			tAcc = tA[ai];
		}
	}

	for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) //1-n轴
	{
		if ((tAcc < tA[ai]) && (fabs(posOffset[ai]) > 0.1))
		{
			tAcc = tA[ai];
		}
	}
	//DEBUG("\ntAcc = %f;",tAcc);
	if (!tAcc)
	{
		return PLCOPEN_MOVE_INSTRUCT_CALC_ERROR;
	}

	//第三步：确定匀速段时间tConst
	double tf = T + tAcc; //最终整个过程运行时间为tf
	double tConst = tf - 2 * tAcc;

	if (tConst<0)
	{
		//todo: 不采用速度减半方法，而是改为只有变速段，没有匀速段处理
		//速度减半
		velPerc = velPerc * 98;
		accPerc = accPerc * 98;
		goto _AdjustVel;
		//return PLCOPEN_MOVE_INSTRUCT_NOT_APPLICABLE;
	}

	//第四步：确定加速段、匀速段、减速段的插补步数（N1,N2,N3），并确定最终的运行时间
	size_t N1 = static_cast<size_t>(tAcc / Ts);
	size_t N2 = static_cast<size_t>(tConst / Ts);
	size_t N3 = N1;

	std::cout << "\n JointInterp: N1=" << N1 << " " << "N2=" << N2 << " " << "N3=" << N3 << " " << std::endl;

	tAcc = N1 * Ts;
	tConst = N2 * Ts;
	T = tConst + tAcc;

	//第五步：计算各轴最终运行时的平均速度(也即匀速段的速度velConst，注意这个速度与设定的速度可能不一样)
	dvector velConst(n);//有正负号
	for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n轴*/
	{
		velConst[ai] = posOffset[ai] / T;
	}

	//第六步：轨迹插补，其中：用五次多项式对加速段和减速段进行插补
	nAglSeqPtr.resize(n);
	for (N_AxisSeqPtrIdx idx = 0; idx != n; ++idx)
	{
		nAglSeqPtr[idx].reset(new AglSeq());
	}
	dvector posInterp(n);//用于暂存各轴某周期的插补点位置
	Position_ACS_rad pLast(n);

	pLast = deg2rad(originPoint);

	if (polynomiType == 0)
	{
		//(1)加速段插补 [ 第 0 ~ N1 点]
		//计算五次多项式系数
		dvector accPosEnd(n);//各轴加速段的结束点位置
		std::deque<QuinticCoe> coeQuintic(n);//各个轴的五次多项式

		for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n轴*/
		{
			accPosEnd[ai] = posOrigin[ai] + velConst[ai] * tAcc / 2;
			getQuinticCoe(posOrigin[ai], accPosEnd[ai], 0, velConst[ai], 0, 0,
				tAcc, coeQuintic[ai]);
		}

		//生成链表
		for (size_t i = 0; i != (N1 + 1); ++i) //i用于记录加速段插补点的序号
		{
			double t = i*Ts; //时间t
			for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n轴*/
			{
				QuinticPolynomi(t, coeQuintic[ai], &(posInterp[ai]));
				//std::cout << t << "posInterp[]=" << posInterp[ai] << "coeQuintic[ai]=" << coeQuintic[ai].a0 << std::endl;
				if ((posInterp[ai] > jointLimit[2 * ai]) || (posInterp[ai] < jointLimit[2 * ai + 1]))
				{
					jointLimitNum = ai + 1;
					std::cout << "jointLimitNum=" << jointLimitNum << "," << posInterp[ai] << "," << jointLimit[2 * ai] << "," << jointLimit[2 * ai + 1] << std::endl;
					return INVERSE_KIN_NOT_REACHABLE;
				}
				posacs[ai] = deg2rad(posInterp[ai]);
			}
			if ((lRet = pRobot->calcForwardKin(posacs, posmcs))
				!= RBTINTERF_NO_ERROR)//读回弧度值
			{
				return lRet;
			}
			rectLimit = pRobot->RobotRangeLimit(posmcs);	//计算是否超限
			if (rectLimit != RANGE_NOOVERLIMIT)
			{
				return rectLimit;
			}
			speedLimit = pRobot->RobotSpeedLimit(pLast, posacs, Ts, n);
			if (speedLimit != SPEED_NOOVERLIMIT)
			{
				std::cout << "speedLimit" << std::endl;
				return speedLimit;
			}
			for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n轴*/
			{
				nAglSeqPtr[ai]->push_back(posInterp[ai]);
			}
			pLast = deg2rad(posInterp);
		}
		//(2)匀速段插补 [ 第 (N1+1) ~ (N1+N2) 点]
		//生成链表
		for (size_t j = 1; j != (N2 + 1); ++j) //j用于记录匀速段插补点的序号
		{
			for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n轴*/
			{
				posInterp[ai] += velConst[ai] * Ts;
				if ((posInterp[ai] > jointLimit[2 * ai]) || (posInterp[ai] < jointLimit[2 * ai + 1]))
				{
					jointLimitNum = ai + 1;
					std::cout << "jointLimitNum=" << jointLimitNum << "," << posInterp[ai] << "," << jointLimit[2 * ai] << "," << jointLimit[2 * ai + 1] << std::endl;
					return INVERSE_KIN_NOT_REACHABLE;
				}
				posacs[ai] = deg2rad(posInterp[ai]);
			}
			if ((lRet = pRobot->calcForwardKin(posacs, posmcs))
				!= RBTINTERF_NO_ERROR)//读回弧度值
			{
				return lRet;
			}
			rectLimit = pRobot->RobotRangeLimit(posmcs);	//计算是否超限
			if (rectLimit != RANGE_NOOVERLIMIT)
			{
				return rectLimit;
			}
			speedLimit = pRobot->RobotSpeedLimit(pLast, posacs, Ts, n);
			if (speedLimit != SPEED_NOOVERLIMIT)
			{
				std::cout << "speedLimit" << std::endl;
				return speedLimit;
			}
			for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n轴*/
			{
				nAglSeqPtr[ai]->push_back(posInterp[ai]);
			}
			pLast = deg2rad(posInterp);
		}

		//(3)减速段插补 [ 第 (N1+N2+1) ~ (N1+N2+N3) 点]
		//计算五次多项式系数
		for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n轴*/
		{
			getQuinticCoe(posInterp[ai], posTarget[ai], velConst[ai], 0, 0, 0,
				tAcc, coeQuintic[ai]);
		}
		//生成链表
		for (size_t k = 1; k != (N3 + 1); ++k) //k用于记录减速段插补点的序号
		{
			double t = k*Ts; //时间t
			for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n轴*/
			{
				QuinticPolynomi(t, coeQuintic[ai], &(posInterp[ai]));
				if ((posInterp[ai] > jointLimit[2 * ai]) || (posInterp[ai] < jointLimit[2 * ai + 1]))
				{
					jointLimitNum = ai + 1;
					return INVERSE_KIN_NOT_REACHABLE;
				}
				posacs[ai] = deg2rad(posInterp[ai]);
			}
			if ((lRet = pRobot->calcForwardKin(posacs, posmcs))
				!= RBTINTERF_NO_ERROR)//读回弧度值
			{
				return lRet;
			}
			rectLimit = pRobot->RobotRangeLimit(posmcs);	//计算是否超限
			if (rectLimit != RANGE_NOOVERLIMIT)
			{
				return rectLimit;
			}
			speedLimit = pRobot->RobotSpeedLimit(pLast, posacs, Ts, n);
			if (speedLimit != SPEED_NOOVERLIMIT)
			{
				return speedLimit;
			}
			for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n轴*/
			{
				nAglSeqPtr[ai]->push_back(posInterp[ai]);
			}
			pLast = deg2rad(posInterp);
		}
	}
	else if (polynomiType == 1)
	{
		//(1)加速段插补 [ 第 0 ~ N1 点]
		//计算二次多项式系数
		dvector accPosEnd(n);//各轴加速段的结束点位置
		std::deque<trapezoidalCoe> coeTrapezoidal(n);//各个轴的二次多项式
		for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n轴*/
		{
			accPosEnd[ai] = posOrigin[ai] + velConst[ai] * tAcc / 2;
			getTrapezoidalCoe(posOrigin[ai], accPosEnd[ai], 0, velConst[ai],
				tAcc, coeTrapezoidal[ai]);
		}
		//生成链表
		for (size_t i = 0; i != (N1 + 1); ++i) //i用于记录加速段插补点的序号
		{
			double t = i*Ts; //时间t
			for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n轴*/
			{
				trapezoidalPolynomi(t, coeTrapezoidal[ai], &(posInterp[ai]));
				posacs[ai] = deg2rad(posInterp[ai]);
			}
			if ((lRet = pRobot->calcForwardKin(posacs, posmcs))
				!= RBTINTERF_NO_ERROR)//读回弧度值
			{
				return lRet;
			}
			rectLimit = pRobot->RobotRangeLimit(posmcs);	//计算是否超限
			if (rectLimit != RANGE_NOOVERLIMIT)
			{
				return rectLimit;
			}
			speedLimit = pRobot->RobotSpeedLimit(pLast, posacs, Ts, n);
			if (speedLimit != SPEED_NOOVERLIMIT)
			{
				return speedLimit;
			}
			for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n轴*/
			{
				nAglSeqPtr[ai]->push_back(posInterp[ai]);
			}
			pLast = deg2rad(posInterp);
		}

		//(2)匀速段插补 [ 第 (N1+1) ~ (N1+N2) 点]
		//生成链表
		for (size_t j = 1; j != (N2 + 1); ++j) //j用于记录匀速段插补点的序号
		{
			for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n轴*/
			{
				posInterp[ai] += velConst[ai] * Ts;
				posacs[ai] = deg2rad(posInterp[ai]);
			}
			if ((lRet = pRobot->calcForwardKin(posacs, posmcs))
				!= RBTINTERF_NO_ERROR)//读回弧度值
			{
				return lRet;
			}
			rectLimit = pRobot->RobotRangeLimit(posmcs);	//计算是否超限
			if (rectLimit != RANGE_NOOVERLIMIT)
			{
				return rectLimit;
			}
			speedLimit = pRobot->RobotSpeedLimit(pLast, posacs, Ts, n);
			if (speedLimit != SPEED_NOOVERLIMIT)
			{
				return speedLimit;
			}
			for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n轴*/
			{
				nAglSeqPtr[ai]->push_back(posInterp[ai]);
			}
			pLast = deg2rad(posInterp);
		}

		//(3)减速段插补 [ 第 (N1+N2+1) ~ (N1+N2+N3) 点]
		//计算五次多项式系数
		for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n轴*/

		{
			getTrapezoidalCoe(posInterp[ai], posTarget[ai], velConst[ai], 0,
				tAcc, coeTrapezoidal[ai]);
		}
		//生成链表
		for (size_t k = 1; k != (N3 + 1); ++k) //k用于记录减速段插补点的序号
		{
			double t = k*Ts; //时间t
			for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n轴*/
			{
				trapezoidalPolynomi(t, coeTrapezoidal[ai], &(posInterp[ai]));
				posacs[ai] = deg2rad(posInterp[ai]);
			}
			if ((lRet = pRobot->calcForwardKin(posacs, posmcs))
				!= RBTINTERF_NO_ERROR)//读回弧度值
			{
				return lRet;
			}
			rectLimit = pRobot->RobotRangeLimit(posmcs);	//计算是否超限
			if (rectLimit != RANGE_NOOVERLIMIT)
			{
				return rectLimit;
			}
			speedLimit = pRobot->RobotSpeedLimit(pLast, posacs, Ts, n);
			if (speedLimit != SPEED_NOOVERLIMIT)
			{
				return speedLimit;
			}
			for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n轴*/
			{
				nAglSeqPtr[ai]->push_back(posInterp[ai]);
			}
			pLast = deg2rad(posInterp);
		}
	}
	return RBTINTERF_NO_ERROR;
}
