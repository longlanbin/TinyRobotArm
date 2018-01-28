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
ErrorID Interpolation::ArcInterp(const Position_MCS_rad &middlePoint,
	const Position_MCS_rad &targetPoint,
	const Position_MCS_rad &originPoint, const Position_ACS_rad &originACS,
	double Ts, double maxVel, double maxAcc, double maxDecel,
	double maxJerk, double maxVelOri, double maxAccOri, double maxDecelOri,
	double maxJerkOri, int polynomiType, bool isFullCircle, N_AxisSeqPtr &nAglSeqPtr, GrpTcpSeq *pGrpTcp/* = NULL*/)
{
	if ((targetPoint.rows() != 6) || (originPoint.rows() != 6)
		|| (middlePoint.rows() != 6))
		return PLCOPEN_MOVE_INSTRUCT_ARG_ERROR;

	ErrorID lRet(RBTINTERF_NO_ERROR);
	AXIS_REFS_INDEX n = pRobot->numAxes();

	XYZ pA(originPoint.head(3)), pB(middlePoint.head(3)),
		pC(targetPoint.head(3));

	//判断共点、共线
	Vector3 v1 = pA - pB;
	Vector3 v2 = pA - pC;
	if (v1.norm() < 0.1 || v2.norm() < 0.1) //值很小
		return PLCOPEN_MOVE_INSTRUCT_ARC_COLLINEATION;

	double agl = acos(v1.dot(v2) / v1.norm() / v2.norm());
	if (fabs(agl) < 0.1 * M_PI / 180) //小于1度
		return PLCOPEN_MOVE_INSTRUCT_ARC_COLLINEATION;

	//第一步：确定圆心
	XYZ pO;//圆心
	pO = calcCircleCenter(pA, pB, pC);
	//第二步：确定旋转轴
	Vector3 f, vOA, vOB, vOC;
	vOA = pA - pO;
	vOB = pB - pO;
	vOC = pC - pO;
	f = calcOrthoNormalVector(vOA, vOC);//单位正交向量


	//第三步：计算角度幅度
	Vector3 beta1, beta2, beta3;
	double eta1, eta2, eta3;
	beta1 = vOA.cross(vOB);
	beta2 = vOB.cross(vOC);
	beta3 = vOC.cross(vOA);
	eta1 = beta1.dot(beta2);
	eta2 = beta2.dot(beta3);
	eta3 = beta3.dot(beta1);

	//若绕正方向旋转
	double deltaTheta = acos(vOA.dot(vOC) / vOA.norm() / vOC.norm());
	//绕反方向旋转
	if ((eta1<0) || (eta2>0) || (eta3>0))//判断转向
	{
		deltaTheta = deltaTheta - 2 * M_PI;
	}
	if (isFullCircle == true) //若为360圆
	{
		if (deltaTheta > 0)
		{
			deltaTheta = 2 * M_PI;
		}
		else
		{
			deltaTheta = -2 * M_PI;
		}
	}

	//第四步：计算半径，开始同步规划
	double R = vOA.norm();//半径
	std::cout << "R = " << R << std::endl;

	//确定绕轴最大旋转角速度;
	double maxAngleVel = maxVel / R;
	double maxAngleAcc = maxAcc / R;
	double maxAngleDecel = maxDecel / R;
	double maxAngleJerk = maxJerk / R;

	//第五步：终于可以开始规划了（还是按时间固定的老方法做吧，这个方法有bug，正确方法看我毕业论文）

	// 插补
	// todo: 当值未考虑加减速过程
	// 对于线性段带抛物线拟合
	// displcement->1
	// times = displacement/1.0
	// 时间相同
	// maxVel -> maxVel/times
	// maxAcc -> maxAcc/times
	// maxJerk -> maxJerk/times

	std::deque<double> rlst;

	if (sqrt(maxAngleAcc * deltaTheta) < maxAngleVel)
	{
		if (maxAngleAcc / sqrt(maxAngleAcc * deltaTheta) > 5)
		{
			maxAngleVel = 5 * deltaTheta;
			maxAngleAcc = 25 * deltaTheta;
			maxAngleDecel = 25 * deltaTheta;
		}
		else
		{
			maxAngleVel = sqrt(maxAngleAcc * deltaTheta);
		}
	}

	lRet = calcInterp_5_1_5(0, 1, Ts, fabs(maxAngleVel / deltaTheta),
		fabs(maxAngleAcc / deltaTheta), fabs(maxAngleDecel / deltaTheta),
		fabs(maxAngleJerk / deltaTheta), polynomiType, rlst);
	if (lRet != RBTINTERF_NO_ERROR)
		return lRet;

	nAglSeqPtr.resize(n);
	for (N_AxisSeqPtrIdx idx = 0; idx != n; ++idx)
	{
		nAglSeqPtr[idx].reset(new AglSeq());
	}
	//球面线性插补
	Matrix3 mOrigin = rpy2r(originPoint.tail(3));
	Matrix3 mMid = rpy2r(middlePoint.tail(3));
	Matrix3 mTarget = rpy2r(targetPoint.tail(3));

	dQuaternion qOrigin(mOrigin);
	dQuaternion qMid(mMid);
	dQuaternion qTarget(mTarget);
	dQuaternion qr1;
	dQuaternion qr2;
	dQuaternion qr;
	Vector3 pOrigin = originPoint.head(3);
	Vector3 pTarget = targetPoint.head(3);
	Vector3 pr;
	dmatrix Tr(4, 4);
	Position_MCS_rad posr(6);
	Position_ACS_rad posACS(n);
	Position_ACS_rad pLast(n);

	//以下为求360°整圆建立的变量
	Vector3 fx, fy;
	Matrix3 rotFrame(3, 3);
	dmatrix mInterp(4, 4);
	dmatrix tcp_under_frame(4, 4);
	//dmatrix frame(4, 4);
	Matrix4 frame(4, 4);
	dmatrix Rotr(4, 4);
	dmatrix RotTcp(4, 4);
	dmatrix originT(4, 4);
	originT = rpy2tr(originPoint);  //机器人TCP点坐标系在初始位置的旋转矩阵

	double alpha;
	Vector3 tcpZ(mOrigin.col(2));  //机器人TCP点坐标系Z轴

	if (isFullCircle == true) //若为360°圆
	{
		fx = vOA;
		fy = f.cross(fx);
		fx.normalize();
		fy.normalize();
		rotFrame << fx, fy, f;   //叉乘出圆心坐标系
		frame << rotFrame, pO, dmatrix::Zero(1, 3), 1;  //frame 为圆心坐标系相对于基座标系的旋转矩阵
		tcp_under_frame = frame.inverse() * originT;    //机器人TCP点坐标系在初始位置相对于圆心坐标系
		alpha = acos(tcpZ.dot(f) / tcpZ.norm() / f.norm());  // alpha 判断圆心坐标系Z轴与TCP点坐标系Z轴夹角，用于判断TCP点坐标系绕自身Z轴旋转方向
	}

	for (std::deque<double>::iterator rit = rlst.begin(); rit != rlst.end(); ++rit)
	{
		double r = *rit;
		if (isFullCircle == true) //若为360°圆
		{
			Rotr << Eigen::AngleAxisd(r*deltaTheta, Vector3::UnitZ()).toRotationMatrix(), dmatrix::Zero(3, 1), dmatrix::Zero(1, 3), 1;
			//RotTcp旋转矩阵是为了抵消TCP坐标系随圆心坐标系frame旋转而导致TCP点坐标系绕自身Z轴旋转360°，注意判断方向
			if (alpha > M_PI / 2)
			{
				RotTcp << Eigen::AngleAxisd(r*deltaTheta, Vector3::UnitZ()).toRotationMatrix(), dmatrix::Zero(3, 1), dmatrix::Zero(1, 3), 1;
			}
			else
			{
				RotTcp << Eigen::AngleAxisd(-r*deltaTheta, Vector3::UnitZ()).toRotationMatrix(), dmatrix::Zero(3, 1), dmatrix::Zero(1, 3), 1;
			}
			mInterp = frame * Rotr * tcp_under_frame * RotTcp; //姿态和位置，通过将机器人TCP坐标系变换到圆心坐标系frame中，实现姿态保持功能
			pr = pO + Eigen::AngleAxisd(r*deltaTheta, f) * vOA;//位置
			Tr << mInterp.block(0, 0, 3, 3), pr, dmatrix::Zero(1, 3), 1;
		}
		else  //若为圆弧
		{
			qr1 = qOrigin.slerp(r, qMid);
			qr2 = qMid.slerp(r, qTarget);
			qr = qr1.slerp(r, qr2); //姿态	
			pr = pO + Eigen::AngleAxisd(r*deltaTheta, f)*vOA; //位置
			Tr << qr.toRotationMatrix(), pr, dmatrix::Zero(1, 3), 1;
		}

		posr = tr2MCS(Tr);
		//判断在直角坐标系下是否超限
		rectLimit = pRobot->RobotRangeLimit(posr);
		if (rectLimit != RANGE_NOOVERLIMIT)
		{
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
			return speedLimit;
		}

		if (lRet != RBTINTERF_NO_ERROR)
		{
			//			logMsg((char*)"Error: pRobot->calcInverseKin_RPY()--count\n",0,0,0,0,0,0);
			if (pGrpTcp)
				pGrpTcp->clear();
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