#include "General6S.h"
#include <iostream>


General6S::General6S()
{
	printf("General6S::construct!\n");

}


General6S::~General6S()
{
	printf("General6S::destruct!\n");

}

/**
* @brief 设置机器人模型参数
*/
void General6S::setRobotModelParameter()
{
	printf("General6S::setRobotModelParameter\n");
	int totalAxes = 6;
	axes.resize(totalAxes);
	for (AXIS_REFS_INDEX idx = 0; idx != totalAxes; ++idx)
	{
		axes[idx].reset(new Axis());
	}

	////(1)读入Joint
	//axes[0]->AxisNumber = 1;
	//axes[0]->paramLmt.posSWLimit = 155;
	//axes[0]->paramLmt.negSWLimit = -155;
	//axes[0]->paramLmt.maxVel = 198;
	//axes[0]->paramLmt.maxRdcVel = -198;
	//axes[0]->paramLmt.maxRotSpeed = 4000;
	//axes[0]->paramLmt.maxDeRotSpeed = -4000;
	//axes[0]->paramLmt.maxAcc = 420;
	//axes[0]->paramLmt.maxDecel = -420;
	//axes[0]->reducRatio = 121;
	//axes[0]->direction = 1;
	//axes[0]->encoderResolution = 17;

	//axes[1]->AxisNumber = 2;
	//axes[1]->paramLmt.posSWLimit = 83;
	//axes[1]->paramLmt.negSWLimit = -120;
	//axes[1]->paramLmt.maxVel = 198;
	//axes[1]->paramLmt.maxRdcVel = -198;
	//axes[1]->paramLmt.maxRotSpeed = 4000;
	//axes[1]->paramLmt.maxDeRotSpeed = -4000;
	//axes[1]->paramLmt.maxAcc = 420;
	//axes[1]->paramLmt.maxDecel = -420;
	//axes[1]->reducRatio = 121;
	//axes[1]->direction = 1;
	//axes[1]->encoderResolution = 17;

	//axes[2]->AxisNumber = 3;
	//axes[2]->paramLmt.posSWLimit = 98;
	//axes[2]->paramLmt.negSWLimit = -70;
	//axes[2]->paramLmt.maxVel = 237;
	//axes[2]->paramLmt.maxRdcVel = -237;
	//axes[2]->paramLmt.maxRotSpeed = 4000;
	//axes[2]->paramLmt.maxDeRotSpeed = -4000;
	//axes[2]->paramLmt.maxAcc = 460;
	//axes[2]->paramLmt.maxDecel = -460;
	//axes[2]->reducRatio = 101;
	//axes[2]->direction = 1;
	//axes[2]->encoderResolution = 17;

	//axes[3]->AxisNumber = 4;
	//axes[3]->paramLmt.posSWLimit = 171;
	//axes[3]->paramLmt.negSWLimit = -171;
	//axes[3]->paramLmt.maxVel = 318;
	//axes[3]->paramLmt.maxRdcVel = -318;
	//axes[3]->paramLmt.maxRotSpeed = 4000;
	//axes[3]->paramLmt.maxDeRotSpeed = -4000;
	//axes[3]->paramLmt.maxAcc = 900;
	//axes[3]->paramLmt.maxDecel = -900;
	//axes[3]->reducRatio = 75.48;
	//axes[3]->direction = 1;
	//axes[3]->encoderResolution = 17;

	//axes[4]->AxisNumber = 5;
	//axes[4]->paramLmt.posSWLimit = 100;
	//axes[4]->paramLmt.negSWLimit = -100;
	//axes[4]->paramLmt.maxVel = 237;
	//axes[4]->paramLmt.maxRdcVel = -237;
	//axes[4]->paramLmt.maxRotSpeed = 4000;
	//axes[4]->paramLmt.maxDeRotSpeed = -4000;
	//axes[4]->paramLmt.maxAcc = 900;
	//axes[4]->paramLmt.maxDecel = -900;
	//axes[4]->reducRatio = 101;
	//axes[4]->direction = 1;
	//axes[4]->encoderResolution = 17;

	//axes[5]->AxisNumber = 6;
	//axes[5]->paramLmt.posSWLimit = 342;
	//axes[5]->paramLmt.negSWLimit = -342;
	//axes[5]->paramLmt.maxVel = 480;
	//axes[5]->paramLmt.maxRdcVel = -480;
	//axes[5]->paramLmt.maxRotSpeed = 4000;
	//axes[5]->paramLmt.maxDeRotSpeed = -4000;
	//axes[5]->paramLmt.maxAcc = 900;
	//axes[5]->paramLmt.maxDecel = -900;
	//axes[5]->reducRatio = 50;
	//axes[5]->direction = -1;
	//axes[5]->encoderResolution = 17;

	////(2)读入Link
	//axes[0]->DHJ.Parameters.Alpha = deg2rad(90);
	//axes[0]->DHJ.Parameters.A = 50;
	//axes[0]->DHJ.Parameters.Theta = deg2rad(0);
	//axes[0]->DHJ.Parameters.D = 321.5;
	//axes[0]->Offset = axes[0]->DHJ.Parameters.Theta;

	//axes[1]->DHJ.Parameters.Alpha = deg2rad(0);
	//axes[1]->DHJ.Parameters.A = 270;
	//axes[1]->DHJ.Parameters.Theta = deg2rad(90);
	//axes[1]->DHJ.Parameters.D = 0;
	//axes[1]->Offset = axes[1]->DHJ.Parameters.Theta;

	//axes[2]->DHJ.Parameters.Alpha = deg2rad(90);
	//axes[2]->DHJ.Parameters.A = 70;
	//axes[2]->DHJ.Parameters.Theta = deg2rad(0);
	//axes[2]->DHJ.Parameters.D = 0;
	//axes[2]->Offset = axes[2]->DHJ.Parameters.Theta;

	//axes[3]->DHJ.Parameters.Alpha = deg2rad(90);
	//axes[3]->DHJ.Parameters.A = 0;
	//axes[3]->DHJ.Parameters.Theta = deg2rad(0);
	//axes[3]->DHJ.Parameters.D = 299;
	//axes[3]->Offset = axes[3]->DHJ.Parameters.Theta;

	//axes[4]->DHJ.Parameters.Alpha = deg2rad(-90);
	//axes[4]->DHJ.Parameters.A = 0;
	//axes[4]->DHJ.Parameters.Theta = deg2rad(90);
	//axes[4]->DHJ.Parameters.D = 0;
	//axes[4]->Offset = axes[4]->DHJ.Parameters.Theta;

	//axes[5]->DHJ.Parameters.Alpha = deg2rad(0);
	//axes[5]->DHJ.Parameters.A = 0;
	//axes[5]->DHJ.Parameters.Theta = deg2rad(0);
	//axes[5]->DHJ.Parameters.D = 78.5;
	//axes[5]->Offset = axes[5]->DHJ.Parameters.Theta;

	////(3)输入点动速度
	//axes[0]->JogJointSpeed.MaxSpeed = 100;
	//axes[0]->JogJointSpeed.MaxAcc = 100;

	////输入CoupleCoe
	//_coupleCoe.Couple_Coe_1_2 = 0;
	//_coupleCoe.Couple_Coe_2_3 = 0;
	//_coupleCoe.Couple_Coe_3_2 = 0;
	//_coupleCoe.Couple_Coe_3_4 = 0;
	//_coupleCoe.Couple_Coe_4_5 = 0;
	//_coupleCoe.Couple_Coe_4_6 = 0;
	//_coupleCoe.Couple_Coe_5_6 = 0;
	////Set Robot Range
	//range_limit.maxX = 9999;
	//range_limit.maxY = 9999;
	//range_limit.maxZ = 9999;
	//range_limit.minX = -9999;
	//range_limit.minY = -9999;
	//range_limit.minZ = -9999;

}


/**
* @brief      将TCP的位姿转换为关节角
* @param[in]  posMCS  TCP的位姿，弧度
* @param[in]  posLast 之前的关节角，弧度
* @param[in]  posACS  求逆解后的关节角
* @note       先将TCP的位姿转换为矩阵，在对矩阵求逆解
*/
ErrorID General6S::calcInverseKin_RPY(const Position_MCS_rad& posMCS, const Position_ACS_rad& posLast, Position_ACS_rad& posACS) const
{
	printf("General6S::calcInverseKin_RPY\n");

	//dmatrix transMatrix(4, 4);
	//if ((posMCS.rows() != 6) || (posLast.rows() != 6) || (posACS.rows() != 6)
	//	|| (axes.size() != 6))
	//	return INVERSE_KIN_ARG_ERROR;
	//transMatrix = rpy2tr(posMCS);
	//if ((transMatrix.rows() != 4) || (transMatrix.cols() != 4) || (posLast.rows()
	//	!= 6) || (posACS.rows() != 6) || (axes.size() != 6))
	//{
	//	return INVERSE_KIN_TRANS_ARG_ERROR;
	//}
	//dmatrix T(transMatrix);
	//T *= toolMatrix.inverse();

	//Position_ACS_rad pLast(posLast);//读到的角度，经下面处理后，成为theta值

	//double nx(T(0, 0)), ny(T(1, 0)), nz(T(2, 0));
	//double ox(T(0, 1)), oy(T(1, 1)), oz(T(2, 1));
	//double ax(T(0, 2)), ay(T(1, 2)), az(T(2, 2));
	//double px(T(0, 3) / 1000.0), py(T(1, 3) / 1000.0), pz(T(2, 3) / 1000.0);

	////读取theat5偏移量
	//double theta5_offset = axes[4]->Offset;

	////vars defined temporarily
	//double d1 = axes[0]->DHJ.Parameters.D / 1000;
	//double d4 = axes[3]->DHJ.Parameters.D / 1000;
	//double d6 = axes[5]->DHJ.Parameters.D / 1000;
	//double a1 = axes[0]->DHJ.Parameters.A / 1000;
	//double a2 = axes[1]->DHJ.Parameters.A / 1000;
	//double a3 = axes[2]->DHJ.Parameters.A / 1000;

	////第一轴角度限制范围
	//double smax = axes[0]->paramLmt.posSWLimit;
	//double smin = axes[0]->paramLmt.negSWLimit;
	////第二轴角度限制范围
	//double lmax = axes[1]->paramLmt.posSWLimit;
	//double lmin = axes[1]->paramLmt.negSWLimit;
	////第三轴角度限制范围
	//double umax = axes[2]->paramLmt.posSWLimit;
	//double umin = axes[2]->paramLmt.negSWLimit;
	////第四轴角度限制范围
	//double rmax = axes[3]->paramLmt.posSWLimit;
	//double rmin = axes[3]->paramLmt.negSWLimit;
	////第五轴角度限制范围
	//double bmax = axes[4]->paramLmt.posSWLimit;
	//double bmin = axes[4]->paramLmt.negSWLimit;
	////第六轴角度限制范围
	//double tmax = axes[5]->paramLmt.posSWLimit;
	//double tmin = axes[5]->paramLmt.negSWLimit;

	////solve for theta1
	//double theta1_1;
	//if ((fabs((py - d6*ay)) < 10e-13) && (fabs((px - d6*ax)) < 10e-13))
	//	theta1_1 = pLast(0);
	//else
	//	theta1_1 = atan2((py - d6*ay), (px - d6*ax));
	//double theta1_2 = theta1_1;
	//double theta1;

	///*if (theta1_1 <= 0.0L)
	//theta1_2 = theta1_1 + M_PI;
	//else
	//theta1_2 = theta1_1 - M_PI;*/

	//theta1 = calcRealAngle(pLast(0), theta1_1, theta1_2);
	//// the limit of theta1 according to the reference
	//if ((theta1 < M_PI*smin / 180.0L) || (theta1 > M_PI*smax / 180.0L))
	//{
	//	theta1 = pLast(0);
	//	printf("theta1 exceeds pos limit.\n");
	//	jointLimitNum = 1;
	//	return INVERSE_KIN_NOT_REACHABLE;
	//}
	//posACS(0) = theta1;
	////solve for theta3

	//double k1 = cos(theta1)*px + sin(theta1)*py - a1 - d6*(cos(theta1)*ax + sin(theta1)*ay);
	//double k2 = pz - d1 - d6*az;
	//double k3 = pow(k1, 2) + pow(k2, 2) - pow(a2, 2) - pow(a3, 2) - pow(d4, 2);
	//double k4 = k3 / (2 * a2);
	//double temp_var = pow(a3, 2) + pow(d4, 2) - pow(k4, 2);
	//if (temp_var < 0.0L)
	//{
	//	printf("Theta3 can not be solved, so can not reach the point!\n");
	//	jointLimitNum = 3;
	//	return INVERSE_KIN_NOT_REACHABLE;
	//}
	//double delta = sqrt(pow(a3, 2) + pow(d4, 2) - pow(k4, 2));
	//double theta3_1 = atan2(d4, a3) + atan2(delta, k4);
	//double theta3_2 = atan2(d4, a3) - atan2(delta, k4);
	//double theta3;

	//theta3 = calcRealAngle(pLast(2), theta3_1, theta3_2);
	//// the limit of theta3 according to the reference
	//if ((theta3 < umin / 180.0L*M_PI) || (theta3 > umax / 180.0L*M_PI))
	//{
	//	theta3 = pLast(2);
	//	printf("theta3 exceeds pos limit.\n");
	//	jointLimitNum = 3;
	//	return INVERSE_KIN_NOT_REACHABLE;
	//}
	//posACS(2) = theta3;

	////solve for theta2
	//k1 = cos(theta1)*px + sin(theta1)*py - a1 - d6*(cos(theta1)*ax + sin(theta1)*ay);
	//k2 = -d1 + pz - d6*az;
	//double a = d4*cos(theta3) - a3*sin(theta3);
	//double b = d4*sin(theta3) + a2 + a3*cos(theta3);
	////已经加入了theta2的offset -pi/2    theta(运算) = theta(电机) - pi/2
	//double theta2_1;
	//if ((fabs(a*k1 + b*k2) < 10e-13) && (fabs(b*k1 - a*k2) < 10e-13))
	//	theta2_1 = pLast(1);
	//else
	//	theta2_1 = atan2((a*k1 + b*k2), (b*k1 - a*k2)) - M_PI / 2.0;
	//double theta2;

	//theta2 = calcRealAngle(pLast(1), theta2_1, theta2_1);
	//// the limit of theta2 according to the reference
	//if ((theta2 < lmin / 180.0L*M_PI) || (theta2 > lmax / 180.0L*M_PI))
	//{
	//	theta2 = pLast(1);
	//	printf("theta2 exceeds pos limit.\n");
	//	jointLimitNum = 2;
	//	return INVERSE_KIN_NOT_REACHABLE;
	//}
	//posACS(1) = theta2;

	////solve for theta4

	//k1 = sin(theta1)*ax - cos(theta1)*ay;
	//k2 = cos(theta1)*cos(theta2 + M_PI / 2.0 + theta3)*ax + sin(theta1)*
	//	cos(theta2 + M_PI / 2.0 + theta3)*ay + sin(theta2 + M_PI / 2.0 + theta3)*az;

	//double theta4;
	//double theta4_2;
	////此处的判断阈值不能过小，过小的话，当0/0时，它无法识别出来
	//if ((fabs(k1) < 10e-13) && (fabs(k2) < 10e-13))
	//{
	//	theta4 = pLast(3);
	//	//cout << "A" << endl;
	//}
	//else
	//{
	//	double theta4_1 = atan2(k1, k2);
	//	if (theta4_1 > 0.0L)
	//		theta4_2 = theta4_1 - M_PI;
	//	else
	//		theta4_2 = theta4_1 + M_PI;
	//	theta4 = calcRealAngle(pLast(3), theta4_1, theta4_2);
	//}

	//// the limit of theta4 according to the reference
	//if ((theta4 < rmin*M_PI / 180.0L) || (theta4 > rmax*M_PI / 180.0L))
	//{
	//	theta4 = pLast(3);
	//	printf("theta4 exceeds pos limit.\n");
	//	jointLimitNum = 4;
	//	return INVERSE_KIN_NOT_REACHABLE;
	//}
	//posACS(3) = theta4;

	////solve for theta5
	//double k1_1 = sin(theta1)*sin(theta4) + cos(theta1)*cos(theta4)*cos(theta2 + M_PI / 2.0 + theta3);
	//double k1_2 = -cos(theta1)*sin(theta4) + sin(theta1)*cos(theta4)*cos(theta2 + M_PI / 2.0 + theta3);
	//double k1_3 = cos(theta4)*sin(theta2 + M_PI / 2.0 + theta3);
	//k1 = k1_1*ax + k1_2*ay + k1_3*az;
	//k2 = cos(theta1)*sin(theta2 + M_PI / 2.0 + theta3)*ax + sin(theta1)*sin(theta2 + M_PI / 2.0 + theta3)*
	//	ay - cos(theta2 + M_PI / 2.0 + theta3)*az;
	//double theta5_1;
	//if ((fabs(k1) < 10e-13) && (fabs(k2) < 10e-13))
	//{
	//	theta5_1 = pLast(4);
	//}
	//else
	//{
	//	if (fabs(theta5_offset - M_PI / 2) < 10e-4)	//判断第五轴是垂直还是平行
	//	{
	//		theta5_1 = atan2(-k1, k2) - M_PI / 2.0;
	//	}
	//	else if (fabs(theta5_offset) < 10e-4)
	//	{
	//		theta5_1 = atan2(-k1, k2);
	//	}
	//}
	//double theta5;

	//theta5 = calcRealAngle(pLast(4), theta5_1, theta5_1);
	//// the limit of theta5 according to the reference
	//if ((theta5 < bmin / 180.0L*M_PI) || (theta5 > bmax / 180.0L*M_PI))
	//{
	//	theta5 = pLast(4);
	//	printf("theta5 exceeds pos limit.\n");
	//	jointLimitNum = 5;
	//	return INVERSE_KIN_NOT_REACHABLE;
	//}
	//posACS(4) = theta5;

	////solve for theta6
	//double k1_4 = sin(theta4)*cos(theta2 + M_PI / 2.0 + theta3)*cos(theta1) - cos(theta4)*sin(theta1);
	//double k1_5 = sin(theta4)*cos(theta2 + M_PI / 2.0 + theta3)*sin(theta1) + cos(theta4)*cos(theta1);
	//double k1_6 = sin(theta4)*sin(theta2 + M_PI / 2.0 + theta3);
	//k2 = -k1_4*ox - k1_5*oy - k1_6*oz;
	//k1 = -k1_4*nx - k1_5*ny - k1_6*nz;

	//double theta6_1;
	//if ((fabs(k1) < 10e-13) && (fabs(k2) < 10e-13))
	//	theta6_1 = pLast(5);
	//else
	//	theta6_1 = atan2(k1, k2);
	//double theta6;

	//theta6 = calcRealAngle(pLast(5), theta6_1, theta6_1);
	//// the limit of theta6 according to the reference
	//if ((theta6 < tmin / 180.0L*M_PI) || (theta6 > tmax / 180.0L*M_PI))
	//{
	//	theta6 = pLast(5);
	//	printf("theta6 exceeds pos limit.\n");
	//	jointLimitNum = 6;
	//	return INVERSE_KIN_NOT_REACHABLE;
	//}
	//posACS(5) = theta6;


	//for (std::deque<AXIS_REF>::size_type sz = 0; sz != axes.size(); ++sz)
	//{
	//	if ((posACS(sz)>deg2rad(axes[sz]->paramLmt.posSWLimit)) || (posACS(sz) <deg2rad(axes[sz]->paramLmt.negSWLimit)))
	//	{
	//		std::cerr << "theta " << sz + 1 << " exceeds pos limit\n";
	//		std::cerr << "posACS = [ \n" << rad2deg(posACS) << " \n]\n";

	//		posACS = pLast;
	//		return INVERSE_KIN_NOT_REACHABLE;
	//	}
	//}

	return RBTINTERF_NO_ERROR;
} 
