#include "Scara4S.h"



Scara4S::Scara4S()
{
	printf("Scara4S::construct!\n");

}


Scara4S::~Scara4S()
{
	printf("Scara4S::destruct!\n");

}

/**
* @brief 设置机器人模型参数
*/
void Scara4S::setRobotModelParameter()
{
	printf("Scara4S::setRobotModelParameter\n");
	int totalAxes = 4;
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
ErrorID Scara4S::calcInverseKin_RPY(const Position_MCS_rad& posMCS, const Position_ACS_rad& posLast, Position_ACS_rad& posACS) const
{
	printf("Scara4S::calcInverseKin_RPY\n");
	return RBTINTERF_NO_ERROR;
}