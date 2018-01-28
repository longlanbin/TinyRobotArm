#pragma once
#include "RobotData.h"
#include "RbtInterfStatus.h"
#include <string>

class AbstractRobot
{
public:

	AbstractRobot()
	{
		printf("AbstractRobot::construct!\n");

	}

	virtual ~AbstractRobot()
	{
		printf("AbstractRobot::destruct!\n");

	}
	//配置参数
	virtual	void setRobotModelParameter() = 0;
	/**
	* @brief  将关节角转换为空间中TCP点位姿
	* @param[in] posACS 机器人的关节角，单位是弧度值
	* @param[in] posMCS TCP点位姿，单位是毫米和弧度值
	* Gets or sets the model tcp.* @TODO:基于串联机器人正解算法一样，为了复用代码，可以设计为普通成员方法；为了扩展到并联机器人，其正解算法完全不一样
	*/
	//virtual size_t calcForwardKin(const dvector& posACS, dvector6& posMCS) = 0; /**< @note 弧度值*/
	virtual ErrorID calcForwardKin(const Position_ACS_rad& posACS, Position_MCS_rad& posMCS) = 0;

	/**
	* @brief      将TCP的位姿转换为关节角
	* @param[in]  posMCS  TCP的位姿，弧度
	* @param[in]  posLast 之前的关节角，弧度
	* @param[in]  posACS  求逆解后的关节角
	* @note       先将TCP的位姿转换为矩阵，在对矩阵求逆解
	*/
	//virtual size_t calcInverseKin_RPY(const dvector6& posMCS, const dvector& posLast, dvector& posACS) const = 0;/**< @note 弧度值*/

	virtual ErrorID calcInverseKin_RPY(const Position_MCS_rad& posMCS,
										const Position_ACS_rad& posLast, Position_ACS_rad& posACS) const = 0;
private:
	std::string robotName;
};

