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
	//���ò���
	virtual	void setRobotModelParameter() = 0;
	/**
	* @brief  ���ؽڽ�ת��Ϊ�ռ���TCP��λ��
	* @param[in] posACS �����˵Ĺؽڽǣ���λ�ǻ���ֵ
	* @param[in] posMCS TCP��λ�ˣ���λ�Ǻ��׺ͻ���ֵ
	* Gets or sets the model tcp.* @TODO:���ڴ��������������㷨һ����Ϊ�˸��ô��룬�������Ϊ��ͨ��Ա������Ϊ����չ�����������ˣ��������㷨��ȫ��һ��
	*/
	//virtual size_t calcForwardKin(const dvector& posACS, dvector6& posMCS) = 0; /**< @note ����ֵ*/
	virtual ErrorID calcForwardKin(const Position_ACS_rad& posACS, Position_MCS_rad& posMCS) = 0;

	/**
	* @brief      ��TCP��λ��ת��Ϊ�ؽڽ�
	* @param[in]  posMCS  TCP��λ�ˣ�����
	* @param[in]  posLast ֮ǰ�Ĺؽڽǣ�����
	* @param[in]  posACS  ������Ĺؽڽ�
	* @note       �Ƚ�TCP��λ��ת��Ϊ�����ڶԾ��������
	*/
	//virtual size_t calcInverseKin_RPY(const dvector6& posMCS, const dvector& posLast, dvector& posACS) const = 0;/**< @note ����ֵ*/

	virtual ErrorID calcInverseKin_RPY(const Position_MCS_rad& posMCS,
										const Position_ACS_rad& posLast, Position_ACS_rad& posACS) const = 0;
private:
	std::string robotName;
};

