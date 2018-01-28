#pragma once
#include <deque>
#include "RbtInterfStatus.h"
#include "Seq.h"
#include "SerialRobotModel.h"


class Interpolation
{
public:
	Interpolation(std::shared_ptr<SerialRobotModel> pR) : pRobot(pR)
	{
	};
	~Interpolation()
	{
	};

	/**
	* @class tagQuinticCoe Interpolation.h
	* @brief ��ζ���ʽϵ��
	*/
	typedef struct tagQuinticCoe
	{
		double a0; /**< @brief ϵ��0 */
		double a1; /**< @brief ϵ��1 */
		double a2; /**< @brief ϵ��2 */
		double a3; /**< @brief ϵ��3 */
		double a4; /**< @brief ϵ��4 */
		double a5; /**< @brief ϵ��5 */
	} QuinticCoe; /**< @brief ��ζ���ʽ��ϵ�� */

	typedef struct tagTrapezoidalCoe
	{
		double a0; /**< @brief ϵ��0 */
		double a1; /**< @brief ϵ��1 */
		double a2; /**< @brief ϵ��2 */
	} trapezoidalCoe; /**< @brief���ζ���ʽ��ϵ�� */

	void getQuinticCoe(double q0, double qf, double dq0, double dqf,
		double ddq0, double ddqf, double tf, QuinticCoe &Coe);
	void getTrapezoidalCoe(double q0, double qf, double dq0, double dqf,
		double tf, trapezoidalCoe &Coe);
	void trapezoidalPolynomi(double r, trapezoidalCoe& coe, double *qptr);
	void QuinticPolynomi(double r, QuinticCoe& coe, double *qptr,
		double *dqptr = NULL, double *ddqptr = NULL, double *dddqptr = NULL);

	/**< @brief ��ζ���ʽ */
	/**
	* @class tagQuarticCoe JogInterp.h
	* @brief �Ĵζ���ʽϵ��
	*/
	typedef struct tagQuarticCoe
	{
		double a0; /**< @brief ϵ��0 */
		double a1; /**< @brief ϵ��1 */
		double a2; /**< @brief ϵ��2 */
		double a3; /**< @brief ϵ��3 */
		double a4; /**< @brief ϵ��4 */
	} QuarticCoe; /**< @brief �Ĵζ���ʽ��ϵ�� */
	void getQuarticCoe1(double q0, double dq0, double ddq0, double ddqh,
		double ddqf, double tf, QuarticCoe &Coe);
	void getQuarticCoe2(double q0, double dq0, double dqf, double ddq0,
		double ddqf, double tf, QuarticCoe &Coe);
	void getQuarticCoe3(double q0, double qf, double dq0, double ddq0,
		double ddqf, double tf, QuarticCoe &Coe);
	void QuarticPolynomi(double r, QuarticCoe& coe, double *qptr,
		double *dqptr = NULL, double *ddqptr = NULL, double *dddqptr = NULL);

	/**
	* @class tagLinearCoe JogInterp.h
	* @brief һ�ζ���ʽϵ��
	*/
	typedef struct tagLinearCoe
	{
		double a0; /**< @brief ϵ��0 */
		double a1; /**< @brief ϵ��1 */
	} LinearCoe; /**< @brief һ�ζ���ʽ��ϵ�� */

	void LinearPolynomi(double r, LinearCoe& coe, double *qptr,
		double *dqptr = NULL, double *ddqptr = NULL, double *dddqptr = NULL);/**< @brief һ�ζ���ʽ */
	void getLinearCoe(double q0, double dq0, double tf, LinearCoe &Coe);


	//���������ԶεĲ岹��
	void InterpNoLinear_5_0_5(double q0, double qf, double Ts, double tf, int polynomiType,
						   	  std::deque<double> &posInterp, double *posEdptr = NULL,
							  double *velEdptr = NULL, double *accEdptr = NULL,
							  double *jerkEdptr = NULL);
	//����5����ϵ��߶Σ��������ַ�����bug�ģ�Ӧ��4�׾͹��ˣ�������
	ErrorID calcInterp_5_1_5(double q0, double q1, double Ts, double maxVel,
							double maxAcc, double maxDecel, double maxJerk, int polynomiType,
							std::deque<double> &seq);

public:
	//�ؽڲ岹
	ErrorID JointInterp(const Position_ACS_deg &targetPoint,
						const Position_ACS_deg &originPoint, double Ts, double velPerc,
						double accPerc, int polynomiType, N_AxisSeqPtr &nAglSeqPtr); //������ǻ���ֵ���ڲ�����ʹ�õ��ǽǶ�ֵ

public:
	//ֱ�߲岹
	ErrorID LineInterp(const Position_MCS_rad &targetPoint,
		const Position_MCS_rad &originPoint,
		const Position_ACS_rad &originACS, double Ts, double maxVel,
		double maxAcc, double maxDecel, double maxJerk, double maxVelOri,
		double maxAccOri, double maxDecelOri, double maxJerkOri, int polynomiType,
		N_AxisSeqPtr &nAglSeqPtr, GrpTcpSeq *pGrpTcp = NULL);

public:
	//Բ���岹
	ErrorID ArcInterp(const Position_MCS_rad &middlePoint,
		const Position_MCS_rad &targetPoint,
		const Position_MCS_rad &originPoint,
		const Position_ACS_rad &originACS, double Ts, double maxVel,
		double maxAcc, double maxDecel, double maxJerk, double maxVelOri,
		double maxAccOri, double maxDecelOri, double maxJerkOri, int polynomiType, bool isFullCircle,
		N_AxisSeqPtr &nAglSeqPtr, GrpTcpSeq *pGrpTcp = NULL);
private:
	//����Բ��	
	XYZ calcCircleCenter(const XYZ& pA, const XYZ& pB, const XYZ& pC);
	//���㵥λ������
	Vector3 calcOrthoNormalVector(const Vector3& v1, const Vector3& v2);
private:
	std::shared_ptr<SerialRobotModel> pRobot;
};

