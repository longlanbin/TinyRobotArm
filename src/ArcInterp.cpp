#include "Interpolation.h"
#include <iostream>
/*****************************************************************************/
/**
* @brief Sercos III - λ�ÿ���ģʽ�£��ѿ�������ϵ�°�ֱ�߲岹�����е�����λ�� unit: deg
* @param[in] targetPoint Ŀ��λ�ã�����
* @param[in] originPoint ��ʼλ��(MCS)������
* @param[in] originACS ��ʼλ��(ACS)�� ����
* @param[in] Ts �岹����
* @param[in] maxVel ����ٶ�
* @param[in] maxAcc �����ٶ�
* @param[in] maxDecel �����ٶ�
* @param[in] maxJerk ���Ӽ��ٶ�
* @param[in] maxVelOri �����ٶ�
* @param[in] maxAccOri ���Ǽ��ٶ�
* @param[in] maxDecelOri ���Ǽ��ٶ�
* @param[in] maxJerkOri ���ǼӼ��ٶ�
* @param[out] nAglSeqPtr [ָ��]n�����λ�ò岹���ؽڿռ䣩���У�nΪ����
* @param[out] pGrpTcp [��ѡ][ָ��]�����λ�ò岹��ֱ�ǿռ䣩����
* @return ���ɹ������� RBTINTERF_NO_ERROR
* @attention ע����ڲ�����λΪ����
* @note �˶��������̫С���м��и��ݹ����
* @todo ���ٶ�����
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

	//�жϹ��㡢����
	Vector3 v1 = pA - pB;
	Vector3 v2 = pA - pC;
	if (v1.norm() < 0.1 || v2.norm() < 0.1) //ֵ��С
		return PLCOPEN_MOVE_INSTRUCT_ARC_COLLINEATION;

	double agl = acos(v1.dot(v2) / v1.norm() / v2.norm());
	if (fabs(agl) < 0.1 * M_PI / 180) //С��1��
		return PLCOPEN_MOVE_INSTRUCT_ARC_COLLINEATION;

	//��һ����ȷ��Բ��
	XYZ pO;//Բ��
	pO = calcCircleCenter(pA, pB, pC);
	//�ڶ�����ȷ����ת��
	Vector3 f, vOA, vOB, vOC;
	vOA = pA - pO;
	vOB = pB - pO;
	vOC = pC - pO;
	f = calcOrthoNormalVector(vOA, vOC);//��λ��������


	//������������Ƕȷ���
	Vector3 beta1, beta2, beta3;
	double eta1, eta2, eta3;
	beta1 = vOA.cross(vOB);
	beta2 = vOB.cross(vOC);
	beta3 = vOC.cross(vOA);
	eta1 = beta1.dot(beta2);
	eta2 = beta2.dot(beta3);
	eta3 = beta3.dot(beta1);

	//������������ת
	double deltaTheta = acos(vOA.dot(vOC) / vOA.norm() / vOC.norm());
	//�Ʒ�������ת
	if ((eta1<0) || (eta2>0) || (eta3>0))//�ж�ת��
	{
		deltaTheta = deltaTheta - 2 * M_PI;
	}
	if (isFullCircle == true) //��Ϊ360Բ
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

	//���Ĳ�������뾶����ʼͬ���滮
	double R = vOA.norm();//�뾶
	std::cout << "R = " << R << std::endl;

	//ȷ�����������ת���ٶ�;
	double maxAngleVel = maxVel / R;
	double maxAngleAcc = maxAcc / R;
	double maxAngleDecel = maxDecel / R;
	double maxAngleJerk = maxJerk / R;

	//���岽�����ڿ��Կ�ʼ�滮�ˣ����ǰ�ʱ��̶����Ϸ������ɣ����������bug����ȷ�������ұ�ҵ���ģ�

	// �岹
	// todo: ��ֵδ���ǼӼ��ٹ���
	// �������Զδ����������
	// displcement->1
	// times = displacement/1.0
	// ʱ����ͬ
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
	//�������Բ岹
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

	//����Ϊ��360����Բ�����ı���
	Vector3 fx, fy;
	Matrix3 rotFrame(3, 3);
	dmatrix mInterp(4, 4);
	dmatrix tcp_under_frame(4, 4);
	//dmatrix frame(4, 4);
	Matrix4 frame(4, 4);
	dmatrix Rotr(4, 4);
	dmatrix RotTcp(4, 4);
	dmatrix originT(4, 4);
	originT = rpy2tr(originPoint);  //������TCP������ϵ�ڳ�ʼλ�õ���ת����

	double alpha;
	Vector3 tcpZ(mOrigin.col(2));  //������TCP������ϵZ��

	if (isFullCircle == true) //��Ϊ360��Բ
	{
		fx = vOA;
		fy = f.cross(fx);
		fx.normalize();
		fy.normalize();
		rotFrame << fx, fy, f;   //��˳�Բ������ϵ
		frame << rotFrame, pO, dmatrix::Zero(1, 3), 1;  //frame ΪԲ������ϵ����ڻ�����ϵ����ת����
		tcp_under_frame = frame.inverse() * originT;    //������TCP������ϵ�ڳ�ʼλ�������Բ������ϵ
		alpha = acos(tcpZ.dot(f) / tcpZ.norm() / f.norm());  // alpha �ж�Բ������ϵZ����TCP������ϵZ��нǣ������ж�TCP������ϵ������Z����ת����
	}

	for (std::deque<double>::iterator rit = rlst.begin(); rit != rlst.end(); ++rit)
	{
		double r = *rit;
		if (isFullCircle == true) //��Ϊ360��Բ
		{
			Rotr << Eigen::AngleAxisd(r*deltaTheta, Vector3::UnitZ()).toRotationMatrix(), dmatrix::Zero(3, 1), dmatrix::Zero(1, 3), 1;
			//RotTcp��ת������Ϊ�˵���TCP����ϵ��Բ������ϵframe��ת������TCP������ϵ������Z����ת360�㣬ע���жϷ���
			if (alpha > M_PI / 2)
			{
				RotTcp << Eigen::AngleAxisd(r*deltaTheta, Vector3::UnitZ()).toRotationMatrix(), dmatrix::Zero(3, 1), dmatrix::Zero(1, 3), 1;
			}
			else
			{
				RotTcp << Eigen::AngleAxisd(-r*deltaTheta, Vector3::UnitZ()).toRotationMatrix(), dmatrix::Zero(3, 1), dmatrix::Zero(1, 3), 1;
			}
			mInterp = frame * Rotr * tcp_under_frame * RotTcp; //��̬��λ�ã�ͨ����������TCP����ϵ�任��Բ������ϵframe�У�ʵ����̬���ֹ���
			pr = pO + Eigen::AngleAxisd(r*deltaTheta, f) * vOA;//λ��
			Tr << mInterp.block(0, 0, 3, 3), pr, dmatrix::Zero(1, 3), 1;
		}
		else  //��ΪԲ��
		{
			qr1 = qOrigin.slerp(r, qMid);
			qr2 = qMid.slerp(r, qTarget);
			qr = qr1.slerp(r, qr2); //��̬	
			pr = pO + Eigen::AngleAxisd(r*deltaTheta, f)*vOA; //λ��
			Tr << qr.toRotationMatrix(), pr, dmatrix::Zero(1, 3), 1;
		}

		posr = tr2MCS(Tr);
		//�ж���ֱ������ϵ���Ƿ���
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
				pGrpTcp->push_back(posr);//��¼tcp
			for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) //1-n��
			{
				nAglSeqPtr[ai]->push_back(rad2deg(posACS[ai]));
			}
			pLast = posACS;
		}
	}
	return RBTINTERF_NO_ERROR;
}