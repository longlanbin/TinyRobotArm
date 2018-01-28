#include "Interpolation.h"
#include <iostream>
/*****************************************************************************/
/**
* @brief Sercos III - λ�ÿ���ģʽ�£��������е�����λ�ã������ǹ켣�� unit: deg
* @param[in] targetPoint Ŀ��λ�ã��Ƕ�
* @param[in] originPoint ��ʼλ�ã��Ƕ�
* @param[in] Ts �岹����
* @param[in] velPerc �ٶȰٷֱȣ�0.5-95.0�����磺�趨��50%������ٶ����У���ֵ50��
* @param[in] accPerc ���ٶȰٷֱȣ�0.5-95.0��
* @param[out] nAglSeqPtr [ָ��]n�����λ�ò岹���ؽڿռ䣩���У�nΪ����
* @return ���ɹ������� RBTINTERF_NO_ERROR
* @attention ע����ڲ�����λΪ�Ƕ�
* @note �˶��Ƕ����̫С���м��и��ݹ����
*/
/*****************************************************************************/
ErrorID Interpolation::JointInterp(const Position &targetPoint,
									const Position_ACS_deg &originPoint, double Ts, double velPerc,
									double accPerc, int polynomiType, N_AxisSeqPtr &nAglSeqPtr) //������ǽǶ�ֵ
{
	AXIS_REFS_INDEX n = pRobot->numAxes();

	ErrorID lRet(RBTINTERF_NO_ERROR);
	Position_ACS_rad posacs(n);
	Position_MCS_rad posmcs(6);	//������ֱ������λ��

	if ((n != AXIS_REFS_INDEX(targetPoint.rows())) || (n
		!= AXIS_REFS_INDEX(originPoint.rows())))
		return JOINT_INTERPLATE_ARG_ERROR;


	//	����λ������δִ����ϣ��ȴ�����������״̬
	//	MC_WaitIsFree();

	Position_ACS_deg posOrigin(originPoint);
	Position_ACS_deg posTarget(targetPoint);

	double jointLimit[32];
	for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai)
	{
		jointLimit[2 * ai] = pRobot->axes[ai]->paramLmt.posSWLimit;
		jointLimit[2 * ai + 1] = pRobot->axes[ai]->paramLmt.negSWLimit;
	}

	//��������λ��ƫ����
	Position_ACS_deg posOffset(n);
	posOffset = posTarget - posOrigin;

	dvector t(n); //��ȫ�����٣�����ʱ��
	double T = 0; //�ؽ��˶�����Ǹ�ʱ��
	dvector tA(n); //�������ʱ��
	double tAcc = 0; //�ؽڼ��ٹ�������Ǹ�ʱ��

_AdjustVel:
	//��һ������ȫ�����������˶���ȷ����������˶�ʱ��T
	velPerc = velPerc / 100;
	accPerc = accPerc / 100;

	for (AXIS_REFS_INDEX ai = 0; ai<n; ++ai) /*1-n��*/
	{
		t[ai] = fabs(posOffset[ai]) / (velPerc*(pRobot->axes[ai]->paramLmt.maxVel));
	}
	//�Ƚ�ʱ���С��ȡ���ֵ��ֵ��T
	T = t[0];
	for (AXIS_REFS_INDEX ai = 1; ai != n; ++ai) //2-n��
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

	//�ڶ�����ȷ���ӣ������ٶ�ʱ��tAcc
	for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n��*/
	{
		tA[ai] = (velPerc * (pRobot->axes[ai]->paramLmt.maxVel))
			/ (accPerc * (pRobot->axes[ai]->paramLmt.maxAcc));
	}
	tAcc = tA[0];
	for (AXIS_REFS_INDEX ai = 1; ai != n; ++ai) //2-n��
	{
		if (tAcc > tA[ai])
		{
			tAcc = tA[ai];
		}
	}

	for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) //1-n��
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

	//��������ȷ�����ٶ�ʱ��tConst
	double tf = T + tAcc; //����������������ʱ��Ϊtf
	double tConst = tf - 2 * tAcc;

	if (tConst<0)
	{
		//todo: �������ٶȼ��뷽�������Ǹ�Ϊֻ�б��ٶΣ�û�����ٶδ���
		//�ٶȼ���
		velPerc = velPerc * 98;
		accPerc = accPerc * 98;
		goto _AdjustVel;
		//return PLCOPEN_MOVE_INSTRUCT_NOT_APPLICABLE;
	}

	//���Ĳ���ȷ�����ٶΡ����ٶΡ����ٶεĲ岹������N1,N2,N3������ȷ�����յ�����ʱ��
	size_t N1 = static_cast<size_t>(tAcc / Ts);
	size_t N2 = static_cast<size_t>(tConst / Ts);
	size_t N3 = N1;

	std::cout << "\n JointInterp: N1=" << N1 << " " << "N2=" << N2 << " " << "N3=" << N3 << " " << std::endl;

	tAcc = N1 * Ts;
	tConst = N2 * Ts;
	T = tConst + tAcc;

	//���岽�����������������ʱ��ƽ���ٶ�(Ҳ�����ٶε��ٶ�velConst��ע������ٶ����趨���ٶȿ��ܲ�һ��)
	dvector velConst(n);//��������
	for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n��*/
	{
		velConst[ai] = posOffset[ai] / T;
	}

	//���������켣�岹�����У�����ζ���ʽ�Լ��ٶκͼ��ٶν��в岹
	nAglSeqPtr.resize(n);
	for (N_AxisSeqPtrIdx idx = 0; idx != n; ++idx)
	{
		nAglSeqPtr[idx].reset(new AglSeq());
	}
	dvector posInterp(n);//�����ݴ����ĳ���ڵĲ岹��λ��
	Position_ACS_rad pLast(n);

	pLast = deg2rad(originPoint);

	if (polynomiType == 0)
	{
		//(1)���ٶβ岹 [ �� 0 ~ N1 ��]
		//������ζ���ʽϵ��
		dvector accPosEnd(n);//������ٶεĽ�����λ��
		std::deque<QuinticCoe> coeQuintic(n);//���������ζ���ʽ

		for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n��*/
		{
			accPosEnd[ai] = posOrigin[ai] + velConst[ai] * tAcc / 2;
			getQuinticCoe(posOrigin[ai], accPosEnd[ai], 0, velConst[ai], 0, 0,
				tAcc, coeQuintic[ai]);
		}

		//��������
		for (size_t i = 0; i != (N1 + 1); ++i) //i���ڼ�¼���ٶβ岹������
		{
			double t = i*Ts; //ʱ��t
			for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n��*/
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
				!= RBTINTERF_NO_ERROR)//���ػ���ֵ
			{
				return lRet;
			}
			rectLimit = pRobot->RobotRangeLimit(posmcs);	//�����Ƿ���
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
			for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n��*/
			{
				nAglSeqPtr[ai]->push_back(posInterp[ai]);
			}
			pLast = deg2rad(posInterp);
		}
		//(2)���ٶβ岹 [ �� (N1+1) ~ (N1+N2) ��]
		//��������
		for (size_t j = 1; j != (N2 + 1); ++j) //j���ڼ�¼���ٶβ岹������
		{
			for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n��*/
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
				!= RBTINTERF_NO_ERROR)//���ػ���ֵ
			{
				return lRet;
			}
			rectLimit = pRobot->RobotRangeLimit(posmcs);	//�����Ƿ���
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
			for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n��*/
			{
				nAglSeqPtr[ai]->push_back(posInterp[ai]);
			}
			pLast = deg2rad(posInterp);
		}

		//(3)���ٶβ岹 [ �� (N1+N2+1) ~ (N1+N2+N3) ��]
		//������ζ���ʽϵ��
		for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n��*/
		{
			getQuinticCoe(posInterp[ai], posTarget[ai], velConst[ai], 0, 0, 0,
				tAcc, coeQuintic[ai]);
		}
		//��������
		for (size_t k = 1; k != (N3 + 1); ++k) //k���ڼ�¼���ٶβ岹������
		{
			double t = k*Ts; //ʱ��t
			for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n��*/
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
				!= RBTINTERF_NO_ERROR)//���ػ���ֵ
			{
				return lRet;
			}
			rectLimit = pRobot->RobotRangeLimit(posmcs);	//�����Ƿ���
			if (rectLimit != RANGE_NOOVERLIMIT)
			{
				return rectLimit;
			}
			speedLimit = pRobot->RobotSpeedLimit(pLast, posacs, Ts, n);
			if (speedLimit != SPEED_NOOVERLIMIT)
			{
				return speedLimit;
			}
			for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n��*/
			{
				nAglSeqPtr[ai]->push_back(posInterp[ai]);
			}
			pLast = deg2rad(posInterp);
		}
	}
	else if (polynomiType == 1)
	{
		//(1)���ٶβ岹 [ �� 0 ~ N1 ��]
		//������ζ���ʽϵ��
		dvector accPosEnd(n);//������ٶεĽ�����λ��
		std::deque<trapezoidalCoe> coeTrapezoidal(n);//������Ķ��ζ���ʽ
		for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n��*/
		{
			accPosEnd[ai] = posOrigin[ai] + velConst[ai] * tAcc / 2;
			getTrapezoidalCoe(posOrigin[ai], accPosEnd[ai], 0, velConst[ai],
				tAcc, coeTrapezoidal[ai]);
		}
		//��������
		for (size_t i = 0; i != (N1 + 1); ++i) //i���ڼ�¼���ٶβ岹������
		{
			double t = i*Ts; //ʱ��t
			for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n��*/
			{
				trapezoidalPolynomi(t, coeTrapezoidal[ai], &(posInterp[ai]));
				posacs[ai] = deg2rad(posInterp[ai]);
			}
			if ((lRet = pRobot->calcForwardKin(posacs, posmcs))
				!= RBTINTERF_NO_ERROR)//���ػ���ֵ
			{
				return lRet;
			}
			rectLimit = pRobot->RobotRangeLimit(posmcs);	//�����Ƿ���
			if (rectLimit != RANGE_NOOVERLIMIT)
			{
				return rectLimit;
			}
			speedLimit = pRobot->RobotSpeedLimit(pLast, posacs, Ts, n);
			if (speedLimit != SPEED_NOOVERLIMIT)
			{
				return speedLimit;
			}
			for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n��*/
			{
				nAglSeqPtr[ai]->push_back(posInterp[ai]);
			}
			pLast = deg2rad(posInterp);
		}

		//(2)���ٶβ岹 [ �� (N1+1) ~ (N1+N2) ��]
		//��������
		for (size_t j = 1; j != (N2 + 1); ++j) //j���ڼ�¼���ٶβ岹������
		{
			for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n��*/
			{
				posInterp[ai] += velConst[ai] * Ts;
				posacs[ai] = deg2rad(posInterp[ai]);
			}
			if ((lRet = pRobot->calcForwardKin(posacs, posmcs))
				!= RBTINTERF_NO_ERROR)//���ػ���ֵ
			{
				return lRet;
			}
			rectLimit = pRobot->RobotRangeLimit(posmcs);	//�����Ƿ���
			if (rectLimit != RANGE_NOOVERLIMIT)
			{
				return rectLimit;
			}
			speedLimit = pRobot->RobotSpeedLimit(pLast, posacs, Ts, n);
			if (speedLimit != SPEED_NOOVERLIMIT)
			{
				return speedLimit;
			}
			for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n��*/
			{
				nAglSeqPtr[ai]->push_back(posInterp[ai]);
			}
			pLast = deg2rad(posInterp);
		}

		//(3)���ٶβ岹 [ �� (N1+N2+1) ~ (N1+N2+N3) ��]
		//������ζ���ʽϵ��
		for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n��*/

		{
			getTrapezoidalCoe(posInterp[ai], posTarget[ai], velConst[ai], 0,
				tAcc, coeTrapezoidal[ai]);
		}
		//��������
		for (size_t k = 1; k != (N3 + 1); ++k) //k���ڼ�¼���ٶβ岹������
		{
			double t = k*Ts; //ʱ��t
			for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n��*/
			{
				trapezoidalPolynomi(t, coeTrapezoidal[ai], &(posInterp[ai]));
				posacs[ai] = deg2rad(posInterp[ai]);
			}
			if ((lRet = pRobot->calcForwardKin(posacs, posmcs))
				!= RBTINTERF_NO_ERROR)//���ػ���ֵ
			{
				return lRet;
			}
			rectLimit = pRobot->RobotRangeLimit(posmcs);	//�����Ƿ���
			if (rectLimit != RANGE_NOOVERLIMIT)
			{
				return rectLimit;
			}
			speedLimit = pRobot->RobotSpeedLimit(pLast, posacs, Ts, n);
			if (speedLimit != SPEED_NOOVERLIMIT)
			{
				return speedLimit;
			}
			for (AXIS_REFS_INDEX ai = 0; ai != n; ++ai) /*1-n��*/
			{
				nAglSeqPtr[ai]->push_back(posInterp[ai]);
			}
			pLast = deg2rad(posInterp);
		}
	}
	return RBTINTERF_NO_ERROR;
}
