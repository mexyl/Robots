#include <Eigen/Eigen>

#include <aris_plan.h>
#include <Robot_Type_I.h>


Robots::RobotTypeI rbt;

struct SimpleWalkParam_Aris final :public Aris::DynKer::PlanParamBase
{
	double beginPee[18]{ 0 };
	double beginPeb[6]{ 0 };
	std::int32_t totalCount{ 500 };
	std::int32_t n{ 1 };
	double d{ 0.5 };
	double h{ 0.05 };
};
int SimpleWalk_Aris(Aris::DynKer::ModelBase &model, const Aris::DynKer::PlanParamBase & param)
{
	auto &sp = static_cast<const SimpleWalkParam_Aris&>(param);
	auto &robot = static_cast<Robots::RobotBase&>(model);

	static Aris::DynKer::FloatMarker beginBodyMak(robot.Ground(), nullptr, "313");
	static double beginEE[18];

	/*在每次脚着地时更新与身体坐标系重合的位于地面的坐标系*/
	if ((sp.count % sp.totalCount) == 0)
	{
		beginBodyMak.SetPrtPm(*robot.Body().Pm());
		beginBodyMak.Update();
		robot.GetPee(beginEE, beginBodyMak);
	}

	double pEE[18], pe[6]{ 0 };
	std::copy_n(beginEE, 18, pEE);

	/*当前相位的count*/
	int count = sp.count % sp.totalCount;

	if ((sp.count / sp.totalCount) == 0)/*加速段*/
	{
		pe[2] = Aris::Plan::acc_even(sp.totalCount, count + 1)*0.25*sp.d;

		double s = -(PI / 2)*cos(PI * (count + 1) / sp.totalCount) + PI / 2;

		for (int i = 0; i < 18; i += 6)
		{
			pEE[i + 1] = sp.h*sin(s) + beginEE[i + 1];
			pEE[i + 2] = sp.d / 2 * (1 - cos(s)) / 2 + beginEE[i + 2];
		}
	}
	else if ((sp.count / sp.totalCount) == (sp.n * 2 - 1))/*减速段*/
	{
		pe[2] = Aris::Plan::dec_even(sp.totalCount, count + 1)*0.25*sp.d;

		double s = -(PI / 2)*cos(PI * (count + 1) / sp.totalCount) + PI / 2;

		for (int i = 3; i < 18; i += 6)
		{
			pEE[i + 1] = sp.h*sin(s) + beginEE[i + 1];
			pEE[i + 2] = sp.d / 2 * (1 - cos(s)) / 2 + beginEE[i + 2];
		}
	}
	else if ((sp.count / sp.totalCount) % 2 == 1)/*第一匀速段，紧接着加速段*/
	{
		pe[2] = Aris::Plan::even(sp.totalCount, count + 1)*0.5*sp.d;

		double s = -(PI / 2)*cos(PI * (count + 1) / sp.totalCount) + PI / 2;

		for (int i = 3; i < 18; i += 6)
		{
			pEE[i + 1] = sp.h*sin(s) + beginEE[i + 1];
			pEE[i + 2] = sp.d * (1 - cos(s)) / 2 + beginEE[i + 2];
		}
	}
	else/*第二匀速段，后面就是减速段*/
	{
		pe[2] = Aris::Plan::even(sp.totalCount, count + 1)*0.5*sp.d;

		double s = -(PI / 2)*cos(PI * (count + 1) / sp.totalCount) + PI / 2;

		for (int i = 0; i < 18; i += 6)
		{
			pEE[i + 1] = sp.h*sin(s) + beginEE[i + 1];
			pEE[i + 2] = sp.d * (1 - cos(s)) / 2 + beginEE[i + 2];
		}
	}

	robot.SetPeb(pe, beginBodyMak);
	robot.SetPee(pEE, beginBodyMak);

	return 2 * sp.n * sp.totalCount - sp.count - 1;
}

struct SimpleWalkParam final :public Robots::GaitParamBase
{
	std::int32_t totalCount{ 500 };
	std::int32_t n{ 1 };
	double d{ 0.5 };
	double h{ 0.05 };
};
int SimpleWalk(Robots::RobotBase * pRobot, const Robots::GaitParamBase * pParam)
{
	auto pSP = static_cast<const SimpleWalkParam*>(pParam);

	static Aris::DynKer::FloatMarker beginBodyMak(pRobot->Ground(), nullptr, "313");
	static double beginEE[18];

	/*在每次脚着地时更新与身体坐标系重合的位于地面的坐标系*/
	if ((pSP->count % pSP->totalCount) == 0)
	{
		beginBodyMak.SetPrtPm(*pRobot->Body().Pm());
		beginBodyMak.Update();
		pRobot->GetPee(beginEE, beginBodyMak);
	}

	double pEE[18], pe[6]{ 0 };
	std::copy_n(beginEE, 18, pEE);

	/*当前相位的count*/
	int count = pSP->count % pSP->totalCount;

	if ((pSP->count / pSP->totalCount) == 0)/*加速段*/
	{
		pe[2] = Aris::Plan::acc_even(pSP->totalCount, count + 1)*0.25*pSP->d;

		double s = -(PI / 2)*cos(PI * (count + 1) / pSP->totalCount) + PI / 2;

		for (int i = 0; i < 18; i += 6)
		{
			pEE[i + 1] = pSP->h*sin(s) + beginEE[i + 1];
			pEE[i + 2] = pSP->d / 2 * (1 - cos(s)) / 2 + beginEE[i + 2];
		}
	}
	else if ((pSP->count / pSP->totalCount) == (pSP->n * 2 - 1))/*减速段*/
	{
		pe[2] = Aris::Plan::dec_even(pSP->totalCount, count + 1)*0.25*pSP->d;

		double s = -(PI / 2)*cos(PI * (count + 1) / pSP->totalCount) + PI / 2;

		for (int i = 3; i < 18; i += 6)
		{
			pEE[i + 1] = pSP->h*sin(s) + beginEE[i + 1];
			pEE[i + 2] = pSP->d / 2 * (1 - cos(s)) / 2 + beginEE[i + 2];
		}
	}
	else if ((pSP->count / pSP->totalCount) % 2 == 1)/*第一匀速段，紧接着加速段*/
	{
		pe[2] = Aris::Plan::even(pSP->totalCount, count + 1)*0.5*pSP->d;

		double s = -(PI / 2)*cos(PI * (count + 1) / pSP->totalCount) + PI / 2;

		for (int i = 3; i < 18; i += 6)
		{
			pEE[i + 1] = pSP->h*sin(s) + beginEE[i + 1];
			pEE[i + 2] = pSP->d * (1 - cos(s)) / 2 + beginEE[i + 2];
		}
	}
	else/*第二匀速段，后面就是减速段*/
	{
		pe[2] = Aris::Plan::even(pSP->totalCount, count + 1)*0.5*pSP->d;

		double s = -(PI / 2)*cos(PI * (count + 1) / pSP->totalCount) + PI / 2;

		for (int i = 0; i < 18; i += 6)
		{
			pEE[i + 1] = pSP->h*sin(s) + beginEE[i + 1];
			pEE[i + 2] = pSP->d * (1 - cos(s)) / 2 + beginEE[i + 2];
		}
	}

	pRobot->SetPeb(pe, beginBodyMak);
	pRobot->SetPee(pEE, beginBodyMak);

	return 2 * pSP->n * pSP->totalCount - pSP->count - 1;
}

int main(int argc, char *argv[])
{
#ifdef WIN32
	rbt.LoadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_VIII\\Robot_VIII.xml");
#endif
#ifdef UNIX
	rbt.LoadXml("/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml");
#endif

	const double beginEE[]{
		-0.3, -0.75, -0.65,
		-0.45, -0.75, 0,
		-0.3, -0.75, 0.65,
		0.3, -0.75, -0.65,
		0.45, -0.75, 0,
		0.3, -0.75, 0.65 };

	double beginPE[6]{0};

	rbt.SetPeb(beginPE);
	rbt.SetPee(beginEE, rbt.Body());
	
	SimpleWalkParam_Aris param;
	rbt.GetPee(param.beginPee);
	rbt.GetPeb(param.beginPeb);

	rbt.SimScriptClear();
	rbt.SimScriptSetTopologyA();
	rbt.SimScriptSimulate(500, 10);
	rbt.SimScriptSetTopologyB();
	rbt.SimScriptSimulate(500, 10);

	
	rbt.SetPeb(beginPE);
	rbt.SetPee(beginEE);
	rbt.Model::SimToAdams("C:\\Users\\yang\\Desktop\\test.cmd", SimpleWalk_Aris, param, 10, true);
	
	Aris::DynKer::SimResult result;
	rbt.SetPeb(beginPE);
	rbt.SetPee(beginEE);
	rbt.Model::SimDynAkima(SimpleWalk_Aris, param, result, 10, true);
	result.SaveToFile("C:\\Users\\yang\\Desktop\\test");
	

	double x[] = { 0,0.01,0.02,0.03,0.04,0.05,0.06,0.07,0.08,0.09,0.1,0.11,0.12,0.13,0.14,0.15,0.16,0.17,0.18,0.19,0.2,0.21,0.22,0.23,0.24,0.25,0.26,0.27,0.28,0.29,0.3,0.31,0.32,0.33,0.34,0.35,0.36,0.37,0.38,0.39,0.4,0.41,0.42,0.43,0.44,0.45,0.46,0.47,0.48,0.49,0.5,0.51,0.52,0.53,0.54,0.55,0.56,0.57,0.58,0.59,0.6,0.61,0.62,0.63,0.64,0.65,0.66,0.67,0.68,0.69,0.7,0.71,0.72,0.73,0.74,0.75,0.76,0.77,0.78,0.79,0.8,0.81,0.82,0.83,0.84,0.85,0.86,0.87,0.88,0.89,0.9,0.91,0.92,0.93,0.94,0.95,0.96,0.97,0.98,0.99,1 };
	double y[] = { 0.788395476259395,0.788278076130106,0.787923777836767,0.787326394234666,0.786475971222442,0.785359332789471,0.783960858426017,0.782263501825451,0.780250054737889,0.777904649370488,0.775214476251184,0.772171671959896,0.768775303418601,0.765033344438682,0.760964509103381,0.756599779693273,0.751983449613536,0.747173499993993,0.742241147778784,0.737269447166716,0.732350896324615,0.727584094325297,0.72306960109146,0.718905262360803,0.715181354905649,0.711975965780152,0.709351027466997,0.707349380199783,0.705993126307656,0.705283394161696,0.705201465835583,0.70571107153914,0.706761540706789,0.708291440475391,0.710232330853915,0.71251231403072,0.715059136740841,0.717802700212179,0.720676924481144,0.723620990267037,0.726580035612739,0.72950541514647,0.732354640102726,0.735091112337565,0.73768375120383,0.740106593417344,0.7423384267962,0.744362501411814,0.74616634737604,0.747741717334142,0.749084663226129,0.750313051165011,0.751543935682581,0.752775526290681,0.754006079891963,0.755233900421093,0.756457338454043,0.757674790789167,0.758884700003587,0.760085553988191,0.761275885464413,0.762454271485742,0.763619332926767,0.76476973396238,0.765904181539604,0.767021424844336,0.768120254765181,0.769199503356361,0.77025804330159,0.771294787380622,0.772308687940117,0.773298736370275,0.774263962588648,0.775203434532375,0.776116257660021,0.777001574464086,0.77785856399517,0.778686441398698,0.779484457465026,0.780251898193669,0.780988084372351,0.78169237117149,0.782364147754671,0.783002836905632,0.783607894672209,0.784178810027664,0.784715104549754,0.785216332117891,0.785682078628673,0.786111961730065,0.786505630574455,0.786862765590796,0.787183078276024,0.787466311005895,0.787712236865403,0.787920659498877,0.788091412979875,0.788224361700954,0.788319400283387,0.788376453506891,0.788395476259395 };

	Aris::DynKer::Akima akima(101, x, y);

	std::cout << std::setprecision(10) << akima(0.001) << std::endl;

	/*
	SimpleWalkParam param;
	rbt.GetPee(param.beginPee);
	rbt.GetPeb(param.beginPeb);

	rbt.SimByMatlab("C:\\Users\\yang\\Desktop\\test\\", SimpleWalk, param);
	
	rbt.SimScriptClear();
	rbt.SimScriptSetTopologyA();
	rbt.SimScriptSimulate(500, 10);
	rbt.SimScriptSetTopologyB();
	rbt.SimScriptSimulate(500, 10);
	rbt.SimByAdams("C:\\Users\\yang\\Desktop\\test.cmd", SimpleWalk, param, 10, true);
	rbt.SimByAdamsResultAt(101);

	double fin[18];
	rbt.GetFinDyn(fin);
	Aris::DynKer::dsp(fin, 18, 1);

	{
		rbt.ClbPre();
		rbt.ClbUpd();
		rbt.ClbSetInverseMethod([](int n, double *A)
		{
			Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >Am(A, n, n);
			auto im = Am.inverse();
			Am = im;
		});
		rbt.ForEachMotion([](Aris::DynKer::MotionBase *p) 
		{
			p->SetMotFce(p->MotFce());
		});
		
		Aris::DynKer::Matrix D(rbt.ClbDimM(), rbt.ClbDimN());
		Aris::DynKer::Matrix b(rbt.ClbDimM(), 1);
		Aris::DynKer::Matrix x(rbt.ClbDimN(), 1);

		rbt.ClbMtx(D.Data(), b.Data());
		rbt.ClbUkn(x.Data());

		auto result = D*x;
		result.dsp();
		b.dsp();
	}
	
	*/
	std::cout << "finished" << std::endl;

	char aaa;
	std::cin >> aaa;
	return 0;
}