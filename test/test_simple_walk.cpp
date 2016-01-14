#include <Eigen\Eigen>

#include <Aris_Plan.h>



#include <Robot_Type_I.h>


Robots::RobotTypeI rbt;

struct SimpleWalkParam final :public Robots::GaitParamBase
{
	std::int32_t totalCount{ 1000 };
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
	rbt.LoadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_III\\Robot_III.xml");
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

	SimpleWalkParam param;
	rbt.SimByMatlab("C:\\Users\\yang\\Desktop\\test\\", SimpleWalk, &param);
	
	Aris::DynKer::Element *pFirst[]
	{
		rbt.pLegs[0]->pM1,rbt.pLegs[1]->pF1, rbt.pLegs[2]->pM1,rbt.pLegs[3]->pF1,rbt.pLegs[4]->pM1, rbt.pLegs[5]->pF1,
		rbt.pLegs[1]->pSf,rbt.pLegs[3]->pSf,rbt.pLegs[5]->pSf
	};
	Aris::DynKer::Element *pSecond[]
	{
		rbt.pLegs[0]->pF1,rbt.pLegs[1]->pM1, rbt.pLegs[2]->pF1,rbt.pLegs[3]->pM1,rbt.pLegs[4]->pF1, rbt.pLegs[5]->pM1,
		rbt.pLegs[0]->pSf,rbt.pLegs[2]->pSf,rbt.pLegs[4]->pSf
	};
	
	for (auto &ele : pFirst)rbt.Script().Activate(*ele, true);
	for (auto &ele : pSecond)rbt.Script().Activate(*ele, false);

	rbt.Script().Simulate(1000, 5);

	for (auto &ele : pFirst)rbt.Script().Activate(*ele, false);
	for (auto &ele : pSecond)rbt.Script().Activate(*ele, true);

	rbt.Script().Simulate(1000, 5);

	rbt.SimByAdams("C:\\Users\\yang\\Desktop\\test", SimpleWalk, &param, 5);
	//rbt.SaveAdams("C:\\Users\\yang\\Desktop\\test");
	//rbt.Script().MoveMarker(rbt.pLF->Base(), beginPE);
	/*
	rbt.ForEachForce([](Aris::DynKer::ForceBase* p)
	{
		p->Activate(false);
	});
	for (auto &ele : pFirst)ele->Activate(true);
	for (auto &ele : pSecond)ele->Activate(false);

	rbt.SimByAdamsResultAt(101);

	double fin[18];
	rbt.GetFinDyn(fin);
	Aris::DynKer::dsp(fin, 18, 1);

	

	rbt.ForEachMotion([](Aris::DynKer::MotionBase* p)
	{
		double frc[3]{ 0,0,0 };
		p->SetFrcCoe(frc);
	});

	{
		rbt.ClbPre();
		rbt.ClbSetInverseMethod([](int n, double *A)
		{
			Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >Am(A, n, n);
			auto im = Am.inverse();
			Am = im;
		});
		Aris::DynKer::Matrix D(rbt.ClbDimM(), rbt.ClbDimN());
		Aris::DynKer::Matrix b(rbt.ClbDimM(), 1);
		Aris::DynKer::Matrix x(rbt.ClbDimN(), 1);

		rbt.DynUpd();
		rbt.ClbMtx(D.Data(), b.Data());
		rbt.ClbUkn(x.Data());

		x.dsp();

		auto result = D*x;
		result.dsp();
		b.dsp();
	}
	*/
	//rbt.SimByAdams()

	//rbt.SetPee
	std::cout << "finished" << std::endl;

	char aaa;
	std::cin >> aaa;
	return 0;
}