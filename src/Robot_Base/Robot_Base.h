#ifndef ROBOT_BASE_H
#define ROBOT_BASE_H

#include <cstdint>
#include <vector>

#include "Aris_XML.h"
#include <Aris_DynModel.h>

namespace Robots
{
	class ROBOT_BASE;
	struct GAIT_PARAM_BASE;
	typedef std::function<int(ROBOT_BASE *, const GAIT_PARAM_BASE *)> GAIT_FUNC;

	class LEG_BASE:public Aris::DynKer::OBJECT
	{
	public:
		using COORDINATE = Aris::DynKer::COORDINATE;
		void GetPee(double *pEE, COORDINATE coordinate) const;
		void SetPee(const double *pEE, COORDINATE coordinate);
		void GetVee(double *pEE, COORDINATE coordinate) const;
		void SetVee(const double *pEE, COORDINATE coordinate);
		void GetAee(double *pEE, COORDINATE coordinate) const;
		void SetAee(const double *pEE, COORDINATE coordinate);
		void GetFeeSta(double *fEE_sta, COORDINATE coordinate) const;
		void SetFeeSta(const double *fEE_sta, COORDINATE coordinate);

		void GetJfd(double *jac, COORDINATE coordinate) const;
		void GetJfi(double *jac, COORDINATE coordinate) const;
		void GetJvd(double *jac, COORDINATE coordinate) const;
		void GetJvi(double *jac, COORDINATE coordinate) const;
		void GetDifJfd(double *dJac, COORDINATE coordinate) const;
		void GetDifJfi(double *dJac, COORDINATE coordinate) const;
		void GetDifJvd(double *dJac, COORDINATE coordinate) const;
		void GetDifJvi(double *dJac, COORDINATE coordinate) const;
		void GetCvd(double *c, COORDINATE coordinate) const;
		void GetCvi(double *c, COORDINATE coordinate) const;
		/*!
		* \brief follow equation: Aee = Jvd * Ain + dJvd * Vin + Cad
		* \param c Cad
		*/
		void GetCad(double *c, COORDINATE coordinate) const;
		/*!
		* \brief follow equation: Ain = Jvi * Aee + dJvi * Vee + Cai
		* \param c Cai
		*/
		void GetCai(double *c, COORDINATE coordinate) const;


		/*!
		* \brief Get the position of end-effector
		* \param pEE position of end-effector, double array with 3 elements
		* \param RelativeCoodinate coordinate, which can be "O" or "G" both means origin ground, "M" or "B" mainbody, "L" leg
		*/
		void GetPee(double *pEE, const char *RelativeCoodinate = "G") const;
		/*!
		* \brief Set the position of end-effector
		* \param pEE position of end-effector, double array with 3 elements
		* \param RelativeCoodinate coordinate, which can be "O" or "G" both means origin ground, "M" or "B" mainbody, "L" leg
		*/
		void SetPee(const double *pEE, const char *RelativeCoodinate = "G");
		/*!
		* \brief Get the velocity of end-effector
		* \param vEE velocity of end-effector, double array with 3 elements
		* \param RelativeCoodinate coordinate, which can be "O" or "G" both means origin ground, "M" or "B" mainbody, "L" leg
		*/
		void GetVee(double *vEE, const char *RelativeCoodinate = "G") const;
		/*!
		* \brief Set the velocity of end-effector, must be called after position is set
		* \param vEE velocity of end-effector, double array with 3 elements
		* \param RelativeCoodinate coordinate, which can be "O" or "G" both means origin ground, "M" or "B" mainbody, "L" leg
		*/
		void SetVee(const double *vEE, const char *RelativeCoodinate = "G");
		/*!
		* \brief Get the acceleration of end-effector
		* \param pEE acceleration of end-effector, double array with 3 elements
		* \param RelativeCoodinate coordinate, which can be "O" or "G" both means origin ground, "M" or "B" mainbody, "L" leg
		*/
		void GetAee(double *aEE, const char *RelativeCoodinate = "G") const;
		/*!
		* \brief Set the velocity of end-effector, must be called after velocity is set
		* \param vEE acceleration of end-effector, double array with 3 elements
		* \param RelativeCoodinate coordinate, which can be "O" or "G" both means origin ground, "M" or "B" mainbody, "L" leg
		*/
		void SetAee(const double *aEE, const char *RelativeCoodinate = "G");
		/*!
		* \brief Get the static end-effector force of inputs, which is equal to Jfd * Fin_sta
		* \param fIn actuation force of inputs, double array with 3 elements
		*/
		void GetFeeSta(double *fEE_sta, const char *RelativeCoodinate = "G") const;
		/*!
		* \brief Set the static end-effector force of inputs, meanwhile the Fin_sta will be Jfi * Fee_sta
		* \param aIn actuation force of inputs, double array with 3 elements
		*/
		void SetFeeSta(const double *fEE_sta, const char *RelativeCoodinate = "G");
		/*!
		* \brief Get the position of inputs
		* \param pIn position of inputs, double array with 3 elements
		*/
		void GetPin(double *pIn) const;
		/*!
		* \brief Set the position of inputs
		* \param pIn position of inputs, double array with 3 elements
		*/
		void SetPin(const double *pIn);
		/*!
		* \brief Get the velocity of inputs
		* \param vIn velocity of inputs, double array with 3 elements
		*/
		void GetVin(double *vIn) const;
		/*!
		* \brief Set the velocity of inputs
		* \param vIn velocity of inputs, double array with 3 elements
		*/
		void SetVin(const double *vIn);
		/*!
		* \brief Get the acceleration of inputs
		* \param aIn acceleration of inputs, double array with 3 elements
		*/
		void GetAin(double *aIn) const;
		/*!
		* \brief Set the acceleration of inputs
		* \param aIn acceleration of inputs, double array with 3 elements
		*/
		void SetAin(const double *aIn);
		/*!
		* \brief Get the static actuation force of inputs, which is equal to Jfi * Fee_sta
		* \param fIn actuation force of inputs, double array with 3 elements
		*/
		void GetFinSta(double *fIn_sta) const;
		/*!
		* \brief Set the static actuation force of inputs, meanwhile the Fee_sta will be Jfd * Fin_sta
		* \param aIn actuation force of inputs, double array with 3 elements
		*/
		void SetFinSta(const double *fIn_sta);
		
		void GetJfd(double *jac, const char *RelativeCoodinate = "G") const;
		void GetJfi(double *jac, const char *RelativeCoodinate = "G") const;
		void GetJvd(double *jac, const char *RelativeCoodinate = "G") const;
		void GetJvi(double *jac, const char *RelativeCoodinate = "G") const;
		void GetDifJfd(double *jac, const char *RelativeCoodinate = "G") const;
		void GetDifJfi(double *jac, const char *RelativeCoodinate = "G") const;
		void GetDifJvd(double *jac, const char *RelativeCoodinate = "G") const;
		void GetDifJvi(double *jac, const char *RelativeCoodinate = "G") const;
		void GetCvd(double *c, const char *RelativeCoodinate = "G") const;
		void GetCvi(double *c, const char *RelativeCoodinate = "G") const;
		/*!
		* \brief follow equation: Aee = Jvd * Ain + dJvd * Vin + Cad
		* \param c Cad
		*/
		void GetCad(double *c, const char *RelativeCoodinate = "G") const;
		/*!
		* \brief follow equation: Ain = Jvi * Aee + dJvi * Vee + Cai
		* \param c Cai
		*/
		void GetCai(double *c, const char *RelativeCoodinate = "G") const;

		void TransformCoordinatePee(const double *bodyPe, const char *fromMak, const double *fromPee
			, const char *toMak, double *toPee) const;

	protected:
		LEG_BASE(ROBOT_BASE* pRobot, const char *Name);
		virtual ~LEG_BASE() = default;
		
		virtual void calculate_from_pEE(){};
		virtual void calculate_from_pIn(){};
		virtual void calculate_from_vEE(){};
		virtual void calculate_from_vIn(){};
		virtual void calculate_from_aEE(){};
		virtual void calculate_from_aIn(){};
		virtual void calculate_jac(){};
		virtual void calculate_diff_jac(){};

	protected:		
		ROBOT_BASE *pRobot;
		Aris::DynKer::MARKER *pBase;

		union 
		{
			double pEE[3]{ 0 };
			struct
			{
				double x;
				double y;
				double z;
			};
		};
		union 
		{
			double vEE[3]{ 0 };
			struct
			{
				double vx;
				double vy;
				double vz;
			};
		};
		union 
		{
			double aEE[3]{ 0 };
			struct
			{
				double ax;
				double ay;
				double az;
			};
		};
		union 
		{
			double fEE_sta[3]{ 0 };
			struct
			{
				double fx_sta;
				double fy_sta;
				double fz_sta;
			};
		};
		union 
		{
			double pIn[3]{ 0 };
			struct
			{
				double l1;
				double l2;
				double l3;
			};
		};
		union 
		{
			double vIn[3]{ 0 };
			struct
			{
				double vl1;
				double vl2;
				double vl3;
			};
		};
		union 
		{
			double aIn[3]{ 0 };
			struct
			{
				double al1;
				double al2;
				double al3;
			};
		};
		union 
		{
			double fIn_sta[3]{ 0 };
			struct
			{
				double f1_sta;
				double f2_sta;
				double f3_sta;
			};
		};

		double _c_acc_dir[3]{ 0 };
		double _c_acc_inv[3]{ 0 };

		double Jvd[3][3]{ { 0 } };
		double Jvi[3][3]{ { 0 } };

		double vJvd[3][3]{ { 0 } };
		double vJvi[3][3]{ { 0 } };

		friend class ROBOT_BASE;
	};

	class ROBOT_BASE:public Aris::DynKer::MODEL
	{
	public:
		using COORDINATE = Aris::DynKer::COORDINATE;
		
		ROBOT_BASE();
		virtual ~ROBOT_BASE() = default;

		const Aris::DynKer::PART& Ground() const { return *pGround; };
		Aris::DynKer::PART& Ground() { return *pGround; };
		const Aris::DynKer::PART& Body() const { return *pBody; };
		Aris::DynKer::PART& Body() { return *pBody; };
		COORDINATE* pLegCoors()
		{ 
			static Aris::DynKer::COORDINATE pCoors[6];
			for (int i = 0; i < 6; ++i)
			{
				pCoors[i] = *pLegs[i]->pBase;
			}


			return pCoors;
		};


		void GetBodyPm(double *bodyPm, COORDINATE coordinate) const;
		void SetBodyPm(const double *bodyPm, COORDINATE coordinate);
		void GetBodyPe(double *bodyPe, const char *eurType, COORDINATE coordinate) const;
		void SetBodyPe(const double *bodyPe, const char *eurType, COORDINATE coordinate);
		void GetBodyVel(double *bodyVel, COORDINATE coordinate) const;
		void SetBodyVel(const double *bodyVel, COORDINATE coordinate);
		void GetBodyAcc(double *bodyAcc, COORDINATE coordinate) const;
		void SetBodyAcc(const double *bodyAcc, COORDINATE coordinate);

		void GetPee(double *pEE, COORDINATE coordinate) const;
		void SetPee(const double *pEE, COORDINATE coordinate);
		void GetVee(double *vEE, COORDINATE coordinate) const;
		void SetVee(const double *vEE, COORDINATE coordinate);
		void GetAee(double *aEE, COORDINATE coordinate) const;
		void SetAee(const double *aEE, COORDINATE coordinate);
		void GetFeeSta(double *fee_sta, COORDINATE coordinate) const;
		void SetFeeSta(const double *fee_sta, COORDINATE coordinate);
		void GetPee(double *pEE, COORDINATE *coordinate) const;
		void SetPee(const double *pEE, COORDINATE *coordinate);
		void GetVee(double *vEE, COORDINATE *coordinate) const;
		void SetVee(const double *vEE, COORDINATE *coordinate);
		void GetAee(double *aEE, COORDINATE *coordinate) const;
		void SetAee(const double *aEE, COORDINATE *coordinate);
		void GetFeeSta(double *fee_sta, COORDINATE *coordinate) const;
		void SetFeeSta(const double *fee_sta, COORDINATE *coordinate);




		/*!
		* \brief Get the body pose matrix
		* \param body pose matrix, double array with 16 elements
		*/
		void GetBodyPm(double *bodyPm) const;
		/*!
		* \brief Get the body PE
		* \param body pose matrix, double array with 16 elements
		*/
		void GetBodyPe(double *bodyPe, const char *eurType = "313") const;
		/*!
		* \brief Get the body velocity
		* \param body velocity, double array with 6 elements
		*/
		void GetBodyVel(double *bodyVel) const;
		/*!
		* \brief Get the body acceleration
		* \param body acceleration, double array with 6 elements
		*/
		void GetBodyAcc(double *bodyAcc) const;

		




		void GetPee(double *pEE, const char *RelativeCoodinate = "G") const;
		void SetPee(const double *pEE = nullptr, const double *bodyPe = nullptr, const char *coodinate = "G", const char *eurType = "313");
		void GetVee(double *aEE, const char *RelativeCoodinate = "G") const;
		void SetVee(const double *vEE = nullptr, const double *bodyVel = nullptr, const char *coodinate = "G");
		void GetAee(double *vEE, const char *RelativeCoodinate = "G") const;
		void SetAee(const double *aEE = nullptr, const double *bodyAcc = nullptr, const char *coodinate = "G");
		void GetFeeSta(double *fee_sta, const char *RelativeCoodinate = "G") const;
		void SetFeeSta(const double *fee_sta, const char *RelativeCoodinate = "G");
		
		void GetPin(double *pIn) const;
		void SetPin(const double *pIn = nullptr, const double *bodyPe = nullptr, const char *eurType = "313");
		void GetVin(double *vIn) const;
		void SetVin(const double *vIn = nullptr, const double *bodyPe = nullptr);
		void GetAin(double *aIn) const;
		void SetAin(const double *aIn = nullptr, const double *bodyPe = nullptr);
		void GetFinSta(double *fIn_sta) const;
		void SetFinSta(const double *fIn_sta);

		/*!
		* \brief 根据雅可比矩阵和初值迭代求解，主要用来在已知足端位置和输入位置时，机器人身体的位置。
		* 对于固定于地面的腿，对其SetPee，然后迭代求解身体位姿，之后对不在地面上的腿SetPin
		*/
		void SetPinFixFeet(const double *pIn, const char *fixFeet, const char *activeMotor, const double *initBodyPE);
		void SetVinFixFeet(const double *vIn, const char *fixFeet, const char *activeMotor);
		void SetAinFixFeet(const double *aIn, const char *fixFeet, const char *activeMotor);

		void GetJfd(double *jac_out, const char *activeMotion = "111111111111111111") const;
		void GetJvi(double *jac_out, const char *activeMotion = "111111111111111111") const;
		void GetDifJfd(double *jac_out, const char *activeMotion = "111111111111111111") const;
		void GetDifJvi(double *jac_out, const char *activeMotion = "111111111111111111") const;

		void TransformCoordinatePee(const double *bodyPe, const char *fromMak, const double *fromPee
			, const char *toMak, double *toPee) const;

		virtual void LoadXml(const char *file) { this->MODEL::LoadXml(file); };
		virtual void LoadXml(const Aris::Core::DOCUMENT &doc) { this->MODEL::LoadXml(doc); };

		LEG_BASE *pLegs[6];

	protected:
		Aris::DynKer::PART *pBody;

	private:
		double _BodyPm[4][4]{ {0} }, _BodyVel[6]{ 0 }, _BodyAcc[6]{ 0 };
		double Jvi[18][6]{ {0} };
		double vJvi[18][6]{ {0} };
		void calculate_jac();
		void calculate_jac_c();

		friend class LEG_BASE;
	};
}

#endif
