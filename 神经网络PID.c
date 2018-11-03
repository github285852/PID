
//神经网络PID
#define M0K 0.06
#define M1K 0.12
#define M2K 0.12
float   M3K = 3.2;

float xiteP = 0.50;//P的学习率；
float xiteI = 0.25;
float xiteD = 0.50;
float error1,error2,error;
float u,u1;
float x1,x2,x3;
float wkp,wki,wkd;
float SignleNeuralAdaptivPID(u16 yd ,u16 y,u8 M)
{
//	static float wkp1 = 0.5;
//	static float wki1 = 0.1;
//	static float wkd1 = 0.5; //初始加权
	
	static float wkp1 = 0.5;
	static float wki1 = 0.5;
	static float wkd1 = 0.5; //初始加权
	float norm,w11,w22,w33;
	error = yd-y;
	//反馈
	switch(M)
	{
		case 0:
			wkp = wkp1 + xiteP*u1*x1;
			wki = wki1 + xiteI*u1*x2;
			wkd = wkd1 + xiteD*u1*x3;
		break;
		case 1:
			wkp = wkp1 + xiteP*u1*error;
			wki = wki1 + xiteI*u1*error;
			wkd = wkd1 + xiteD*u1*error;
		break;
		case 2:
			wkp = wkp1 + xiteP*u1*error*x1;
			wki = wki1 + xiteI*u1*error*x2;
			wkd = wkd1 + xiteD*u1*error*x3;
		break;
		case 3:
			wkp = wkp1 + xiteP*u1*error*(2*error-error1);
			wki = wki1 + xiteI*u1*error*(2*error-error1);
			wkd = wkd1 + xiteD*u1*error*(2*error-error1);//学习算法
		break;
		default :break;
	}
	//增量式PID
	x1 = error - error1;
	x2 = error;
	x3 = error - 2*error1+error2;
	
	norm = ABS(wkp)+ABS(wki)+ABS(wkd); 
	//归一化，利于收敛
	w11 = wkp/norm;
	w22 = wki/norm;
	w33 = wkd/norm;
	//神经网络PID 计算
	switch(M)
	{
		case 0:
		u = u1 + M0K*(w11*x1+w22*x2+w33*x3);
		break;
		case 1:
		u = u1 + M1K*(w11*x1+w22*x2+w33*x3);
		break;
		case 2:
		u = u1 + M2K*(w11*x1+w22*x2+w33*x3);
		break;
		case 3:
		u = u1 + M3K*(w11*x1+w22*x2+w33*x3);
		break;
		default:break;
	}
	error2 = error1;
	error1 = error;
	u1 = u;
	wkp1 = wkp;
	wki1 = wki;
	wkd1 = wkd;
	return u;
}
