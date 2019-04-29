#include<stdio.h>
#include<stdlib.h>
#include <time.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include<softPwm.h>
#include<math.h>
#include<sys/time.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <unistd.h>

#define A_ENABLE_1 26
#define A_ENABLE_2 27
#define A_IN_1 22
#define A_IN_2 23

#define B_ENABLE_1 28
#define B_ENABLE_2 29
#define B_IN_1 24
#define B_IN_2 25

enum direction{UP, DOWN, LEFT, RIGHT};
char started = 0;//false

/*****************************/

#define SMPLRT_DIV              0x19    //陀螺仪采样率，典型值：0x07(125Hz)
#define CONFIG                  0x1A    //低通滤波频率，典型值：0x06(5Hz)
#define GYRO_CONFIG             0x1B    //不自检，转速250deg/s 陀螺仪自检及测量范围
#define ACCEL_CONFIG    0x1C    //加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define ACCEL_XOUT_H    0x3B
#define ACCEL_XOUT_L    0x3C
#define ACCEL_YOUT_H    0x3D
#define ACCEL_YOUT_L    0x3E
#define ACCEL_ZOUT_H    0x3F
#define ACCEL_ZOUT_L    0x40
#define TEMP_OUT_H              0x41
#define TEMP_OUT_L              0x42
#define GYRO_XOUT_H             0x43
#define GYRO_XOUT_L             0x44
#define GYRO_YOUT_H             0x45
#define GYRO_YOUT_L             0x46
#define GYRO_ZOUT_H             0x47
#define GYRO_ZOUT_L             0x48
#define PWR_MGMT_1              0x6B    //电源管理，典型值：0x00(正常启用)
#define WHO_AM_I                0x75    //IIC地址寄存器(默认数值0x68，只读)
#define MPU1 0x69
#define MPU2  0x68 //MPU-6050的I2C地址
#define nValCnt  7 //一次读取寄存器的数量
#define nCalibTimes  500 //校准时读数的次数
#define fRad2Deg 57.295779513 //将弧度转为角度的乘数

int fd1,fd2;
short calibData[nValCnt*2]; //校准数据
unsigned long nLastTime = 0; //上一次读数的时间
unsigned long nCurTime = 0;
double fLastRoll1 = 0.0; //上一次滤波得到的Roll角
double fLastPitch1 = 0.0; //上一次滤波得到的Pitch角
double fLastYaw1 = 0.0;
double fLastRoll2 = 0.0; //上一次滤波得到的Roll角
double fLastPitch2 = 0.0; //上一次滤波得到的Pitch角
double fLastYaw2 = 0.0;
char detectStarted = 0;//false

struct Kalman
{
	double Q_angle; // Process noise variance for the accelerometer
	double Q_bias; // Process noise variance for the gyro bias
	double R_measure; // Measurement noise variance - this is actually the variance of the measurement noise
	double angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
	double bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
	double rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

	double P[2][2]; // Error covariance matrix - This is a 2x2 matrix
}*kalmanRoll1, *kalmanPitch1, *kalmanRoll2, *kalmanPitch2;

/***************************/

#define TOPRANGE 200
#define BOTTOMRANGE 164

const int BOTTOMPIN = 1;
const int TOPPIN = 0;

/***************************/

#define LED1 21
#define LED2 6

/***************************/

#define Trig_pin 4
#define Echo_pin 5

double distance = 0;

/***************************/

#define VOICE 2

/***************************/

#define IPSTR "127.0.0.1"
#define PORT 80
#define BUFSIZE 1024

int sockfd, ret, i, h;
struct sockaddr_in servaddr;
char buf[BUFSIZE], *str;
socklen_t len;
fd_set   t_set1;
struct timeval  tv;

void reset(void)
{
		digitalWrite(A_ENABLE_1, HIGH);
		digitalWrite(A_ENABLE_2, HIGH);
		digitalWrite(A_IN_1, LOW);
		digitalWrite(A_IN_2, LOW);
		digitalWrite(B_ENABLE_1, HIGH);
		digitalWrite(B_ENABLE_2, HIGH);
		digitalWrite(B_IN_1, LOW);
		digitalWrite(B_IN_2, LOW);
}

void go(int direction)
{
	switch(direction)
	{
		case UP:
			digitalWrite(A_IN_1, LOW);
			digitalWrite(A_IN_2, HIGH);
			digitalWrite(B_IN_1, LOW);
			digitalWrite(B_IN_2, HIGH);
			break;
		case DOWN:
			digitalWrite(A_IN_1, HIGH);
			digitalWrite(A_IN_2, LOW);
			digitalWrite(B_IN_1, HIGH);
			digitalWrite(B_IN_2, LOW);
			break;
		case LEFT:
			digitalWrite(A_IN_1, HIGH);
			digitalWrite(A_IN_2, LOW);
			digitalWrite(B_IN_1, LOW);
			digitalWrite(B_IN_2, HIGH);
			break;
		case RIGHT:
			digitalWrite(A_IN_1, LOW);
			digitalWrite(A_IN_2, HIGH);
			digitalWrite(B_IN_1, HIGH);
			digitalWrite(B_IN_2, LOW);
			break;
	}
}

void stop(void)
{
	digitalWrite(A_IN_1, LOW);
	digitalWrite(A_IN_2, LOW);
	digitalWrite(B_IN_1, LOW);
	digitalWrite(B_IN_2, LOW);
}

void init(struct Kalman *this)
{
	this->Q_angle = 0.001;
	this->Q_bias = 0.003;
	this->R_measure = 0.03;
	this->angle = 0.0;
	this->bias = 0.0;
	this->rate = 0.0;
	for(int i=0;i<4;i++)
		this->P[i/2][i%2]=0.0;
}

double getAngle(struct Kalman *this, double newAngle, double newRate, double dt) 
{
	// KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
	// Modified by Kristian Lauszus
	// See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

	// Discrete Kalman filter time update equations - Time Update ("Predict")
	// Update xhat - Project the state ahead
	/* Step 1 */
	this->rate = newRate - this->bias;
	this->angle += dt * this->rate;

	// Update estimation error covariance - Project the error covariance ahead
	/* Step 2 */
	this->P[0][0] += dt * (dt*(this->P[1][1]) - this->P[0][1] - this->P[1][0] + this->Q_angle);
	this->P[0][1] -= dt * (this->P[1][1]);
	this->P[1][0] -= dt * (this->P[1][1]);
	this->P[1][1] += this->Q_bias * dt;

	// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
	// Calculate Kalman gain - Compute the Kalman gain
	/* Step 4 */
	double S = this->P[0][0] + this->R_measure; // Estimate error
	/* Step 5 */
	double K[2]; // Kalman gain - This is a 2x1 vector
	K[0] = this->P[0][0] / S;
	K[1] = this->P[1][0] / S;

	// Calculate angle and bias - Update estimate with measurement zk (newAngle)
	/* Step 3 */
	double y = newAngle - this->angle; // Angle difference
	/* Step 6 */
	this->angle += K[0] * y;
	this->bias += K[1] * y;

	// Calculate estimation error covariance - Update the error covariance
	/* Step 7 */
	double P00_temp = this->P[0][0];
	double P01_temp = this->P[0][1];

	this->P[0][0] -= K[0] * P00_temp;
	this->P[0][1] -= K[0] * P01_temp;
	this->P[1][0] -= K[1] * P00_temp;
	this->P[1][1] -= K[1] * P01_temp;

	return this->angle;
};

//向MPU6050写入一个字节的数据
//指定寄存器地址与一个字节的值
void WriteMPUReg(int fd, int nReg, unsigned char nVal)
{
	wiringPiI2CWriteReg8(fd, nReg, nVal);
}

//从MPU6050读出一个字节的数据
//指定寄存器地址，返回读出的值
unsigned char ReadMPUReg(int fd, int nReg) 
{
	return wiringPiI2CReadReg8(fd, nReg);
}


//从MPU6050读出加速度计三个分量、温度和三个角速度计
//保存在指定的数组中
void ReadAccGyr(int fd, short *pVals) 
{
	for (long i = 0; i < nValCnt; i++) 
	{
		pVals[i] = ReadMPUReg(fd, 0x3B+i*2) << 8 | ReadMPUReg(fd, 0x3C+i*2);
	}
}

//对大量读数进行统计，校准平均偏移量
void Calibration(int identifier)
{
	memset(calibData, 0, sizeof(calibData));
	double valSums[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	//先求和
	for (int i = 0; i < nCalibTimes; ++i) 
	{
		short mpuVals[nValCnt];
		ReadAccGyr(identifier==1?fd1:fd2, mpuVals);
		for (int j = 0; j < nValCnt; ++j) 
		{
			valSums[j] += mpuVals[j];
		}
	}
	//再求平均
	for (int i = 0; i < nValCnt; ++i) 
	{
		calibData[(identifier-1)*7+i] = (int)(valSums[i] / nCalibTimes);
	}
	calibData[(identifier-1)*7+2] -= 16384; //设芯片Z轴竖直向下，设定静态工作点。
}

char* itoa(int num,char* str,int radix)
{
	char index[]="0123456789ABCDEF";
	unsigned unum;
	int i=0,j,k;
	if(radix==10&&num<0)
	{
		unum=(unsigned)-num;
		str[i++]='-';
	}
	else unum=(unsigned)num;
	do
	{
		str[i++]=index[unum%(unsigned)radix];
		unum/=radix;
	}while(unum);
	str[i]='\0';
	if(str[0]=='-')
		k=1;
	else
		k=0;
	char temp;
	for(j=k;j<=(i-1)/2;j++)
	{
		temp=str[j];
		str[j]=str[i-1+k-j];
		str[i-1+k-j]=temp;
	}
	return str;
}

// void Calibration(int identifier)
// {
//   double valSums1[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//   double valSums2[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//   //先求和
//   for (int i = 0; i < nCalibTimes; ++i) {
//     short mpuVals1[nValCnt];
//     short mpuVals2[nValCnt];
//     ReadAccGyr(fd1, mpuVals1);
//     ReadAccGyr(fd2, mpuVals2);
//     for (int j = 0; j < nValCnt; ++j) {
//       valSums1[j] += mpuVals1[j];
//       valSums2[j] += mpuVals2[j];
//     }
//   }
//   //再求平均
//   for (int i = 0; i < nValCnt; ++i) {
//     calibData[i] = (int)(valSums1[i] / nCalibTimes);
//     calibData[7+i] = (int)(valSums2[i] / nCalibTimes);
//   }
//   calibData[2] -= 16384;
//   calibData[9] -= 16384; //设芯片Z轴竖直向下，设定静态工作点。
// }

//算得Roll角。算法见文档。
double GetRoll(double *pRealVals, double fNorm)
{
	double fz = fabs(pRealVals[2]), fy = fabs(pRealVals[1]);
	double ftan = fy / fz;
	// // printf("**********roll:%lf***********\n", atan(ftan) * fRad2Deg);
	return atan(ftan) * fRad2Deg;
}

//算得Pitch角。算法见文档。
double GetPitch(double *pRealVals, double fNorm)
{
	double fz = fabs(pRealVals[2]), fx = fabs(pRealVals[0]);
//   // printf("**********%lf***********\n", pRealVals[2]);
	double ftan = fx / fz;
	// printf("**********pitch:%lf***********\n", atan(ftan) * fRad2Deg);
	return atan(ftan) * fRad2Deg;
}

//利用陀螺仪积分
double GetYaw(double yaw, double rotateSpeed, double dt) 
{
	// printf("**********yaw:%lf***********\n", yaw+rotateSpeed*dt);
	return yaw+rotateSpeed*dt;
}

//对读数进行纠正，消除偏移，并转换为物理量。公式见文档。
void Rectify(int identifier, short *pReadout, double *pRealVals)
{
	for (int i = 0; i < 3; ++i)
	{
		pRealVals[i] = (double)(pReadout[i] - calibData[(identifier-1)*7+i]) / 16384.0;
	}
	pRealVals[3] = pReadout[3] / 340.0 + 36.53;
	for (int i = 4; i < 7; ++i)
	{
		pRealVals[i] = (double)(pReadout[i] - calibData[(identifier-1)*7+i]) / 131.0;
		// printf("----------%lf------------\n", pRealVals[i]);
	}
}

void getDis()
{
	struct timeval start_time;
	struct timeval stop_time;

	digitalWrite (Trig_pin, LOW); 
	delay(10); // 延时10 毫秒  等待让电平稳定
	digitalWrite (Trig_pin, HIGH);
	delayMicroseconds(10); // 给一个10us的高电平
	// 这里发现 delayMicroseconds()，这个函数是不准确的，实际是大于10us，具体结果如何，需要示波器测量一下。
	digitalWrite (Trig_pin, LOW);

	while(!(digitalRead(Echo_pin) == 1));      
	gettimeofday(&start_time, NULL);           // 获取当前时间 开始接收到返回信号的时候     
	while(!(digitalRead(Echo_pin) == 0));      
	gettimeofday(&stop_time, NULL);           // 获取当前时间 结束信号

	double start, stop;
	start = start_time.tv_sec * 1000000 + start_time.tv_usec;   //微秒级的时间  
	stop  = stop_time.tv_sec * 1000000 + stop_time.tv_usec;
	distance = (stop - start) / 1000000 * 34000 / 2;  //计算时间差求出距离
	// 这里测试的 距离 实际就是上面时序图的回响时间长度乘以声速的结果。
}

void setup() 
{
	wiringPiSetup();
	fd1 = wiringPiI2CSetup(MPU1);
	fd2 = wiringPiI2CSetup(MPU2);
	//WriteMPUReg(0x6B, 0);
	if (fd1 >= 0 && fd2 >= 0) 
	{ // fd 为负数，说明IIC连接失败
		// printf("fd1 = %d\n",fd1);
		// printf("fd2 = %d\n",fd2);
		wiringPiI2CWriteReg8(fd1,PWR_MGMT_1,0x00); // 开启温度检测 关闭休眠
		wiringPiI2CWriteReg8(fd1,SMPLRT_DIV, 0x07);
		wiringPiI2CWriteReg8(fd1,CONFIG, 0x06);
		wiringPiI2CWriteReg8(fd1,GYRO_CONFIG, 0x00);
		wiringPiI2CWriteReg8(fd1,ACCEL_CONFIG, 0x01);
		wiringPiI2CWriteReg8(fd2,PWR_MGMT_1,0x00); // 开启温度检测 关闭休眠
		wiringPiI2CWriteReg8(fd2,SMPLRT_DIV, 0x07);
		wiringPiI2CWriteReg8(fd2,CONFIG, 0x06);
		wiringPiI2CWriteReg8(fd2,GYRO_CONFIG, 0x00);
		wiringPiI2CWriteReg8(fd2,ACCEL_CONFIG, 0x01);
	}
	else 
	{
		// printf("IIC初始化失败");
	}

	softPwmCreate(BOTTOMPIN, 15, BOTTOMRANGE);//initialize
	softPwmCreate(TOPPIN, 5, TOPRANGE);

	pinMode(A_ENABLE_1, OUTPUT);
	pinMode(A_ENABLE_2, OUTPUT);
	pinMode(A_IN_1, OUTPUT);
	pinMode(A_IN_2, OUTPUT);
	pinMode(B_ENABLE_1, OUTPUT);
	pinMode(B_ENABLE_2, OUTPUT);
	pinMode(B_IN_1, OUTPUT);
	pinMode(B_IN_2, OUTPUT);
	
	digitalWrite(A_ENABLE_1, HIGH);
	digitalWrite(A_ENABLE_2, HIGH);
	digitalWrite(B_ENABLE_1, HIGH);
	digitalWrite(B_ENABLE_2, HIGH);

	pinMode(LED1, OUTPUT);
	pinMode(LED2, OUTPUT);

	pinMode (Trig_pin, OUTPUT);
	pinMode (Echo_pin, INPUT);

	pinMode(VOICE, INPUT);

	digitalWrite(LED1, HIGH);
	digitalWrite(LED2, HIGH);

	Calibration(1); //执行校准
	Calibration(2);//TODO

	digitalWrite(LED1, LOW);
	digitalWrite(LED2, LOW);

	kalmanRoll1 = (struct Kalman *)malloc(sizeof(struct Kalman));
	init(kalmanRoll1);
	kalmanPitch1 = (struct Kalman *)malloc(sizeof(struct Kalman));
	init(kalmanPitch1);
	// kalmanRoll2 = (struct Kalman *)malloc(sizeof(struct Kalman));
	// init(kalmanRoll2);
	// kalmanPitch2 = (struct Kalman *)malloc(sizeof(struct Kalman));
	// init(kalmanPitch2);
	do
	{
		sleep(2);
		if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0 ) 
		{
			// printf("创建网络连接失败,尝试重连---socket error!\n");
			sleep(1);
			continue;
		};

		bzero(&servaddr, sizeof(servaddr));
		servaddr.sin_family = AF_INET;
		servaddr.sin_port = htons(PORT);
		if (inet_pton(AF_INET, IPSTR, &servaddr.sin_addr) <= 0 )
		{
			// printf("创建网络连接失败,尝试重连--inet_pton error!\n");
			sleep(1);
			continue;
		};

		if (connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
		{
			// printf("连接到服务器失败,connect error!\n");
			sleep(1);
			continue;
		}
		// printf("与远端建立了连接\n");
		break;
	}while(1);
	nLastTime = micros(); //记录当前时间
	nCurTime = micros();
}

int main()
{
	setup();
	while(1)
	{
		short readouts1[nValCnt], readouts2[nValCnt];
		ReadAccGyr(fd1, readouts1); //读出测量值
		ReadAccGyr(fd2, readouts2);

		double realVals1[7];
		double realVals2[7];
		Rectify(1, readouts1, realVals1); //根据校准的偏移量进行纠正
		Rectify(2, readouts2, realVals2);

		//计算加速度向量的模长，均以g为单位
		double fNorm1 = sqrt(realVals1[0] * realVals1[0] + realVals1[1] * realVals1[1] + realVals1[2] * realVals1[2]);
		double fNorm2 = sqrt(realVals2[0] * realVals2[0] + realVals2[1] * realVals2[1] + realVals2[2] * realVals2[2]);

		// printf("NORMAL:%lf  %lf\n",fNorm1, fNorm2);
		double fRoll1 = GetRoll(realVals1, fNorm1); //计算Roll角
		if (realVals1[1] < 0)
			fRoll1 = 0;
		//   double fPitch1 = GetPitch(realVals1, fNorm1); //计算Pitch角
		//   if (realVals1[0] < 0) {
		//   fPitch1 = -fPitch1;
		//   }

		double fRoll2 = GetRoll(realVals2, fNorm2); //计算Roll角
		if (realVals2[1] > 0)
			fRoll2 = -fRoll2;
		double fPitch2 = GetPitch(realVals2, fNorm2); //计算Pitch角
		if (realVals2[0] > 0)
			fPitch2 = -fPitch2;

		//计算两次测量的时间间隔dt，以秒为单位
		nCurTime = micros();
		double dt = (double)(nCurTime - nLastTime) / 1000000.0;
		//对Yaw角度进行积分
		double fYaw1 = GetYaw(fLastYaw1, realVals1[6], dt);
		//   double fYaw2 = GetYaw(fLastYaw2, realVals2[6], dt);
		//对Roll角和Pitch角进行卡尔曼滤波
		double fNewRoll1 = getAngle(kalmanRoll1, fRoll1, realVals1[4], dt);
		if(fNewRoll1<0)fNewRoll1=0;
		//   double fNewPitch1 = getAngle(kalmanPitch1, fPitch1, realVals1[5], dt);

		double fNewRoll2 = fRoll2;//getAngle(kalmanRoll2, fRoll2, realVals2[4], dt);
		double fNewPitch2 = fPitch2;//getAngle(kalmanPitch2, fPitch2, realVals2[5], dt);

		//更新Roll角和Pitch角
		fLastRoll1 = fNewRoll1;
		fLastRoll2 = fNewRoll2;
		//   fLastPitch1 = fNewPitch1;
		fLastPitch2 = fNewPitch2;
		fLastYaw1 = fYaw1;
		//   fLastYaw2 = fYaw2;
		//更新本次测的时间
		nLastTime = nCurTime;

		// printf("Roll:");
		// printf("%lf %lf\tPitch: %lf\tYaw:%lf", fNewRoll1, fNewRoll2, fNewPitch2, fYaw1);
		
		if(started)
		{
			if(!detectStarted)
			{
				if(fNewPitch2<-30)
				{
					go(LEFT);
					// printf("\n***left to go***\n\n");
				}
				else if(fNewPitch2>30)
				{
					go(RIGHT);
					// printf("\n***right to go***\n\n");
				}
				else if(fNewRoll2>30)
				{
					go(UP);
					// printf("\n***up to go***\n\n");
				}
				else if(fNewRoll2<-30)
				{
					go(DOWN);
					// printf("\n***down to go***\n\n");
				}
				else
				{
					// printf("\n***stand by***\n\n");
					stop();
				}
			}
			
			if(realVals2[2]>1.5&&started)
			{
				// if(!started)
				// {
				// 	started=1;
				// 	digitalWrite(LED1, HIGH);
				// 	digitalWrite(LED2, HIGH);
				// 	Calibration(2);
				// 	digitalWrite(LED1, LOW);
				// 	digitalWrite(LED2, LOW);
				// }
				// else
				// {
				stop();
				detectStarted ^= 1;
				softPwmWrite(BOTTOMPIN, 15.0);
				softPwmWrite(TOPPIN, 5.0);
				fLastYaw1 = 0;
				fYaw1 = 0;
				digitalWrite(LED1, HIGH);
				digitalWrite(LED2, HIGH);
				Calibration(1);
				digitalWrite(LED1, LOW);
				digitalWrite(LED2, LOW);
				// }
			}
			if(detectStarted)
			{
				stop();
				softPwmWrite(BOTTOMPIN, fYaw1/(1800/BOTTOMRANGE)+15.0);
				softPwmWrite(TOPPIN, fNewRoll1/(1800/TOPRANGE)+5.0);
				// printf("bottom:%lf top:%lf\n", fYaw1/(1800/BOTTOMRANGE)+15.0, fNewRoll1/(1800/TOPRANGE)+5.0);
			}
			getDis();
			char str1[4096], para[16];
			sprintf(para, "%.2lf",distance);
			memset(str1, 0, 4096);
			strcat(str1, "GET /api/setDistance?distance=");
			strcat(str1, para);
			strcat(str1," HTTP/1.1\n");
			strcat(str1, "\r\n\r\n");
			// printf("%s\n",str1);

			ret = write(sockfd,str1,strlen(str1));
			if (ret < 0) 
			{
				// printf("发送失败！错误代码是%d，错误信息是'%s'\n",errno, strerror(errno));
				continue;
			}else
			{
				// printf("消息发送成功，共发送了%d个字节！\n\n", ret);
			}
		}
		if (digitalRead(VOICE) == LOW)
		{
			// printf("sound!\n");
			started^=1;
			stop();
			digitalWrite(LED1, HIGH);
			digitalWrite(LED2, HIGH);
			Calibration(2);
			digitalWrite(LED1, LOW);
			digitalWrite(LED2, LOW);
		}
		delay(10);
	}
	return 0;
}
