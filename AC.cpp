//该程序是以蚁群系统为模型写的蚁群算法程序(强调：非蚂蚁周模型)，以TSP问题为测试对象

#include <iostream>
#include <math.h>
#include <time.h>
using namespace std;

#define N 75  //城市节点数目
//城市坐标
double C[N][2]={
{6,25}, {7,43}, {9,56}, {10,70}, {11,28},
{12,17}, {12,38}, {15,5}, {15,14}, {15,56},
{16,19}, {17,64}, {20,30}, {21,48}, {21,45},
{21,36}, {22,53}, {22,22}, {26,29}, {26,13},
{26,59}, {27,24}, {29,39}, {30,50}, {30,20},
{30,60}, {31,76}, {33,34}, {33,44}, {35,51},
{35,16}, {35,60}, {36,6}, {36,26}, {38,33},
{40,37}, {40,66}, {40,60}, {40,20}, {41,46},
{43,26}, {44,13}, {45,42}, {45,35}, {47,66},
{48,21}, {50,30}, {50,40}, {50,50}, {50,70},
{50,4}, {50,15}, {51,42}, {52,26}, {54,38},
{54,10}, {55,34}, {55,45}, {55,50}, {55,65},
{55,57}, {55,20}, {57,72}, {59,5}, {60,15},
{62,57}, {62,48}, {62,35}, {62,24}, {64,4},
{65,27}, {66,14}, {66,8}, {67,41}, {70,64}
};

#define M 75	 //蚂蚁数量
int NcMax =100;  //最大循环次数NcMax

double alpha = 2, beta = 5, rou = 0.1, alpha1 = 0.1,  qzero = 0.1;
//信息启发因子，期望启发式因子，全局信息素挥发参数，局部信息素挥发参数, 状态转移公式中的q0

double allDistance[N][N];	//矩阵表示两两城市之间的距离
double Lnn;		//局部更新时候使用的的常量，它是由最近邻方法得到的一个长度
//最近邻方法:就是从源节点出发，每次选择一个距离最短的点来遍历所有的节点得到的路径，每个节点都可能作为源节点来遍历

int ChooseNextNode(int currentNode, int visitedNode[]);	  //选择下一个节点，配合下面的函数来计算的长度
double CalAdjacentDistance(int node);	//给一个节点由最近邻距离方法计算长度Lnn
double calculateDistance(int i, int j);	//计算两个城市之间的距离
void calculateAllDistance();	  //由矩阵表示两两城市之间的距离
double calculateSumOfDistance(int* tour);	//获得经过n个城市的路径长度

class ACSAnt;   //蚂蚁个体

class AntColonySystem     //蚁群系统
{
private:
	double info[N][N], visible[N][N];//节点之间的信息素量,节点之间的启发式信息量
public:
	AntColonySystem()
	{
	}	
	double Transition(int i, int j);	//计算当前节点到下一节点转移的概率
	void UpdateLocalPathRule(int i, int j);	//局部更新规则	
	void InitParameter(double value);	//初始化	
	void UpdateGlobalPathRule(int* bestTour, int globalBestLength);	//全局信息素更新
};

class ACSAnt                //蚂蚁个体
{
private:
	AntColonySystem* antColony;   //蚁群
protected:
	int startCity, cururentCity;//初始城市编号，当前城市编号
	int allowed[N];//禁忌表	
	int Tour[N][2];//当前路径，是一个路径段，即（currentcity，nextcity），用(Tour[i][0],Tour[i][1])表示
	int currentTourIndex;//当前路径索引，从0开始，存储蚂蚁经过城市的编号
public:
	ACSAnt(AntColonySystem* acs, int start)
	{
		antColony = acs;
		startCity = start;
	}	
	int* Search();	//开始搜索	
	int Choose();	//选择下一节点	
	void MoveToNextCity(int nextCity);	//移动到下一节点
};


int main()
{
	time_t timer, timerl;   //time_t 数据类型就是用来存储从1970年到现在经过了多少秒
	time(&timer);		//函数time()的返回值仍然是从1970年1月1日至今所经历的时间（以秒为单位）
						//返回值同时也赋给作为参数的指针（p）所指向的实体
	unsigned long seed = timer;
	seed %= 56000;
	srand((unsigned int)seed);  //初始化随机种子，保证后面的rand（）函数产生不一样的随机数
								
	calculateAllDistance();	//计算表示两两城市之间的距离
	
	AntColonySystem* acs = new AntColonySystem();	//蚁群系统对象
	ACSAnt* ants[M];
	for (int k = 0; k < M; k++)	//蚂蚁均匀分布在城市上
	{
		ants[k] = new ACSAnt(acs, (int)(k%N));		 //M = N = 75
	}
	int node = rand() % N;	//随机选择一个节点计算由最近邻方法得到的一个长度
	Lnn = CalAdjacentDistance(node);
	double initInfo = 1 / (N * Lnn);	//各条路径上初始化的信息素强度
	acs->InitParameter(initInfo);	  //初始化
									 
	int globalTour[N][2];        //全局最优路径，就是路径序列								
	double globalBestLength = 0.0;	 //全局最优长度
	for (int i = 0; i < NcMax; i++)    //NcMax最大循环次数
	{	
		int localTour[N][2];	//局部最优路径	
		double localBestLength = 0.0;	//局部最优长度	
		double tourLength;	//当前路径长度
		for (int j = 0; j < M; j++)
		{
			int* tourPath = ants[j]->Search();
			tourLength = calculateSumOfDistance(tourPath);
			//局部比较，并记录路径和长度
			if (tourLength < localBestLength || abs(localBestLength - 0.0) < 0.000001)
			{
				for (int m = 0; m< N; m++)
				{
					int row = *(tourPath + 2 * m);		
					int col = *(tourPath + 2 * m + 1);
					localTour[m][0] = row;
					localTour[m][1] = col;
				}
				localBestLength = tourLength;
			}
		}

		//全局比较，并记录路径和长度
		if (localBestLength < globalBestLength || abs(globalBestLength - 0.0) < 0.000001)
		{
			for (int m = 0; m< N; m++)
			{
				globalTour[m][0] = localTour[m][0];
				globalTour[m][1] = localTour[m][1];
			}
			globalBestLength = localBestLength;
		}
		acs->UpdateGlobalPathRule(*globalTour, globalBestLength);
		//输出所有蚂蚁循环一次后的迭代最优路径
		cout << "第 " << i + 1 << " 迭代最优路径:" << localBestLength << "	  " << endl;
		for (int m = 0; m< N; m++)
		{
			cout << localTour[m][0] << "	";
		}
		cout << endl;
	}

	//输出全局最优路径
	cout << "全局最优路径长度:" << globalBestLength << endl;
	cout << "全局最优路径:";
	for (int m = 0; m< N; m++)
	{
		cout << globalTour[m][0] << "	";
	}
	cout << endl;
	system("pause");
	return 0;
}

//计算当前节点到下一节点转移的概率
double AntColonySystem::Transition(int i, int j)
{
	if (i != j)
	{
		return (pow(info[i][j], alpha) * pow(visible[i][j], beta));  //采用的5.1的概率计算公式
	}
	else
	{
		return 0.0;
	}
}

//局部更新规则
void AntColonySystem::UpdateLocalPathRule(int i, int j)
{
	info[i][j] = (1.0 - alpha1) * info[i][j] + alpha1 * (1.0 / (N * Lnn));
	info[j][i] = info[i][j];
}

//初始化
void AntColonySystem::InitParameter(double value)
{
	//初始化路径上的信息素强度tao0
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < N; j++)
		{
			info[i][j] = value;   //信息素强度
			info[j][i] = value;
			if (i != j)
			{
				visible[i][j] = 1.0 / allDistance[i][j];   //启发式信息强度
				visible[j][i] = visible[i][j];
			}
		}
	}
}

//全局信息素更新
void AntColonySystem::UpdateGlobalPathRule(int* bestTour, int globalBestLength)
{
	for (int i = 0; i < N; i++)
	{
		int row = *(bestTour + 2 * i);          
		int col = *(bestTour + 2 * i + 1);
		info[row][col] = (1.0 - rou) * info[row][col] + rou * (1.0 / globalBestLength);   
		info[col][row] = info[row][col];
	}
}

//选择下一个节点，配合下面的函数来计算的长度
int ChooseNextNode(int currentNode, int visitedNode[])
{
	int nextNode = -1;
	double shortDistance = 0.0;
	for (int i = 0; i < N; i++)
	{
		//去掉已走过的节点,从剩下节点中选择距离最近的节点
		if (1 == visitedNode[i])
		{
			if (shortDistance == 0.0)
			{
				shortDistance = allDistance[currentNode][i];
				nextNode = i;
			}
			if (shortDistance < allDistance[currentNode][i])
			{
				nextNode = i;
			}
		}
	}
	return nextNode;
}

//给一个节点由最近邻距离方法计算长度
double CalAdjacentDistance(int node)
{
	double sum = 0.0;
	int visitedNode[N];
	for (int j = 0; j < N; j++)
	{
		visitedNode[j] = 1;
	}
	visitedNode[node] = 0;
	int currentNode = node;
	int nextNode;
	do
	{
		nextNode = ChooseNextNode(currentNode, visitedNode);
		if (nextNode >= 0)
		{
			sum += allDistance[currentNode][nextNode];  //allDistance为两城市间的距离矩阵
			currentNode = nextNode;
			visitedNode[currentNode] = 0;
		}
	} while (nextNode >= 0);           
	sum += allDistance[currentNode][node];
	return sum;
}


//开始搜索
int* ACSAnt::Search()
{
	cururentCity = startCity;
	int toCity;
	currentTourIndex = 0;        //当前路径索引，存储蚂蚁经过城市的编号
	for (int i = 0; i < N; i++)
	{
		allowed[i] = 1;          //禁忌表
	}
	allowed[cururentCity] = 0;	//cururentCity为当前城市编号
	int endCity;
	int count = 0;
	do
	{
		count++;
		endCity = cururentCity;
		toCity = Choose();	     //选择下一个节点	
		if (toCity >= 0)
		{
			MoveToNextCity(toCity);    //移动到下一个节点
			antColony->UpdateLocalPathRule(endCity, toCity);  //进行局部更新
			cururentCity = toCity;
		}
	} while (toCity >= 0);
	MoveToNextCity(startCity);
	antColony->UpdateLocalPathRule(endCity, startCity);

	return *Tour;  //Tour是一个二维数组，Tour表示首元素地址的地址
	/*
	tourPath为指向int数的指针，相当于一维数组tourpath[]；
	tourpath=*Tour，即将二维数组Tour首元素地址给tourpath；
	所以tourpath[0]=Tour首元素；
	tourpath[]={Tour[0][0],Tour[0][1],Tour[1][0],Tour[1][1],...Tour[74][0],Tour[74][1]}
	tourpath下标：   0			1			2		3				148			149
	对应路径序列：第1段路径:（*（tourpath），*（tourpath+1））
				 ...
				 第i段路径：（*（tourpath+2*（i-1）），*（tourpath+2*（i-1）+1））
	*/
}

//选择下一节点
int ACSAnt::Choose()
{
	int nextCity = -1;
	double q = rand() / (double)RAND_MAX;    //产生一个0~1之间的随机数q												 
	if (q <= qzero)	//如果 q <= q0,按先验知识，否则则按概率转移
	{
		double probability = -1.0;//转移到下一节点的概率
		for (int i = 0; i < N; i++)
		{
			//去掉禁忌表中已走过的节点,从剩下节点中选择最大概率的可行节点
			if (1 == allowed[i])
			{
				double prob = antColony->Transition(cururentCity, i);  //计算当前节点转移到下一节点的概率
				if (prob  > probability)
				{
					nextCity = i;
					probability = prob;
				}
			}
		}
	}
	else
	{
		//按概率转移			
		double p = rand() / (double)RAND_MAX;	//生成一个随机数,用来判断落在哪个区间段
		double sum = 0.0;
		double probability = 0.0;	//概率的区间点，p 落在哪个区间段，则该点是转移的方向										
		for (int i = 0; i < N; i++)	//计算概率公式的分母的值
		{
			if (1 == allowed[i])
			{
				sum += antColony->Transition(cururentCity, i);
			}
		}
		for (int j = 0; j < N; j++)
		{
			if (1 == allowed[j] && sum > 0)
			{
				probability += antColony->Transition(cururentCity, j) / sum; //往城市j转移的概率
				if (probability >= p || (p > 0.9999 && probability > 0.9999))
				{
					nextCity = j;
					break;
				}
			}
		}
	}
	return nextCity;
}

//移动到下一节点
void ACSAnt::MoveToNextCity(int nextCity)
{
	allowed[nextCity] = 0;    //禁忌表
	Tour[currentTourIndex][0] = cururentCity;	//当前路径
	Tour[currentTourIndex][1] = nextCity;
	currentTourIndex++;
	cururentCity = nextCity;
}


//计算两个城市之间的距离
double calculateDistance(int i, int j)
{
	return sqrt(pow((C[i][0]-C[j][0]),2.0) + pow((C[i][1]-C[j][1]),2.0));
}

//由矩阵表示两两城市之间的距离
void calculateAllDistance()
{
	for(int i = 0; i < N; i++)
	{
		for(int j = 0; j < N; j++)
		{
			if (i != j)
			{
				allDistance[i][j] = calculateDistance(i, j);
				allDistance[j][i] = allDistance[i][j];
			}
		}
	}
}

//获得经过n个城市的路径长度  
double calculateSumOfDistance(int* tour)
{
	double sum = 0;
	for(int i = 0; i< N ;i++)
	{
		int row = *(tour + 2 * i);
		int col = *(tour + 2* i + 1);
		sum += allDistance[row][col];
	}
	return sum;
}
