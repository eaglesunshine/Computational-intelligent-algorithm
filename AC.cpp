//�ó���������ȺϵͳΪģ��д����Ⱥ�㷨����(ǿ������������ģ��)����TSP����Ϊ���Զ���

#include <iostream>
#include <math.h>
#include <time.h>
using namespace std;

#define N 75  //���нڵ���Ŀ
//��������
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

#define M 75	 //��������
int NcMax =100;  //���ѭ������NcMax

double alpha = 2, beta = 5, rou = 0.1, alpha1 = 0.1,  qzero = 0.1;
//��Ϣ�������ӣ���������ʽ���ӣ�ȫ����Ϣ�ػӷ��������ֲ���Ϣ�ػӷ�����, ״̬ת�ƹ�ʽ�е�q0

double allDistance[N][N];	//�����ʾ��������֮��ľ���
double Lnn;		//�ֲ�����ʱ��ʹ�õĵĳ���������������ڷ����õ���һ������
//����ڷ���:���Ǵ�Դ�ڵ������ÿ��ѡ��һ��������̵ĵ����������еĽڵ�õ���·����ÿ���ڵ㶼������ΪԴ�ڵ�������

int ChooseNextNode(int currentNode, int visitedNode[]);	  //ѡ����һ���ڵ㣬�������ĺ���������ĳ���
double CalAdjacentDistance(int node);	//��һ���ڵ�������ھ��뷽�����㳤��Lnn
double calculateDistance(int i, int j);	//������������֮��ľ���
void calculateAllDistance();	  //�ɾ����ʾ��������֮��ľ���
double calculateSumOfDistance(int* tour);	//��þ���n�����е�·������

class ACSAnt;   //���ϸ���

class AntColonySystem     //��Ⱥϵͳ
{
private:
	double info[N][N], visible[N][N];//�ڵ�֮�����Ϣ����,�ڵ�֮�������ʽ��Ϣ��
public:
	AntColonySystem()
	{
	}	
	double Transition(int i, int j);	//���㵱ǰ�ڵ㵽��һ�ڵ�ת�Ƶĸ���
	void UpdateLocalPathRule(int i, int j);	//�ֲ����¹���	
	void InitParameter(double value);	//��ʼ��	
	void UpdateGlobalPathRule(int* bestTour, int globalBestLength);	//ȫ����Ϣ�ظ���
};

class ACSAnt                //���ϸ���
{
private:
	AntColonySystem* antColony;   //��Ⱥ
protected:
	int startCity, cururentCity;//��ʼ���б�ţ���ǰ���б��
	int allowed[N];//���ɱ�	
	int Tour[N][2];//��ǰ·������һ��·���Σ�����currentcity��nextcity������(Tour[i][0],Tour[i][1])��ʾ
	int currentTourIndex;//��ǰ·����������0��ʼ���洢���Ͼ������еı��
public:
	ACSAnt(AntColonySystem* acs, int start)
	{
		antColony = acs;
		startCity = start;
	}	
	int* Search();	//��ʼ����	
	int Choose();	//ѡ����һ�ڵ�	
	void MoveToNextCity(int nextCity);	//�ƶ�����һ�ڵ�
};


int main()
{
	time_t timer, timerl;   //time_t �������;��������洢��1970�굽���ھ����˶�����
	time(&timer);		//����time()�ķ���ֵ��Ȼ�Ǵ�1970��1��1��������������ʱ�䣨����Ϊ��λ��
						//����ֵͬʱҲ������Ϊ������ָ�루p����ָ���ʵ��
	unsigned long seed = timer;
	seed %= 56000;
	srand((unsigned int)seed);  //��ʼ��������ӣ���֤�����rand��������������һ���������
								
	calculateAllDistance();	//�����ʾ��������֮��ľ���
	
	AntColonySystem* acs = new AntColonySystem();	//��Ⱥϵͳ����
	ACSAnt* ants[M];
	for (int k = 0; k < M; k++)	//���Ͼ��ȷֲ��ڳ�����
	{
		ants[k] = new ACSAnt(acs, (int)(k%N));		 //M = N = 75
	}
	int node = rand() % N;	//���ѡ��һ���ڵ����������ڷ����õ���һ������
	Lnn = CalAdjacentDistance(node);
	double initInfo = 1 / (N * Lnn);	//����·���ϳ�ʼ������Ϣ��ǿ��
	acs->InitParameter(initInfo);	  //��ʼ��
									 
	int globalTour[N][2];        //ȫ������·��������·������								
	double globalBestLength = 0.0;	 //ȫ�����ų���
	for (int i = 0; i < NcMax; i++)    //NcMax���ѭ������
	{	
		int localTour[N][2];	//�ֲ�����·��	
		double localBestLength = 0.0;	//�ֲ����ų���	
		double tourLength;	//��ǰ·������
		for (int j = 0; j < M; j++)
		{
			int* tourPath = ants[j]->Search();
			tourLength = calculateSumOfDistance(tourPath);
			//�ֲ��Ƚϣ�����¼·���ͳ���
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

		//ȫ�ֱȽϣ�����¼·���ͳ���
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
		//�����������ѭ��һ�κ�ĵ�������·��
		cout << "�� " << i + 1 << " ��������·��:" << localBestLength << "	  " << endl;
		for (int m = 0; m< N; m++)
		{
			cout << localTour[m][0] << "	";
		}
		cout << endl;
	}

	//���ȫ������·��
	cout << "ȫ������·������:" << globalBestLength << endl;
	cout << "ȫ������·��:";
	for (int m = 0; m< N; m++)
	{
		cout << globalTour[m][0] << "	";
	}
	cout << endl;
	system("pause");
	return 0;
}

//���㵱ǰ�ڵ㵽��һ�ڵ�ת�Ƶĸ���
double AntColonySystem::Transition(int i, int j)
{
	if (i != j)
	{
		return (pow(info[i][j], alpha) * pow(visible[i][j], beta));  //���õ�5.1�ĸ��ʼ��㹫ʽ
	}
	else
	{
		return 0.0;
	}
}

//�ֲ����¹���
void AntColonySystem::UpdateLocalPathRule(int i, int j)
{
	info[i][j] = (1.0 - alpha1) * info[i][j] + alpha1 * (1.0 / (N * Lnn));
	info[j][i] = info[i][j];
}

//��ʼ��
void AntColonySystem::InitParameter(double value)
{
	//��ʼ��·���ϵ���Ϣ��ǿ��tao0
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < N; j++)
		{
			info[i][j] = value;   //��Ϣ��ǿ��
			info[j][i] = value;
			if (i != j)
			{
				visible[i][j] = 1.0 / allDistance[i][j];   //����ʽ��Ϣǿ��
				visible[j][i] = visible[i][j];
			}
		}
	}
}

//ȫ����Ϣ�ظ���
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

//ѡ����һ���ڵ㣬�������ĺ���������ĳ���
int ChooseNextNode(int currentNode, int visitedNode[])
{
	int nextNode = -1;
	double shortDistance = 0.0;
	for (int i = 0; i < N; i++)
	{
		//ȥ�����߹��Ľڵ�,��ʣ�½ڵ���ѡ���������Ľڵ�
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

//��һ���ڵ�������ھ��뷽�����㳤��
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
			sum += allDistance[currentNode][nextNode];  //allDistanceΪ�����м�ľ������
			currentNode = nextNode;
			visitedNode[currentNode] = 0;
		}
	} while (nextNode >= 0);           
	sum += allDistance[currentNode][node];
	return sum;
}


//��ʼ����
int* ACSAnt::Search()
{
	cururentCity = startCity;
	int toCity;
	currentTourIndex = 0;        //��ǰ·���������洢���Ͼ������еı��
	for (int i = 0; i < N; i++)
	{
		allowed[i] = 1;          //���ɱ�
	}
	allowed[cururentCity] = 0;	//cururentCityΪ��ǰ���б��
	int endCity;
	int count = 0;
	do
	{
		count++;
		endCity = cururentCity;
		toCity = Choose();	     //ѡ����һ���ڵ�	
		if (toCity >= 0)
		{
			MoveToNextCity(toCity);    //�ƶ�����һ���ڵ�
			antColony->UpdateLocalPathRule(endCity, toCity);  //���оֲ�����
			cururentCity = toCity;
		}
	} while (toCity >= 0);
	MoveToNextCity(startCity);
	antColony->UpdateLocalPathRule(endCity, startCity);

	return *Tour;  //Tour��һ����ά���飬Tour��ʾ��Ԫ�ص�ַ�ĵ�ַ
	/*
	tourPathΪָ��int����ָ�룬�൱��һά����tourpath[]��
	tourpath=*Tour��������ά����Tour��Ԫ�ص�ַ��tourpath��
	����tourpath[0]=Tour��Ԫ�أ�
	tourpath[]={Tour[0][0],Tour[0][1],Tour[1][0],Tour[1][1],...Tour[74][0],Tour[74][1]}
	tourpath�±꣺   0			1			2		3				148			149
	��Ӧ·�����У���1��·��:��*��tourpath����*��tourpath+1����
				 ...
				 ��i��·������*��tourpath+2*��i-1������*��tourpath+2*��i-1��+1����
	*/
}

//ѡ����һ�ڵ�
int ACSAnt::Choose()
{
	int nextCity = -1;
	double q = rand() / (double)RAND_MAX;    //����һ��0~1֮��������q												 
	if (q <= qzero)	//��� q <= q0,������֪ʶ�������򰴸���ת��
	{
		double probability = -1.0;//ת�Ƶ���һ�ڵ�ĸ���
		for (int i = 0; i < N; i++)
		{
			//ȥ�����ɱ������߹��Ľڵ�,��ʣ�½ڵ���ѡ�������ʵĿ��нڵ�
			if (1 == allowed[i])
			{
				double prob = antColony->Transition(cururentCity, i);  //���㵱ǰ�ڵ�ת�Ƶ���һ�ڵ�ĸ���
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
		//������ת��			
		double p = rand() / (double)RAND_MAX;	//����һ�������,�����ж������ĸ������
		double sum = 0.0;
		double probability = 0.0;	//���ʵ�����㣬p �����ĸ�����Σ���õ���ת�Ƶķ���										
		for (int i = 0; i < N; i++)	//������ʹ�ʽ�ķ�ĸ��ֵ
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
				probability += antColony->Transition(cururentCity, j) / sum; //������jת�Ƶĸ���
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

//�ƶ�����һ�ڵ�
void ACSAnt::MoveToNextCity(int nextCity)
{
	allowed[nextCity] = 0;    //���ɱ�
	Tour[currentTourIndex][0] = cururentCity;	//��ǰ·��
	Tour[currentTourIndex][1] = nextCity;
	currentTourIndex++;
	cururentCity = nextCity;
}


//������������֮��ľ���
double calculateDistance(int i, int j)
{
	return sqrt(pow((C[i][0]-C[j][0]),2.0) + pow((C[i][1]-C[j][1]),2.0));
}

//�ɾ����ʾ��������֮��ľ���
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

//��þ���n�����е�·������  
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
