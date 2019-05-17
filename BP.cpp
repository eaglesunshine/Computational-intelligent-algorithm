#include<stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <string>
#include <fstream>
#include <iomanip>
using namespace std;

#define innode 4//���������
#define hidenode 10//��������
#define outnode 3//���������
#define trainsample 75//ѵ��������
#define testsample 75//����������

double trainData[trainsample][innode];//��������
double outData[trainsample][outnode];//�������

double testData[testsample][innode];//��������

double w[innode][hidenode];//����㵽�����Ȩֵ
double w1[hidenode][outnode];//���㵽������Ȩֵ
double b1[hidenode];//������ֵ
double b2[outnode];//�������ֵ

double e = 0.0;//������
double error = 1.0;//�����������

double rate_w = 0.9;//����㵽�����ѧϰ��
double rate_w1 = 0.9;//���㵽������ѧϰ��
double rate_b1 = 0.9;//������ֵѧϰ��
double rate_b2 = 0.9;//�������ֵѧϰ��

double result[outnode];//bp���

					   //��ʼ������
void init(double w[], int n);
//Bpѵ������
void train(double trainData[trainsample][innode], double label[trainsample][outnode]);
//Bpʶ�� 
double *recognize(double *p);
//���ļ��ж�ȡ����
void readData(std::string filename, double data[][innode], int x);
//���ݹ�һ������
void changeData(double data[][innode], int x);

int main()
{
	int i, j;
	int trainNum = 0;//����ѵ������
	double *r; //���Խ��
	int count = 0;//��ȷ���Խ����
	double maxRate = 1.0;//�������е�������
						 //��Ȩֵ����ֵ���г�ʼ��
	init((double*)w, innode*hidenode);       //double *��ʾָ��double�͵�ָ�룬wΪ����㵽�����Ȩֵ���飬innode*hidenode=4*10=40
	init((double*)w1, hidenode*outnode);
	init(b1, hidenode);    //b1Ϊ������ֵ��hidenodeΪ��������
	init(b2, outnode);	  //b2Ϊ�������ֵ��outnodeΪ���������

						  //��ȡѵ������
	readData("./Iris-train.txt", trainData, trainsample);
	//��ѵ�����ݽ��й�һ������
	changeData(trainData, trainsample);

	for (i = 0; i<trainsample; i++)    //trainsampleΪ����������
	{
		printf("%d: ", i + 1);
		for (j = 0; j<innode; j++)
			printf("%5.2lf", trainData[i][j]);
		printf("\n");
	}
	
	//׼�������������3�໨��ÿ�໨��25������
	for (i = 0; i<trainsample; i++)
	{
		if (i<25)
		{
			outData[i][0] = 1;
			outData[i][1] = 0;
			outData[i][2] = 0;
		}
		else if (i<50)
		{
			outData[i][0] = 0;
			outData[i][1] = 1;
			outData[i][2] = 0;
		}
		else
		{
			outData[i][0] = 0;
			outData[i][1] = 0;
			outData[i][2] = 1;
		}
	}

	printf("��ʼѵ��\n");
	while (trainNum < 10000)    //trainNumΪ����ѵ������
	{
		e = 0.0;                 //eΪ���
		trainNum++;
		train(trainData, outData);      //BPѵ��
		printf("ѵ����%d�Σ� error=%8.4lf\n", trainNum, error);
	}
	printf("ѵ�����\n\n");

	//�����������
	readData("./Iris-test.txt", testData, testsample);
	//��һ����������
	changeData(testData, testsample);
	for (i = 0; i<testsample; i++)
	{
		r = recognize(testData[i]);
		for (j = 0; j<outnode; j++)
			printf("\t%7.4lf\t", r[j]);
		printf("\n");
		//�жϼ�����Ƿ���ȷ
		if (i<25 && r[0]>r[1] && r[0]>r[2])
			count++;
		if (i >= 25 && i<50 && r[1]>r[0] && r[1]>r[2])
			count++;
		if (i >= 50 && r[2]>r[0] && r[2]>r[1])
			count++;
	}

	printf("\n\n����%d����������� ��ȷ����%d���� ׼ȷ��: %7.4lf\n\n", testsample, count, (double)count / testsample);
	system("pause");
	system("pause");
	return 0;
}

//��ʼ��������0��1֮�������
void init(double w[], int n)
{
	int i;
	srand((unsigned int)time(NULL));    //��������Ӻ���srand����Ϊrand�����ṩ��ͬ�����ӣ�ÿ�����г��������ͬ�����������Ȼrand����ÿ�����г�����������������һ����
	for (i = 0; i<n; i++)
	{
		w[i] = 2.0*((double)rand() / RAND_MAX) - 1;     //RAND_MAX �� <stdlib.h> ��α��������ɺ��� rand ���ܷ��ص������ֵ��
														//����ζ�ţ��κ�һ�ζ� rand �ĵ��ã������õ�һ�� 0~RAND_MAX ֮���α�������RAND_MAX=0x7fff
	}
}

//BPѵ������
void train(double trainData[trainsample][innode], double label[trainsample][outnode])
{
	double x[innode];//����������ֵ
	double yd[outnode];//���������ֵ

	double o1[hidenode];//�����㼤��ֵ
	double o2[hidenode];//������㼤��ֵ
	double x1[hidenode];//����������������
	double x2[outnode];//����������
					   /*********************************************************************
					   o1: ��������ļ���ֵ������ý�������ĸ�·����Ȩֵ���·���ϵ�������˺�ȫ�����
					   **********************************************************************
					   x1: �����������������o1����ܼ���x1������ 1.0/(1.0 + exp-(����ֵ+�ý�����ֵ))
					   ***********************************************************************
					   o2: �������ļ���ֵ������ý�������ĸ�·���ϵ�Ȩֵ���·����������˺�ȫ�����
					   ***********************************************************************
					   x2: ������������������o2����ܼ���x2������ 1.0/(1.0 + exp-(����ֵ+�ý�����ֵ))
					   ***********************************************************************/

					   /*��ǰ���󡪡������Ԫƫ��qq���㷽ʽ��  ��������� - ʵ����������� ʵ����� ���� ��1-ʵ�������  */
	double qq[outnode];//�����������ʵ�������ƫ��

	double pp[hidenode];//�������У�����


	int issamp;
	int i, j, k;
	for (issamp = 0; issamp<trainsample; issamp++)
	{
		for (i = 0; i<innode; i++)
			x[i] = trainData[issamp][i];

		for (i = 0; i<outnode; i++)
			yd[i] = label[issamp][i];

		//������������ļ���ֵ����������ֵ
		for (i = 0; i<hidenode; i++)
		{
			o1[i] = 0.0;
			for (j = 0; j<innode; j++)
				o1[i] = o1[i] + w[j][i] * x[j];              //w[][]Ϊ����㵽�����������Ȩ�أ�o1�������㼤��ֵ
			x1[i] = 1.0 / (1.0 + exp(-o1[i] - b1[i]));  //x1������������
		}

		//�������������ļ���ֵ�����ֵ
		for (i = 0; i<outnode; i++)
		{
			o2[i] = 0.0;
			for (j = 0; j<hidenode; j++)
				o2[i] = o2[i] + w1[j][i] * x1[j];                //w1[][]Ϊ���㵽������Ȩֵ��o2Ϊ������㼤��ֵ
			x2[i] = 1.0 / (1.0 + exp(-o2[i] - b2[i]));        //x2Ϊ����������
		}

		//�õ���x2������������Ҫ���з��򴫲���

		//����ʵ����������������ƫ�����������㵽������·���ϵ�Ȩֵ
		for (i = 0; i<outnode; i++)
		{
			qq[i] = (yd[i] - x2[i]) * x2[i] * (1 - x2[i]);      //����ڵ�j��ƫ��
			for (j = 0; j<hidenode; j++)
				w1[j][i] = w1[j][i] + rate_w1*qq[i] * x1[j];   //���㵽������·���ϵ�Ȩֵ
		}

		//�������򴫲���������㵽����ĸ�·���ϵ�Ȩֵ
		for (i = 0; i<hidenode; i++)
		{
			pp[i] = 0.0;
			for (j = 0; j<outnode; j++)
				pp[i] = pp[i] + qq[j] * w1[i][j];
			pp[i] = pp[i] * x1[i] * (1.0 - x1[i]);      //������ڵ�i��ƫ��

			for (k = 0; k<innode; k++)
				w[k][i] = w[k][i] + rate_w*pp[i] * x[k];  //����㵽����ĸ�·���ϵ�Ȩֵ
		}

		//���������������
		for (k = 0; k<outnode; k++)
		{
			e += fabs(yd[k] - x2[k])*fabs(yd[k] - x2[k]); //���������  
		}
		error = e / 2.0;

		//����������������ֵ
		for (k = 0; k<outnode; k++)
			b2[k] = b2[k] + rate_b2*qq[k];

		//���������������ֵ
		for (j = 0; j<hidenode; j++)
			b1[j] = b1[j] + rate_b1*pp[j];
	}
}

//Bpʶ��
double *recognize(double *p)
{
	double x[innode];//�����ĸ�����ֵ
	double o1[hidenode];//�����㼤��ֵ
	double o2[hidenode];//������㼤��ֵ
	double x1[hidenode];//����������������
	double x2[outnode];//����������

	int i, j, k;

	for (i = 0; i<innode; i++)
		x[i] = p[i];

	for (j = 0; j<hidenode; j++)
	{
		o1[j] = 0.0;
		for (i = 0; i<innode; i++)
			o1[j] = o1[j] + w[i][j] * x[i]; //���������Ԫ����ֵ  
		x1[j] = 1.0 / (1.0 + exp(-o1[j] - b1[j])); //���������Ԫ���  
	}

	for (k = 0; k<outnode; k++)
	{
		o2[k] = 0.0;
		for (j = 0; j<hidenode; j++)
			o2[k] = o2[k] + w1[j][k] * x1[j];//��������Ԫ����ֵ  
		x2[k] = 1.0 / (1.0 + exp(-o2[k] - b2[k]));//��������Ԫ���   
	}

	for (k = 0; k<outnode; k++)
	{
		result[k] = x2[k];
	}
	return result;
}

//���ļ��ж�ȡ����
void readData(std::string filename, double data[][innode], int x)
{
	ifstream inData(filename, std::ios::in);
	int i, j;
	double dataLabel;
	for (i = 0; i<x; i++)
	{
		for (j = 0; j<innode; j++)
		{
			inData >> data[i][j];
		}
		inData >> dataLabel;
	}
	inData.close();
}

//���ݹ�һ�����������ǰ�����Ϊ��0��1��֮���С������Ҫ��Ϊ�����ݴ�����
void changeData(double data[][innode], int x)
{
	//��һ����ʽ:(x-min)/(max-min)  
	double minNum, maxNum;
	int i, j;
	minNum = data[0][0];
	maxNum = data[0][0];
	//�������Сֵ
	for (i = 0; i<x; i++)
	{
		for (j = 0; j<innode; j++)
		{
			if (minNum > data[i][j])
				minNum = data[i][j];
			if (maxNum < data[i][j])
				maxNum = data[i][j];
		}
	}
	//��һ��
	for (i = 0; i<x; i++)
	{
		for (j = 0; j<innode; j++)
			data[i][j] = (data[i][j] - minNum) / (maxNum - minNum);
	}
}