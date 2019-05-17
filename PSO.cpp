/*
* ������Ѱ�ŷ����Ժ���Ϊ
* f(x, y) = sin(sqrt(x ^ 2 + y ^ 2)) / (sqrt(x ^ 2 + y ^ 2)) + exp((cos(2 * PI*x) + cos(2 * PI*y)) / 2) - 2.71289
* �ú����кܶ�ֲ�����ֵ�㣬������λ��Ϊ(0, 0), ��(0, 0)����ȡ�ü���ֵ
*/

#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<time.h>
#define c1 1.49445 //���ٶ�����һ���Ǹ��ݴ���ʵ������
#define c2 1.49445
#define maxgen 300  // ��������
#define repeat 100 // �ظ�ʵ�����
#define sizepop 20 // ��Ⱥ��ģ
#define popmax 2 // ����λ�����ȡֵ
#define popmin -2 // ����λ����Сȡֵ
#define Vmax 0.5 // �ٶ����ֵ
#define Vmin -0.5 //�ٶ���Сֵ
#define dim 2 // ���ӵ�ά��
#define w_start 0.9
#define w_end 0.4
#define PI 3.1415926 //Բ����

double pop[sizepop][dim]; // ������Ⱥ����
double V[sizepop][dim]; // ������Ⱥ�ٶ�����
double fitness[sizepop]; // ������Ⱥ����Ӧ������
double result[maxgen];  //������ÿ�ε�����Ⱥ����ֵ������
double pbest[sizepop][dim];  // ���弫ֵ��λ��
double gbest[dim]; //Ⱥ�弫ֵ��λ��
double fitnesspbest[sizepop]; //���弫ֵ��Ӧ�ȵ�ֵ
double fitnessgbest; // Ⱥ�弫ֵ��Ӧ��ֵ
double genbest[maxgen][dim]; //ÿһ������ֵȡֵ����

double func(double * arr);	//��Ӧ�Ⱥ���
void pop_init(void);	 // ��Ⱥ��ʼ��
double * max(double * fit, int size);	// max()��������
void PSO_func(int n);	// ����Ѱ�ţ�����Ĳ���Ϊһ��������ȡֵΪ1��5,�ֱ����5�ֲ�ͬ�ļ���w�ķ���


int main(void)
{
	clock_t start, finish; //����ʼ�ͽ���ʱ��
	start = clock(); //��ʼ��ʱ
	srand((unsigned)time(NULL)); // ��ʼ�����������
	for (int i = 1; i <= 5; i++)
	{
		int near_best = 0; // �ӽ����Ž�Ĵ���
		double best_sum = 0; // �ظ�����ֵ���
		double best = 0; // �ظ�ʵ��õ������Ž�
		for (int j = 0; j < repeat; j++)    //repeatΪ��������
		{
			PSO_func(i); // ��i��w����ȡֵ����ʾ���õ�i�ֹ���Ȩ��w�����Եݼ���ʽ���������Ⱥ����
			double * best_fit_index = max(result, maxgen);  //result�Ǵ��ÿ�ε�������ֵ�����飬maxgen��������
			double best_result = *(best_fit_index + 1); //���Ž�
			if (best_result > 0.95)
				near_best++;
			if (best_result > best)
				best = best_result;
			best_sum += best_result;
		}
		double average_best = best_sum / repeat; //�ظ�ʵ��ƽ������ֵ
		printf("w�����ĵ�%d�ַ���:\n", i);
		printf("�ظ�ʵ��%d�Σ�ÿ��ʵ�����%d��,�ӽ����Ž��ʵ�����Ϊ%d�Σ��������ֵΪ:%lf,ƽ������ֵΪ:%lf\n",
			    repeat, maxgen, near_best, best, average_best);
	}
	finish = clock(); //����ʱ��
	double duration = (double)(finish - start) / CLOCKS_PER_SEC; // ��������ʱ��
	printf("�������к�ʱ:%lf\n", duration);
	system("pause");
	return 0;
}
							 
double func(double * arr)	//��Ӧ�Ⱥ���
{
	double x = *arr; //x ��ֵ
	double y = *(arr + 1); //y��ֵ
	double fitness = sin(sqrt(x*x + y*y)) / (sqrt(x*x + y*y)) + exp((cos(2 * PI*x) + cos(2 * PI*y)) / 2) - 2.71289;
	return fitness;

}

void pop_init(void)	 // ��Ⱥ��ʼ��
{
	for (int i = 0; i<sizepop; i++)
	{
		for (int j = 0; j<dim; j++)   //dimΪ����ά��2
		{
			pop[i][j] = (((double)rand()) / RAND_MAX - 0.5) * 4; //-2��2֮��������
			V[i][j] = ((double)rand()) / RAND_MAX - 0.5; //-0.5��0.5֮��
		}
		fitness[i] = func(pop[i]); //������Ӧ�Ⱥ���ֵ
								   //pop[i]��ʾ��ά����pop��pop[i][0]�ĵ�ַ
	}
}

double * max(double * fit, int size)	// ����Ӧ�Ⱥ���ֵ���ֵ����λ��
{
	int index = 0; // ��ʼ�����
	double max = *fit; //��ʼ�����ֵΪ�����һ��Ԫ��
	static double best_fit_index[2];
	for (int i = 1; i<size; i++)
	{
		if (*(fit + i) > max)
			max = *(fit + i);
		index = i;
	}
	best_fit_index[0] = index;
	best_fit_index[1] = max;
	return best_fit_index;

}

void PSO_func(int n)	// ����Ѱ�ţ�����Ĳ���Ϊһ��������ȡֵΪ1��5,�ֱ����5�ֲ�ͬ�ļ���w�ķ���
{
	pop_init();
	double * best_fit_index; // ���ڴ��Ⱥ�弫ֵ����λ��(���)
	best_fit_index = max(fitness, sizepop); //��Ⱥ�弫ֵ
	int index = (int)(*best_fit_index);
	// Ⱥ�弫ֵλ��
	for (int i = 0; i<dim; i++)    //dimΪ����ά��2
	{
		gbest[i] = pop[index][i];
	}
	// ���弫ֵλ��
	for (int i = 0; i<sizepop; i++)
	{
		for (int j = 0; j<dim; j++)
		{
			pbest[i][j] = pop[i][j];
		}
	}
	// ���弫ֵ��Ӧ��ֵ
	for (int i = 0; i<sizepop; i++)
	{
		fitnesspbest[i] = fitness[i];
	}
	//Ⱥ�弫ֵ��Ӧ��ֵ
	double bestfitness = *(best_fit_index + 1);
	fitnessgbest = bestfitness;

	//����Ѱ��
	for (int i = 0; i<maxgen; i++)
	{
		for (int j = 0; j<sizepop; j++)
		{
			//�ٶȸ��¼����Ӹ���
			for (int k = 0; k<dim; k++)
			{
				// �ٶȸ���
				double rand1 = (double)rand() / RAND_MAX; //0��1֮��������
				double rand2 = (double)rand() / RAND_MAX;
				double w;
				double Tmax = (double)maxgen;  //��������
				switch (n)
				{
				case 1:
					w = 1;
				case 2:
					w = w_end + (w_start - w_end)*(Tmax - i) / Tmax;
				case 3:
					w = w_start - (w_start - w_end)*(i / Tmax)*(i / Tmax);
				case 4:
					w = w_start + (w_start - w_end)*(2 * i / Tmax - (i / Tmax)*(i / Tmax));
				case 5:
					w = w_end*(pow((w_start / w_end), (1 / (1 + 10 * i / Tmax))));
				default:
					w = 1;
				}
				V[j][k] = w*V[j][k] + c1*rand1*(pbest[j][k] - pop[j][k]) + c2*rand2*(gbest[k] - pop[j][k]);
				//�ٶ�����
				if (V[j][k] > Vmax)
					V[j][k] = Vmax;
				if (V[j][k] < Vmin)
					V[j][k] = Vmin;
				// ���Ӹ���
				pop[j][k] = pop[j][k] + V[j][k];
				//λ������
				if (pop[j][k] > popmax)
					pop[j][k] = popmax;
				if (pop[j][k] < popmin)
					pop[j][k] = popmin;
			}
			fitness[j] = func(pop[j]); //�����ӵ���Ӧ��ֵ
		}
		for (int j = 0; j<sizepop; j++)
		{
			// ���弫ֵ����
			if (fitness[j] > fitnesspbest[j])
			{
				for (int k = 0; k<dim; k++)
				{
					pbest[j][k] = pop[j][k];
				}
				fitnesspbest[j] = fitness[j];
			}
			// Ⱥ�弫ֵ����
			if (fitness[j] > fitnessgbest)
			{
				for (int k = 0; k<dim; k++)
					gbest[k] = pop[j][k];
				fitnessgbest = fitness[j];
			}
		}
		for (int k = 0; k<dim; k++)
		{
			genbest[i][k] = gbest[k]; // ÿһ������ֵȡֵ����λ�ü�¼
		}
		result[i] = fitnessgbest; // ÿ��������ֵ��¼������
	}
}

