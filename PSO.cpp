/*
* 本例的寻优非线性函数为
* f(x, y) = sin(sqrt(x ^ 2 + y ^ 2)) / (sqrt(x ^ 2 + y ^ 2)) + exp((cos(2 * PI*x) + cos(2 * PI*y)) / 2) - 2.71289
* 该函数有很多局部极大值点，而极限位置为(0, 0), 在(0, 0)附近取得极大值
*/

#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<time.h>
#define c1 1.49445 //加速度因子一般是根据大量实验所得
#define c2 1.49445
#define maxgen 300  // 迭代次数
#define repeat 100 // 重复实验次数
#define sizepop 20 // 种群规模
#define popmax 2 // 个体位置最大取值
#define popmin -2 // 个体位置最小取值
#define Vmax 0.5 // 速度最大值
#define Vmin -0.5 //速度最小值
#define dim 2 // 粒子的维数
#define w_start 0.9
#define w_end 0.4
#define PI 3.1415926 //圆周率

double pop[sizepop][dim]; // 定义种群数组
double V[sizepop][dim]; // 定义种群速度数组
double fitness[sizepop]; // 定义种群的适应度数组
double result[maxgen];  //定义存放每次迭代种群最优值的数组
double pbest[sizepop][dim];  // 个体极值的位置
double gbest[dim]; //群体极值的位置
double fitnesspbest[sizepop]; //个体极值适应度的值
double fitnessgbest; // 群体极值适应度值
double genbest[maxgen][dim]; //每一代最优值取值粒子

double func(double * arr);	//适应度函数
void pop_init(void);	 // 种群初始化
double * max(double * fit, int size);	// max()函数定义
void PSO_func(int n);	// 迭代寻优，传入的参数为一个整数，取值为1到5,分别代表5种不同的计算w的方法


int main(void)
{
	clock_t start, finish; //程序开始和结束时间
	start = clock(); //开始计时
	srand((unsigned)time(NULL)); // 初始化随机数种子
	for (int i = 1; i <= 5; i++)
	{
		int near_best = 0; // 接近最优解的次数
		double best_sum = 0; // 重复最优值求和
		double best = 0; // 重复实验得到的最优解
		for (int j = 0; j < repeat; j++)    //repeat为迭代次数
		{
			PSO_func(i); // 第i种w参数取值，表示采用第i种惯性权重w的线性递减方式，完成粒子群搜索
			double * best_fit_index = max(result, maxgen);  //result是存放每次迭代最优值的数组，maxgen迭代次数
			double best_result = *(best_fit_index + 1); //最优解
			if (best_result > 0.95)
				near_best++;
			if (best_result > best)
				best = best_result;
			best_sum += best_result;
		}
		double average_best = best_sum / repeat; //重复实验平均最优值
		printf("w参数的第%d种方法:\n", i);
		printf("重复实验%d次，每次实验迭代%d次,接近最优解的实验次数为%d次，求得最优值为:%lf,平均最优值为:%lf\n",
			    repeat, maxgen, near_best, best, average_best);
	}
	finish = clock(); //结束时间
	double duration = (double)(finish - start) / CLOCKS_PER_SEC; // 程序运行时间
	printf("程序运行耗时:%lf\n", duration);
	system("pause");
	return 0;
}
							 
double func(double * arr)	//适应度函数
{
	double x = *arr; //x 的值
	double y = *(arr + 1); //y的值
	double fitness = sin(sqrt(x*x + y*y)) / (sqrt(x*x + y*y)) + exp((cos(2 * PI*x) + cos(2 * PI*y)) / 2) - 2.71289;
	return fitness;

}

void pop_init(void)	 // 种群初始化
{
	for (int i = 0; i<sizepop; i++)
	{
		for (int j = 0; j<dim; j++)   //dim为粒子维数2
		{
			pop[i][j] = (((double)rand()) / RAND_MAX - 0.5) * 4; //-2到2之间的随机数
			V[i][j] = ((double)rand()) / RAND_MAX - 0.5; //-0.5到0.5之间
		}
		fitness[i] = func(pop[i]); //计算适应度函数值
								   //pop[i]表示二维数组pop的pop[i][0]的地址
	}
}

double * max(double * fit, int size)	// 求适应度函数值最大值及其位置
{
	int index = 0; // 初始化序号
	double max = *fit; //初始化最大值为数组第一个元素
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

void PSO_func(int n)	// 迭代寻优，传入的参数为一个整数，取值为1到5,分别代表5种不同的计算w的方法
{
	pop_init();
	double * best_fit_index; // 用于存放群体极值和其位置(序号)
	best_fit_index = max(fitness, sizepop); //求群体极值
	int index = (int)(*best_fit_index);
	// 群体极值位置
	for (int i = 0; i<dim; i++)    //dim为粒子维数2
	{
		gbest[i] = pop[index][i];
	}
	// 个体极值位置
	for (int i = 0; i<sizepop; i++)
	{
		for (int j = 0; j<dim; j++)
		{
			pbest[i][j] = pop[i][j];
		}
	}
	// 个体极值适应度值
	for (int i = 0; i<sizepop; i++)
	{
		fitnesspbest[i] = fitness[i];
	}
	//群体极值适应度值
	double bestfitness = *(best_fit_index + 1);
	fitnessgbest = bestfitness;

	//迭代寻优
	for (int i = 0; i<maxgen; i++)
	{
		for (int j = 0; j<sizepop; j++)
		{
			//速度更新及粒子更新
			for (int k = 0; k<dim; k++)
			{
				// 速度更新
				double rand1 = (double)rand() / RAND_MAX; //0到1之间的随机数
				double rand2 = (double)rand() / RAND_MAX;
				double w;
				double Tmax = (double)maxgen;  //迭代次数
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
				//速度修正
				if (V[j][k] > Vmax)
					V[j][k] = Vmax;
				if (V[j][k] < Vmin)
					V[j][k] = Vmin;
				// 粒子更新
				pop[j][k] = pop[j][k] + V[j][k];
				//位置修正
				if (pop[j][k] > popmax)
					pop[j][k] = popmax;
				if (pop[j][k] < popmin)
					pop[j][k] = popmin;
			}
			fitness[j] = func(pop[j]); //新粒子的适应度值
		}
		for (int j = 0; j<sizepop; j++)
		{
			// 个体极值更新
			if (fitness[j] > fitnesspbest[j])
			{
				for (int k = 0; k<dim; k++)
				{
					pbest[j][k] = pop[j][k];
				}
				fitnesspbest[j] = fitness[j];
			}
			// 群体极值更新
			if (fitness[j] > fitnessgbest)
			{
				for (int k = 0; k<dim; k++)
					gbest[k] = pop[j][k];
				fitnessgbest = fitness[j];
			}
		}
		for (int k = 0; k<dim; k++)
		{
			genbest[i][k] = gbest[k]; // 每一代最优值取值粒子位置记录
		}
		result[i] = fitnessgbest; // 每代的最优值记录到数组
	}
}


