#include "CFrelement2DGrid.h"

using namespace std;

static bool debug = true;

extern float *periods;

// 构造函数
CFrelement2DGrid::CFrelement2DGrid(const char* name)
{
  strcpy(id, name);  // 对 Frelement 地图的 ID 赋值为 name
  numFrelements = 0;
  height = 0;
  width = 0;
  originX = 0;
  originY = 0;
  resolution = 0;
  frelementArray = NULL;
}

// 析构函数
CFrelement2DGrid::~CFrelement2DGrid()
{
  if (frelementArray != NULL)
  {
    for (int i = 0; i < numFrelements; i++)
    {
      if (frelementArray[i] != NULL)
	  {
        delete frelementArray[i];  // 清空 Frelement 地图每一个单元格
	  }
    }

    free(frelementArray);  // 清空 Frelement 地图
  }
}

int CFrelement2DGrid::add(uint32_t time, int8_t states[], int widthi, int heighti,
                          float originXi, float originYi, float resolutioni)
{
  int result = -1;

  /*is this a new map ? if yes, initialise all stuff*/
  // 检查 Frelement 地图是否是新的，如果是，则进行初始化
  if (numFrelements == 0)
  {
    height = heighti;
    width = widthi;
    numFrelements = height * width;  // numFrelements 表示栅格地图总的单元格数量，即地图总的像素点个数
    originX = originXi;
    originY = originYi;
    resolution = resolutioni;

    // 为 frelementArray 分配 numFrelements 个大小为 sizeof(CFrelement*) 的内存空间。
	// frelementArray 为一个二维数组，第一个维度的大小为 numFrelements，表示总的单元格，
	// 第一维度的每一个元素为指向 CFrelement 数据类型的指针，这是一个 CFrelement 类型的数组。
    frelementArray = (CFrelement**)malloc(numFrelements * sizeof(CFrelement*));

    for (int i = 0; i < numFrelements; i++)
	{
      frelementArray[i] = NULL;
	}

    result = 1;
  }

  // 检查新加入地图的 size 维度以及地图分辨率是否相同
  if (height == heighti && width == widthi && resolution == resolutioni)
  { 
    for (int i = 0; i < numFrelements; i++)  // 遍历栅格地图
    {
      if (states[i] != -1)  // 查找栅格地图中的已探索区域（-1 表示未探索区域），即自由空间或者障碍物
      {
        if (frelementArray[i] == NULL)
        {
          frelementArray[i] = new CFrelement();  // 为 CFrelement2DGrid 地图中已探索区域的单元格新建一个 CFrelement
		}

        float signal = ((float)states[i]) / 100.0;  // signal 按 100 缩放
		// 对每一个单元格添加观察值，第三个参数表示观察值的数量，这里1表示每次只添加一个观察值
        frelementArray[i]->add(&time, &signal, 1);
      }
    }

    result = 0;
  }

  return result;
}

// 使用给定的阶数 order，在给定时间 time 对栅格地图每一个单元格的占据概率进行估计，占据概率的范围为[0..100]
// 最后估计的栅格地图的占用率保存在 states[]
int CFrelement2DGrid::estimate(uint32_t time, int8_t states[], int order)
{
  // 单个单元格的占据率，按 100 进行缩放（缩小了 100 倍）
  float prob;

  // 遍历栅格地图
  for (int i = 0; i < numFrelements; i++)
  {
	// 检查单元格是否已经分配空间进行初始化，单元格分配空间是在 add 中进行，
	// 只有在添加地图时该单元格是已探索区域（自由空间或者障碍物），才会分配空间。
    if (frelementArray[i] == NULL)  // 该单元格还是未探索区域
	{
      states[i] = -1;  // 把地图对应单元格赋值为 -1，-1 表示未探索区域
    }
    else  // 该单元格是已探索区域
    {
	  // 对单元格的占据率进行预测和估计，1 表示 length，order 表示阶数，prob 为估计的占据率
      frelementArray[i]->estimate(&time, &prob, 1, order);
	  // states[i] 为最终返回的单元格的占据率，比 prob 扩大 100 倍
      states[i] = (int8_t)(100.0 * prob);
    }
  }

  return 0;
}

int CFrelement2DGrid::estimateEntropy(uint32_t time,int8_t states[],int order)
{
	float prob;
	for (int i = 0;i<numFrelements;i++)
	{
		if (frelementArray[i] == NULL){
			states[i] = -1;
		}else{
			frelementArray[i]->estimateEntropy(&time,&prob,1,order);
			states[i] = (int8_t)(100.0*prob);
		}
	}
	return 0;
}

//not functional!
// 使用给定的阶数 order，在给定时刻 time 对新构建的地图数据 states[] 进行预测/评估，并返回误差数组 errs[]。
// 主要是计算输入地图数据 states[] 与 Frelement 地图之间的误差，即计算新地图与 Frelement 地图的匹配程度（一致性）。
// 值得注意的是，errs[] 数组的长度为 order + 1
int CFrelement2DGrid::evaluate(uint32_t time, int8_t states[], int order, float errs[])
{
  // 对地图误差进行初始化
  for (int i = 0; i <= order; i++)
  {
    errs[i] = 0;
  }

  float evals[order + 1];  // evals[] 数组的长度跟 errs[] 一样，也为 order + 1
  int numEvaluated = 0;    // numEvaluated 表示栅格地图中进行评估的单元格总的数量

  // 遍历栅格地图
  for (int i = 0; i < numFrelements; i++)
  {
	// 检查该单元格是否为输入地图数据 states[] 中的 free（自由空间）或者 occupancy（障碍物）区域，
	// 以及检查该单元格是否为 Frelement 地图的已分配空间的栅格。
    if (states[i] != -1 && frelementArray[i] != NULL)
    {
      float signal = ((float)states[i]) / 100.0;  // signal 按 100 进行缩放

      // 在给定时间 time，评估单元格的测量值 signal 的预测误差，即对该单元格进行一致性的评估。
	  // 1 表示 length，order 表示阶数，evals 表示返回的估计数组。
      frelementArray[i]->evaluate(&time, &signal, 1, order, evals);

      for (int i = 0; i <= order; i++)
	  {
        errs[i] += evals[i];  // 在阶数 i 下，对所有单元格的评估数值的累加
	  }

      numEvaluated++;
    }
  }

  if (numEvaluated > 0)
  {
    for (int i = 0; i <= order; i++)
	{
      errs[i] = errs[i] / numEvaluated;  // errs[i] 为在阶数 i 下，所有单元格的平均评估数值
	}
  }

  int index = 0;
  float minError = 100000;

  for (int i = 0; i <= order; i++)
  {
    if (errs[i] < minError)
    {
      index = i;
      minError = errs[i];
    }
  }

  return 0;
}

bool CFrelement2DGrid::print(int order)
{
	for (int i = 0;i<numFrelements;i++)
	{
		printf("Cell %i: ",i); 
		if (frelementArray[i] != NULL) frelementArray[i]->print(order); else printf("None\n"); 
	}
	return true;
}

int CFrelement2DGrid::load(const char* name)
{
	/*destroy old stuff*/
	if (frelementArray !=NULL){
		for (int i=0;i<numFrelements;i++){
			if (frelementArray[i] != NULL) delete frelementArray[i];
		}
		free(frelementArray);
	}

	/*read meta-information*/
	int result = 0;
	FILE* file=fopen(name,"r");
	if (file == NULL) return -1;
	result += fread(&numFrelements,sizeof(int),1,file);
	result += fread(&height,sizeof(int),1,file);
	result += fread(&width,sizeof(int),1,file);
	result += fread(&originX,sizeof(float),1,file);
	result += fread(&originY,sizeof(float),1,file);
	result += fread(&resolution,sizeof(float),1,file);
	if (debug) printf("%i %i %.3f %.3f %.3f %i\n",width,height,originX,originY,resolution,result);
	/*initialize array*/	
	frelementArray = (CFrelement**)malloc(numFrelements*sizeof(CFrelement*));
	for (int i = 0;i<numFrelements;i++) frelementArray[i] = NULL;
	unsigned char *occupancyArray = (unsigned char*)calloc(numFrelements/8+1,sizeof(unsigned char));
	result += fread(occupancyArray,sizeof(unsigned char),numFrelements/8+1,file);

	unsigned char a = 128;
	for (int i = 0;i<numFrelements;i++)
	{
		if ((occupancyArray[i/8]&a) > 0){
			frelementArray[i] = new CFrelement();
		       	frelementArray[i]->load(file);
		}
		if (a == 1) a=128; else a=a/2;
	}
	free(occupancyArray);
	fclose(file);
	return result;
}

int CFrelement2DGrid::save(const char* name)
{
	int result = 0;
	FILE* file=fopen(name,"w");
	if (file == NULL) return -1;

	/*the occupancy array allows to represent never-observed states as single bits*/
	int dummy = 0;
	unsigned char *occupancyArray = (unsigned char*)calloc(numFrelements/8+1,sizeof(unsigned char));
	unsigned char a = 128;
	for (int i = 0;i<numFrelements;i++)
	{
		if (frelementArray[i] != NULL) occupancyArray[i/8] = occupancyArray[i/8]|a; 
		if (a == 1) a=128; else a=a/2;
	}
	/*printf("Occupancy array:");
	for (int i = 0;i<=numFrelements/8;i++) printf("%x",occupancyArray[i]);
	printf("\n");*/
	unsigned char len = strlen(id);	
	result += fwrite(&len,sizeof(unsigned char),1,file);
	result += fwrite(id,len,1,file);
	result += fwrite(&numFrelements,sizeof(int),1,file);
	result += fwrite(&height,sizeof(int),1,file);
	result += fwrite(&width,sizeof(int),1,file);
	result += fwrite(&originX,sizeof(float),1,file);
	result += fwrite(&originY,sizeof(float),1,file);
	result += fwrite(&resolution,sizeof(float),1,file);
	result += fwrite(occupancyArray,sizeof(unsigned char),numFrelements/8+1,file);
	if (debug) printf("Map %s with dimensions %ix%i, origin %.3fx%.3f and resolution %.3f saved in file %s.\n",id,width,height,originX,originY,resolution,name);
	for (int i = 0;i<numFrelements;i++)
	{
		if (frelementArray[i] != NULL) result += frelementArray[i]->save(file);  
	}
	free(occupancyArray);
	fclose(file);
	return result;
}
