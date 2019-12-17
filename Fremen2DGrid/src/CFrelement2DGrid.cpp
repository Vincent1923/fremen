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

// 析够函数
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
      if (states[i] != -1)  // 查找栅格地图中的已探索区域（-1表示未探索区域），即自由空间或者障碍物
      {
        if (frelementArray[i] == NULL)
        {
          frelementArray[i] = new CFrelement();  // 为 CFrelement2DGrid 地图中已探索区域的单元格新建一个 CFrelement
		}

        float signal = ((float)states[i]) / 100.0;  // signal 按 100 缩放
        frelementArray[i]->add(&time, &signal, 1);  // 对每一个单元格添加观察值
      }
    }

    result = 0;
  }

  return result;
}

int CFrelement2DGrid::estimate(uint32_t time,int8_t states[],int order)
{
	float prob;
	for (int i = 0;i<numFrelements;i++)
	{
		if (frelementArray[i] == NULL){
			states[i] = -1;
		}else{
			frelementArray[i]->estimate(&time,&prob,1,order);
			states[i] = (int8_t)(100.0*prob);
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
int CFrelement2DGrid::evaluate(uint32_t time,int8_t states[],int order,float errs[])
{
	for (int i = 0;i<=order;i++) errs[i] = 0;
	float evals[order+1];
	int numEvaluated = 0;
	for (int i = 0;i<numFrelements;i++)
	{
		if (states[i] != -1 && frelementArray[i] != NULL)
		{
			float signal = ((float)states[i])/100.0;
			frelementArray[i]->evaluate(&time,&signal,1,order,evals);
			for (int i = 0;i<=order;i++) errs[i] += evals[i];
			numEvaluated++;
		}
	}
	if (numEvaluated > 0) for (int i = 0;i<=order;i++) errs[i]=errs[i]/numEvaluated;
	
	int index = 0;
	float minError = 100000;
	for (int i = 0;i<=order;i++){
	       	if (errs[i]<minError)
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
