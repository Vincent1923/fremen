#ifndef CFRELEMENT2DGRID_H
#define CFRELEMENT2DGRID_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include "CFrelement.h"

/**
@author Tom Krajnik
*/

using namespace std;

class CFrelement2DGrid
{
 public:
  CFrelement2DGrid(const char* name);
  ~CFrelement2DGrid();

  /*add new measurements  
    - adds a new 2d grid, if it's the first grid added, then its params are stored and the following grids will be fitted
    - states are [-1..100], where -1 is unknown, 0 is free and 100 is occupied
    - returns the number of stored grids*/
  /*添加新的测量值
    - 添加一个新的2d栅格地图，如果它是第一个添加的栅格地图，则存储其参数并有以下栅格
    - 状态是[-1..100]，其中-1是未知的，0是空闲的，100是被占用的
    - 返回存储的栅格数*/
  int add(uint32_t time, int8_t states[], int width, int height, float originX, float originY, float resolution);

  /*estimates the occupancy probabilities [0..100] for the given time with a given order 
    returns the number of predicted cells*/
  /*使用给定的阶数估计在给定时间的占用概率[0..100]
    返回预测的单元格数量*/
  int estimate(uint32_t time, int8_t states[], int order);

  /*estimates the occupancy entropies [0..100] for the given time with a given order 
    returns the number of cells*/
  int estimateEntropy(uint32_t time, int8_t entropy[], int order);

  /*evaluate the prediction/estimation for the given time
    returns -1 if the state with the given ID is not present in the collection
    otherwise returns the best performing model order and the errors in the eval array*/
  /*评估给定时间的预测/估计
    如果集合中不存在具有给定ID的状态，则返回-1
    否则返回表现最佳的模型阶数和 eval 数组中的 errors*/
  int evaluate(uint32_t time, int8_t states[], int order, float []);

  /*print grid info*/
  bool print(int order);

  /*load the 2D grid from a file*/
  int load(const char* file);

  /*save the 2D grid to a file*/
  int save(const char* file);

  /*idecko voe*/
  char id[255];  // Frelement 地图的名字

  int numFrelements;  // numFrelements 表示栅格地图总的单元格数量
  CFrelement** frelementArray;
  int width;
  int height;
  float resolution;
  float originX;
  float originY;
};

#endif
