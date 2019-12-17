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
  /**
   * @brief add         添加一个新的2d栅格地图，如果它是第一个添加的栅格地图，则存储其参数并它的每个单元格有以下形式
   *                    状态是[-1..100]，其中 -1 是未知区域，0 是自由空间，100 是被占据的障碍物
   * @param time        添加新地图的时间
   * @param states      新地图每一个单元格的占据率
   * @param width       新地图的 width
   * @param height      新地图的 height
   * @param originX     新地图原点的x坐标
   * @param originY     新地图原点的y坐标
   * @param resolution  新地图的分辨率
   * @return            如果它是第一个添加的栅格地图，则返回 1；
   *                    如果它不是第一个添加的栅格地图，并且添加成功，则返回 0；
   *                    如果新地图的 width，height 或 resolution 与原有的 Frelement 地图不同，则添加失败并返回 -1
   */
  int add(uint32_t time, int8_t states[], int width, int height,
          float originX, float originY, float resolution);

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

  int numFrelements;  // Frelement 地图总的单元格数量
  // frelementArray 为一个二维数组。
  // 数组的第一个维度大小为 numFrelements，这是 Frelement 地图总的单元格数量，所以第一个维度是一张 Frelement 地图，
  // 第一个维度的每一个元素为指向 CFrelement 的指针，即每一个元素为一个 CFrelement 类型的数组。
  // 第二个维度表示 Frelement 地图单个单元格的占据率，大小随着观察值的加入而不断增加。
  CFrelement** frelementArray;
  int width;         // Frelement 地图的 width
  int height;        // Frelement 地图的 height
  float resolution;  // Frelement 地图的分辨率
  float originX;     // Frelement 地图原点的x坐标
  float originY;     // Frelement 地图原点的y坐标
};

#endif
