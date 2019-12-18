#ifndef CFRELEMENT_H
#define CFRELEMENT_H

#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <string.h>
#include <stdint.h>

#define NUM_PERIODICITIES 24
#define SATURATION 0.000
#define FREMEN_AMPLITUDE_THRESHOLD 0.0

/**
@author Tom Krajnik
*/
typedef struct
{
  float realStates;
  float imagStates;
  float realBalance;
  float imagBalance;
}SSpectralComponent;

typedef struct
{
  float amplitude;
  float phase;
  float period;
}SFrelement;

using namespace std;

extern float *periods;

class CFrelement
{
 public:
  CFrelement();
  ~CFrelement();

  //this is called only when the state becomes dynamic
  // 只有当状态变为动态时才会调用它
  void initializeFrequencies();

  //adds a serie of measurements to the data
  // 向数据添加一系列测量值
  int add(uint32_t times[], float states[], int length);

  //estimates the probability for the given times
  // 估计在给定时间的概率
  int estimate(uint32_t times[], float probs[], int length, int order);

  //estimates the state's entropy for the given times 
  int estimateEntropy(uint32_t times[], float entropy[], int length, int order);

  //evaluates the error of the predictions for the given times and measurements
  // 在给定时间和测量值下，评估预测误差
  int evaluate(uint32_t times[], float signal[], int length, int order, float evals[]);

  /*prepares for predictions*/
  /*为预测做准备*/
  int update(unsigned char modelOrder);

  void print(int order);

  int save(FILE* file,bool lossy = false);
  int load(FILE* file);
  int save(char* name,bool lossy = false);
  int load(char* name);

  float gain;

  SSpectralComponent *components;
  SFrelement *frelements;

  int measurements;
  unsigned char order;
  int64_t firstTime;
  int64_t  lastTime;
  float lastMeasurement;
};

#endif
