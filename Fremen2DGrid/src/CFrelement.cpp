#include "CFrelement.h"

using namespace std;

static bool debug = false; 

int fremenSort(const void* i,const void* j)
{
  if (((SFrelement*)i)->amplitude < ((SFrelement*)j)->amplitude)
  {
    return +1;
  }

  return -1;
}

// 构造函数
CFrelement::CFrelement()
{
  //initialization of the frequency set
  // 初始化
  frelements = NULL;
  components = NULL;
  gain = 0.5;
  lastMeasurement = 0.5;
  firstTime = -1;
  lastTime = -1;
  measurements = 0;
  order = 0;
}

void CFrelement::initializeFrequencies()
{
  // 初始化频谱分量，components 的数组长度为 NUM_PERIODICITIES
  components = (SSpectralComponent*)calloc(NUM_PERIODICITIES, sizeof(SSpectralComponent));
}

// 析构函数
CFrelement::~CFrelement()
{
  if (components != NULL)
  {
    free(components);
  }
  if (frelements != NULL)
  {
    free(frelements);
  }
}

// adds new state observations at given times
// 在给定的时间 times 添加新的状态观察值 states
int CFrelement::add(uint32_t times[], float states[], int length)
{
  if (measurements == 0 && length > 0)
  {
    firstTime = times[0];  // times[0] 表示 goal 请求的时间，即添加观察值的时间
  }

  int firstIndex = 0;

  //discard already known observations
  // 丢弃已知的观察结果
  for (int i = 0; i < length; i++)
  {
    if (times[i] <= lastTime)
	{
      firstIndex++;
	}
  }

  // numUpdated 表示添加的观察值（即单元格占据率）的数量，一般 length = 1，所以 numUpdated = 1
  int numUpdated = length - firstIndex;

  //verify if there is an actual update
  // 验证是否有实际的更新
  if (numUpdated <= 0)
  {
    return numUpdated;
  }

  lastTime = times[length - 1];          // lastTime 记录上一次添加观察值的时间
  lastMeasurement = states[length - 1];  // lastMeasurement 记录上一次加入的观察值

  //update the gains accordingly
  // 相应地更新 gains
  float oldGain = 0;
  float newGain = 0;
  for (int j = firstIndex; j < length; j++)
  {
    // newGain 表示所有新添加的观察值的总和。
	// 若 length = 1，则 newGain = states[0]，等于加入的单个单元格的占据率。
    newGain += states[j];
  }
  // 添加长度为 length 的观测值后，更新该单元格的平均占据率
  gain = (gain * measurements + newGain) / (measurements + length);

  // 如果单元格的平均占据率 gain 大于0，并且该单元格是第一次添加观察值，则初始化频谱
  if (gain > 0 && components == NULL)
  {
    initializeFrequencies();  // 初始化频谱
  }

  if (components != NULL)
  {
    //recalculate spectral balance - this is beneficial is the process period does not match the length of the data
	// 重新计算谐波平衡 - 如果处理周期与数据长度不匹配，这是有好处的
    if (oldGain > 0)
	{
      for (int i = 0; i < NUM_PERIODICITIES; i++)
      {
        components[i].realBalance  = gain * components[i].realBalance / oldGain;
        components[i].imagBalance  = gain * components[i].imagBalance / oldGain;
      }
    }

    float angle = 0;

	//recalculate the spectral components
    // 重新计算谐波成分
	for (int j = firstIndex; j < length; j++)
    {
      for (int i = 0; i < NUM_PERIODICITIES; i++)
      {
        angle = 2 * M_PI * (float)times[j] / periods[i];     // 根据周期 periods[i] 计算复数在复平面旋转的角度
        components[i].realStates += states[j] * cos(angle);  // 计算复平面的实部，states[j] 为模长
        components[i].imagStates += states[j] * sin(angle);  // 计算复平面的虚部，states[j] 为模长
        components[i].realBalance += gain * cos(angle);
        components[i].imagBalance += gain * sin(angle);
      }
    }

    order = -1;
  }

  measurements += length;

  return numUpdated;  // 返回添加的观察值的数量
}

int CFrelement::update(unsigned char orderi)
{
	if (gain == 0 || measurements == 0){
	       	order = 0;
		return 0;
	}
	if (orderi != order)
	{
		order = orderi;
		free(frelements);
		//establish amplitudes and phase shifts
		if (components != NULL){
			SFrelement *freq = (SFrelement*)malloc(NUM_PERIODICITIES*sizeof(SFrelement));
			float re,im;
			int duration = lastTime-firstTime;
			for (int i = 0;i<NUM_PERIODICITIES;i++)
			{
				re = components[i].realStates-components[i].realBalance;
				im = components[i].imagStates-components[i].imagBalance;
				if (periods[i] <= duration) freq[i].amplitude = sqrt(re*re+im*im)/measurements; else freq[i].amplitude = 0;
				if (freq[i].amplitude < FREMEN_AMPLITUDE_THRESHOLD) freq[i].amplitude = 0;
				freq[i].phase = atan2(im,re);
				freq[i].period = periods[i];
			}
			//sort the spectral components
			qsort(freq,NUM_PERIODICITIES,sizeof(SFrelement),fremenSort);
			frelements = (SFrelement*)malloc(order*sizeof(SFrelement));
			for (int i = 0;i<order;i++)frelements[i] = freq[i];
			free(freq);
		}else{
			frelements = (SFrelement*)malloc(order*sizeof(SFrelement));
			for (int i = 0;i<order;i++){
				frelements[i].amplitude = frelements[i].phase = 0;
				frelements[i].period = periods[i];
			}
		}
	}
	return 0;
}

int CFrelement::evaluate(uint32_t times[], float signal[],int length,int orderi,float evals[])
{
	update(orderi);
	float estimate = 0;
	float time;
	float state;
	for (int j = 0;j<=order;j++) evals[j] = 0;
	for (int j = 0;j<length;j++)
	{
		time = times[j];
		state = signal[j];
		estimate = gain;
		evals[0]+= fabs(state-estimate);
		for (int i = 0;i<order;i++){
			 estimate+=2*frelements[i].amplitude*cos(time/frelements[i].period*2*M_PI-frelements[i].phase);
			 evals[i+1]+= fabs(state-estimate);
		}
	}
	for (int j = 0;j<=order;j++)evals[j]=evals[j]/length;

	//get best model order
	float error = 10.0;
	int index = 0;
	for (int j = 0;j<=order;j++)
	{
		if (evals[j] < error-0.001){
			index = j;
			error = evals[j]; 
		}
	}
	return index;
}

/*text representation of the fremen model*/
void CFrelement::print(int orderi)
{
	update(orderi);
	int errs = 0;
	std::cout << " Prior: " << gain << " Size: " << measurements << " ";
	if (order > 0) std::cout  << endl;
	float ampl = gain;
	for (int i = 0;i<order;i++){
		std::cout << "Frelement " << i << " " << frelements[i].amplitude << " " << frelements[i].phase << " " << frelements[i].period << endl;
	}
	std::cout << endl; 
}

int CFrelement::estimate(uint32_t times[],float probs[],int length,int orderi)
{
	update(orderi);
	float estimate = 0;
	float time;
	for (int j = 0;j<length;j++)
	{
		time = times[j];
		estimate = gain;
		for (int i = 0;i<order;i++) estimate+=2*frelements[i].amplitude*cos(time/frelements[i].period*2*M_PI-frelements[i].phase);
		estimate+=(lastMeasurement-estimate)*exp(-(float)fabs(time-lastTime)/3600);
		if (estimate > 1.0 - SATURATION) estimate =  1.0 - SATURATION;
		if (estimate < 0.0 + SATURATION) estimate =  0.0 + SATURATION;
		probs[j]=estimate;
	}
	return length;
}

int CFrelement::estimateEntropy(uint32_t times[],float entropy[],int length,int orderi)
{
	update(orderi);
	float estimate = 0;
	float time;
	for (int j = 0;j<length;j++)
	{
		time = times[j];
		estimate = gain;
		for (int i = 0;i<order;i++) estimate+=2*frelements[i].amplitude*cos(time/frelements[i].period*2*M_PI-frelements[i].phase);
		if (estimate <= 0 || estimate >= 1) entropy[j] = 0; else  entropy[j] = -(estimate*log2f(estimate)+(1-estimate)*log2f((1-estimate)));
	}
	return length;
}

int CFrelement::save(char* name,bool lossy)
{
	FILE* file = fopen(name,"w");
	save(file);
	fclose(file);
}

int CFrelement::load(char* name)
{
	FILE* file = fopen(name,"r");
	load(file);
	fclose(file);
	return 0;
}

int CFrelement::save(FILE* file,bool lossy)
{
	int frk = NUM_PERIODICITIES;
	fwrite(&measurements,sizeof(int),1,file);
	fwrite(&firstTime,sizeof(int64_t),1,file);
	fwrite(&lastTime,sizeof(int64_t),1,file);
	fwrite(&gain,sizeof(float),1,file);
	if (measurements != 0 && gain != 0) fwrite(components,sizeof(SFrelement),NUM_PERIODICITIES,file);
	return 0;
}

int CFrelement::load(FILE* file)
{
	if (components != NULL) free(components);
	int frk = NUM_PERIODICITIES;
	int ret = 0;
	ret+=fread(&measurements,sizeof(int),1,file);
	ret+=fread(&firstTime,sizeof(int64_t),1,file);
	ret+=fread(&lastTime,sizeof(int64_t),1,file);
	ret+=fread(&gain,sizeof(float),1,file);
	if (measurements != 0 && gain != 0)
	{
		initializeFrequencies();
	       	ret+=fread(components,sizeof(SFrelement),NUM_PERIODICITIES,file);
	}
	return 0;
}

