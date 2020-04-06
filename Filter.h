#ifndef FILTER_HEADER
#define FILTER_HEADER

class Filter {
public:
  float cutoff, frequency;

  Filter() : Filter(10, 100) {};
  Filter(float cutoff, float frequency) : cutoff(cutoff), frequency(frequency) { };
  virtual void reset() = 0;
  virtual float process(float sample) = 0;
};

#endif 
