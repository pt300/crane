#ifndef FILTER_HEADER
#define FILTER_HEADER

/*
 * Base class for filters
 */

class Filter {
public:
  /*
   * cutoff - cutoff frequency of the filter
   * frequency - sampling frequency
   */
  float cutoff, frequency;

  Filter() : Filter(10, 100) {};
  Filter(float cutoff, float frequency) : cutoff(cutoff), frequency(frequency) { };
  /*
   * Zeros filter's memory and recalculates coefficients based on cutoff and sampling frequency
   */
  virtual void reset() = 0;
  /*
   * Passes a single sampel through the filter
   * Returns the result of filtering
   */
  virtual float process(float sample) = 0;
};

#endif 
