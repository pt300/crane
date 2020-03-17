#ifndef VARKV_HEADER
#define VARKV_HEADER

#include <initializer_list>
#include <utility>

class VarKV {
public:
  VarKV(std::initializer_list<std::pair<String, float&>> init);
  bool update(String& name, float value);
  
  const int size;
  String *keys;
  float* *values;
};

#endif
