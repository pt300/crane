#include <arduino.h>
#include "VarKV.h"

VarKV::VarKV(std::initializer_list<std::pair<String, float&>> init) : size(init.size()) {
  keys = (String *)malloc(size * sizeof *keys);
  values = (float* *)malloc(size * sizeof *values);

  int i = 0;
  for(auto& n : init) {
    keys[i] = n.first;
    values[i] = &n.second;
    ++i;
  }
}

bool VarKV::update(String& name, float value) {

  for(int i = 0; i < size; i++) {
    if(name.compareTo(keys[i]) == 0) {
      *values[i] = value;
      return true;
    }
  }
  return false;
}
