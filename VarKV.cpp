#include <arduino.h>
#include "VarKV.h"

VarKV::VarKV(std::initializer_list<std::pair<String, float*>> init) : size(init.size()) {
  keys = new String[size];
  values = new float*[size];

  int i = 0;
  for(auto n : init) {
    keys[i] = n.first;
    values[i] = n.second;
    ++i;
  }
}

VarKV::~VarKV() {
  delete[] keys;
  delete[] values;
}

bool VarKV::update(String& name, float value) {
  for(int i = 0; i < size; i++) {
    if(name == keys[i]) {
      *values[i] = value;
      return true;
    }
  }
  return false;
}
