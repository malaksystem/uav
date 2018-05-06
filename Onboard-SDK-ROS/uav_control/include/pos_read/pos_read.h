#ifndef POS_READ_H
#define POS_READ_H

#include <fstream>


class iopos
{
  std::ifstream fs;
  
  
public:
  unsigned int current_point;
  float pos[2];

  iopos(void)
  {
    
  }

  iopos(const char* filename)
  {
    open(filename);
  }

  bool open(const char* filename);

  unsigned int read_next_pos(void);

  unsigned int read_next_pos(float *data);

};



#endif /* POS_READ_H */