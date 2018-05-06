#include <iostream>
#include <fstream>



#include "pos_read.h"


/* namespace */
using namespace std;


bool iopos::open(const char* filename)
{
	fs.open(filename);

  if (!fs){
      cout << "pos input file err" << endl;
      return false;
  }
  return true;
}

unsigned int iopos::read_next_pos(void)
{
	char mark;
	fs >> mark;
	if(mark == '!') {
		fs >> current_point >> pos[0] >> pos[1];
		cout << "point_num:" << current_point << " posX:" << pos[0] << " posY:" << pos[1] << endl;
		return current_point;
	} else {
		return 0;
	}
}

unsigned int iopos::read_next_pos(float *data)
{
	char mark;
	fs >> mark;
	if(mark == '!') {
		fs >> current_point >> data[0] >> data[1];
		cout << "point_num:" << current_point << " posX:" << data[0] << " posY:" << data[1] << endl;
		return current_point;
	} else {
		return 0;
	}
}
