#ifndef _READ_TXT_H_
#define _READ_TXT_H_
#include <iostream>
#include <string>
#include <fstream>
using namespace std;

int countLines(const char *filename);

string readLine(const char *filename, int line);

#endif