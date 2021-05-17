#include <iostream>
#include <string.h>

using namespace std;

template<typename ... Args>
std::string format_string(const std::string& format, Args ... args)
{
  size_t size = snprintf(nullptr, 0, format.c_str(), args ...) + 1;
  std::unique_ptr<char[]> buffer(new char[size]);
  snprintf(buffer.get(), size, format.c_str(), args ...);
  return std::string(buffer.get(), buffer.get() + size - 1);
}


void main(){

    std::string text;

    std::string mesh = format_string("happy", );

    char* mystr = new char;
    strcpy_s(mystr, )

    return ;
}