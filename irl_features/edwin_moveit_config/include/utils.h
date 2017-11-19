#ifndef __UTILS_H__
#define __UTILS_H__

#include <cmath>
#include <vector>
#include <string>
#include <iostream>

#ifndef M_PI
#define M_PI 3.141592653589793238
#endif

extern float lerp(float a, float b, float w);
extern float d2r(float d);
extern float r2d(float r);
extern float posr(float r);

extern void split(std::string& s, const std::string& delim, std::vector<std::string>& res);
extern std::vector<std::string> split(std::string s, const std::string& delim);
extern std::vector<std::string> split(std::string s, const char delim_c);

template<typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v){
    os << '[';
    for(auto& e : v){
        os << e << ';';
    }
    os << ']';
}
#endif
