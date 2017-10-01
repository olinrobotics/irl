#include "utils.h"
#include <vector>
#include <sstream>

float lerp(float a, float b, float w){
	return a*w + b*(1-w);
}

float d2r(float d){
	return d*M_PI/180;
}

float r2d(float r){
	return r * 180 / M_PI;
}

float posr(float x){
	float TWO_PI = 2*M_PI;
    x = fmod(x,TWO_PI);
    if (x < 0)
        x += TWO_PI;
    return x;
}

void split(std::string& s, const std::string& delim, std::vector<std::string>& res){
    res.clear();
    size_t pos=0;
    std::string token;
    while ((pos = s.find(delim)) != std::string::npos) {
        token = s.substr(0, pos);
        res.push_back(token);
        s.erase(0, pos + delim.length());
    }
    res.push_back(s);
}

// string manip.
std::vector<std::string> split(
        std::string s,
        const std::string& delim){
    std::vector<std::string> res;
    split(s,delim,res);
    return res;
}

std::vector<std::string> split(
        std::string s,
        const char delim_c){
    std::string delim(1,delim_c);
    return split(s, delim);
}
