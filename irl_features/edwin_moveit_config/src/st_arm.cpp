#include "st_arm.h"
#include <iostream>
#include <sstream>
#include <string>

STArm::STArm(const std::string port):energized(true){
	// open serial port
	ser.Open(port);
	ser.SetBaudRate(SerialStreamBuf::BAUD_19200); //only supports baud rate = 19200 for now
	ser.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
	ser.SetParity(SerialStreamBuf::PARITY_NONE);
	ser.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
}
STArm::~STArm(){
	if(ser.IsOpen()){
		ser.Close();
	}
}
void STArm::initialize(){
	ser.sync();
	//sequence of operations
	write("ROBOFORTH");
	set_mode(MODE_JOINT);
	start();
	calibrate();
	home();
	// perform initial calibration
}

void STArm::start(){
	std::string str = write("START");
	//std::cout << str << std::endl;
}

void STArm::home(){
	write("HOME");
}
void STArm::purge(){
	write("PURGE");
}
void STArm::calibrate(){
	write("CALIBRATE");
}

void STArm::joint(){
	write("JOINT");
}

void STArm::de_energize(){
    write("DE-ENERGIZE");
    energized=false;
}

void STArm::energize(){
    write("ENERGIZE");
    energized=true;
}

void STArm::move(const std::string& j, int val, bool rel){
    if(!energized)
        return;
	//TELL <JOINT> <VAL> MOVETO
	std::stringstream ss;
	// make uppercase
	std::string s = j;
	for(std::string::iterator it = s.begin(); it != s.end(); ++it){
		char& c = *it;
		c = std::toupper(c);
	}

	// format string
	ss << "TELL " << s << ' ' << val << ' ' << (rel?"MOVE":"MOVETO");
	write(ss.str());
}

void STArm::move(const std::vector<double>& v){
    if(!energized)
        return;
	// JMA command lists the joint in the opposite order of what's given by commands like WHERE.
	// therefore it must be enumerated in reverse, for consistency.
	std::stringstream ss;
	for(std::vector<double>::const_reverse_iterator it=v.rbegin(); it!=v.rend(); ++it){
		int v = *it;
		ss << v << ' ';
	}
	ss << "JMA";
	write(ss.str());
}

void STArm::where(std::vector<double>& v){
	std::string s = write("WHERE");
	std::vector<std::string> l = split(s,'\r');

	// TO TEST : 
	//for(std::vector<std::string>::iterator it=l.begin();it!=l.end();++it){
	//	std::string& s = *it;
	//	std::cout << s << std::endl;
	//}
	
	//l[2] holds regular motor count
	//l[3] holds actual encoder count
	//l[4] holds what motor count should be

	if(l.size() > 2){
        v.resize(5);
        std::istringstream ss(l[2]);
        for(int i=0; i<N_JOINTS; ++i){
            ss >> v[i];
        }
	}	
	
}

std::vector<double> STArm::where(){
	std::vector<double> res;
	where(res);
	return res;
}


void create_route(){
	//TODO : implement
}

void execute_command(){
	// TODO : implement
}

void STArm::set(const std::string& field, const std::string& value){
    std::stringstream ss;
    ss << value << ' ' << field << " !";
    write(ss.str());
}
std::string STArm::get(const std::string& field){
    return write(field + " ?");
}

void STArm::set_speed(int speed){
    set("SPEED", std::to_string(speed));
}
int STArm::get_speed(){
    auto s = get("SPEED");
    std::cout << "Get speed returned :" << s << std::endl;
    //ROS_INFO("Get Speed Returned : %s", s.c_str());
    return 0;
    //return std::atoi(write("SPEED ?"));
}
void STArm::set_accel(int accel){
    set("ACCEL", std::to_string(accel));
}
int STArm::get_accel(){
    // Format : ">ACCEL ? 1000 OK"
    auto s = get("ACCEL");
    std::cout << "Get accel returned :" << s << std::endl;
    //ROS_INFO("Get Accel Returned : %s", s.c_str());
    return 0;
}
void STArm::set_decimal(){
    write("DECIMAL");
}

void STArm::set_segmented(){
	write("SEGMENTED");
}

void STArm::set_continuous(){
	write("CONTINUOUS");
}
// lower level calls
void STArm::set_mode(Mode m){
	switch(m){
		case MODE_JOINT:
			write("JOINT");
			break;
		case MODE_CARTESIAN:
			write("CARTESIAN");
			break;
	}
}

std::string STArm::write(const std::string str){
	ser.sync(); // flush input
	ser << str << '\r';
	ser.flush(); // flush output
	return block_on_result();
}

std::string STArm::block_on_result(){
	std::stringstream s;
	std::string str;
	//wait till ok
	
	while(str.find("OK") == std::string::npos){ // TODO : verify this is valid
		std::getline(ser,str);
		s << str;
	}

	return s.str();
}


