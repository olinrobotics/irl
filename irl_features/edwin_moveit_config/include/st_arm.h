#ifndef __ST_ARM_H__
#define __ST_ARM_H__

#include <SerialStream.h>
#include <vector>
#include <string>
#include <sstream>
#include "utils.h"

#define N_JOINTS 5

#define B_RATIO 9818
#define S_RATIO 18000
#define E_RATIO 9000
#define W_RATIO 3000
#define T_RATIO 3000

#define B_LIMIT 15915
#define S_LIMIT 4718
#define E_LIMIT 12912
#define W_LIMIT 2185
#define T_LIMIT 3594

using namespace LibSerial;
enum Mode {MODE_CARTESIAN, MODE_JOINT};

class STArm{
	private:
		SerialStream ser;
        bool energized;
	public:
		STArm(const std::string port); //default port
		~STArm();

		std::string write(const std::string str);
		void initialize();
		void start();

		//one-liner commands
		void home();
		void purge();
		void calibrate();
		void joint();

        void de_energize();
        void energize();

		void move(const std::string& j, int val, bool rel=false);
		void move(const std::vector<double>& v);

		std::vector<double> where();
		void where(std::vector<double>& v);

		void create_route();
		std::string block_on_result(); // necessary?

		// custom command
		void execute_command();

		void set(const std::string& field, const std::string& value);
        std::string get(const std::string& field);
		void set_speed(int speed);
        int get_speed();
        void set_accel(int accel);
        int get_accel();
		void set_mode(Mode); //joint or cartesian
		void set_decimal();
		void set_continuous();
		void set_segmented();

};

#endif
