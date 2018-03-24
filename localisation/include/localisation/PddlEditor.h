#include <sstream>
#include <fstream>
#include <iostream>

class PddlEditor {

private:
	void modify(std::string baseFile, std::string targetFile, char lineType, bool removeFile = false);
	int _gear_part_count;
    int _piston_rod_part_count;

public:
	PddlEditor() {

	}

	void editPDDL(std::string baseFile, std::string targetFile, char lineType);
	void setPartCount(int gear_part_count, int piston_rod_part_count);
};