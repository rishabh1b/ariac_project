#include "localisation/PddlEditor.h"
using namespace std;

void PddlEditor::modify(std::string baseFile, std::string targetFile, char lineType, bool removeFile) {
	fstream readfile;
	fstream writefile;
	readfile.open(baseFile.c_str(), ios::in);
	writefile.open(targetFile.c_str(), ios::out);
	std::string line, dummy; 
	std::size_t found;

    // for testing
    // int _gear_part_count = 3;
    // int _piston_rod_part_count = 3;

    // For changing part locations - future use
    int _bin_loc_gear = 6;
    int _bin_loc_piston = 7;

	if (readfile.is_open() && writefile.is_open()) {
		std::cout << "Files Opened successfully!" << std::endl;


		while (getline(readfile, line)) {
			switch(lineType) {
				case 'o':
					found = line.find(":objects");
					if (found != std::string::npos) {
						std::cout << "Ready to Modify Objects" << std::endl;
						writefile<< line << "\n";
						getline(readfile, dummy);
						writefile << "\t\t" << "gear_part_1 - Block" << "\n";
						getline(readfile, dummy);
						writefile << "\t\t" << "r - Robot" << "\n"; 
						getline(readfile, dummy);
						writefile << "\t\t" << "bin5 bin6 bin7 bin8 - Bin"  << "\n";
						getline(readfile, dummy);
						writefile << "\t\t" <<"tray1 tray2 - Tray"  << "\n";
					}

					else {
						writefile << line << "\n";
					}
					break;
				case 'q':
					found = line.find("= (order_parts");
					if (found != std::string::npos) {
						std::cout << "Ready To Modify Order Parts" << std::endl;
						std::stringstream ss;
						ss << "(= (order_parts gear) " << _gear_part_count << ")";
						writefile << "\t\t" << ss.str() << "\n";
						ss.str(std::string());
						getline(readfile, dummy);
						ss << "(= (order_parts piston) " << _piston_rod_part_count << ")";
						writefile << "\t\t" << ss.str() << "\n";
					}

					else {
						writefile << line << "\n";
					}
					break;
				case 'l':
					found = line.find("inbin");
					if (found != std::string::npos) {
						std::cout << "Ready To Modify Part Location" << std::endl;
						std::stringstream ss;
						ss << "(inbin gear bin" << _bin_loc_gear << ")";
						writefile << "\t\t" << ss.str() << "\n";
						ss.str(std::string());
						getline(readfile, dummy);
						ss << "(inbin gear bin" << _bin_loc_piston << ")";
						writefile << "\t\t" << ss.str() << "\n";
					}

					else {
						writefile << line << "\n";
					}
					break;
				default:
					writefile << line << "\n";
			}

	}
}

	writefile.close();
	readfile.close();

	if (removeFile)
		remove(baseFile.c_str());

}

void PddlEditor::editPDDL(std::string baseFile, std::string targetFile, char lineType) {
	modify(baseFile, targetFile, lineType, false);
	modify(targetFile, baseFile, lineType, true);
}

void PddlEditor::setPartCount(int gear_part_count, int piston_rod_part_count) {
	_gear_part_count = gear_part_count;
	_piston_rod_part_count = piston_rod_part_count;
}

/* int main() {
	editPDDL('l');
	return 0;
}*/ 