#include "rosMast/Garbage.h"
#include  <iostream>

void Garbage::testPrint() {
	std::cout << fuu << "\n";
	fuu = 10;
	std::cout << fuu << "\n";
}