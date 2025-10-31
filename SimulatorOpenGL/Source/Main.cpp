#include "ch.h"
#include "Application.h"


// Main method fpr 
int main() {
	auto app = std::make_unique<Application>("Simulator V0.1");

	app->run();

	return 0;
}