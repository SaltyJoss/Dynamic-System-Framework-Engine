#include "CoreIncludes.h"
#include "GUIManager.h"
#include "Application.h"


// Main method fpr 
int main() {
	Application app;

	if (!app.Initialise())
		return -1;

	app.Run();
	app.Shutdown();
	
	return 0;
}