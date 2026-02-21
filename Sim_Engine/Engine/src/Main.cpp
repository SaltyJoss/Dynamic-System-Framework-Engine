#include "pch.h"
// File:   Main.cpp
// GitHub: SaltyJoss
#include <EngineCore.h>
#include <Application.h>

int main(int argc, char** argv) {
    if (argc > 1 && std::string(argv[1]) == "--batch") {
        return runBatchMode();
    }

    Application app("DSFE");
    app.run();
    return 0;
}