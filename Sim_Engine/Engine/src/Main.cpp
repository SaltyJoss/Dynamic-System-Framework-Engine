#include "pch.h"
// File:   Main.cpp
// GitHub: SaltyJoss
#include <EngineCore.h>
#include <Application.h>
#include "include/BatchEntry.h"

int main(int argc, char** argv) {
    printf("argc = %d\n", argc);
    for (int i = 0; i < argc; ++i)
        printf("argv[%d] = %s\n", i, argv[i]);

    if (argc > 1 && std::string(argv[1]) == "--batch") {
        return runBatchMode();
    }

    Application app("DSFE");
    app.run();
    return 0;
}