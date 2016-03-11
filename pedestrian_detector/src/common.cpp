#include "common.h"
using namespace std;

std::stack<clock_t> tictoc_stack;

extern void tic()
{
    tictoc_stack.push(clock());
}

extern void toc() {
    std::cout << "Time elapsed: "
              << ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC
              << std::endl;
    tictoc_stack.pop();
}

extern float tocMatteo()
{
    float currentTime=((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC;
    //std::cout << "Time elapsed: "
    //         << ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC
    //          << std::endl;
    tictoc_stack.pop();
    return currentTime;
}
