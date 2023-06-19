#include <frt/frt.hpp>

int main (int argc, char *argv[]) 
{
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::cout.tie(nullptr);

    if (argc < 2) {
        FRT::Logger::error("Robot ID not specified. Usage: ./main <robot_id>");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
