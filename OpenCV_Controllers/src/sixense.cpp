#include "../headers/sixense.h"
#include "../headers/shared_memory.h"

freepie_io::shared_memory<std::array<SixenseEmulatedData, 2>> shared_memory;
freepie_io::view<std::array<SixenseEmulatedData, 2>> mapping;

void initSixense()
{
	shared_memory = freepie_io::shared_memory<std::array<SixenseEmulatedData, 2>>("SixenseEmulatedData");
	mapping = shared_memory.open_view();
}

void sixenseSetData(std::array<SixenseEmulatedData, 2> input) {
	mapping.write(input);
}