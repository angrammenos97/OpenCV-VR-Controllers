#include "../headers/FreeTrackClient.h"
#include "../headers/shared_memory.h"

freepie_io::shared_memory<FreeTrackData> shared_memory;
freepie_io::view<FreeTrackData> mapping;

void initFreeTrack() {
	shared_memory = freepie_io::shared_memory<FreeTrackData>("FT_SharedMem");
	mapping = shared_memory.open_view();
}

void freetrackSetData(FreeTrackData& input) {
	if (input.DataID > (1 << 29))
		input.DataID = 0;
	input.DataID++;	
	mapping.write(input);
}