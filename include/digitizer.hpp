#pragma once

#include <vector>
#include <cstdint>
#include <stdexcept>

extern "C" {
#include <CAENDigitizer.h>
}

class V1730Digitizer {
public:
    V1730Digitizer(int link_type, int link_num, int conet_node, uint32_t base_addr);
    ~V1730Digitizer();

    void configure_external_trigger();
    void start_acquisition();
    void stop_acquisition();
    void set_record_length(uint32_t size);
    void set_post_trig_size(uint32_t percent);
    void set_threshold(uint16_t channel, float threshold);

    std::vector<int> count_hits(int waveforms_sampled);
    std::vector<std::vector<std::vector<uint16_t>>> acquire_multiple(int qty);
    std::vector<std::vector<uint16_t>> read_waveforms();

    std::vector<std::vector<float>> pull_charge(int waveforms_sampled);

private:
    int handle;
    CAEN_DGTZ_BoardInfo_t boardInfo;
    std::vector<std::vector<uint16_t>> waveforms;

    char* buffer;
    char* eventPtr = nullptr;
    CAEN_DGTZ_UINT16_EVENT_t* evt = nullptr;
    uint32_t bufferSize;

    uint16_t trigger_channel = 1;

    void check_error(int ret);

    std::vector<float> thresholds = {-10000, -1000, -100, 0};
};