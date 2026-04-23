
#include "digitizer.hpp"
#include <iostream>

V1730Digitizer::V1730Digitizer(int link_type, int link_num, int conet_node, uint32_t base_addr)
    : buffer(nullptr), bufferSize(0)
{
    int ret = CAEN_DGTZ_OpenDigitizer(
        (CAEN_DGTZ_ConnectionType)link_type,
        link_num,
        conet_node,
        base_addr,
        &handle
    );

    check_error(ret);

    CAEN_DGTZ_GetInfo(handle, &boardInfo);

    CAEN_DGTZ_Reset(handle);

    // Allocate readout buffer
    ret = CAEN_DGTZ_MallocReadoutBuffer(handle, &buffer, &bufferSize);
    check_error(ret);
}

V1730Digitizer::~V1730Digitizer() {
    if (buffer)
        CAEN_DGTZ_FreeReadoutBuffer(&buffer);

    CAEN_DGTZ_CloseDigitizer(handle);
}

void V1730Digitizer::check_error(int ret) {
    if (ret != CAEN_DGTZ_Success) {
        throw std::runtime_error("CAEN error code: " + std::to_string(ret));
    }
}

void V1730Digitizer::set_record_length(uint32_t size){
    check_error(CAEN_DGTZ_SetRecordLength(handle, size));
}

void V1730Digitizer::set_post_trig_size(uint32_t percent){
    if (percent>0 && percent<100){
        check_error(CAEN_DGTZ_SetPostTriggerSize(handle, percent));   
    }else{
        throw std::runtime_error("Invalid percentage: "+ std::to_string(percent));
    }
}

void V1730Digitizer::configure_external_trigger() {
    check_error(CAEN_DGTZ_SetAcquisitionMode(handle, CAEN_DGTZ_FIRST_TRG_CONTROLLED));
    // CAEN_DGTZ_GetSWTriggerMode 

    // External trigger on TRG-IN
    check_error(CAEN_DGTZ_SetExtTriggerInputMode(
        handle,
        CAEN_DGTZ_TRGMODE_ACQ_ONLY
    ));

    // Enable channels (example: first 3)
    uint32_t channelMask = 0x7;
    check_error(CAEN_DGTZ_SetChannelEnableMask(handle, channelMask));

    // Set record length (samples)
    check_error(CAEN_DGTZ_SetRecordLength(handle, 128));
    check_error(CAEN_DGTZ_SetPostTriggerSize(handle, 100));

    //add delay time between trigger and acquisition start to capture the rising edge of the signal
    // use 60 samples (or 30ns)
    //check_error(CAEN_DGTZ_SetDPPPreTriggerSize(handle, 2, 60));

    uint32_t maxnumaggregates; 
    check_error(CAEN_DGTZ_SetMaxNumAggregatesBLT(handle, 100));
    check_error(CAEN_DGTZ_SetMaxNumEventsBLT(handle, 100));
    check_error(CAEN_DGTZ_GetMaxNumAggregatesBLT(handle, &maxnumaggregates));

}

void V1730Digitizer::start_acquisition() {
    check_error(CAEN_DGTZ_SWStartAcquisition(handle));
}

void V1730Digitizer::stop_acquisition() {
    check_error(CAEN_DGTZ_SWStopAcquisition(handle));
}

void V1730Digitizer::set_threshold(uint16_t channel, float threshold) {
    if (channel >= boardInfo.Channels) {
        throw std::runtime_error("Invalid channel: " + std::to_string(channel));
    }

    thresholds[channel] = threshold;
}

std::vector<std::vector<float>> V1730Digitizer::pull_charge(int waveforms_sampled){
    /*
    Pull `waveforms_sampled` number of waveforms and collect some charges from the waveforms. Skip the trigger channel 
    */
    
    std::vector<std::vector<float>> charges;

    charges.push_back({}); 
    charges.push_back({}); 

    std::vector<int> these_channels = {0, 2};

    int sample_index = 0;
    uint32_t size = 0;
    uint32_t numEvents;
    float this_charge;

    for (int i=0; i<waveforms_sampled; i++){
        waveforms.clear(); 
        check_error(CAEN_DGTZ_ReadData(
            handle,
            CAEN_DGTZ_SLAVE_TERMINATED_READOUT_MBLT,
            buffer,
            &size
        ));
        if (size==0){
            continue; // No data, skip
        }

        CAEN_DGTZ_EventInfo_t eventInfo;
        check_error(CAEN_DGTZ_GetNumEvents(handle, buffer, size, &numEvents)); 
        for (uint32_t i = 0; i < numEvents; ++i) {
            check_error(CAEN_DGTZ_GetEventInfo(
                handle, buffer, size, i, &eventInfo, &eventPtr
            ));
            
            check_error(CAEN_DGTZ_DecodeEvent(handle, eventPtr, (void**)&evt));
            
            for(int channel_index=0; channel_index<these_channels.size(); channel_index++){
                uint16_t last_value=0;
                sample_index = 0;
                this_charge = 0;


                if (evt->ChSize[these_channels[channel_index]] > 0) {
                    std::vector<uint16_t> wf(
                        evt->DataChannel[these_channels[channel_index]],
                        evt->DataChannel[these_channels[channel_index]] + evt->ChSize[these_channels[channel_index]]
                    );

                    for (int sample_index=0; sample_index<wf.size(); sample_index++){
                        this_charge=this_charge+wf[sample_index]/wf.size();
                    }

                    charges[channel_index].push_back(this_charge);
                }
            }
            check_error(CAEN_DGTZ_FreeEvent(handle, (void**)&evt));
        }

    }

    return charges;
}

std::vector<int> V1730Digitizer::count_hits(int waveforms_sampled){
    /*
    Pull `waveforms_sampled` number of waveforms and count how many times each channel exceeds the threshold.
        Expect channel 1 to always show a hit - this is the trigger channel. 

    */
    std::vector<int> hit_counts = {0,0,0,0};
    
    bool found_hit = false; 
    int sample_index = 0;
    uint32_t size = 0;
    uint32_t numEvents;
       

    for (int i=0; i<waveforms_sampled; i++){
        waveforms.clear(); 
        check_error(CAEN_DGTZ_ReadData(
            handle,
            CAEN_DGTZ_SLAVE_TERMINATED_READOUT_MBLT,
            buffer,
            &size
        ));
        if (size==0){
            continue; // No data, skip
        }

        CAEN_DGTZ_EventInfo_t eventInfo;
        check_error(CAEN_DGTZ_GetNumEvents(handle, buffer, size, &numEvents)); 
        for (uint32_t i = 0; i < numEvents; ++i) {
            hit_counts[3]++;
            check_error(CAEN_DGTZ_GetEventInfo(
                handle, buffer, size, i, &eventInfo, &eventPtr
            ));
            
            check_error(CAEN_DGTZ_DecodeEvent(handle, eventPtr, (void**)&evt));
            
            for(int channel_index=0; channel_index<boardInfo.Channels; channel_index++){
                found_hit = false; 
                uint16_t last_value=0;
                sample_index = 0;

                if (evt->ChSize[channel_index] > 0) {
                    std::vector<uint16_t> wf(
                        evt->DataChannel[channel_index],
                        evt->DataChannel[channel_index] + evt->ChSize[channel_index]
                    );

                    while((!found_hit) && sample_index<wf.size()){

                        if(sample_index==0){
                            last_value = wf[sample_index];   
                        }
                        if ((wf[sample_index]-last_value) < thresholds[channel_index]){
                            hit_counts[channel_index]++;
                            found_hit = true;
                            
                        }
                        last_value = wf[sample_index];
                        sample_index++; 
                    }
                }
            }
            check_error(CAEN_DGTZ_FreeEvent(handle, (void**)&evt));
        }

    }
    return hit_counts;
}

std::vector<std::vector<std::vector<uint16_t>>> V1730Digitizer::acquire_multiple(int qty){
    std::vector<std::vector<std::vector<uint16_t>>> waveforms; 

    for (int i=0; i<qty; i++){
        waveforms.push_back(
            read_waveforms()
        );
    }

    return waveforms;
};

std::vector<std::vector<uint16_t>> V1730Digitizer::read_waveforms() {
    uint32_t size = 0;

    check_error(CAEN_DGTZ_ReadData(
        handle,
        CAEN_DGTZ_SLAVE_TERMINATED_READOUT_MBLT,
        buffer,
        &size
    ));

    if (size == 0)
        return {};

    CAEN_DGTZ_EventInfo_t eventInfo;

    std::vector<std::vector<uint16_t>> waveforms;

    uint32_t numEvents;
    
    // only ever assigns 1? 
    check_error(CAEN_DGTZ_GetNumEvents(handle, buffer, size, &numEvents)); 

    for (uint32_t i = 0; i < numEvents; ++i) {
        check_error(CAEN_DGTZ_GetEventInfo(
            handle, buffer, size, i, &eventInfo, &eventPtr
        ));

        CAEN_DGTZ_UINT16_EVENT_t* evt = nullptr;
        check_error(CAEN_DGTZ_DecodeEvent(handle, eventPtr, (void**)&evt));

        for (int ch = 0; ch < boardInfo.Channels; ++ch) {
            if (evt->ChSize[ch] > 0) {
                std::vector<uint16_t> wf(
                    evt->DataChannel[ch],
                    evt->DataChannel[ch] + evt->ChSize[ch]
                );
                waveforms.push_back(wf);

            }
        }

        CAEN_DGTZ_FreeEvent(handle, (void**)&evt);
    }

    return waveforms;
}