# What is this?

This is a c++ framework for reading data off of a CAEN v1730 digitizer, using the existing CAEN API, designed to work with a python bindings for easier useage. 
Designed with a Midas interface in mind, this does _not_ require Midas to work, and could in principle be used with any backend database management. 


# Installing

## Requirements

- pybind11 
- the CAENComm, CAENVME, and CAENDigitizer libraries
- Python3 (tested on 3.10 and 3.12)
- cmake 3.14 
- A C++17 capable compiler 

## Compile it 

From the `pyV1730` repository, create a build directory.
Enter this build directory and run `cmake ../`, then run `make`. 

This might take a moment. It will create a shared object compiled against **the specific version of python** picked out by cmake and your machine. 
Install this shared object `caen_v1730.[****].so` into a folder pointed to in your PYTHONPATH. 

If you're using a venv, it'll be something like `python3_env/lib/python3.12/site-packages`. 

You should now be able to import caen_v1730 as if it's a normal python library. 

# Bindings
The following methods are accepted 

- `start_acquisition` - tells the digitizer to start streaming data
- `stop_acquisition` - tells the digitizer to stop streaming data
- `read_waveforms` - returns a collection of waveforms for each open channel. May be zero-length immediately after acquisition begins. 

- `count_hits` - returns a vector of the number of hits observed over a period of acqusition. The last entry in the vector is the number of triggers in that period. 
- `set_threshold` - the step-size necessary for a spike to be counted as a `hit`: see `count_hits` 
- `pull charge` - still a WIP
- `set_record_length` - how many samples to collect per trigger 
- `set_pos_trig_size` - what fraction (0-100) of the collected samples should be after the trigger 
- `read_register` - reads a register on the digitizer and returns it


# Useage

A `V1730Digitizer` object first needs to be created with the appropriate link type.
An external trigger should be configured (currently, this is untested with the software/internal trigger). Afterwards you can start/stop the data acqusition and/or configure thresholds. 

```
digi = caen_v1730.V1730Digitizer(
    link_type=1,
    link_num = 0,
    conet_node = 0,
    base_addr = 0
)

digi.configure_external_trigger()
digi.start_acquisition()
digi.set_threshold(2, -25)
```

Then, hits or waveforms can be counted using 
```
digi.count_hits()
```