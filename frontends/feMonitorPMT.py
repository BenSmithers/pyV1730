import midas.client
import midas
import midas.frontend
import midas.event
import caen_v1730
import time 
darkrate_mode = False 
counter = 0 
def run_start(client, run_no, digi):
    global darkrate_mode, counter
    try: 
        data_mode = client.odb_get("/Equipment/controller0/Settings/data_mode") 
        darkrate_mode = data_mode==0
        
        if darkrate_mode:
            digi.start_acquisition()
    except Exception as e:
        return 2 
    return 1 

def run_stop(client, run_no, digi):
    global counter 
    counter = 0
    try:
        digi.stop_acquisition()
    except Exception as e:
        return 2
    return 1 

if __name__ == "__main__":
    this_time = int(time.time())
    client =  midas.client.MidasClient("feMonitorPMT")

    digi = caen_v1730.V1730Digitizer(
        link_type=1,
        link_num = 0,
        conet_node = 0,
        base_addr = 0
    )
    digi.configure_external_trigger()
    digi.set_threshold(2, -25)
    digi.set_post_trig_size(75)
    digi.recalibrate()


    buffer_handle = client.open_event_buffer("SYSTEM")

    scan_request_handle = client.register_event_request(buffer_handle,event_id=1)
    client.register_transition_callback(midas.TR_START, 1, lambda _client,_run_no: run_start(_client, _run_no, digi))
    client.register_transition_callback(midas.TR_STOP, 1,  lambda _client,_run_no: run_stop(_client, _run_no, digi))
    
    accept_events = False 
    while True: 
        event = client.receive_event(buffer_handle, async_flag=True)
        if not darkrate_mode:
            if event is not None:
                this_time = int(time.time())
                for bank in event.banks.values():
                    if "EOM"== bank.name[:3]:
                        digi.start_acquisition()
                        accept_events=True 
                    elif "BONM"==bank.name:
                        digi.stop_acquisition() 
                        accept_events=False 

        if accept_events or darkrate_mode:
            res = digi.count_hits(500)
            
            new_event = midas.event.Event()
            new_event.create_bank("CNTS", midas.TID_INT, res)

            new_event.header.event_id=74
            new_event.header.trigger_mask=0
            new_event.header.serial_number=counter 
            new_event.header.timestamp= this_time
            new_event.populate_bank_and_event_size()    

            counter +=1 
            client.send_event(buffer_handle, new_event)


        client.communicate(1,True)
