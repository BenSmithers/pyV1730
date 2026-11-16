import midas.client
import midas
import midas.frontend
import midas.event
import caen_v1730
import time 
darkrate_mode = False 
charge_mode = False 
counter = 0 
def run_start(client, run_no, digi):
    global darkrate_mode, counter, charge_mode
    digi.reset_counter()
    try: 
        data_mode = client.odb_get("/Equipment/controller0/Settings/data_mode") 
        darkrate_mode = data_mode==0
        digi.recalibrate()

        collection_mode = client.odb_get("/Equipment/controller0/Settings/collection_mode")
        charge_mode = collection_mode==1 
        if darkrate_mode:
            digi.start_acquisition()
    except Exception as e:
        return 2 
    return 1 

def update_values(client, path, update_what):
    if update_what==0:
        return 

    monitor_thresh = update_what== (update_what & 8)
    if monitor_thresh:
        new_thresh = client.odb_get("/Equipment/controller0/Settings/mon_thresh_working")
        client.msg("Updating mon PMT Thresh to {}".format(new_thresh))
        digi.set_threshold(2, -8)
        client.odb_set("/Equipment/controller0/Settings/mon_thresh", new_thresh)


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
    digi.set_threshold(2, -8)
    digi.set_post_trig_size(75)
    digi.recalibrate()

    communication_counter = 0

    buffer_handle = client.open_event_buffer("SYSTEM")

    scan_request_handle = client.register_event_request(buffer_handle,event_id=1)
    client.register_transition_callback(midas.TR_START, 1, lambda _client,_run_no: run_start(_client, _run_no, digi))
    client.register_transition_callback(midas.TR_STOP, 1,  lambda _client,_run_no: run_stop(_client, _run_no, digi))
    client.register_transition_callback(midas.TR_STARTABORT, 1,  lambda _client,_run_no: run_stop(_client, _run_no, digi))
    client.odb_watch("/Equipment/controller0/Settings/updater", update_values, False)
    
    accept_events = False 
    while True: 
        event = client.receive_event(buffer_handle, async_flag=True)
        if not darkrate_mode:
            if event is not None:
                for bank in event.banks.values():
                    if "EOM"== bank.name[:3]:
                        digi.start_acquisition()
                        accept_events=True 
                    elif "BONM"==bank.name:
                        digi.stop_acquisition() 
                        accept_events=False 

        if accept_events or darkrate_mode:
            new_event = midas.event.Event()
            if charge_mode :
                res = digi.pull_charge(500) 
                new_event.create_bank("MCHG", midas.TID_UINT16, res[1])
                new_event.create_bank("NOWT", midas.TID_DOUBLE, [digi.time_into_run(), ])
            else:
                res = digi.count_hits(500)
                new_event.create_bank("CNTS", midas.TID_INT, res)
                new_event.create_bank("NOWT", midas.TID_DOUBLE,[digi.time_into_run(), ])
            
            new_event.header.event_id=74
            new_event.header.trigger_mask=0
            new_event.header.serial_number=counter 
            new_event.header.timestamp= int(time.time())
            new_event.populate_bank_and_event_size()    

            counter +=1 
            client.send_event(buffer_handle, new_event)

        communication_counter+=1

        if communication_counter%50==0:
            communication_counter = 1
            client.communicate(1,True)
