import serial, time, io, keyboard, winsound, random
from psychopy import event, core, sound
from pyglet.window import key
import matplotlib.pyplot as plt
import numpy as np
import winsound
#Check available ports
import serial.tools.list_ports
ports = serial.tools.list_ports.comports()

from bit_functions import bin, bitget, bitand

print('Available ports:')
for port, desc, hwid in sorted(ports):
        print("{}: {} [{}]".format(port, desc, hwid))
 

############################################
### MARKERS
##2.1.3 Set up parallel port
ppPresent = 0 #If no parallel port connection set up (e.g. Spike) = 0; else = 1 

#2.1.3.2 Set the address of the parallel port
if ppPresent:
        from ctypes import windll
        from intart_functions import sendParallelTrigger, readParallelTrigger        
        pport_address_out=888 #0x378
        pport_address_in = 889
        pport=windll.inpoutx64 #create the connection via specific dll 
        pport.Out32(pport_address_out,0) #set all the parallel port pins to zero at the beginning of the experiment

#2.1.3.3 Set Starting markers 
mrkrCode_start=240
mrkrCode_end=241

#2.1.3.4 Set Resting state Markers
mrkrCode_rs_start=200
mrkrCode_rs_end=201

#2.1.3.5 Set Block Marker
mrkrCode_block_start=100 #sommalo ad n_blocks
mrkrCode_block_end=101 #sommalo ad n_blocks

############################################

#3.5 Sounds
# 3.5.1 settings
sound_Hz = 250#note name (“C”,”Bfl”), filename or frequency (Hz)
sound_secs = 0.1#duration (for synthesised tones)

sound_vol_low = 0.005
sound_vol_high = 0.05
sound_vol = sound_vol_high

vol_step = 0.0005 #by how much the volume changes with each adjustment (+/-)
sound_sampleRate = 44100#sample rate for synthesized tones

#3.5.2 Tkc marking start and end of trial
tick = sound.Sound(600,secs=sound_secs,sampleRate=sound_sampleRate)#sample rate ignored because already set

#3.5.3 sound stim setup
soundStim = sound.Sound(value = sound_Hz,secs = sound_secs,sampleRate = sound_sampleRate)
soundStim.setVolume(sound_vol)

#3.5.3.1 sound stim presentation randomizer
soundStim_isi_list1 = [0.5,0.75,1,1.25,1.5] #inter-stimulus intervals (isi)
soundStim_isi_list2 = [0.5,0.75,1,1.25,1.5,1.75,2,2.25,2.5]

sound_Hz_range = range(150,251) #sound pitch (Hz) range
sound_Hz_range_min = 225
sound_Hz_range_max = 275

soundStim_isi_lists = [soundStim_isi_list1,soundStim_isi_list2] #pick list
soundStim_isi_list = random.choice(soundStim_isi_lists) #initial choice of interval for soundStim presentation
soundStim_isi = random.choice(soundStim_isi_list) #select isi from selected list



def connectPort():
        global ser    
        ser = serial.Serial(
            port='COM7',\
            baudrate=9600,\
            parity=serial.PARITY_NONE,\
            stopbits=serial.STOPBITS_ONE,\
            bytesize=serial.EIGHTBITS,\
                timeout=0)
        
        print("connected to: " + ser.portstr)
        
        def get_bit(byteval,idx):
                return ((byteval&(1<<idx))!=0)



play_tone = False
tick = 0
beat = 0
counter = 0
byte = []
Listening = True

output     = []
output_bin = []
count      = []

def sync_test():
        global ser
        data_raw   = []
        buff_save  = []
        bin_save   = []
        bit_counter = 0
        
        print('starting sync test\n')   
        
        buff = np.zeros(5)
        sync_ok = False
        duration = 5
        tic = time.time()
        while sync_ok == False:
                bit = int(np.frombuffer(ser.read(1), np.uint8))
                buff = np.append(buff[1:5],[bit])
                
                data_raw.append(bit)
                buff_save.append(buff)
                buff = buff.astype('int')
                if len(buff_save) > 10:
                        if sum(bitget(int(buff[0]),range(0,6)))==0 and sum(bitget(buff[0],7))==1 and sum(bitget(buff[1],0))==1 and (bitand(sum(buff[0:4]),255)-buff[4])==0:
                                sync_ok= True;
                        elif  time.time() > tic + duration:
                                print('no signal')
                                break
        print('sync test over')
        if sync_ok:
                print('sync OK')
        else:
                print('sync not OK')
        return sync_ok

def sync_test2():
        global ser
        data_raw   = []
        buff_save  = []
        bin_save   = []
        bit_counter = 0
        
        print('starting sync test\n')   
        
        buff = np.zeros(5)
        sync_ok = False
        duration = 5
        tic = time.time()

        buff = np.frombuffer(ser.readline(5), np.uint8)
        while sync_ok == False:
                buff_save.append(buff)
                buff = buff.astype('int')
                if sum(bitget(int(buff[0]),range(0,6)))==0 and sum(bitget(buff[0],7))==1 and sum(bitget(buff[1],0))==1 and (bitand(sum(buff[0:4]),255)-buff[4])==0:
                        sync_ok= True;
                        return sync_ok
                elif  time.time() > tic + duration:
                        print('no signal')
                        return
                else:
                        buff = np.append(buff[1:5],[int(np.frombuffer(ser.read(1), np.uint8))])
        return sync_ok

def bitPlot():
        global ser
        data_raw   = []
        buff_save  = []
        bin_save   = []
        bit_counter = 0
        data_counter = []
        data_plot = []
        
        print('starting sync test\n')   
        
        buff = np.zeros(5)
        running = False
        duration = 15
        tic = time.time()
        while running == False:
                for c in ser.readline():
                        bit = np.frombuffer(c, np.uint8)
                        buff = np.append(buff[1:5],[bit])
                        data_raw.append(bit)
                        buff_save.append(buff)
                        buff = buff.astype('int')
                        if sum(bitget(int(buff[0]),range(0,6)))==0 and sum(bitget(buff[0],7))==1 and sum(bitget(buff[1],0))==1 and (bitand(sum(buff[0:4]),255)-buff[4])==0:
                                data_plot = np.append(data_plot,buff)          
                        if  time.time() > tic + duration:
                                running = True
        print('sync test over')
        for i in range(len(data_plot)):
                data_counter.append(i)
        plt.scatter(data_counter,data_plot)


def beat_test():
        global ser
        data_raw   = []
        buff_save  = []
        bin_save   = []
        beats      = 0
        previous_beat = 0
        
        soundPlayed = False
        
        print('starting test\n')   

        buff = np.zeros(5)
        sync_ok = sync_test()
        sync_ok = sync_test2()
        duration = 7
        tic = time.time()
        start_delay = 3
        
        
        while sync_ok == True and time.time() < (tic + duration + start_delay):
                
                buff = np.frombuffer(ser.readline(5), np.uint8)
                buff = buff.astype('int')

                if len(buff) == 5 and time.time() > tic + start_delay:
                        if soundPlayed == False:
                                winsound.Beep(600,200)
                                soundPlayed = True
                        if sum(bitget(int(buff[0]),range(0,6)))==0 and sum(bitget(buff[0],7))==1 and sum(bitget(buff[1],0))==1 and (bitand(sum(buff[0:4]),255)-buff[4])==0:
                                if sum(bitget(int(buff[1]),range(1,4)))>0:
                                        print('sensor problem (3)')
                                elif sum(bitget(int(buff[1]),range(6,7)))>0:
                                        if not previous_beat:
                                                previous_beat = 1
                                                beats += 1
                                                print('beat. New total: %d' % beats)
                                else:
                                        previous_beat = 0

        winsound.Beep(600,200)
        print('test over')
        print('Number of beats: %d' % beats)



def beat_test2():
        global ser
        data_raw   = []
        buff_save  = []
        bin_save   = []
        beats      = 0
        previous_beat = 0
        
        soundPlayed = False
        
        tr_running = True
         
        buff = np.zeros(5)
        sync_ok = sync_test()
        sync_ok = sync_test2()
        tr_dur = 10
        tic = time.time()
        start_delay = 3
        start_print = True
        beat_reg = False
        
        
        while sync_ok == True and tr_running:  
                buff = np.frombuffer(ser.readline(5), np.uint8)
                buff = buff.astype('int')

                if len(buff) == 5 and time.time() > (tic+start_delay):
                        if start_print:
                                print('starting test\n')
                                start_print = False
                        if soundPlayed == False:
                                winsound.Beep(600,200)
                                soundPlayed = True
                        if sum(bitget(int(buff[0]),range(0,6)))==0 and sum(bitget(buff[0],7))==1 and sum(bitget(buff[1],0))==1 and (bitand(sum(buff[0:4]),255)-buff[4])==0:
                                if sum(bitget(int(buff[1]),range(1,4)))>0:
                                        print('sensor problem (3)')
                                elif sum(bitget(int(buff[1]),range(6,7)))>0:
                                        if not previous_beat:
                                                if beat_reg:
                                                        previous_beat = 1
                                                        beats += 1
                                                        print('beat. New total: %d' % beats)
                                                elif beat_reg == False:
                                                        previous_beat = 1
                                                        beat_reg = True
                                                        
                                                #print(buff) 
                                else:
                                        previous_beat = 0
                if time.time() > (tic + start_delay + tr_dur):
                        tr_running = False

        winsound.Beep(600,200)
        print('test over')
        print('Number of beats: %d' % beats)
 

def run_trial():
        global ser
        data_raw   = []
        buff_save  = []
        bin_save   = []
        beats      = 0
        previous_beat = 0                                
        soundPlayed = False
        
        # trial configs
        tr_n = 3 # nr of trials per block
        tr_dur = 20 # length of trial (seconds)
        tr_count = 0
         
        #buff = np.zeros(5,'uint8')
        buff = np.zeros(5)
        sync_ok = sync_test()
        sync_ok = sync_test2()
        tr_dur = 10
        start_delay = 3
        beat_reg = False
        
        task_running = True
        
        if sync_ok:
        
                while task_running:
                        buff = np.frombuffer(ser.readline(5), np.uint8)
                        #buff_save.append(buff)
                        buff = buff.astype('int')
                        
                        for t in range(tr_n):
                                tr_count += 1
                                print('trial %d start' % tr_count)
                                tic = time.time()
                                tr_start = tic + 10
                                tr_running = True
                                
                                resp          = 0
                                beats         = 0
                                previous_beat = 0  
                                beat_reg      = False
                                
                                print('+')
                                
                                while tr_running:  
                                        buff = np.frombuffer(ser.readline(5), np.uint8) # read bits from pulseOx
                                        #buff_save.append(buff)
                                        buff = buff.astype('int')                                
                                        if len(buff) == 5 and time.time() > (tic+start_delay):
                                                if sum(bitget(int(buff[0]),range(0,6)))==0 and sum(bitget(buff[0],7))==1 and sum(bitget(buff[1],0))==1 and (bitand(sum(buff[0:4]),255)-buff[4])==0:
                                                        if sum(bitget(int(buff[1]),range(1,4)))>0:
                                                                print('sensor problem (3)')
                                                        elif sum(bitget(int(buff[1]),range(6,7)))>0:
                                                                if not previous_beat:
                                                                        if beat_reg:
                                                                                previous_beat = 1
                                                                                beats += 1
                                                                                print('beat. New total: %d' % beats)
                                                                        elif beat_reg == False: # wait for the signal to stabilize and beats to be registered before starting a new trial
                                                                                previous_beat = 1
                                                                                tr_start = time.time()
                                                                                winsound.Beep(600,200)
                                                                                beat_reg = True
                                                        else:
                                                                previous_beat = 0
                                        if time.time() > (tr_start + start_delay + tr_dur):
                                                winsound.Beep(600,200)
                                                tr_running = False
          
                                resp = input('How many beats? ')
                                print('actual number of beats: %d' % beats)
                                acc = 1-(abs(float(beats-resp))/(float(beats)))
                                print('trial hit-rate: %f' % float(acc))
                                print('#####################')
                                
                                if tr_count == tr_n:
                                        task_running = False

        print('test over')


def trial_ext():
        global ser
        data_raw   = []
        buff_save  = []
        bin_save   = []
        beats      = 0
        previous_beat = 0                                
        soundPlayed = False
        
        # trial configs
        tr_n = 3 # nr of trials per block
        tr_dur = 20 # length of trial (seconds)
        tr_count = 0
         
        #buff = np.zeros(5,'uint8')
        buff = np.zeros(5)
        sync_ok = sync_test()
        sync_ok = sync_test2()
        tr_dur = 10
        start_delay = 3
        beat_reg = False
        
        task_running = True
        
        if sync_ok:
                while task_running:
                        buff = np.frombuffer(ser.readline(5), np.uint8)
                        #buff_save.append(buff)
                        buff = buff.astype('int')
                        
                        for t in range(tr_n):
                                tr_count += 1
                                print('trial %d start' % tr_count)
                                tic = time.time()
                                tr_start = tic + 10
                                tr_running = True
                                trial_start_flag = False
                                
                                resp          = 0
                                beats         = 0
                                previous_beat = 0  
                                beat_reg      = False
                                
                                # sound stim configurations
                                soundStim_isi_list = random.choice(soundStim_isi_lists)
                                soundStim_isi = random.choice(soundStim_isi_list)   
                                soundStim_check = False
                                soundStim_counter = 0
                                
                                # spike HB configs
                                HB_spike = 0
                                beat_1st = False
                                beat_time_list = [] # reset heartbeat time tracker                                
                                
                                print('+')
                                
                                while tr_running:  
                                        buff = np.frombuffer(ser.readline(5), np.uint8) # read bits from pulseOx
                                        #buff_save.append(buff)
                                        buff = buff.astype('int')                                
                                        if len(buff) == 5 and time.time() > (tic+start_delay):
                                                if sum(bitget(int(buff[0]),range(0,6)))==0 and sum(bitget(buff[0],7))==1 and sum(bitget(buff[1],0))==1 and (bitand(sum(buff[0:4]),255)-buff[4])==0:
                                                        if sum(bitget(int(buff[1]),range(1,4)))>0:
                                                                print('sensor problem (3)')
                                                        elif sum(bitget(int(buff[1]),range(6,7)))>0:
                                                                if not previous_beat:
                                                                        if beat_reg:
                                                                                previous_beat = 1
                                                                                beats += 1
                                                                                #print('beat. New total: %d' % beats)
                                                                        elif beat_reg == False: # wait for the signal to stabilize and beats to be registered before starting a new trial
                                                                                previous_beat = 1
                                                                                tr_start = time.time()
                                                                                stimTime = (tr_start + 2)
                                                                                trial_start_flag = True
                                                                                if ppPresent:
                                                                                        #triggerCode=i
                                                                                        triggerCode = 255
                                                                                        sendParallelTrigger(pport_address_out,triggerCode)    #Trial START marker                                                                                
                                                                                winsound.Beep(600,200)
                                                                                beat_reg = True
                                                        else:
                                                                previous_beat = 0
                                        if ppPresent:
                                                signal_in = readParallelTrigger(pport_address_in)
                                                if signal_in > 63 and beat_1st == True and time.time() > beat_time + 0.1: # If not first heartbeat of trial. Added minimal possible time between beats (0.03 seconds)
                                                        HB_spike += 1
                                                        beat_time = time.time()  
                                                        beat_time_list.append(beat_time) # record time of each heartbeat
                                                elif signal_in > 63 and beat_1st == False: # If first heartbeat of trial
                                                        HB_spike += 1
                                                        beat_1st = True
                                                        beat_time = time.time() 
                                                        beat_time_list.append(beat_time) # record time of each heartbeat                                        
                                        # play external stim
                                        if trial_start_flag:
                                                if time.time() > (tr_start + 1) and time.time() < (tr_start + tr_dur - 0.8): #After 1 second from trial start, start counter for generating sound stimulus, with buffer between last stim and END tone
                                                        if time.time() > (stimTime + soundStim_isi) and soundStim_check == False: 
                                                                sound_Hz = random.randint(sound_Hz_range_min, sound_Hz_range_max) #randomize stimulus pitch
                                                                #soundStim = sound.Sound(value = sound_Hz,secs = sound_secs,volume = sound_vol, sampleRate = sound_sampleRate)
                                                                soundStim = sound.Sound(value = sound_Hz,secs = sound_secs,sampleRate = sound_sampleRate)#regenerate stim signal
                                                                soundStim.setVolume(sound_vol) #set current volume               
                                                                soundStim.play() 
                                                                if ppPresent:
                                                                        sendParallelTrigger(pport_address_out,triggerCode) # Flag sound stim played
                                                                soundStim_counter += 1 #increment stim counter
                                                                stimTime = time.time() # keep track of time since last stim
                                                                soundStim_isi = random.choice(soundStim_isi_list)                    
                                                                soundStim_check = True
                                                        elif soundStim_check == True and time.time() > (stimTime + soundStim_isi):
                                                                soundStim_check = False # reset to load new sound stim
                                                
                                                
                                        if time.time() > (tr_start + start_delay + tr_dur):
                                                winsound.Beep(600,200)
                                                tr_running = False
          
                                resp = input('How many beats? ')
                                print('actual number of beats: %d' % beats)
                                acc = 1-(abs(float(beats-resp))/(float(beats)))
                                print('trial hit-rate: %f' % float(acc))
                                print('Spike HBs: %d' % HB_spike)
                                print('#####################')
                                
                                if tr_count == tr_n:
                                        task_running = False
                       
        print('test over')

def register_beats(tr_running):
        global ser
        data_raw   = []
        buff_save  = []
        bin_save   = []
        beats      = 0
        previous_beat = 0                                
         
        #buff = np.zeros(5,'uint8')
        buff = np.zeros(5)
        sync_ok = sync_test()
        sync_ok = sync_test2()
        start_delay = 3
        if sync_ok:                
                while tr_running:  
                        buff = np.frombuffer(ser.readline(5), np.uint8) # read bits from pulseOx
                        #buff_save.append(buff)
                        buff = buff.astype('int')                                
                        if len(buff) == 5 and time.time() > (tic+start_delay):
                                if sum(bitget(int(buff[0]),range(0,6)))==0 and sum(bitget(buff[0],7))==1 and sum(bitget(buff[1],0))==1 and (bitand(sum(buff[0:4]),255)-buff[4])==0:
                                        if sum(bitget(int(buff[1]),range(1,4)))>0:
                                                print('sensor problem (3)')
                                        elif sum(bitget(int(buff[1]),range(6,7)))>0:
                                                if not previous_beat:
                                                        if beat_reg:
                                                                previous_beat = 1
                                                                beats += 1
                                                        elif beat_reg == False: # wait for the signal to stabilize and beats to be registered before starting a new trial
                                                                previous_beat = 1
                                                                tr_start = time.time()
                                                                winsound.Beep(600,200)
                                                                beat_reg = True
                                        else:
                                                previous_beat = 0
                return beats


