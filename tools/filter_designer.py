from libs.filter_design import *


if __name__ == "__main__":

    sampling_frequency  = 120000


    bass_filters    = FilterNode("bandstop",  50,    10, sampling_frequency)
    bass_filters    = FilterNode("lowpass",   100,   10, sampling_frequency, bass_filters)
    bass_filters    = FilterNode("bandpass",  200,   10, sampling_frequency, bass_filters)
    bass_filters    = FilterNode("bandpass",  300,   10, sampling_frequency, bass_filters)
    bass_filters    = FilterNode("bandpass",  400,   10, sampling_frequency, bass_filters)


    
    middles_filters = FilterNode("highpass", 1000,   10, sampling_frequency)
    middles_filters = FilterNode("lowpass",  2000,  10, sampling_frequency, middles_filters)
    middles_filters = FilterNode("bandpass", 3000,   10, sampling_frequency, middles_filters)

    trebles_filters = FilterNode("highpass", 8000,   10, sampling_frequency)
    trebles_filters = FilterNode("bandpass", 9000,  10, sampling_frequency, trebles_filters)

    filters         = FilterNode("lowpass", 10000)
    filters         = FilterNode("lowpass", 20000, prev=filters)
    filters.append(bass_filters)
    filters.append(middles_filters)
    filters.append(trebles_filters)
    
    
    #filters.traverse()
    
    bass_filters.traverse(stage=7)
