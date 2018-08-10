def format(*data):
    field = ['cc_mag_field', 'cc_offset0', 'cc_offset1', 'cc_offset2', 'cc_gain0', 'cc_gain1', 'cc_gain2', 'cc_t0', 'cc_t1', 'cc_t2', 'cc_t3', 'cc_t4', 'cc_t5']
    for k,v in zip(field, data):
        print '<param name="%s" value="%f"/>' % (k, v)

format(0.314500, -2.334668, -1.182157, 0.000000, 2.909083, 3.450224, 3.179654, -0.000277, 0.000000, -0.000234, 0.000000, 0.000000, 0.000000)
