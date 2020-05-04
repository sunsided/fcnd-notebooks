import numpy as np 

def gps_and_barometer_measurements():
    z_true = 50

    sigma_gps = 10
    sigma_baro = 1
    rate_gps = 10
    rate_baro = 100
    baro_drift = 0.1

    measuring_time = 10 # sec

    # GPS data 
    gps_sample_number = measuring_time * rate_gps
    gps_t = np.linspace(0.0,measuring_time,gps_sample_number)
    gps_data = z_true + np.random.normal(0.0, sigma_gps,(gps_sample_number,1)) 

    # Baro data
    baro_sample_number = measuring_time * rate_baro
    baro_t = np.linspace(0.0,measuring_time,baro_sample_number)
    baro_data = z_true + np.random.normal(0.0, sigma_baro,(baro_sample_number,1)) + baro_drift * baro_t.reshape(-1,1)

    return gps_t, gps_data, sigma_gps, baro_t, baro_data, sigma_baro