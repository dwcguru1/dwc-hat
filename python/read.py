#!/usr/bin/env python3
import sys
from smbus import SMBus
import struct
import time
import datetime

from influxdb import InfluxDBClient
import configparser

config = configparser.ConfigParser()
config.read("config.ini")

ifuser = config["grafana"]["user"]
ifpass = config["grafana"]["pass"]
ifdb = config["grafana"]["home"]
ifhost = config["grafana"]["host"]
ifport = int(config["grafana"]["port"])

#i2c
#default akku constant 0x079D5157
i2c_ec = int(config["i2c"]["i2c_ec"])
i2c_ph = int(config["i2c"]["i2c_ph"])
i2c_pressure = int(config["i2c"]["i2c_pressure"])
i2c_ht = int(config["i2c"]["i2c_ht"])

# 1Wire 
ds18b20 = config["1wire"]["ds18b20"]

# PH parameter
ph_gain = int(config["ph"]["PH_GAIN"])    # amplifier in adc for PH
ph_lsb = 2.0 * 2.048 / 262144.0
ph_pga = float(ph_gain)

# measured EC
# resistors give values like measuring a fluid at 25C
ec_adc_390R = float(config["ec"]["EC_ADC_390R"])
ec_adc_1k = float(config["ec"]["EC_ADC_1K"])

# temperature compensation factor commonly used for nutirent solutions
ec_alpha = float(config["ec"]["EC_ALPHA"])

# constants EC (do not edit)
EC_390R = 2564.10
EC_1K  = 1000.0

# measured PH 
PH_401 =  float(config["ph"]["PH_401"])
PH_686 =  float(config["ph"]["PH_686"])


# reading of sensor without pressure applied
P_OFS = float(config["pressure"]["P_OFS"])


i2c = SMBus(1)  # Create a new I2C bus

def _crc(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc <<= 1
                crc ^= 0x131
            else:
                crc <<= 1
    return crc

def twist(x):
    return struct.unpack('>I', struct.pack('<I', x))[0]

def twist_uint16(x):
    return struct.unpack('>H', struct.pack('<H', x))[0]

def read_uint32(i2c_addr, reg_address):
    data = i2c.read_i2c_block_data(i2c_addr, reg_address, 4)
    return struct.unpack('<I', bytearray(data))[0]

def read_uint16(i2c_addr, reg_address):
    data = i2c.read_i2c_block_data(i2c_addr, reg_address, 2)
    return struct.unpack('<H', bytearray(data))[0]

def write_uint32(i2c_addr, reg_address, value):
    data = struct.pack('<I', value)
    i2c.write_i2c_block_data(i2c_addr, reg_address, list(data))

def ec_adc_to_ec(adcval):
    # f(x) = m*x+t
    # m=dy/dx
    m = (ec_adc_390R - ec_adc_1k)/(EC_390R - EC_1K)
    # t=f(x)-m*x
    t = ec_adc_1k - m * EC_1K
    # x = (f(x)-t) / m
    ec = (adcval - t) / m
    # print("{:f} {:f} {:f}".format(m, t, ec))
    return ec

def ec_temp_compensate(ec, temp):
    # EC25 = EC/(1+a(T-25))
    return ec / (1 + ec_alpha * (temp - 25.0))

def ph_adc_to_ph(adcval):
    # f(x)=m*x+t
    m = (PH_686 - PH_401) / (6.86 - 4.01)
    # t=f(x)-m*x
    t = PH_401 - m * 4.01
    # x=(f(x)-t)/m
    ph = (float(adcval) - t) / m
    return ph


def ph_read():
    config_byte = 0x1c
    if ph_gain==2:
        g=1
    elif ph_gain==4:
        g=2
    elif ph_gain==8:
        g=3
    else:
        pass # illegal value TODO

    i2c.write_byte(i2c_ph, config_byte)

    time.sleep(0.5)

    # format: SSSSSSSD | DDDDDDDD | DDDDDDDD | CCCCCCCC
    unsigned = twist(read_uint32(i2c_ph, config_byte))

    # remove config byte
    # after: 000000000 | SSSSSSSD | DDDDDDDD | DDDDDDDD
    unsigned >>= 8


    # extend sign
    # after: SSSSSSSSS | SSSSSSSD | DDDDDDDD | DDDDDDDD
    if unsigned & 0x00800000:
        unsigned |= 0xff000000

    # unsigned to signed
    signed = struct.unpack('<i', struct.pack('<I', unsigned))[0]

#    print(str(signed))
#    print("voltage: {:f}".format(voltage))
    ph = ph_adc_to_ph(signed)
    # print(signed)
    # print("{:08x}".format(unsigned))
    return (signed, ph)

def ec_read_akku():
    return read_uint32(i2c_ec, 0x00)

def ec_write_akku(value):
    write_uint32(i2c_ec, 0x00, value)

def ec_read_adcval():
    return read_uint16(i2c_ec, 0x04)

def ec_dds(enabled):
    data = [0x80 if not enabled else 0x00]
    i2c.write_i2c_block_data(i2c_ec, 0x04, data)


def temp_read():
    # Read 1-wire Slave file
    file = open('/sys/bus/w1/devices/'+ds18b20+'/w1_slave')
    filecontent = file.read()
    file.close()
 
    # Read and convert temperature
    stringvalue = filecontent.split("\n")[1].split(" ")[9]
    return float(stringvalue[2:]) / 1000

def ec_read(temp):
    ec_dds(True)
    time.sleep(0.5)

    sum = 0
    for i in range(0,256):
        sum += ec_read_adcval()
        time.sleep(0.01)

    adcval = float(sum) / 255.0

    ec_dds(False)
    # print('0x{:04X}'.format(adcval))
    #print("temp:"+str(temp))
    ec = ec_adc_to_ec(adcval)
    #print("ec:"+str(ec))
    ec = ec_temp_compensate(ec, temp)
    #print("ec:"+str(ec))
    return (adcval, ec)
    

def pressure_read():
    sum = 0
    for i in range(0,32):
        sum += twist_uint16(read_uint16(i2c_pressure, 0x04))
        time.sleep(0.01)
    raw = sum / 32
    return raw

def ht_read_reg(reg):
    valid = False
    while True:
        data = i2c.read_i2c_block_data(i2c_ht, reg, 3)
        time.sleep(0.1)
        crc = data[2]
        if _crc(data[:2]) != crc:
            print(str((data)))
            print("crc error reading function 0x{:02x} ({:02x} vs {:02x}) . Trying again".format(reg, _crc(data[:2]), crc))
            time.sleep(1.0)
            continue
        return data


def xht_read():
    rh = ht_read_reg(0xe5)
    humidity = (float(rh[0] * 256 + rh[1]) * 125.0 / 65536.0) - 6.0

    temp = ht_read_reg(0xe3)
    cTemp = (float(temp[0] * 256.0 + temp[1]) * 175.72 / 65536.0) - 46.85

    return (humidity, cTemp)


def main():
    (humidity, airtemp) = xht_read()
    temp = temp_read()
    (raw_ec, ec) = ec_read(temp)
    (raw_ph, ph) = ph_read()
    pressure_raw = pressure_read()
    time = datetime.datetime.utcnow()
    timestamp = int(time.timestamp()*1000)

    print(str(time))
    print("time={}, ec_raw={}, ec={:f}, ph_raw={}, ph={:f}, temp={:f}, pressure_raw={}, humidity={}, airtemp={}".format(timestamp, raw_ec, ec, raw_ph, ph, temp, pressure_raw, humidity, airtemp))

    iso = timestamp + 2*3600*1000

    if True:
        # format the data as a single measurement for influx
        body = [
            {
                "measurement": "temp",
                "time": iso,
                "fields": {
                    "value": temp
                }
            },
            {
                "measurement": "ec",
                "time": iso,
                "fields": {
                    "ec": ec,
                    "ec_raw": raw_ec
                }
            },
            {
                "measurement": "ph",
                "time": iso,
                "fields": {
                    "ph": ph,
                    "ph_raw": raw_ph
                }
            },
            {
                "measurement": "si7102",
                "time": iso,
                "fields": {
                    "hum": humidity,
                    "temp": airtemp
                }
            },
            {
                "measurement": "Pressure",
                "time": iso,
                "fields": {
                    "pressure_raw": pressure_raw,
                    "pressure": 0
                }
            }
        ]

        # connect to influx
        ifclient = InfluxDBClient(ifhost,ifport,ifuser,ifpass,ifdb)
        
        # write the measurement
        ifclient.write_points(body, time_precision='ms')



if __name__ == "__main__":
    main()

