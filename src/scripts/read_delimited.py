# Based on https://www.datadoghq.com/blog/engineering/protobuf-parsing-in-python/

import os
from pprint import pprint
import sys
 
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "protos", "generated"))
import time_step_pb2

from google.protobuf.internal.decoder import _DecodeVarint32
 
def main():

    with open('../../results/cpp_sensor.data', 'rb') as f:
        buf = f.read()
        n = 0
        while n < len(buf):
            msg_len, new_pos = _DecodeVarint32(buf, n)
            n = new_pos
            msg_buf = buf[n:n+msg_len]
            n += msg_len
            read_metric = time_step_pb2.TimeStep()
            read_metric.ParseFromString(msg_buf)
            print(read_metric)
 
if __name__ == "__main__":
    main()
