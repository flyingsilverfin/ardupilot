from pymavlink import DFReader

class DFGPSConverter():
    def __init__(self, file):
        if file.endswith('.bin'): 
            df = DFReader.DFReader_binary(file)
        else:
            df = DFReader.DFReader_text(file)

    def convert(self):
        result = []     #(time, (lat, long, altitude))
        


    