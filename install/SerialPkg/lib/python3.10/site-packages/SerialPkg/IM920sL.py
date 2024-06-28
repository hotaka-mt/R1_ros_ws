import serial
import struct
import time

class IM920(serial.Serial):
    def __init__(self,port):
        super().__init__(port,19200)
        self.TXDA = bytes("TXDA ","utf-8")
        self.ENDCH = bytes([0xA,0xD])
        self.tx_data = ""
        self.rx_NN = 0
        self.rx_data = []
        self.data = ""

    def write(self,data):
        if len(data)%2 != 0:
            print("送信データは偶数個にしてください")
            return False
        super().write(self.TXDA)
        for i in range(len(data)):
            self.data = format(int.from_bytes(self.to_binary(data[i]),byteorder='big'),'x')
            self.tx_data = bytes(self.data.zfill(8),"utf-8")
            print(self.tx_data)
            super().write(self.tx_data)
        super().write(self.ENDCH)
    
    def raw_read(self,size):
        self.rx_data = super().read(13+size*3-1)
        if self.rx_data[0]==0x30 and self.rx_data[1]==0x30 and self.rx_data[-2]==0xD and self.rx_data[-1]==0xA: 
            return False
        return self.rx_data

    def read(self,size,val):
        self.rx_data = super().read(13+size*3-1)
        self.rx_data = self.rx_data.decode(encoding='utf-8')
        self.rx_NN = int(self.rx_data[3:7],16)
        self.rx_data = self.rx_data[11:-2].replace(',','')
        self.get_data = []
        self.return_data = []
        for i in range(len(self.rx_data)//2):
            self.get_data.append(int(self.rx_data[i*2:(i*2)+2],16))
        for i in range(size//4):
            self.return_data.append(self.from_binary(bytes(self.get_data[i*4:(i+1)*4]),val))
        return self.rx_NN,self.return_data
        
    def to_binary(self,val):
        #int、float型をバイナリー変換
        if type(val) is int:
            binary = struct.pack('<i',val)
        elif type(val) is float:
            binary = struct.pack('<f',val)
        else:
            binary = None
        return binary

    def from_binary(self,val,type_):
        #バイナリーデータをint、float型に
        if type_ is int:
            data = struct.unpack('<i',val)[0]
        elif type_ is float:
            data = struct.unpack('<f',val)[0]
        else:
            data = None
        return data

if __name__ == "__main__":
    im920 = IM920("COM13")
    count = 0.5
    while True:
        #im920.write([count,count*(-1)])
        count += 0.03
        num,data = im920.read(8,float)
        print(data)
        #time.sleep(0.5)