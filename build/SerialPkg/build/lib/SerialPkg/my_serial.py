import serial
import struct
from decimal import Decimal

def to_binary(val):
   #int、float型をバイナリー変換
   if type(val) is int:
      binary = struct.pack('<i',val)
   elif type(val) is float:
      binary = struct.pack('<f',val)
   else:
      binary = None
   return binary

def from_binary(val,type_):
   #バイナリーデータをint、float型に
   if type_ is int:
      data = struct.unpack('<i',val)[0]
   elif type_ is float:
      data = struct.unpack('<f',val)[0]
   else:
      data = None
   return data

class MySerial(serial.Serial):
   #バイナリー変換機能を追加したシリアル通信のクラス
   def __init__(self,port,bps):
      super().__init__(port,bps)
      self.__send_binary = 0
      self.__get_binary = 0
      self.__get_data = 0
   
   def write(self,binary):
      #先頭に目印のビットを付け足して送信
      self.__send_binary = binary
      return super().write(self.__send_binary)
      
   def read(self,head_bit,size):
      #受信関数 先頭の目印ビットを取り除いたものを戻り値に
      head_size = len(head_bit)
      self.__get_binary = super().read(size)
      for i in range(0,size):
         #print(self.__get_binary[i:i+head_size])
         if i+head_size >= size:
            if [self.__get_binary[size-1]]+[self.__get_binary[0]] == bytes(head_bit): break
         else:
            if self.__get_binary[i:i+head_size] == bytes(head_bit): break
         #if i == size-1: return False
      self.__get_data = self.__get_binary[i+head_size:size]+self.__get_binary[0:i]
      return self.__get_data

if __name__=="__main__":
   import time
   ser = MySerial('/dev/ttyUSB0',115200)
   a=0.0
   while True:
      #aと-aを送信  それぞれ4バイト長
      a = a + 0.1#Decimal(a) + Decimal('0.1')
      send_data = [int(a),int(-1*a)]
      send_binary = bytes([0x1,0x2,0x3,0x4,0x5,0x6,0x7,0x8])#to_binary(int(a))+to_binary(int(-1*a))
      send_byte = ser.write([0xA5],send_binary)
      time.sleep(0.01)
      
      #データ受信、Arudinoが受け取ったデータをそのまま返してくる
      #get_data = ser.read([0xA5],9)
      '''
      get_int1 = from_binary(get_data[0:4],int)
      get_int2 = from_binary(get_data[4:8],int)
      get_int = [get_int1,get_int2]
      '''
      
      print("送信バイト数:"+str(send_byte),end = "  ")
      print("送信データ:"+str(send_binary))
      #print("受信データ:"+str(get_data))
   ser.close()
