#!/usr/bin/python3
# from ipcqueue import posixmq
# from time import sleep
# from ipcqueue.serializers import RawSerializer
# ipcqueue.posixmq.unlink('/test_queue')
# q1 = posixmq.Queue('/test_queue')
# #q1.put(b'tt00',pickle_protocol=0)
# q1.put()
# print(r'tt00')
# q1.close()
# 
import codecs
from ipcqueue import posixmq
from ipcqueue.serializers import RawSerializer

# q = posixmq.Queue('/test_queue', serializer=RawSerializer)
# q.put(codecs.encode('A000B\n\0', 'utf_8'))
# 
# print (q.qsize())
# q.close()
#q.unlink()


def Send_STTMsg(strMsg, strQ_Name='/STT_Q'): 
  q = posixmq.Queue(strQ_Name, serializer=RawSerializer)
  q.put(codecs.encode(strMsg, 'utf_8'))
  q.close()
  print("Send Msg : " + strMsg)





