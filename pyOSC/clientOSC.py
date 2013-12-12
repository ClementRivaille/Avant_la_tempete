"""
This program sends 10 random values between 0.0 and 1.0 to the /filter address,
waiting for 1 seconds between each value.
"""
import argparse
import random
import time

from pythonosc import osc_message_builder
from pythonosc import udp_client


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("--ip", default="127.0.0.1",
      help="The ip of the OSC server")
  parser.add_argument("--port", type=int, default=8000,
      help="The port the OSC server is listening on")
  parser.add_argument("--msg", type=int, default="",
      help="The message to send (ie the function to be called)")
  parser.add_argument("--value", type=int, default=0,
      help="The value that will be given to the function")
  args = parser.parse_args()

  client = udp_client.UDPClient(args.ip, args.port)

  for x in range(1):
    msg = osc_message_builder.OscMessageBuilder(address = args.msg)
    msg.add_arg(args.value)
    msg = msg.build()
    client.send(msg)
    time.sleep(1) 
